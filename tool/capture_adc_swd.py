#!/usr/bin/env python3
"""Capture 20 kHz dual-channel ADC data through SWD/J-Link.

The target firmware exposes a RAM buffer and three control words. This script
never uses USART or Ozone: it arms capture through J-Link, waits for the DMA
completion flag, saves the RAM buffer, and writes a CSV with physical names.
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

FRAMES = 2000
WORDS = FRAMES * 2
BYTES = WORDS * 2
DEVICE = "STM32F072C8"

def find_tool(name: str, explicit: str | None = None) -> str:
    if explicit:
        return explicit
    # Windows often has Java's `jlink.exe` earlier on PATH. It is unrelated
    # to SEGGER J-Link and does not accept -device/-CommanderScript.
    if name.lower() == "jlink.exe":
        segger_candidates = [
            Path(r"C:\Program Files\SEGGER\JLink\JLink.exe"),
            Path(r"C:\Program Files\JLink_V960\JLink.exe"),
            Path(r"C:\Program Files (x86)\SEGGER\JLink\JLink.exe"),
        ]
        for item in segger_candidates:
            if item.exists():
                return str(item)
        found = shutil.which(name)
        if found and Path(found).name.lower() == "jlink.exe" and "java" not in found.lower():
            return found
        raise FileNotFoundError("找不到 SEGGER JLink.exe；请使用 --jlink 指定，例如 C:\\Program Files\\JLink_V960\\JLink.exe")
    found = shutil.which(name)
    if found:
        return found
    candidates = [
        Path(r"C:\Program Files\SEGGER\JLink\JLink.exe"),
        Path(r"C:\Program Files\SEGGER\JLink_V*\JLink.exe"),
        Path(r"E:\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-nm.exe"),
    ]
    for item in candidates:
        if "*" in str(item):
            matches = list(item.parent.glob(item.name))
            if matches:
                return str(matches[0])
        elif item.exists() and item.name.lower() == name.lower():
            return str(item)
    raise FileNotFoundError(f"找不到 {name}，请用命令行参数指定路径")

def symbols_from_elf(elf: Path, nm: str) -> dict[str, int]:
    out = subprocess.check_output([nm, "-n", str(elf)], text=True, errors="replace")
    wanted = {
        "ir_raw_capture_dma", "ir_raw_capture_arm",
        "ir_raw_capture_active", "ir_raw_capture_done",
        "ir_raw_capture_frame_count", "ir_raw_capture_error_count",
        "ir_raw_capture_triggered", "ir_raw_capture_trigger_frame",
        "ir_raw_capture_start_word", "ir_raw_capture_trigger_index",
        "ir_raw_capture_valid_frames", "ir_raw_capture_baseline_front",
        "ir_raw_capture_baseline_rear",
    }
    result: dict[str, int] = {}
    for line in out.splitlines():
        m = re.match(r"^\s*([0-9a-fA-F]+)\s+\S\s+(\S+)\s*$", line)
        if m and m.group(2) in wanted:
            result[m.group(2)] = int(m.group(1), 16)
    missing = wanted - result.keys()
    if missing:
        raise RuntimeError("ELF 缺少捕获符号: " + ", ".join(sorted(missing)))
    return result

def jlink_call(jlink: str, commands: list[str], verbose: bool = False) -> str:
    with tempfile.NamedTemporaryFile("w", suffix=".jlink", delete=False, encoding="ascii") as f:
        f.write("\n".join(commands) + "\n")
        script = f.name
    try:
        p = subprocess.run(
            [jlink, "-device", DEVICE, "-if", "SWD", "-speed", "4000",
             "-autoconnect", "1", "-CommanderScript", script],
            capture_output=True, text=True, errors="replace", check=False,
        )
        output = p.stdout + p.stderr
        if verbose:
            print(output)
        if p.returncode != 0 or "ERROR" in output.upper():
            raise RuntimeError("J-Link 命令失败:\n" + output)
        return output
    finally:
        Path(script).unlink(missing_ok=True)

def write_word(jlink: str, address: int, value: int, verbose: bool) -> None:
    jlink_call(jlink, [f"w4 0x{address:08X}, 0x{value:08X}", "exit"], verbose)

def read_word(jlink: str, address: int, verbose: bool) -> int:
    output = jlink_call(jlink, [f"mem32 0x{address:08X}, 1", "exit"], verbose)
    # J-Link Commander commonly prints `20000058 = 00000001` without 0x.
    values = re.findall(r"(?:=|0x[0-9A-Fa-f]{8}\s*=?)\s*([0-9A-Fa-f]{8})\b", output)
    if not values:
        values = re.findall(r"0x([0-9A-Fa-f]{8})", output)
    if not values:
        raise RuntimeError(f"无法从 J-Link 读取 0x{address:08X}")
    return int(values[-1], 16)

def main() -> int:
    ap = argparse.ArgumentParser(description="通过 SWD/J-Link 采集 20 kHz 双路 ADC 原始波形")
    ap.add_argument("--elf", type=Path, required=True, help="与正在烧录固件一致的 ELF")
    ap.add_argument("--count", type=int, default=1, help="连续触发采集次数，默认 1")
    ap.add_argument("--out", type=Path, default=Path("ir_capture"), help="输出前缀")
    ap.add_argument("--jlink", help="JLink.exe 路径")
    ap.add_argument("--nm", help="arm-none-eabi-nm 路径")
    ap.add_argument("--flash", action="store_true", help="先用同名 HEX 通过 J-Link 烧录，再开始采集")
    ap.add_argument("--hex", type=Path, help="--flash 时指定 HEX；默认取 ELF 同名 HEX")
    ap.add_argument("--poll-ms", type=int, default=20)
    ap.add_argument("--timeout-s", type=float, default=30.0, help="等待真实 ADC 触发的最长秒数")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    if args.count < 1 or args.timeout_s <= 0:
        ap.error("--count 必须 >=1，--timeout-s 必须 >0（每次采集独立计时）")

    jlink = find_tool("JLink.exe", args.jlink)
    nm = find_tool("arm-none-eabi-nm.exe", args.nm)
    symbols = symbols_from_elf(args.elf, nm)
    print(f"捕获缓冲区: 0x{symbols['ir_raw_capture_dma']:08X}, {FRAMES} 帧, {BYTES} 字节")
    if args.flash:
        hex_path = args.hex or args.elf.with_suffix(".hex")
        if not hex_path.exists():
            raise FileNotFoundError(f"找不到烧录文件: {hex_path}")
        print(f"开始烧录: {hex_path}")
        jlink_call(jlink, [f"loadfile {hex_path.resolve()}", "r", "g", "sleep 100", "exit"], args.verbose)
        print("烧录完成，目标保持运行，准备发出采样允许")

    # Unique session and sequence names preserve every completed capture.
    session = args.out.with_name(args.out.name + "_" + str(time.time_ns()))
    print(f"计划采集 {args.count} 次；请在‘可以发射’提示后发射，保存期间不会采集。", flush=True)
    for number in range(1, args.count + 1):
        prefix = session.with_name(session.name + f"_{number:03d}")
        capture_once(args, jlink, symbols, prefix, number)
        print(f"进度：已保存 {number}/{args.count}，剩余 {args.count-number} 次", flush=True)
    print(f"全部完成：{args.count} 份记录；前缀 {session}", flush=True)
    return 0

def capture_once(args, jlink: str, symbols: dict[str, int], prefix: Path, number: int) -> None:
    # Clear previous status, then explicitly arm from the PC.
    write_word(jlink, symbols["ir_raw_capture_done"], 0, args.verbose)
    write_word(jlink, symbols["ir_raw_capture_frame_count"], 0, args.verbose)
    write_word(jlink, symbols["ir_raw_capture_arm"], 1, args.verbose)
    print(f"[{number}/{args.count}] 已允许采样，正在准备预触发数据...", flush=True)

    deadline = time.monotonic() + args.timeout_s
    ready_reported = False
    active_since = None
    last_progress = time.monotonic()
    while time.monotonic() < deadline:
        done = read_word(jlink, symbols["ir_raw_capture_done"], args.verbose)
        if done != 0:
            break
        if not ready_reported:
            # frame_count is only published when capture finishes, not while filling.
            # After firmware acknowledges active, allow 50 ms for 25 ms prehistory.
            active = read_word(jlink, symbols["ir_raw_capture_active"], args.verbose)
            if active and active_since is None:
                active_since = time.monotonic()
            if active and active_since is not None and time.monotonic() - active_since >= 0.05:
                ready_reported = True
                print(f"[{number}/{args.count}] 可以发射；已保存 {number-1}，还需 {args.count-number+1} 次采集", flush=True)
        if time.monotonic() - last_progress >= 5:
            print(f"[{number}/{args.count}] 仍在等待触发；还需 {args.count-number+1} 次采集", flush=True)
            last_progress = time.monotonic()
        time.sleep(max(args.poll_ms, 1) / 1000.0)
    else:
        active = read_word(jlink, symbols["ir_raw_capture_active"], args.verbose)
        errors = read_word(jlink, symbols["ir_raw_capture_error_count"], args.verbose)
        raise TimeoutError(f"第 {number}/{args.count} 次采集超时，已保存 {number-1} 次: active={active}, error_count={errors}；可增大 --timeout-s")

    print(f"[{number}/{args.count}] DMA 已完成，正在保存；请等待下一次允许提示", flush=True)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    ring_path = prefix.with_name(prefix.name + "_ring_" + str(int(time.time() * 1000)) + ".bin")
    bin_path = prefix.with_suffix(".bin")
    csv_path = prefix.with_suffix(".csv")
    # J-Link SaveBin may overwrite an existing file without truncating it.
    # Remove old outputs first so the byte-length check is meaningful.
    ring_path.unlink(missing_ok=True)
    bin_path.unlink(missing_ok=True)
    csv_path.unlink(missing_ok=True)
    # Use an explicit hexadecimal byte count; J-Link Commander accepts this
    # unambiguously across versions.
    save_command = f"savebin {ring_path.resolve()}, 0x{symbols['ir_raw_capture_dma']:08X}, 0x{BYTES:08X}"
    if args.verbose:
        print(f"J-Link: {save_command}")
    jlink_call(jlink, [save_command, "exit"], args.verbose)
    data = ring_path.read_bytes()
    if len(data) != BYTES:
        raise RuntimeError(f"导出长度错误: {len(data)}，期望 {BYTES}")
    ring_words = struct.unpack("<%dH" % WORDS, data)
    start_word = read_word(jlink, symbols["ir_raw_capture_start_word"], args.verbose)
    trigger_index = read_word(jlink, symbols["ir_raw_capture_trigger_index"], args.verbose)
    valid_frames = read_word(jlink, symbols["ir_raw_capture_valid_frames"], args.verbose)
    if valid_frames != FRAMES or start_word >= WORDS or start_word % 2 or not 0 <= trigger_index < FRAMES:
        raise RuntimeError(f"采集元数据错误: valid_frames={valid_frames}, start_word={start_word}")
    words = tuple(ring_words[(start_word + i) % WORDS] for i in range(WORDS))
    ordered = struct.pack("<%dH" % WORDS, *words)
    bin_path.write_bytes(ordered)
    with csv_path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["sample_index", "time_us", "barrel_front_adc", "barrel_rear_adc"])
        for i in range(FRAMES):
            et1, et2 = words[2 * i], words[2 * i + 1]
            # Raw DMA order is [ET1, ET2]; ET1 is physical rear and ET2 front.
            writer.writerow([i, i * 50, et2, et1])
    png_path = prefix.with_suffix(".png")
    try:
        plot_script = Path(__file__).with_name("plot_adc_capture.py")
        subprocess.run(
            [sys.executable, str(plot_script), str(csv_path), "--out", str(png_path)],
            check=True,
        )
        print(f"完成: {png_path}")
    except Exception as exc:
        # Keep the raw capture even if the optional desktop plotting runtime is
        # unavailable; the capture itself is already complete.
        print(f"警告: PNG 绘图失败，原始 CSV 已保留: {exc}", file=sys.stderr)
    prefix.with_suffix(".json").write_text(json.dumps({
        "ring_file": ring_path.name, "csv_file": csv_path.name,
        "start_word": start_word, "trigger_index": trigger_index,
        "valid_frames": valid_frames, "sample_interval_us": 50,
        "capture_number": number, "capture_total": args.count,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"触发索引: {trigger_index} 帧 ({trigger_index * 50} us)")
    print(f"完成: {bin_path}")
    print(f"环形原始数据: {ring_path}")
    print(f"完成: {csv_path}")

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"错误: {exc}", file=sys.stderr)
        raise SystemExit(1)
