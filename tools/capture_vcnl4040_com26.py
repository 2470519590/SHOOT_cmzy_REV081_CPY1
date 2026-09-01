#!/usr/bin/env python3
"""Request and plot timestamped five-second raw VCNL4040 captures."""

import argparse
import re
import struct
import sys
import time
from datetime import datetime
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import serial


HEADER_RE = re.compile(
    rb"CAP5_BEGIN CH=([FR]) N=(\d+) ELAPSED_MS=(\d+) RATE=(\d+) CRC16=([0-9A-Fa-f]{4})\r\n"
)


def crc16_ccitt_le(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def read_exact(port: serial.Serial, length: int, timeout_s: float) -> bytes:
    result = bytearray()
    deadline = time.monotonic() + timeout_s
    while len(result) < length and time.monotonic() < deadline:
        block = port.read(length - len(result))
        if block:
            result.extend(block)
    if len(result) != length:
        raise TimeoutError(f"only received {len(result)}/{length} data bytes")
    return bytes(result)


def request_capture(port: serial.Serial, channel: str):
    # Firmware accepts a single byte because the normal application loop has
    # a 1 ms delay; a multi-byte command would overrun its UART receive FIFO.
    port.write(channel.encode("ascii"))
    port.flush()
    print(f"{channel}: MCU is sampling silently for 5 seconds...")
    deadline = time.monotonic() + 8.0
    match = None
    while time.monotonic() < deadline:
        line = port.readline()
        match = HEADER_RE.fullmatch(line)
        if match:
            break
        if line:
            print("MCU:", line.decode("ascii", errors="replace").rstrip())
    if not match:
        raise TimeoutError(f"{channel}: CAP5_BEGIN header was not received")

    received_channel, count, elapsed_ms, rate_hz, expected_crc = match.groups()
    received_channel = received_channel.decode("ascii")
    if received_channel != channel:
        raise RuntimeError(f"expected channel {channel}, received {received_channel}")
    count, elapsed_ms, rate_hz = int(count), int(elapsed_ms), int(rate_hz)
    expected_crc = int(expected_crc, 16)
    raw = read_exact(port, count * 2,
                     max(10.0, count * 2 * 12 / port.baudrate))
    end = port.readline()
    if end != f"CAP5_END CH={channel}\r\n".encode("ascii"):
        raise RuntimeError(f"invalid frame terminator: {end!r}")
    actual_crc = crc16_ccitt_le(raw)
    if actual_crc != expected_crc:
        raise RuntimeError(f"CRC mismatch: MCU={expected_crc:04X}, PC={actual_crc:04X}")
    return count, elapsed_ms, rate_hz, struct.unpack("<" + "H" * count, raw)


def save_and_plot(channel: str, count: int, elapsed_ms: int, rate_hz: int,
                  samples: tuple[int, ...], out_base: Path) -> None:
    times_s = [index * elapsed_ms / 1000.0 / max(count - 1, 1) for index in range(count)]
    out_base.parent.mkdir(parents=True, exist_ok=True)
    csv_path = out_base.with_suffix(".csv")
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        file.write("time_s,ps_data\n")
        file.writelines(f"{timestamp:.8f},{sample}\n" for timestamp, sample in zip(times_s, samples))

    mode, mode_count = Counter(samples).most_common(1)[0]
    bin_width_ms = 10
    bin_count = max(1, (elapsed_ms + bin_width_ms - 1) // bin_width_ms)
    bins = [[] for _ in range(bin_count)]
    for timestamp, sample in zip(times_s, samples):
        bin_index = min(bin_count - 1, int(timestamp * 1000 / bin_width_ms))
        bins[bin_index].append(sample)
    bin_times = []
    bin_means = []
    bin_mins = []
    bin_maxs = []
    for index, values in enumerate(bins):
        if values:
            bin_times.append((index + 0.5) * bin_width_ms / 1000)
            bin_means.append(sum(values) / len(values))
            bin_mins.append(min(values))
            bin_maxs.append(max(values))

    figure = plt.figure(figsize=(13, 8), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, height_ratios=[2.2, 1], width_ratios=[3, 1])
    raw_axis = figure.add_subplot(grid[0, :])
    trend_axis = figure.add_subplot(grid[1, 0])
    hist_axis = figure.add_subplot(grid[1, 1])

    # Do not join samples with lines: integer-valued, fast samples otherwise
    # render as misleading vertical needles.
    raw_axis.scatter(times_s, samples, s=5, marker=".", alpha=0.65,
                     color="#1976d2", rasterized=True, label="raw samples")
    raw_axis.axhline(mode, color="#d32f2f", linewidth=1.2, linestyle="--",
                     label=f"mode = {mode} ({mode_count}/{count})")
    raw_axis.set_xlim(0, elapsed_ms / 1000)
    raw_axis.set_ylabel("PS_DATA")
    raw_axis.set_title(
        f"{'Front' if channel == 'F' else 'Rear'} VCNL4040: "
        f"{count} points, {elapsed_ms} ms, {rate_hz} Hz "
        f"(raw scatter; no connecting lines)"
    )
    raw_axis.grid(True, alpha=0.3)
    raw_axis.legend(loc="upper right")

    trend_axis.fill_between(bin_times, bin_mins, bin_maxs, color="#90caf9",
                            alpha=0.65, label="10 ms min–max")
    trend_axis.plot(bin_times, bin_means, color="#0d47a1", linewidth=1.1,
                    label="10 ms mean")
    trend_axis.axhline(mode, color="#d32f2f", linewidth=1.0, linestyle="--")
    trend_axis.set_xlim(0, elapsed_ms / 1000)
    trend_axis.set_xlabel("Time (s)")
    trend_axis.set_ylabel("PS_DATA")
    trend_axis.set_title("10 ms envelope: reveals slow drift / periodic disturbance")
    trend_axis.grid(True, alpha=0.3)
    trend_axis.legend(loc="upper right")

    values = sorted(Counter(samples).items())
    hist_axis.bar([item[0] for item in values], [item[1] for item in values],
                  width=0.75, color="#43a047")
    hist_axis.set_xlabel("PS_DATA")
    hist_axis.set_ylabel("Count")
    hist_axis.set_title("Value distribution")
    hist_axis.grid(True, axis="y", alpha=0.3)

    png_path = out_base.with_suffix(".png")
    figure.savefig(png_path, dpi=180)
    plt.show()
    print(f"CRC OK. Saved {csv_path} and {png_path}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture 5 s from front, show it, then capture and show rear."
    )
    parser.add_argument("--port", default="COM26")
    parser.add_argument("--baud", type=int, default=38400)
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent / "logs",
                        help="base output directory; a new timestamped folder is created")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        port.reset_input_buffer()
        for channel, name in (("F", "front"), ("R", "rear")):
            capture = request_capture(port, channel)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = args.out_dir / f"vcnl4040_single_{name}_{timestamp}"
            output_dir.mkdir(parents=True, exist_ok=False)
            save_and_plot(channel, *capture, output_dir / f"capture_{name}_5s")
    return 0
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (serial.SerialException, TimeoutError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
