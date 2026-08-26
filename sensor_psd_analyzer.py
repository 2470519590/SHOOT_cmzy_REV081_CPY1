"""Capture and diagnose the two VCNL4040 channels sent as VOFA+ JustFloat.

Firmware frame format (little endian):
    float front_ps_data, float rear_ps_data, 0x00 0x00 0x80 0x7F

Example:
    conda run -n py310 python sensor_psd_analyzer.py --port COM12 --seconds 60
"""

from __future__ import annotations

import argparse
import csv
import struct
import time
from pathlib import Path

import numpy as np
from scipy import signal

try:
    import serial
except ImportError as exc:  # pragma: no cover - depends on local environment
    raise SystemExit("缺少 pyserial，请在 py310 环境安装: python -m pip install pyserial") from exc


TAIL = b"\x00\x00\x80\x7f"
FRAME_SIZE = 12


def capture(port: str, baud: int, seconds: float, max_samples: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Capture frames and return host time, front, rear, and bad-frame count."""
    raw = bytearray()
    timestamps: list[float] = []
    front: list[float] = []
    rear: list[float] = []
    bad_frames = 0
    deadline = time.monotonic() + seconds

    print(f"打开 {port} @ {baud} baud，采集 {seconds:g} s；按 Ctrl+C 可提前结束")
    with serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N", stopbits=1,
                       timeout=0.1) as ser:
        ser.reset_input_buffer()
        while time.monotonic() < deadline and len(front) < max_samples:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            raw.extend(chunk)
            while True:
                pos = raw.find(TAIL)
                if pos < 8:
                    if len(raw) > 16:
                        del raw[:-16]
                    break
                if pos + 4 > len(raw):
                    break
                frame_start = pos - 8
                if frame_start + FRAME_SIZE > len(raw):
                    break
                try:
                    f, r = struct.unpack_from("<ff", raw, frame_start)
                except struct.error:
                    break
                del raw[:frame_start + FRAME_SIZE]
                if not (np.isfinite(f) and np.isfinite(r)):
                    bad_frames += 1
                    continue
                timestamps.append(time.perf_counter())
                front.append(float(f))
                rear.append(float(r))
            if len(front) and len(front) % 1000 == 0:
                print(f"已收到 {len(front)} 帧", end="\r")

    print(f"\n有效帧: {len(front)}，异常帧: {bad_frames}")
    return (np.asarray(timestamps), np.asarray(front), np.asarray(rear), bad_frames)


def dominant_peaks(x: np.ndarray, fs: float, max_hz: float = 100.0) -> list[tuple[float, float]]:
    """Return strongest non-DC Welch peaks as (Hz, relative power)."""
    if len(x) < 32:
        return []
    nperseg = min(16384, max(256, len(x) // 4))
    freqs, power = signal.welch(x, fs=fs, window="hann", nperseg=nperseg,
                                noverlap=nperseg // 2, detrend="constant")
    mask = (freqs > 0.05) & (freqs <= max_hz)
    if not np.any(mask):
        return []
    peaks, _ = signal.find_peaks(power[mask], prominence=np.max(power[mask]) * 0.01)
    peak_indices = np.flatnonzero(mask)[peaks]
    strongest = sorted(peak_indices, key=lambda i: power[i], reverse=True)[:8]
    total = np.sum(power[mask]) or 1.0
    return [(float(freqs[i]), float(power[i] / total)) for i in strongest]


def analyze(t: np.ndarray, front: np.ndarray, rear: np.ndarray) -> dict:
    if len(front) < 32:
        raise ValueError("有效样本少于 32，无法进行频谱分析")
    dt = np.diff(t)
    dt = dt[(dt > 0) & (dt < 1.0)]
    fs = 1.0 / float(np.median(dt)) if len(dt) else 0.0
    if not (1.0 <= fs <= 10000.0):
        raise ValueError(f"主机接收采样率异常: {fs:.2f} Hz")

    n = min(len(front), len(rear))
    f = front[:n]
    r = rear[:n]
    f0 = signal.detrend(f)
    r0 = signal.detrend(r)
    corr = float(np.corrcoef(f0, r0)[0, 1])
    common = (f0 + r0) / 2.0
    differential = f0 - r0
    common_rms = float(np.std(common))
    differential_rms = float(np.std(differential))

    # Cross-correlation lag, limited to +/- 100 ms to avoid spurious long lags.
    max_lag = min(len(f0) - 1, int(fs * 0.1))
    xc = signal.correlate(f0, r0, mode="full", method="fft")
    lags = signal.correlation_lags(len(f0), len(r0), mode="full")
    local = np.abs(lags) <= max_lag
    lag_samples = int(lags[local][np.argmax(xc[local])])

    return {
        "samples": n,
        "duration_s": float(t[n - 1] - t[0]),
        "sample_rate_hz": fs,
        "front_mean": float(np.mean(f)),
        "rear_mean": float(np.mean(r)),
        "front_std": float(np.std(f)),
        "rear_std": float(np.std(r)),
        "correlation": corr,
        "common_rms": common_rms,
        "differential_rms": differential_rms,
        "common_ratio": common_rms / (differential_rms + 1e-12),
        "correlation_lag_ms": float(lag_samples / fs * 1000.0),
        "front_peaks": dominant_peaks(f0, fs),
        "rear_peaks": dominant_peaks(r0, fs),
    }


def diagnosis(a: dict) -> list[str]:
    notes: list[str] = []
    corr = a["correlation"]
    ratio = a["common_ratio"]
    if corr >= 0.8 and ratio >= 2.0:
        notes.append("两路高度同步且共模分量明显：优先怀疑电源/地弹噪声、IRED电流或共享光学环境。")
    elif corr >= 0.5:
        notes.append("两路有明显相关性：可能存在共享电源、机械光学或串口采集链路影响；需结合频率峰判断。")
    else:
        notes.append("两路相关性低：更像单个传感器、接线/I2C或局部光学问题，而不是共同干扰源。")
    if abs(a["correlation_lag_ms"]) < 2.0:
        notes.append("两路相关峰接近零延迟，支持共同干扰的判断。")
    else:
        notes.append(f"两路相关峰延迟约 {a['correlation_lag_ms']:.2f} ms，可能是采样时序或传感器响应差异。")
    for channel in ("front_peaks", "rear_peaks"):
        peaks = a[channel]
        low = [(hz, p) for hz, p in peaks if 0.5 <= hz <= 2.0]
        if low:
            hz, p = low[0]
            notes.append(f"{channel.replace('_peaks', '')} 在 {hz:.3f} Hz 附近有周期峰（约 {1/hz:.2f} s），与图中的约1秒扰动吻合。")
    notes.append("若关闭所有WS2812后该峰消失，重点检查灯条供电、共地、去耦和IRED电流；若峰仍在，再检查USB转串口/VOFA解析与传感器供电。")
    return notes


def save_outputs(out_dir: Path, t: np.ndarray, front: np.ndarray, rear: np.ndarray, a: dict) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "sensor_capture.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        writer.writerow(["host_time_s", "front_ps_data", "rear_ps_data"])
        t0 = t[0]
        writer.writerows((float(ts - t0), float(f), float(r)) for ts, f, r in zip(t, front, rear))

    report_path = out_dir / "sensor_psd_report.txt"
    with report_path.open("w", encoding="utf-8") as fp:
        for k, v in a.items():
            fp.write(f"{k}: {v}\n")
        fp.write("\n诊断:\n")
        fp.write("\n".join(f"- {line}" for line in diagnosis(a)))
        fp.write("\n")

    try:
        import matplotlib.pyplot as plt
        fs = a["sample_rate_hz"]
        nperseg = min(16384, max(256, len(front) // 4))
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), constrained_layout=True)
        rel_t = t - t[0]
        axes[0].plot(rel_t, front, label="front", linewidth=0.7)
        axes[0].plot(rel_t, rear, label="rear", linewidth=0.7)
        axes[0].set(xlabel="time (s)", ylabel="PS_DATA", title="Raw sensor channels")
        axes[0].legend(); axes[0].grid(True, alpha=0.3)
        for x, name in ((front, "front"), (rear, "rear")):
            f, p = signal.welch(signal.detrend(x), fs=fs, window="hann", nperseg=nperseg,
                                noverlap=nperseg // 2)
            axes[1].semilogy(f[1:], p[1:], label=name)
        axes[1].set_xlim(0, min(20, fs / 2)); axes[1].set(xlabel="frequency (Hz)", ylabel="PSD", title="Welch power spectral density")
        axes[1].legend(); axes[1].grid(True, alpha=0.3)
        axes[2].plot(rel_t, (front - np.mean(front) + rear - np.mean(rear)) / 2, label="common")
        axes[2].plot(rel_t, (front - np.mean(front)) - (rear - np.mean(rear)), label="differential", alpha=0.8)
        axes[2].set(xlabel="time (s)", ylabel="deviation", title="Common-mode vs differential")
        axes[2].legend(); axes[2].grid(True, alpha=0.3)
        fig.savefig(out_dir / "sensor_psd.png", dpi=150)
        plt.close(fig)
    except Exception as exc:
        print(f"绘图失败（不影响CSV和报告）: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="例如 COM12")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--out", type=Path, default=Path("sensor_analysis"))
    parser.add_argument("--max-samples", type=int, default=2_000_000)
    args = parser.parse_args()
    try:
        t, front, rear, _ = capture(args.port, args.baud, args.seconds, args.max_samples)
    except KeyboardInterrupt:
        raise SystemExit("\n用户提前结束采集")
    except serial.SerialException as exc:
        raise SystemExit(f"串口打开失败: {exc}") from exc
    analysis = analyze(t, front, rear)
    save_outputs(args.out, t, front, rear, analysis)
    print("\n分析结果:")
    for k, v in analysis.items():
        print(f"{k}: {v}")
    print("\n诊断:")
    for line in diagnosis(analysis):
        print(f"- {line}")
    print(f"\n输出目录: {args.out.resolve()}")


if __name__ == "__main__":
    main()
