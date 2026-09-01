#!/usr/bin/env python3
"""Request a 10 s, 200 Hz dual-VCNL4040 capture and analyze both channels."""

import argparse
from datetime import datetime
import re
import struct
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from scipy import signal


HEADER_RE = re.compile(
    rb"CAP10_200_BEGIN N=(\d+) ELAPSED_MS=(\d+) PERIOD_MS=(\d+) CRC16=([0-9A-Fa-f]{4})\r\n"
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


def capture(port: serial.Serial, command: bytes = b"D"):
    port.reset_input_buffer()
    port.write(command)
    port.flush()
    description = "phase-restart test" if command == b"P" else "normal test"
    print(f"MCU is sampling both channels at 200 Hz for 10 seconds ({description})...")
    deadline = time.monotonic() + 13.0
    match = None
    while time.monotonic() < deadline:
        line = port.readline()
        match = HEADER_RE.fullmatch(line)
        if match:
            break
        if line:
            print("MCU:", line.decode("ascii", errors="replace").rstrip())
    if not match:
        raise TimeoutError("CAP10_200_BEGIN header was not received")

    count, elapsed_ms, period_ms, expected_crc = match.groups()
    count, elapsed_ms, period_ms = int(count), int(elapsed_ms), int(period_ms)
    expected_crc = int(expected_crc, 16)
    raw = read_exact(port, count * 4, max(10.0, count * 4 * 12 / port.baudrate))
    end = port.readline()
    if end != b"CAP10_200_END\r\n":
        raise RuntimeError(f"invalid frame terminator: {end!r}")
    actual_crc = crc16_ccitt_le(raw)
    if actual_crc != expected_crc:
        raise RuntimeError(f"CRC mismatch: MCU={expected_crc:04X}, PC={actual_crc:04X}")
    values = struct.unpack("<" + "H" * (count * 2), raw)
    return elapsed_ms, period_ms, values[:count], values[count:]


def strongest_peaks(frequency: np.ndarray, power: np.ndarray) -> list[tuple[float, float]]:
    mask = (frequency >= 0.1) & (frequency <= 10.0)
    peaks, _ = signal.find_peaks(power[mask])
    indices = np.flatnonzero(mask)[peaks]
    return [(float(frequency[index]), float(power[index]))
            for index in sorted(indices, key=lambda item: power[item], reverse=True)[:8]]


def analyze_low_frequency(samples: np.ndarray, sample_rate_hz: float) -> tuple[np.ndarray, np.ndarray, dict]:
    """Find the dominant low-frequency component and measure cycle-to-cycle stability."""
    detrended = signal.detrend(samples)
    nfft = max(4096, 1 << (len(samples) - 1).bit_length())
    frequency, power = signal.periodogram(detrended, fs=sample_rate_hz, window="hann",
                                          scaling="density", nfft=nfft)
    # The suspected disturbance is near 2 Hz.  Limiting the fundamental search
    # avoids selecting its stronger 2nd/3rd/... harmonics from a sharp pulse.
    search = (frequency >= 0.8) & (frequency <= 3.0)
    dominant_frequency = float(frequency[search][np.argmax(power[search])])
    dominant_period_s = 1.0 / dominant_frequency

    sos = signal.butter(2, (0.4, 6.0), btype="bandpass", fs=sample_rate_hz, output="sos")
    filtered = signal.sosfiltfilt(sos, detrended)
    valleys, _ = signal.find_peaks(
        -filtered,
        distance=max(1, round(sample_rate_hz * dominant_period_s * 0.55)),
        prominence=max(0.25, float(np.std(filtered)) * 0.7),
    )
    intervals_s = np.diff(valleys) / sample_rate_hz
    metrics = {
        "dominant_psd_frequency_hz": dominant_frequency,
        "dominant_psd_period_s": dominant_period_s,
        "valley_count": int(len(valleys)),
        "valley_times_s": valleys / sample_rate_hz,
        "filtered": filtered,
        "intervals_s": intervals_s,
    }
    if len(intervals_s):
        mean_period_s = float(np.mean(intervals_s))
        relative_error_pct = (intervals_s - mean_period_s) / mean_period_s * 100.0
        metrics.update({
            "mean_period_s": mean_period_s,
            "mean_frequency_hz": 1.0 / mean_period_s,
            "period_std_ms": float(np.std(intervals_s, ddof=1) * 1000.0) if len(intervals_s) > 1 else 0.0,
            "period_min_s": float(np.min(intervals_s)),
            "period_max_s": float(np.max(intervals_s)),
            "max_abs_period_error_pct": float(np.max(np.abs(relative_error_pct))),
            "intervals_over_5pct": int(np.count_nonzero(np.abs(relative_error_pct) > 5.0)),
            "intervals_total": int(len(intervals_s)),
        })
    return frequency, power, metrics


def write_stability_report(file, channel_name: str, metrics: dict) -> None:
    file.write(f"{channel_name}_low_frequency_stability:\n")
    file.write(f"  dominant_psd_frequency_hz: {metrics['dominant_psd_frequency_hz']:.4f}\n")
    file.write(f"  dominant_psd_period_s: {metrics['dominant_psd_period_s']:.4f}\n")
    file.write(f"  detected_valleys: {metrics['valley_count']}\n")
    if 'mean_period_s' not in metrics:
        file.write("  stability: insufficient valleys to measure\n\n")
        return
    file.write(f"  mean_cycle_frequency_hz: {metrics['mean_frequency_hz']:.4f}\n")
    file.write(f"  mean_cycle_period_s: {metrics['mean_period_s']:.4f}\n")
    file.write(f"  cycle_period_std_ms: {metrics['period_std_ms']:.2f}\n")
    file.write(f"  cycle_period_range_s: {metrics['period_min_s']:.4f} .. {metrics['period_max_s']:.4f}\n")
    file.write(f"  max_abs_period_error_from_mean_pct: {metrics['max_abs_period_error_pct']:.2f}\n")
    file.write(f"  intervals_over_5pct: {metrics['intervals_over_5pct']}/{metrics['intervals_total']}\n")
    file.write(f"  within_5pct: {'yes' if metrics['intervals_over_5pct'] == 0 else 'no'}\n\n")


def save_and_analyze(elapsed_ms: int, period_ms: int, front, rear, out_dir: Path) -> None:
    count = len(front)
    times_s = np.arange(count, dtype=float) * period_ms / 1000.0
    front = np.asarray(front, dtype=float)
    rear = np.asarray(rear, dtype=float)
    sample_rate_hz = 1000.0 / period_ms
    out_dir.mkdir(parents=True, exist_ok=False)
    csv_path = out_dir / "capture.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        file.write("time_s,front_ps_data,rear_ps_data\n")
        file.writelines(f"{t:.3f},{f},{r}\n" for t, f, r in zip(times_s, front, rear))

    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(14, 7), constrained_layout=True)
    for axis, samples, label, color in (
        (axes[0], front, "Front (I2C1)", "#1565c0"),
        (axes[1], rear, "Rear (I2C2)", "#c62828"),
    ):
        axis.plot(times_s, samples, marker=".", markersize=2.5, linewidth=0.7, color=color)
        axis.set_ylabel("PS_DATA")
        axis.set_title(label)
        axis.grid(True, alpha=0.3)
    axes[1].set_xlabel("Time (s)")
    figure.suptitle(f"VCNL4040 dual capture: {count} pairs, nominal 200 Hz, elapsed {elapsed_ms} ms")
    time_png_path = out_dir / "time_domain.png"
    figure.savefig(time_png_path, dpi=180)

    front_detrended = signal.detrend(front)
    rear_detrended = signal.detrend(rear)
    frequency, front_psd, front_stability = analyze_low_frequency(front, sample_rate_hz)
    _, rear_psd, rear_stability = analyze_low_frequency(rear, sample_rate_hz)
    if np.std(front_detrended) > 1e-12 and np.std(rear_detrended) > 1e-12:
        coherence_frequency, coherence = signal.coherence(
            front_detrended, rear_detrended, fs=sample_rate_hz,
            nperseg=min(512, count))
        coherence_label = "Front/rear coherence"
    else:
        # A powered-down VCNL can legitimately return a constant PS_DATA value.
        # Coherence is undefined when either channel has zero variance.
        coherence_frequency = np.linspace(0.0, sample_rate_hz / 2.0, 2)
        coherence = np.full(2, np.nan)
        coherence_label = "Coherence undefined (one channel is constant)"
    front_peaks = strongest_peaks(frequency, front_psd)
    rear_peaks = strongest_peaks(frequency, rear_psd)

    spectrum, spectrum_axes = plt.subplots(3, 1, figsize=(14, 10), constrained_layout=True)
    spectrum_axes[0].semilogy(frequency[1:], front_psd[1:], color="#1565c0", label="Front PSD")
    spectrum_axes[0].semilogy(frequency[1:], rear_psd[1:], color="#c62828", label="Rear PSD")
    for hz, _ in front_peaks[:3]:
        spectrum_axes[0].axvline(hz, color="#1565c0", alpha=0.2, linewidth=0.8)
    for hz, _ in rear_peaks[:3]:
        spectrum_axes[0].axvline(hz, color="#c62828", alpha=0.2, linewidth=0.8)
    spectrum_axes[0].set(xlim=(0.1, 10), ylabel="PSD (count²/Hz)", title="Detailed low-frequency distribution (0.1–10 Hz)")
    spectrum_axes[0].legend()
    spectrum_axes[0].grid(True, alpha=0.3)
    spectrum_axes[1].plot(coherence_frequency, coherence, color="#6a1b9a")
    spectrum_axes[1].set(xlim=(0.1, 10), ylim=(0, 1.05),
                         ylabel="Coherence", title=coherence_label)
    spectrum_axes[1].grid(True, alpha=0.3)
    spectrum_axes[2].plot(times_s, front_stability['filtered'], color="#1565c0", label="Front filtered")
    spectrum_axes[2].plot(times_s, rear_stability['filtered'], color="#c62828", label="Rear filtered")
    for channel, color in ((front_stability, "#1565c0"), (rear_stability, "#c62828")):
        for valley_time in channel['valley_times_s']:
            spectrum_axes[2].axvline(valley_time, color=color, alpha=0.25, linewidth=0.8)
    spectrum_axes[2].set(xlabel="Time (s)", ylabel="Filtered PS_DATA", title="Detected downward-pulse times for cycle stability")
    spectrum_axes[2].grid(True, alpha=0.3)
    spectrum_axes[2].legend()
    spectrum_png_path = out_dir / "frequency_analysis.png"
    spectrum.savefig(spectrum_png_path, dpi=180)

    report_path = out_dir / "frequency_report.txt"
    if np.std(front_detrended) > 1e-12 and np.std(rear_detrended) > 1e-12:
        correlation_text = f"{float(np.corrcoef(front_detrended, rear_detrended)[0, 1]):.6f}"
    else:
        correlation_text = "undefined (one channel is constant)"
    with report_path.open("w", encoding="utf-8") as file:
        file.write(f"samples_per_channel: {count}\n")
        file.write(f"elapsed_ms: {elapsed_ms}\n")
        file.write(f"sample_rate_hz: {sample_rate_hz:.6f}\n")
        file.write(f"front_rear_correlation: {correlation_text}\n\n")
        file.write("front_psd_peaks_hz_power:\n")
        file.writelines(f"  {hz:.3f} Hz, {power:.8g}\n" for hz, power in front_peaks)
        file.write("rear_psd_peaks_hz_power:\n")
        file.writelines(f"  {hz:.3f} Hz, {power:.8g}\n" for hz, power in rear_peaks)
        file.write("\n")
        write_stability_report(file, "front", front_stability)
        write_stability_report(file, "rear", rear_stability)
    print(f"CRC OK. Saved capture and frequency analysis in {out_dir}")
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture 10 s of both VCNL4040 channels at 200 Hz.")
    parser.add_argument("--port", default="COM26")
    parser.add_argument("--baud", type=int, default=38400)
    parser.add_argument("--command", choices=("D", "P"), default="D",
                        help="D: normal capture; P: controlled PS restart with rear delayed 100 ms")
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent / "logs",
                        help="base output directory; a new timestamped folder is created")
    args = parser.parse_args()
    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        capture_result = capture(port, args.command.encode("ascii"))
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.out_dir / f"vcnl4040_dual_200hz_{timestamp}"
    suffix = 1
    while output_dir.exists():
        output_dir = args.out_dir / f"vcnl4040_dual_200hz_{timestamp}_{suffix:02d}"
        suffix += 1
    save_and_analyze(*capture_result, output_dir)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (serial.SerialException, TimeoutError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
