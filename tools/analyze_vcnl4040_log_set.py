#!/usr/bin/env python3
"""Batch-analyze timestamped dual-VCNL4040 200 Hz captures."""

import argparse
import csv
from datetime import datetime
from pathlib import Path

import numpy as np

from capture_vcnl4040_dual_200hz import analyze_low_frequency


def format_percent(value: float) -> str:
    return f"{value:.2f}%"


def summarize_channel(rows: list[dict], channel: str) -> list[str]:
    selected = [row for row in rows if row["channel"] == channel]
    frequencies = np.asarray([row["mean_frequency_hz"] for row in selected])
    intervals = np.concatenate([row["intervals_s"] for row in selected])
    global_period_s = float(np.mean(intervals))
    global_period_error_pct = (intervals - global_period_s) / global_period_s * 100.0
    capture_error_pct = (frequencies - np.mean(frequencies)) / np.mean(frequencies) * 100.0
    return [
        f"{channel} summary ({len(selected)} captures):",
        f"  mean frequency: {np.mean(frequencies):.4f} Hz",
        f"  between-capture frequency std: {np.std(frequencies, ddof=1):.4f} Hz",
        f"  largest between-capture deviation: {format_percent(float(np.max(np.abs(capture_error_pct))))}",
        f"  global mean period: {global_period_s:.4f} s",
        f"  all-cycle period std: {np.std(intervals, ddof=1) * 1000.0:.2f} ms",
        f"  largest all-cycle period deviation: {format_percent(float(np.max(np.abs(global_period_error_pct))))}",
        f"  cycles exceeding ±5% of global mean: {np.count_nonzero(np.abs(global_period_error_pct) > 5.0)}/{len(intervals)}",
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze all timestamped dual-VCNL4040 captures.")
    parser.add_argument("--logs-dir", type=Path, default=Path(__file__).resolve().parent / "logs")
    args = parser.parse_args()
    captures = sorted(args.logs_dir.glob("vcnl4040_dual_200hz_[0-9]*"))
    if not captures:
        raise SystemExit(f"No timestamped captures found in {args.logs_dir}")

    rows: list[dict] = []
    for capture_dir in captures:
        data = np.loadtxt(capture_dir / "capture.csv", delimiter=",", skiprows=1)
        for channel, column in (("front", 1), ("rear", 2)):
            _, _, metrics = analyze_low_frequency(data[:, column], 200.0)
            if "mean_frequency_hz" not in metrics:
                raise RuntimeError(f"{capture_dir.name} {channel}: insufficient detected pulses")
            rows.append({
                "capture": capture_dir.name,
                "channel": channel,
                "psd_frequency_hz": metrics["dominant_psd_frequency_hz"],
                "mean_frequency_hz": metrics["mean_frequency_hz"],
                "period_std_ms": metrics["period_std_ms"],
                "max_period_error_pct": metrics["max_abs_period_error_pct"],
                "over_5": metrics["intervals_over_5pct"],
                "interval_count": metrics["intervals_total"],
                "intervals_s": metrics["intervals_s"],
            })

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.logs_dir / f"vcnl4040_log_set_analysis_{timestamp}"
    suffix = 1
    while output_dir.exists():
        output_dir = args.logs_dir / f"vcnl4040_log_set_analysis_{timestamp}_{suffix:02d}"
        suffix += 1
    output_dir.mkdir(parents=True)

    with (output_dir / "per_capture.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("capture", "channel", "psd_frequency_hz", "cycle_frequency_hz",
                         "period_std_ms", "max_period_error_pct", "intervals_over_5pct", "intervals"))
        for row in rows:
            writer.writerow((row["capture"], row["channel"], f"{row['psd_frequency_hz']:.4f}",
                             f"{row['mean_frequency_hz']:.4f}", f"{row['period_std_ms']:.2f}",
                             f"{row['max_period_error_pct']:.2f}", row["over_5"], row["interval_count"]))

    report_lines = [f"captures: {len(captures)}", ""]
    for channel in ("front", "rear"):
        report_lines.extend(summarize_channel(rows, channel))
        report_lines.append("")
    report_lines.append("Per capture:")
    for row in rows:
        report_lines.append(
            f"  {row['capture']} {row['channel']}: {row['mean_frequency_hz']:.4f} Hz, "
            f"period std {row['period_std_ms']:.2f} ms, max error {row['max_period_error_pct']:.2f}%, "
            f">5% {row['over_5']}/{row['interval_count']}"
        )
    report = "\n".join(report_lines) + "\n"
    (output_dir / "summary.txt").write_text(report, encoding="utf-8")
    print(report, end="")
    print(f"Saved: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
