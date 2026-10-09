#!/usr/bin/env python3
"""Stream ADC CSV samples through the proposed shot detector.

The detector only sees one row at a time. With the default time scale, a
100 ms capture is replayed in 100 ms of wall-clock time. No future sample is
used to make an event decision.
"""
from __future__ import annotations

import argparse
import csv
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from statistics import median


@dataclass
class PulseEvent:
    channel: str
    entry_us: int
    exit_us: int
    peak_us: int
    peak_adc: int
    baseline_at_entry: float
    amplitude: float
    width_us: int
    pre_dip_adc: float | None = None


class PulseDetector:
    """One-channel rising-pulse detector with the requested 6 ms limit."""

    def __init__(self, channel: str, offset: int, max_width_us: int,
                 min_width_us: int = 0, pre_window_us: int = 2000):
        self.channel = channel
        self.offset = offset
        self.max_width_us = max_width_us
        self.min_width_us = min_width_us
        self.pre_window_us = pre_window_us
        self.active = False
        self.entry_us = 0
        self.entry_baseline = 0.0
        self.peak_us = 0
        self.peak_adc = 0
        self.rejected_count = 0

    def update(self, value: int, baseline: float, now_us: int,
               recent: deque[tuple[int, int]]) -> PulseEvent | None:
        level = baseline + self.offset
        if not self.active:
            if value >= level:
                self.active = True
                self.entry_us = now_us
                self.entry_baseline = baseline
                self.peak_us = now_us
                self.peak_adc = value
            return None

        if value > self.peak_adc:
            self.peak_adc = value
            self.peak_us = now_us
        if value >= level:
            return None

        width_us = now_us - self.entry_us
        self.active = False
        if width_us > self.max_width_us or width_us < self.min_width_us:
            self.rejected_count += 1
            return None
        pre_values = [v for t, v in recent
                      if self.entry_us - self.pre_window_us <= t < self.entry_us]
        pre_dip = (self.entry_baseline - min(pre_values)) if pre_values else None
        return PulseEvent(
            channel=self.channel,
            entry_us=self.entry_us,
            exit_us=now_us,
            peak_us=self.peak_us,
            peak_adc=self.peak_adc,
            baseline_at_entry=self.entry_baseline,
            amplitude=self.peak_adc - self.entry_baseline,
            width_us=width_us,
            pre_dip_adc=pre_dip,
        )


class StreamDetector:
    """Stateful detector; all state is updated in sample arrival order."""

    def __init__(self, front_offset: int, rear_offset: int,
                 max_width_us: int, rear_min_width_us: int,
                 distance_mm: float, speed_min_mps: float,
                 speed_max_mps: float, baseline_frames: int,
                 baseline_divisor: int):
        self.front_offset = front_offset
        self.rear_offset = rear_offset
        self.max_width_us = max_width_us
        self.rear_min_width_us = rear_min_width_us
        self.distance_mm = distance_mm
        self.speed_min_mps = speed_min_mps
        self.speed_max_mps = speed_max_mps
        self.baseline_frames = baseline_frames
        self.baseline_divisor = baseline_divisor
        self.calibration_front: list[int] = []
        self.calibration_rear: list[int] = []
        self.front_baseline: float | None = None
        self.rear_baseline: float | None = None
        self.front = PulseDetector("front", front_offset, max_width_us)
        self.rear = PulseDetector("rear", rear_offset, max_width_us,
                                  rear_min_width_us)
        self.recent_front: deque[tuple[int, int]] = deque()
        self.recent_rear: deque[tuple[int, int]] = deque()
        self.rear_events: deque[PulseEvent] = deque(maxlen=8)
        self.shot_count = 0
        self.rejected_front_count = 0

    def _trim_recent(self, recent: deque[tuple[int, int]], now_us: int) -> None:
        while recent and now_us - recent[0][0] > 5000:
            recent.popleft()

    def _update_baseline(self, old: float, value: int) -> float:
        # Same slow time constant as the current firmware candidate.
        return old + (value - old) / self.baseline_divisor

    def feed(self, time_us: int, front_adc: int, rear_adc: int) -> list[dict]:
        """Consume exactly one timestamped row and return new output records."""
        self.recent_front.append((time_us, front_adc))
        self.recent_rear.append((time_us, rear_adc))
        self._trim_recent(self.recent_front, time_us)
        self._trim_recent(self.recent_rear, time_us)

        if self.front_baseline is None:
            self.calibration_front.append(front_adc)
            self.calibration_rear.append(rear_adc)
            if len(self.calibration_front) >= self.baseline_frames:
                self.front_baseline = float(median(self.calibration_front))
                self.rear_baseline = float(median(self.calibration_rear))
            return []

        front_event = self.front.update(front_adc, self.front_baseline,
                                        time_us, self.recent_front)
        rear_event = self.rear.update(rear_adc, self.rear_baseline,
                                      time_us, self.recent_rear)
        if rear_event is not None:
            self.rear_events.append(rear_event)

        outputs: list[dict] = []
        if front_event is not None:
            self.shot_count += 1
            pair = None
            # Rear is physically observed before front in the current data.
            for candidate in sorted(self.rear_events,
                                    key=lambda event: event.peak_us, reverse=True):
                delta_us = front_event.entry_us - candidate.entry_us
                peak_delta_us = front_event.peak_us - candidate.peak_us
                if not 3333 <= peak_delta_us <= 25000:
                    continue
                speed_mps = self.distance_mm * 1000.0 / peak_delta_us
                if self.speed_min_mps <= speed_mps <= self.speed_max_mps:
                    pair = (candidate, delta_us, peak_delta_us, speed_mps)
                    break
            if pair is None:
                speed = None
                rear_entry = None
                delta_us = None
            else:
                candidate, delta_us, peak_delta_us, speed = pair
                self.rear_events.remove(candidate)
                rear_entry = candidate.entry_us
            outputs.append({
                "type": "shot",
                "shot": self.shot_count,
                "front_entry_us": front_event.entry_us,
                "front_exit_us": front_event.exit_us,
                "front_peak_us": front_event.peak_us,
                "front_peak_adc": front_event.peak_adc,
                "front_amplitude_adc": round(front_event.amplitude, 3),
                "front_width_us": front_event.width_us,
                "front_pre_dip_adc": (round(front_event.pre_dip_adc, 3)
                                       if front_event.pre_dip_adc is not None else None),
                "rear_entry_us": rear_entry,
                "rear_front_delta_us": delta_us,
                "rear_front_peak_delta_us": peak_delta_us if pair is not None else None,
                "speed_mps": round(speed, 4) if speed is not None else None,
                "rear_matched": pair is not None,
            })

        # Tracking is deliberately done after the event decision for this row.
        self.front_baseline = self._update_baseline(self.front_baseline, front_adc)
        self.rear_baseline = self._update_baseline(self.rear_baseline, rear_adc)
        return outputs


def stream_csv(path: Path, detector: StreamDetector, realtime: bool,
               time_scale: float) -> list[dict]:
    records: list[dict] = []
    wall_start = time.perf_counter()
    first_time: int | None = None
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            timestamp = int(float(row["time_us"]))
            if first_time is None:
                first_time = timestamp
            if realtime:
                target = (timestamp - first_time) / 1_000_000.0 / time_scale
                target_wall = wall_start + target
                # Sleep away most of the interval, then spin for the final
                # 0.5 ms. This avoids Windows timer quantization accumulating
                # hundreds of milliseconds over a 20 kHz capture.
                wait = target_wall - time.perf_counter()
                if wait > 0.0005:
                    time.sleep(wait - 0.0005)
                while time.perf_counter() < target_wall:
                    pass
            outputs = detector.feed(
                timestamp,
                int(row["barrel_front_adc"]),
                int(row["barrel_rear_adc"]),
            )
            for output in outputs:
                output["file"] = str(path)
                records.append(output)
                speed_text = (f"{output['speed_mps']:.3f} m/s"
                              if output["speed_mps"] is not None else "unpaired")
                print(
                    f"SHOT {output['shot']}: front={output['front_entry_us']} us, "
                    f"rear->front={output['rear_front_delta_us']} us, "
                    f"speed={speed_text}, "
                    f"front_peak=+{output['front_amplitude_adc']:.1f}, "
                    f"width={output['front_width_us']} us",
                    flush=True,
                )
    wall_elapsed = time.perf_counter() - wall_start
    data_elapsed = ((timestamp - first_time) / 1_000_000.0
                    if first_time is not None else 0.0)
    print(f"[{path.name}] data_span={data_elapsed * 1000:.3f} ms, "
          f"wall_stream={wall_elapsed * 1000:.3f} ms, "
          f"wall/data={wall_elapsed / data_elapsed:.3f}x"
          if data_elapsed > 0 else f"[{path.name}] empty capture")
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--front-threshold", type=int, default=60)
    parser.add_argument("--rear-threshold", type=int, default=18)
    parser.add_argument("--baseline-frames", type=int, default=450)
    parser.add_argument("--baseline-divisor", type=int, default=2048)
    parser.add_argument("--max-pulse-ms", type=float, default=6.0)
    parser.add_argument("--rear-min-width-ms", type=float, default=0.3)
    parser.add_argument("--distance-mm", type=float, default=50.0)
    parser.add_argument("--speed-min-mps", type=float, default=2.0)
    parser.add_argument("--speed-max-mps", type=float, default=15.0)
    parser.add_argument("--no-realtime", action="store_true",
                        help="不等待墙钟；算法仍按 CSV 时间戳逐点运行")
    parser.add_argument("--time-scale", type=float, default=1.0,
                        help="墙钟播放倍率，默认 1.0；1.0 才是严格实时")
    parser.add_argument("--out", type=Path,
                        help="输出事件 CSV；多个输入会合并")
    args = parser.parse_args()
    if args.baseline_frames < 1 or args.baseline_divisor < 1:
        parser.error("baseline 参数必须为正数")
    if args.time_scale <= 0:
        parser.error("--time-scale 必须大于 0")

    all_records: list[dict] = []
    for path in args.csv:
        detector = StreamDetector(
            front_offset=args.front_threshold,
            rear_offset=args.rear_threshold,
            max_width_us=round(args.max_pulse_ms * 1000),
            rear_min_width_us=round(args.rear_min_width_ms * 1000),
            distance_mm=args.distance_mm,
            speed_min_mps=args.speed_min_mps,
            speed_max_mps=args.speed_max_mps,
            baseline_frames=args.baseline_frames,
            baseline_divisor=args.baseline_divisor,
        )
        print(f"[{path}] streaming; realtime={not args.no_realtime}, "
              f"thresholds=front+{args.front_threshold}/rear+{args.rear_threshold}")
        records = stream_csv(path, detector, not args.no_realtime,
                             args.time_scale)
        all_records.extend(records)
        print(f"[{path.name}] shots={detector.shot_count}, "
              f"front_long_or_rejected={detector.front.rejected_count}, "
              f"rear_rejected={detector.rear.rejected_count}")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        fields = ["file", "type", "shot", "front_entry_us", "front_exit_us",
                  "front_peak_us", "front_peak_adc", "front_amplitude_adc",
                  "front_width_us", "front_pre_dip_adc", "rear_entry_us",
                  "rear_front_delta_us", "rear_front_peak_delta_us",
                  "speed_mps", "rear_matched"]
        with args.out.open("w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(all_records)
        print(f"事件输出: {args.out}")
    print(f"总射击事件: {len(all_records)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
