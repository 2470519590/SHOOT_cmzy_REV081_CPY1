"""Plot every DMA sample and report descriptive capture statistics."""
from __future__ import annotations
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

def main():
    parser = argparse.ArgumentParser(description='ADC raw capture: two rows, one column')
    parser.add_argument('csv', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--trigger-index', type=int, default=500)
    args = parser.parse_args()
    with args.csv.open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2 or not 0 < args.trigger_index < len(rows):
        raise ValueError('Need at least two frames and a valid trigger index')
    time = np.array([float(row['time_us']) for row in rows]) / 1000
    intervals = np.diff(time)
    if not np.allclose(intervals, intervals[0]):
        raise ValueError('CSV sample intervals are not uniform')
    dt = intervals[0]
    output = args.out or args.csv.with_suffix('.png')
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(14, 8), constrained_layout=True)
    report = [f'Frames: {len(rows)}; interval: {dt * 1000:g} us; trigger: {time[args.trigger_index]:g} ms',
              'Baseline/noise reference: frames before trigger, excluding final 50 frames.',
              'Widths below are sample-count durations, without filtering.']
    for axis, name, pin, color in zip(axes, ['barrel_front_adc', 'barrel_rear_adc'],
                                     ['ET2 / PA3', 'ET1 / PA1'], ['#d62728', '#2ca02c']):
        values = np.array([int(row[name]) for row in rows])
        # This baseline describes the capture; it is not a firmware detector.
        reference = values[:max(1, args.trigger_index - 50)]
        baseline = float(np.median(reference))
        peak = int(np.argmax(values))
        amplitude = values[peak] - baseline
        axis.plot(time, values, color=color, lw=0.8, label='Raw DMA samples')
        axis.axhline(baseline, color='gray', ls=':', label=f'Pretrigger median {baseline:g}')
        axis.axvline(time[args.trigger_index], color='black', ls='--', lw=.8, label='Capture trigger')
        axis.plot(time[peak], values[peak], 'o', color=color, ms=4)
        axis.axvline(time[peak], color=color, ls=':', lw=.7)
        axis.annotate(f'peak i={peak}, t={time[peak]:g} ms',
                      (time[peak], values[peak]), xytext=(5, 8),
                      textcoords='offset points', fontsize=8)
        axis.set_title(f'{name} ({pin}) | peak {values[peak]} (+{amplitude:g}) at {time[peak]:g} ms')
        axis.set_ylabel('ADC counts')
        # Major grid gives readable milliseconds; minor grid exposes individual
        # 50 us sample spacing without changing the raw data.
        axis.xaxis.set_major_locator(MultipleLocator(5.0))
        axis.xaxis.set_minor_locator(MultipleLocator(0.25))
        axis.yaxis.set_minor_locator(MultipleLocator(5.0))
        axis.grid(True, which='major', alpha=.30)
        axis.grid(True, which='minor', alpha=.13, linewidth=.5)
        axis.legend(loc='upper right')
        report.append(f'\n{name}: baseline={baseline:g}, pretrigger std={np.std(reference):.3f}, '
                      f'MAD={np.median(np.abs(reference-baseline)):.3f}, '
                      f'pretrigger range={reference.min()}..{reference.max()}, '
                      f'min={values.min()}, max={values[peak]}, amplitude={amplitude:g}, '
                      f'peak index={peak}, peak time={time[peak]:g} ms')
        # Contiguous intervals above descriptive levels, including short excursions.
        for level in [15, 25, amplitude / 2]:
            mask = values > baseline + level
            edges = np.diff(np.r_[False, mask, False].astype(int))
            starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
            intervals_text = [f'{time[start]:g}..{time[end-1]:g} ms ({(end-start)*dt:.3f} ms, {end-start} samples)'
                              for start, end in zip(starts, ends)]
            report.append(f'  >baseline+{level:g}: ' + ('; '.join(intervals_text) or 'none'))
    axes[-1].set_xlabel('Time from capture start (ms)')
    fig.suptitle(f'Unfiltered ADC capture: {len(rows)} frames, {1 / (dt / 1000):g} samples/s/channel')
    fig.savefig(output, dpi=160)
    plt.close(fig)
    report_path = output.with_suffix('.analysis.txt')
    report_path.write_text('\n'.join(report) + '\n', encoding='utf-8')
    print('\n'.join(report))
    print(f'Plot: {output}\nStatistics: {report_path}')

if __name__ == '__main__':
    main()
