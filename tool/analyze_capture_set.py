"""Describe archived DMA rings. Circular time origin is not capture trigger time."""
from pathlib import Path
import argparse
import csv
import numpy as np
from scipy.signal import find_peaks, peak_widths
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path('data'))
    args = parser.parse_args()
    out = args.data / 'capture_analysis'
    out.mkdir(exist_ok=True)
    results = []
    for path in sorted(args.data.glob('ir_capture_ring_*.bin')):
        raw = np.frombuffer(path.read_bytes(), dtype='<u2')
        if raw.size != 4000:
            raise ValueError(f'Unexpected length: {path}')
        samples = raw.reshape(-1, 2)[:, ::-1].copy()  # ET2 front; ET1 rear.
        # Put the largest rear peak at 25 ms for comparable plots. No trigger reconstruction.
        samples = np.roll(samples, 500 - int(np.argmax(samples[:, 1])), axis=0)
        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(14, 8), constrained_layout=True)
        for channel, name in enumerate(['front', 'rear']):
            x = samples[:, channel].astype(float)
            baseline = float(np.median(x))
            mad = float(np.median(np.abs(x-baseline)))
            # Median describes mostly quiet samples; prominence additionally measures local height.
            # Work on three rings so peaks at wrap boundaries are treated correctly.
            triple = np.tile(x, 3)
            peaks, properties = find_peaks(triple, prominence=12, distance=20)
            selected = (peaks >= len(x)) & (peaks < 2*len(x))
            widths = peak_widths(triple, peaks, rel_height=.5)[0]
            print(path.stem, name, 'baseline', baseline, 'MAD', mad, 'max_delta', x.max()-baseline)
            axes[channel].plot(np.arange(len(x))*.05, x, lw=.8)
            axes[channel].axhline(baseline, color='gray', ls=':')
            axes[channel].set_title(f'{name}: median {baseline:g}, max +{x.max()-baseline:g}, noise MAD {mad:g}')
            axes[channel].set_ylabel('ADC counts')
            axes[channel].grid(alpha=.25)
            for j in np.flatnonzero(selected):
                i = int(peaks[j] - len(x))
                row = dict(capture=path.stem, channel=name, reference_ms=i*.05,
                           baseline=baseline, peak_adc=int(x[i]), amplitude=x[i]-baseline,
                           prominence=properties['prominences'][j], half_prominence_width_ms=widths[j]*.05)
                results.append(row)
                print(' ', row)
                axes[channel].plot(i*.05, x[i], 'ro', ms=3)
                axes[channel].annotate(f'+{x[i]-baseline:g}', (i*.05, x[i]), xytext=(0, 8), textcoords='offset points')
        axes[-1].set_xlabel('Circular reference time (ms); largest rear peak aligned near 25 ms')
        fig.suptitle(path.stem + ' | raw 20 kHz/channel; no smoothing')
        fig.savefig(out / (path.stem+'.png'), dpi=150)
        plt.close(fig)
    if results:
        with (out/'peaks.csv').open('w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=list(results[0]))
            writer.writeheader()
            writer.writerows(results)
    print('Output:', out)

if __name__ == '__main__':
    main()
