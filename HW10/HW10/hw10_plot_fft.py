# hw10_plot_fft.py

import numpy as np
import matplotlib.pyplot as plt
import csv

def load_csv_to_arrays(filename):
    times = []
    values = []
    with open(filename, 'r', newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                t = float(row[0])
                v = float(row[1])
            except ValueError:
                # Skip header or non‐numeric lines
                continue
            times.append(t)
            values.append(v)
    return np.array(times), np.array(values)

def compute_sample_rate(times):
    N = len(times)
    t_total = times[-1] - times[0]
    if t_total <= 0:
        raise ValueError("Time vector must strictly increase.")
    return N / t_total

def compute_one_sided_fft(values, sample_rate):
    x = values - np.mean(values)   # remove DC
    N = len(x)

    R = np.fft.rfft(x)

    freqs = np.fft.rfftfreq(N, d=1.0 / sample_rate)

    mags = np.abs(R) * 2.0 / N
    mags[0] = np.abs(R[0]) / N
    if N % 2 == 0:
        mags[-1] = np.abs(R[-1]) / N

    return freqs, mags

def plot_time_and_fft_loglog(filename):
    t, x = load_csv_to_arrays(filename)
    if len(t) == 0:
        print(f"Warning: '{filename}' has no valid data.")
        return

    sr = compute_sample_rate(t)

    freqs, mags = compute_one_sided_fft(x, sr)

    fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(8, 6), sharex=False)

    ax_time.plot(t, x, color='black', linewidth=1)
    ax_time.set_xlabel('Time (s)')
    ax_time.set_ylabel('Amplitude')
    ax_time.set_title(f"{filename} (time domain)")
    ax_time.grid(True)

    ax_freq.plot(freqs[1:], mags[1:], color='blue', linewidth=1)
    ax_freq.set_xscale('log')
    ax_freq.set_yscale('log')
    ax_freq.set_xlabel('Freq (Hz)')
    ax_freq.set_ylabel(r'$|Y(f)|$')
    ax_freq.set_title(f"{filename} (one‐sided FFT, DC removed, log–log)")
    ax_freq.grid(which='both', linestyle='--', alpha=0.5)

    ax_freq.set_xlim(freqs[1], sr/2)

    fig.tight_layout(pad=3.0)

    outname = f"plots/{filename.split('.csv')[0]}_time_fft_loglog.png"
    plt.savefig(outname, dpi=300)
    plt.show()
    print(f"Saved: {outname}")

if __name__ == '__main__':
    for fname in ['sigA.csv', 'sigB.csv', 'sigC.csv', 'sigD.csv']:
        plot_time_and_fft_loglog(fname)
