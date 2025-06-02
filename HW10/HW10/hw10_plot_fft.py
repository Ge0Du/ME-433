# hw10_plot_fft.py

import numpy as np
import matplotlib.pyplot as plt
import csv

def load_csv_to_arrays(filename):
    """
    Reads a two‐column CSV (time, value) into two NumPy arrays: times, values.
    Skips any header or malformed lines.
    """
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
    """
    Given a strictly increasing NumPy array of time stamps (in seconds),
    return the sample rate (in Hz): N / (t_last - t_first).
    """
    N = len(times)
    t_total = times[-1] - times[0]
    if t_total <= 0:
        raise ValueError("Time vector must strictly increase.")
    return N / t_total

def compute_one_sided_fft(values, sample_rate):
    """
    Given a real‐valued array 'values' and its sample_rate (Hz),
    return (freqs, mags) for the one‐sided FFT **after removing DC**.

    1) Remove the mean (DC) from values → x_zero_mean
    2) Compute R = np.fft.rfft(x_zero_mean)
    3) freqs = np.fft.rfftfreq(N, d=1/sr)
    4) Scale magnitudes so that a pure sinusoid of amplitude A shows up as peak = A.
       mags = |R(k)| * (2/N), except at DC (k=0) and Nyquist (k=N/2 if N even) use (1/N).
    """
    x = values - np.mean(values)   # remove DC
    N = len(x)

    # Compute real FFT (returns N/2+1 frequency bins from 0 to Nyquist)
    R = np.fft.rfft(x)

    # Frequencies for those bins
    freqs = np.fft.rfftfreq(N, d=1.0 / sample_rate)

    # One‐sided magnitude scaling
    mags = np.abs(R) * 2.0 / N
    mags[0] = np.abs(R[0]) / N
    if N % 2 == 0:
        mags[-1] = np.abs(R[-1]) / N

    return freqs, mags

def plot_time_and_fft_loglog(filename):
    """
    For a given CSV file:
      - Load (t, x)
      - Compute sample rate
      - Plot top: time vs. x (raw time‐domain)
      - Plot bottom: one‐sided FFT magnitude vs. freq on log–log axes
    Saves figure as 'plots/<basename>_time_fft_loglog.png'.
    """
    # 1) Load data
    t, x = load_csv_to_arrays(filename)
    if len(t) == 0:
        print(f"Warning: '{filename}' has no valid data.")
        return

    # 2) Compute sample rate (Hz)
    sr = compute_sample_rate(t)

    # 3) Compute one‐sided FFT: (freqs, mags)
    freqs, mags = compute_one_sided_fft(x, sr)

    # 4) Set up subplots
    fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(8, 6), sharex=False)

    # --- Top: Raw time‐domain ---
    ax_time.plot(t, x, color='black', linewidth=1)
    ax_time.set_xlabel('Time (s)')
    ax_time.set_ylabel('Amplitude')
    ax_time.set_title(f"{filename} (time domain)")
    ax_time.grid(True)

    # --- Bottom: FFT magnitude on log–log ---
    # We’ll exclude the DC bin at freqs[0] = 0 since log(0) is invalid.
    # So plot from freqs[1] onward.
    ax_freq.plot(freqs[1:], mags[1:], color='blue', linewidth=1)
    ax_freq.set_xscale('log')
    ax_freq.set_yscale('log')
    ax_freq.set_xlabel('Freq (Hz)')
    ax_freq.set_ylabel(r'$|Y(f)|$')
    ax_freq.set_title(f"{filename} (one‐sided FFT, DC removed, log–log)")
    ax_freq.grid(which='both', linestyle='--', alpha=0.5)

    # Optionally set x‐limits to cover from ~0.1 Hz up to Nyquist:
    ax_freq.set_xlim(freqs[1], sr/2)

    fig.tight_layout(pad=3.0)

    # 5) Save to plots/
    outname = f"plots/{filename.split('.csv')[0]}_time_fft_loglog.png"
    plt.savefig(outname, dpi=300)
    plt.show()
    print(f"Saved: {outname}")

if __name__ == '__main__':
    # Loop through all four CSVs
    for fname in ['sigA.csv', 'sigB.csv', 'sigC.csv', 'sigD.csv']:
        plot_time_and_fft_loglog(fname)
