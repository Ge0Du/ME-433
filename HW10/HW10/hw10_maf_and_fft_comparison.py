# hw10_maf_and_fft_comparison.py

import numpy as np
import matplotlib.pyplot as plt
from hw10_read_csv import load_csv_to_arrays
from hw10_sample_rate import compute_sample_rate

def moving_average_filter(x, X):
    """
    Returns a length‐N array y = moving‐average of x over window length X.
    Assumes zero‐pre‐samples for indices <0.
    y[i] = (x[0] + ... + x[i]) / X   if i < X
         = (x[i] + x[i−1] + ... + x[i−X+1]) / X  if i >= X
    """
    N = len(x)
    y = np.zeros(N)
    cum = np.cumsum(x)  # cum[i] = x[0] + ... + x[i]
    for i in range(N):
        if i < X:
            y[i] = cum[i] / X
        else:
            y[i] = (cum[i] - cum[i - X]) / X
    return y

def compute_one_sided_fft(y, sr):
    """
    Given a real‐valued sequence y (length N) and sample rate sr,
    subtract its mean, then compute one‐sided FFT using np.fft.fft/np.fft.rfft.
    Returns (freqs, mags) where freqs runs from 0 to sr/2, and
    mags are scaled so that a peak of amplitude A → magnitude ≃ A.
    """
    # 1) Remove DC
    y0 = y - np.mean(y)
    N = len(y0)

    # 2) Compute rfft (N/2+1 bins)
    R = np.fft.rfft(y0)

    # 3) Frequency axis for those bins
    freqs = np.fft.rfftfreq(N, d=1.0 / sr)

    # 4) Scale magnitudes (double‐sided → one‐sided)
    mags = np.abs(R) * 2.0 / N
    mags[0] = np.abs(R[0]) / N
    if N % 2 == 0:
        mags[-1] = np.abs(R[-1]) / N

    return freqs, mags

if __name__ == '__main__':
    import os

    files = ['sigA.csv', 'sigB.csv', 'sigD.csv']
    # “By‐eye” window lengths
    chosen_windows = {
        'sigA.csv': 20,
        'sigB.csv': 50,
        'sigD.csv': 10
    }

    for fname in files:
        # 1) Load raw data
        t, x = load_csv_to_arrays(fname)
        sr = compute_sample_rate(t)

        # 2) Raw FFT
        freqs_raw, mags_raw = compute_one_sided_fft(x, sr)

        # 3) Apply MAF
        X = chosen_windows[fname]
        y_maf = moving_average_filter(x, X)

        # 4) FFT of filtered
        freqs_filt, mags_filt = compute_one_sided_fft(y_maf, sr)

        # ==== PLOTTING ====

        fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(8, 6), sharex=False)

        # --- Top: time‐domain overlay ---
        ax_time.plot(t, x, color='black', alpha=0.5, label='Raw')
        ax_time.plot(t, y_maf, color='red', alpha=0.8, label=f'MAF X={X}')
        ax_time.set_xlabel('Time (s)')
        ax_time.set_ylabel('Amplitude')
        ax_time.set_title(f"{fname}: Time‐domain (MAF X={X})")
        ax_time.legend()
        ax_time.grid(True)

        # --- Bottom: log–log FFT comparison ---
        # Skip index 0 (freq=0) because log(0) is undefined
        ax_freq.loglog(freqs_raw[1:], mags_raw[1:], color='black', linewidth=1, label='Raw FFT')
        ax_freq.loglog(freqs_filt[1:], mags_filt[1:], color='red', linewidth=1, label=f'MAF FFT X={X}')

        ax_freq.set_xlabel('Frequency (Hz)')
        ax_freq.set_ylabel(r'$|Y(f)|$')
        ax_freq.set_title(f"{fname}: FFT (log–log)  —  MAF X={X}")
        ax_freq.set_xlim(freqs_raw[1], sr/2)
        ax_freq.legend()
        ax_freq.grid(which='both', linestyle='--', alpha=0.5)

        fig.tight_layout(pad=3.0)

        # Save under plots/
        outname = f"plots/{fname.split('.csv')[0]}_MAF_loglog.png"
        plt.savefig(outname, dpi=300)
        plt.show()
        print(f"Saved: {outname}")
