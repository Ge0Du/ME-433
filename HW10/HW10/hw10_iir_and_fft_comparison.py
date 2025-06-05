# hw10_iir_and_fft_comparison.py

import numpy as np
import matplotlib.pyplot as plt
from hw10_read_csv import load_csv_to_arrays
from hw10_sample_rate import compute_sample_rate
import os

def iir_filter(x, A, B):
    y = np.zeros_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = A * y[i - 1] + B * x[i]
    return y

def compute_one_sided_fft(y, sr):
    y0 = y - np.mean(y)

    N = len(y0)
    R = np.fft.rfft(y0)

    freqs = np.fft.rfftfreq(N, d=1.0 / sr)

    mags = np.abs(R) * 2.0 / N
    mags[0] = np.abs(R[0]) / N
    if N % 2 == 0:
        mags[-1] = np.abs(R[-1]) / N

    return freqs, mags

if __name__ == '__main__':
    os.makedirs('plots', exist_ok=True)

    files = ['sigA.csv', 'sigB.csv', 'sigD.csv']
    chosen_iir = {
        'sigA.csv': (0.98, 0.02),  # ~50-sample time constant
        'sigB.csv': (0.99, 0.01),  # ~100-sample time constant
        'sigD.csv': (0.95, 0.05)   # ~20-sample time constant
    }

    for fname in files:
        t, x_raw = load_csv_to_arrays(fname)
        if len(t) == 0:
            print(f"Warning: '{fname}' has no data.")
            continue

        sr = compute_sample_rate(t)

        freqs_raw, mags_raw = compute_one_sided_fft(x_raw, sr)

        A, B = chosen_iir[fname]
        x_iir = iir_filter(x_raw, A, B)

        freqs_iir, mags_iir = compute_one_sided_fft(x_iir, sr)

        fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(8, 6), sharex=False)

        ax_time.plot(t, x_raw, color='black', alpha=0.5, label='Raw')
        ax_time.plot(t, x_iir, color='red', alpha=0.8, label=f'IIR A={A}, B={B}')
        ax_time.set_xlabel('Time (s)')
        ax_time.set_ylabel('Amplitude')
        ax_time.set_title(f"{fname}: Time-Domain (IIR A={A}, B={B})")
        ax_time.legend()
        ax_time.grid(True)

        ax_freq.loglog(freqs_raw[1:], mags_raw[1:], color='black', linewidth=1, label='Raw FFT')
        ax_freq.loglog(freqs_iir[1:], mags_iir[1:], color='red', linewidth=1, label=f'IIR FFT A={A}, B={B}')
        ax_freq.set_xlabel('Frequency (Hz)')
        ax_freq.set_ylabel(r'$|Y(f)|$')
        ax_freq.set_title(f"{fname}: FFT (log–log)   IIR A={A}, B={B}")
        ax_freq.set_xlim(freqs_raw[1], sr/2)
        ax_freq.legend()
        ax_freq.grid(which='both', linestyle='--', alpha=0.5)

        fig.tight_layout(pad=3.0)

        base = os.path.splitext(fname)[0]
        outpath = f"plots/{base}_IIR_loglog.png"
        plt.savefig(outpath, dpi=300)
        plt.show()
        print(f"Saved: {outpath}")
