# hw10_fir_design_and_application.py

import numpy as np
import matplotlib.pyplot as plt
from hw10_read_csv import load_csv_to_arrays
from hw10_sample_rate import compute_sample_rate
import os

def design_lowpass_fir(N, fc, sr, window='hamming'):
    n = np.arange(N)
    M = (N - 1) / 2.0
    fc_over_sr = fc / sr
    h0 = np.zeros(N)
    for i in range(N):
        x = (n[i] - M) * fc_over_sr
        h0[i] = (2 * fc_over_sr) * np.sinc(x)

    w_lower = window.lower()
    if w_lower == 'hamming':
        w = np.hamming(N)
    elif w_lower == 'hann':
        w = np.hanning(N)
    elif w_lower == 'blackman':
        w = np.blackman(N)
    else:
        raise ValueError("Window must be 'hamming', 'hann', or 'blackman'")

    h = h0 * w
    h = h / np.sum(h)  # normalize to unity DC
    return h

def apply_fir(x, h):
    return np.convolve(x, h, mode='same')

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
    # Ensure plots/ folder exists
    os.makedirs('plots', exist_ok=True)

    files = ['sigA.csv', 'sigB.csv', 'sigD.csv']
    # Choose (N, fc, window) based on inspecting raw FFT
    chosen_fir = {
        'sigA.csv': {'N': 201, 'fc': 50,  'window': 'hamming'},
        'sigB.csv': {'N': 301, 'fc': 100, 'window': 'hann'},
        'sigD.csv': {'N': 101, 'fc': 20,  'window': 'blackman'},
    }

    for fname in files:
        # 1) Load raw data
        t, x_raw = load_csv_to_arrays(fname)
        if len(t) == 0:
            print(f"Warning: '{fname}' has no data.")
            continue

        # 2) Compute sample rate
        sr = compute_sample_rate(t)

        # 3) Raw FFT
        freqs_raw, mags_raw = compute_one_sided_fft(x_raw, sr)

        # 4) Design FIR & apply
        params = chosen_fir[fname]
        N = params['N']
        fc = params['fc']
        window_type = params['window']
        h = design_lowpass_fir(N, fc, sr, window=window_type)
        x_fir = apply_fir(x_raw, h)

        # 5) FFT of FIR-filtered
        freqs_fir, mags_fir = compute_one_sided_fft(x_fir, sr)

        # ==== PLOT ====
        fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(8, 6), sharex=False)

        # --- Top: time-domain overlay ---
        ax_time.plot(t, x_raw, color='black', alpha=0.5, label='Raw')
        ax_time.plot(t, x_fir, color='red', alpha=0.8, label=f'FIR N={N}, fc={fc}Hz')
        ax_time.set_xlabel('Time (s)')
        ax_time.set_ylabel('Amplitude')
        ax_time.set_title(f"{fname}: Time-Domain (FIR N={N}, fc={fc} Hz)")
        ax_time.legend()
        ax_time.grid(True)

        # --- Bottom: FFT comparison on log-log scale ---
        ax_freq.loglog(freqs_raw[1:], mags_raw[1:], color='black', linewidth=1, label='Raw FFT')
        ax_freq.loglog(freqs_fir[1:], mags_fir[1:], color='red', linewidth=1,
                       label=f'FIR FFT N={N}, fc={fc}Hz, {window_type}')
        ax_freq.set_xlabel('Frequency (Hz)')
        ax_freq.set_ylabel(r'$|Y(f)|$')
        ax_freq.set_title(f"{fname}: FFT (log–log)   FIR N={N}, fc={fc}Hz, {window_type}")
        ax_freq.set_xlim(freqs_raw[1], sr/2)
        ax_freq.legend()
        ax_freq.grid(which='both', linestyle='--', alpha=0.5)

        fig.tight_layout(pad=3.0)

        # Save figure
        base = os.path.splitext(fname)[0]
        outpath = f"plots/{base}_FIR_loglog.png"
        plt.savefig(outpath, dpi=300)
        plt.show()
        print(f"Saved: {outpath}")
