# hw10_sample_rate.py

from hw10_read_csv import load_csv_to_arrays

def compute_sample_rate(times):
    """
    Given a NumPy array of time stamps (strictly increasing),
    return the sample rate in Hz: N / (t_last - t_first).
    """
    N = len(times)
    t_total = times[-1] - times[0]
    if t_total <= 0:
        raise ValueError("Time vector must strictly increase.")
    return N / t_total

if __name__ == '__main__':
    for fname in ['sigA.csv', 'sigB.csv', 'sigC.csv', 'sigD.csv']:
        t, x = load_csv_to_arrays(fname)
        sr = compute_sample_rate(t)
        print(f"{fname}: N={len(t)}, t_start={t[0]:.6f} s, "
              f"t_end={t[-1]:.6f} s → SR ≈ {sr:.2f} Hz")
