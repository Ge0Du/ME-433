# hw10_read_csv.py

import csv
import numpy as np

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
                # Skip header or lines with non‐numeric data
                continue
            times.append(t)
            values.append(v)
    return np.array(times), np.array(values)


if __name__ == '__main__':
    # Quick check: load sigA.csv and print basic info
    tA, xA = load_csv_to_arrays('sigA.csv')
    print(f"sigA.csv → {len(tA)} samples; first time={tA[0]:.6f}, last time={tA[-1]:.6f}")
    tA, xA = load_csv_to_arrays('sigB.csv')
    print(f"sigB.csv → {len(tA)} samples; first time={tA[0]:.6f}, last time={tA[-1]:.6f}")
    tA, xA = load_csv_to_arrays('sigC.csv')
    print(f"sigC.csv → {len(tA)} samples; first time={tA[0]:.6f}, last time={tA[-1]:.6f}")
    tA, xA = load_csv_to_arrays('sigD.csv')
    print(f"sigD.csv → {len(tA)} samples; first time={tA[0]:.6f}, last time={tA[-1]:.6f}")
    # Repeat for others if you like:
    # tB, xB = load_csv_to_arrays('sigB.csv')
    # print(f"sigB.csv → {len(tB)} samples; …")
