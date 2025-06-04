# read_camera.py
# Dependencies: pip install pyserial numpy pillow matplotlib

import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import serial
import sys
import time

# ══ Configuration ══════════════════════════════════════════════════════
# Replace this with your actual USB‐CDC port (e.g. "COM3" on Windows, "/dev/ttyACM0" on Linux).
PORT = "/COM10"
BAUD = 115200

IMG_W = 80
IMG_H = 60
TOTAL_PIXELS = IMG_W * IMG_H

# ══ Open serial port ═════════════════════════════════════════════════
try:
    ser = serial.Serial(PORT, BAUD, timeout=5)
except serial.SerialException as e:
    print(f"Could not open {PORT}: {e}")
    sys.exit(1)

print(f"Opened port {ser.name}. Waiting 2 sec for it to settle...")
time.sleep(2)

# ══ Main capture once ════════════════════════════════════════════════
# Send ’c\n’ so that Pico’s scanf("%s",cmd) sees “c”, if you used a menu-loop
# If you’re using the “unconditional printImage()” loop instead, just read.
print("Requesting one frame...")
ser.write(b"c\n")           # If you incorporated the “scanf( )” version in camera1.c
# If you used the unconditional printImage() version (no scanf), skip the write.

# ══ Read 4800 lines of “i r g b\r\n” ═══════════════════════════════
reds   = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
greens = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
blues  = np.zeros((IMG_H, IMG_W), dtype=np.uint8)

count = 0
while count < TOTAL_PIXELS:
    line = ser.readline().decode('ascii', errors='ignore').strip()
    if not line:
        continue
    # Expect format: "<index> <r> <g> <b>"
    try:
        i_str, r_str, g_str, b_str = line.split()
        i = int(i_str)
        r = int(r_str)
        g = int(g_str)
        b = int(b_str)
    except ValueError:
        continue

    row = i // IMG_W
    col = i % IMG_W

    reds[row, col]   = r
    greens[row, col] = g
    blues[row, col]  = b

    count += 1
    # (Optional) print progress:
    # if count % 1000 == 0:
    #     print(f"  captured {count}/{TOTAL_PIXELS}")

# ══ Stack into one RGB image and display ══════════════════════════════
rgb_array = np.dstack((reds, greens, blues))
image = Image.fromarray(rgb_array, mode="RGB")

plt.figure(figsize=(4, 3))
plt.imshow(image)
plt.axis("off")
plt.title("80×60 Capture w/ green COM dot")
plt.show()

# Optional: save to disk
image.save("capture_hw12.png")
print("Saved capture_hw12.png")

ser.close()
