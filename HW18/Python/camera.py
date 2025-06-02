# read_camera.py
import serial
import numpy as np
import matplotlib.pyplot as plt

# Adjust to whatever COM port the Pico enumerates as (e.g. “COM3” on Windows, “/dev/ttyACM0” on Linux)
SERIAL_PORT = "COM10"
BAUD_RATE   = 115200  # USB-CDC default

WIDTH  = 80
HEIGHT = 60

def read_frame(ser):
    """
    Reads exactly WIDTH*HEIGHT lines of "R G B" from serial, then
    returns an (H, W, 3) NumPy array of dtype uint8.
    """
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    count = 0
    while count < WIDTH * HEIGHT:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 3:
            continue
        try:
            r = int(parts[0])
            g = int(parts[1])
            b = int(parts[2])
        except ValueError:
            continue
        y = count // WIDTH
        x = count % WIDTH
        img[y, x, 0] = r
        img[y, x, 1] = g
        img[y, x, 2] = b
        count += 1
    return img

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        while True:
            print("Waiting for new frame… press button on Pico.")
            frame = read_frame(ser)
            plt.imshow(frame)
            plt.title("80×60 Camera Capture")
            plt.axis("off")
            plt.pause(0.01)      # show the image
            plt.show(block=False)
            input("Press Enter to grab the next frame…")  # wait for user

if __name__ == "__main__":
    main()
