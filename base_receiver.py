#!/usr/bin/env python3
"""
Base-station laptop script.

Press Enter  -> sends '1' to the base ESP32 (capture request)
Type q+Enter -> quit

Incoming images (framed as "IMG_START\\r\\n" | len(4, LE) | JPEG | "\\r\\nIMG_END\\r\\n")
are saved to ./captures/. All other lines from the ESP32 are printed as debug text.

Install:  pip install pyserial opencv-python     (opencv is optional, only used to verify the JPEG decodes)
Run:      python base_receiver.py --port COM6
"""

import argparse
import os
import sys
import threading
import time
import zlib
from datetime import datetime

import serial

try:
    import cv2
    import numpy as np
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

# ===================== USER SETTINGS (can also be passed as arguments) =====================
DEFAULT_PORT = "COM6"        # <-- the BASE ESP32's COM port
DEFAULT_BAUD = 921600        # must match SERIAL_BAUD in base_esp32.ino
DEFAULT_OUTDIR = "captures"
# ===========================================================================================

state = {"t_trigger": 0.0, "count": 0}


def read_exact(ser, n, timeout=15.0):
    buf = bytearray()
    deadline = time.time() + timeout
    while len(buf) < n and time.time() < deadline:
        buf.extend(ser.read(n - len(buf)))
    return bytes(buf)


def save_image(data, outdir):
    state["count"] += 1
    name = datetime.now().strftime("capture_%Y%m%d_%H%M%S") + f"_{state['count']:03d}.jpg"
    path = os.path.join(outdir, name)
    with open(path, "wb") as f:
        f.write(data)

    markers_ok = data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9"
    decode = "n/a (opencv not installed)"
    if HAVE_CV:
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        decode = f"OK {img.shape[1]}x{img.shape[0]}" if img is not None else "FAILED"
    rtt = time.time() - state["t_trigger"] if state["t_trigger"] else 0.0
    print(f"[PC] saved {path}")
    print(f"[PC] {len(data)} bytes | crc32 {zlib.crc32(data) & 0xFFFFFFFF:08X} | "
          f"JPEG markers {'OK' if markers_ok else 'BAD'} | decode {decode} | "
          f"trigger-to-file {rtt:.2f} s")


def reader(ser, outdir, stop):
    pending = b""
    while not stop.is_set():
        try:
            chunk = ser.readline()
        except serial.SerialException as e:
            print(f"[PC] serial error: {e}")
            stop.set()
            return
        if not chunk:
            continue
        pending += chunk
        if not pending.endswith(b"\n"):
            continue
        line = pending.strip()
        pending = b""

        if line == b"IMG_START":
            hdr = read_exact(ser, 4)
            if len(hdr) < 4:
                print("[PC] image header incomplete")
                continue
            length = int.from_bytes(hdr, "little")
            data = read_exact(ser, length)
            if len(data) != length:
                print(f"[PC] image incomplete: got {len(data)} of {length} bytes")
                continue
            save_image(data, outdir)
        elif line and line != b"IMG_END":
            print(line.decode(errors="replace"))


def main():
    ap = argparse.ArgumentParser(description="Base station receiver")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 1
    ser.dtr = False              # avoid resetting the ESP32 on connect (some boards reset anyway)
    ser.rts = False
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}\n(Close the Arduino Serial Monitor / other programs using it.)")
    time.sleep(2)                # give the ESP32 time to boot if it did reset
    ser.reset_input_buffer()
    print(f"[PC] connected to {args.port} @ {args.baud}, saving to ./{args.outdir}/")

    stop = threading.Event()
    t = threading.Thread(target=reader, args=(ser, args.outdir, stop), daemon=True)
    t.start()

    try:
        while not stop.is_set():
            cmd = input("Press Enter to capture (q + Enter to quit): ").strip().lower()
            if cmd == "q":
                break
            state["t_trigger"] = time.time()
            ser.write(b"1")
            time.sleep(0.1)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        stop.set()
        time.sleep(0.2)
        ser.close()
        print("[PC] closed.")


if __name__ == "__main__":
    main()
