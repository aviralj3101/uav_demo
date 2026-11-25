#!/usr/bin/env python3
"""
UAV-side laptop script: stands in for the ESP32-CAM.

Listens on the UAV ESP32's USB serial port. Whenever it sees the request byte 0x01,
it grabs a webcam frame, JPEG-encodes it, and replies with:

    0xA5 0x5A | length (4 bytes, little-endian) | crc32 (4 bytes, LE) | JPEG bytes

(length 0 means "camera failed"). Every other byte coming from the ESP32 is
treated as printable debug text and shown on screen.

Install:  pip install pyserial opencv-python
Run:      python uav_webcam_responder.py --port COM5
"""

import argparse
import os
import struct
import sys
import time
import zlib

import cv2
import serial

# ===================== USER SETTINGS (can also be passed as arguments) =====================
DEFAULT_PORT = "COM5"        # <-- the UAV ESP32's COM port (Linux: /dev/ttyUSB0, macOS: /dev/cu.usbserial-*)
DEFAULT_BAUD = 921600        # must match SERIAL_BAUD in uav_esp32.ino
DEFAULT_CAM = 0              # webcam index (try 1 if 0 is the wrong camera)
DEFAULT_WIDTH = 320
DEFAULT_HEIGHT = 240
DEFAULT_MAX_BYTES = 50000    # must stay below MAX_IMG (60000) in the firmware
# ===========================================================================================

REQ_BYTE = 0x01
MAGIC = b"\xA5\x5A"


def open_camera(index):
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(10):          # let auto-exposure settle
        cap.read()
    return cap


def capture_jpeg(cap, width, height, max_bytes):
    """Return JPEG bytes (<= max_bytes) or None on failure."""
    frame = None
    for _ in range(3):           # skip buffered frames so we get a fresh one
        ok, f = cap.read()
        if ok:
            frame = f
    if frame is None:
        return None
    frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    for quality in (80, 70, 60, 50, 40, 30, 20):
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok and len(buf) <= max_bytes:
            return buf.tobytes()
    return None


def serve_request(ser, cap, args):
    t0 = time.time()
    jpg = capture_jpeg(cap, args.width, args.height, args.max_bytes)
    if jpg is None:
        ser.write(MAGIC + struct.pack("<II", 0, 0))
        ser.flush()
        print("[PC] capture failed - sent error frame")
        return
    crc = zlib.crc32(jpg) & 0xFFFFFFFF
    ser.write(MAGIC + struct.pack("<II", len(jpg), crc) + jpg)
    ser.flush()
    print(f"[PC] frame sent: {len(jpg)} bytes, crc32 {crc:08X}, {(time.time() - t0) * 1000:.0f} ms")


def main():
    ap = argparse.ArgumentParser(description="Webcam responder for the UAV ESP32")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--cam", type=int, default=DEFAULT_CAM)
    ap.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    ap.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    ap.add_argument("--max-bytes", type=int, default=DEFAULT_MAX_BYTES)
    args = ap.parse_args()

    cap = open_camera(args.cam)
    if cap is None:
        sys.exit(f"Could not open webcam index {args.cam}. Close other apps using it or try --cam 1.")
    print(f"[PC] webcam {args.cam} ready ({args.width}x{args.height})")

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 0.1
    ser.write_timeout = 10
    ser.dtr = False              # avoid resetting the ESP32 on connect (some boards reset anyway)
    ser.rts = False
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}\n(Close the Arduino Serial Monitor / other programs using it.)")
    print(f"[PC] listening on {args.port} @ {args.baud}. Ctrl+C to quit.")

    line = bytearray()
    try:
        while True:
            data = ser.read(max(1, ser.in_waiting))
            for b in data:
                if b == REQ_BYTE:
                    serve_request(ser, cap, args)
                elif b == 0x0A:
                    text = line.decode(errors="replace").strip()
                    if text:
                        print(text)
                    line.clear()
                else:
                    line.append(b)
    except KeyboardInterrupt:
        print("\n[PC] stopped.")
    finally:
        ser.close()
        cap.release()


if __name__ == "__main__":
    main()
