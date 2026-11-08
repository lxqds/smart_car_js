"""Bounded RK3576 -> MCU display probe; no gamepad or actuator enable command.

python3 -m host_control.display_probe --speed-mm-s 250 --angle 10
Add --send --port /dev/serial/by-id/... only with the approved outputs-off firmware.
"""
import argparse
import json
import math
import os
import select
import time
from .legacy_protocol import main_control_packet


def make_packet(speed, angle):
    if not math.isfinite(speed) or abs(speed) > 500 or not math.isfinite(angle) or abs(angle) > 30:
        raise ValueError("display test limit: +/-500 mm/s, +/-30 command-angle units")
    return main_control_packet(speed, angle, 3)


def write_all(fd, data, timeout=0.5):
    end = time.monotonic() + timeout
    while data:
        remaining = end - time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise TimeoutError("serial write deadline")
        try:
            count = os.write(fd, data)
        except BlockingIOError:
            continue
        if count <= 0:
            raise OSError("serial write made no progress")
        data = data[count:]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--speed-mm-s", type=float, default=0)
    ap.add_argument("--angle", type=float, default=0)
    ap.add_argument("--duration", type=float, default=3)
    ap.add_argument("--port")
    ap.add_argument("--send", action="store_true")
    args = ap.parse_args()
    packet = make_packet(args.speed_mm_s, args.angle)
    if not math.isfinite(args.duration) or not 0 < args.duration <= 30:
        ap.error("duration must be 0..30 seconds")
    report = dict(speed_mm_s=args.speed_mm_s, angle_command=args.angle,
                  servo_pwm=int(1500 - args.angle * 3), hex=packet.hex(), rate_hz=10,
                  mode="send" if args.send else "dry-run", mcu_confirmed=False)
    print(json.dumps(report), flush=True)
    if not args.send:
        return
    if not args.port:
        ap.error("--send requires --port")
    import termios
    import fcntl
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    locked = False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        locked = True
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        end = time.monotonic() + args.duration
        sent = 0
        try:
            while time.monotonic() < end:
                write_all(fd, packet)
                sent += 1
                time.sleep(0.1)
        finally:
            write_all(fd, make_packet(0, 0))
        print(json.dumps(dict(sent=sent, final_neutral_sent=True, mcu_confirmed=False)), flush=True)
    finally:
        if locked:
            fcntl.ioctl(fd, termios.TIOCNXCL)
        os.close(fd)


if __name__ == "__main__":
    main()
