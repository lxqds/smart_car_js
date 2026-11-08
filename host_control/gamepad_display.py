"""USB gamepad -> MCU speed/steering commands; local motor authorization required.

--list: inventory. --profile FILE: dry-run. Add --send --port PATH for UART.
Hardware wireless power-off must be tested: ioctl success alone is not radio health.
"""
import argparse
import json
import os
import signal
import time
from .gamepad_control import GamepadControl
from .gamepad_evdev import GamepadDevice, InputLost, inventory
from .display_probe import write_all, make_packet


class SerialSink:
    def __init__(self, path):
        self.path, self.fd = path, None

    def __enter__(self):
        import termios
        import fcntl
        fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            a = termios.tcgetattr(fd)
            a[0] = a[1] = a[3] = 0
            a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            a[4] = a[5] = termios.B115200
            a[6][termios.VMIN] = a[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, a)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except BaseException:
            os.close(fd)
            raise
        self.fd = fd
        return self

    def send(self, packet):
        if self.fd is not None:
            write_all(self.fd, packet, timeout=0.1)

    def close(self):
        if self.fd is None:
            return
        import termios
        import fcntl
        try:
            fcntl.ioctl(self.fd, termios.TIOCNXCL)
        finally:
            os.close(self.fd)
            self.fd = None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--racing', action='store_true', help='racing speed targets; can drive an authorized MCU')
    ap.add_argument('--profile')
    ap.add_argument('--send', action='store_true')
    ap.add_argument('--port')
    ap.add_argument('--duration', type=int, default=120, help='seconds, 0 runs until stopped')
    args = ap.parse_args()
    if args.list:
        print(json.dumps(inventory(), ensure_ascii=False, indent=2))
        return
    if not args.profile or not 0 <= args.duration <= 600 or (args.send and not args.port):
        ap.error('profile required, duration 0..600 (0=continuous), --send requires --port')
    with open(args.profile, encoding='utf-8') as f:
        profile = json.load(f)
    if args.racing:
        from .racing_evdev import RacingDevice
        from .racing_control import RacingControl
        if profile.get('racing_mapping_verified') is not True:
            ap.error('racing requires measured RT/LT/B/X/A profile')
        device, control = RacingDevice(profile), RacingControl()
    else:
        device, control = GamepadDevice(profile), GamepadControl()
    sink = SerialSink(args.port) if args.send else None
    running = True
    def stop(signum, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    deadline = time.monotonic() + args.duration if args.duration else float('inf')
    retry = 0
    previous = None
    sent = 0
    state, reason = 'waiting', ''
    try:
        if sink:
            sink.__enter__()
        while running and time.monotonic() < deadline:
            now = time.monotonic()
            sample = None
            try:
                if device.device is None and now >= retry:
                    device.connect()
                if device.device is not None:
                    sample = device.snapshot()
                    state = 'connected'
                    reason = ''
            except (OSError, InputLost, ValueError) as exc:
                device.close()
                retry = now + 0.5
                state = 'input_lost'
                reason = str(exc)
            packet = control.update(sample, int(time.monotonic() * 1000) & 0xffffffff)
            if packet is not None and sink:
                sink.send(packet)
                sent += 1
            raw = tuple(round(x, 3) for x in sample[:2]) if sample else None
            view = (state, control.enabled, control.speed, control.angle, raw, getattr(control,"gear",None))
            if view != previous:
                print(json.dumps(dict(time_ms=int(time.monotonic()*1000), state=state,
                    enabled=control.enabled, speed_mm_s=control.speed, angle_command=control.angle,
                    held=sample[3 if args.racing else 2] if sample else False, axes=raw,
                    gear=getattr(control,"gear",None), reason=reason, sent=sent, mode='send' if sink else 'dry-run')),
                    flush=True)
                previous = view
            time.sleep(0.05)
    finally:
        device.close()
        if sink:
            try:
                if sink.fd is not None:
                    sink.send(control.update(None, int(time.monotonic()*1000)&0xffffffff)
                              if args.racing else make_packet(0, 0))
                    sent += 1
            finally:
                sink.close()
        print(json.dumps(dict(stopped=True, sent=sent)), flush=True)


if __name__ == '__main__':
    main()
