"""Manual pulse calibration for SERVO_ONLY RT1064-direction firmware only.

No automatic sweep, no gamepad input. Use board KEY4/power for physical stop.
Serial port is exclusive. Default is dry-run; --send --port explicitly enables
transmission. Commands: enable, off, center, +10, -10, +5, -5, status, quit.
Local KEY3 is still required by MCU. Targets are us, NOT measured angles.
"""
import argparse
import json
import os
import select
import signal
import sys
import time
from .legacy_protocol import control_packet
from .gamepad_display import SerialSink


class Calibration:
    MIN_US, MAX_US = 1400, 1600

    def __init__(self):
        self.pulse = 1500
        self.enabled = False

    def off(self):
        self.enabled = False
        self.pulse = 1500

    def command(self, text):
        text = text.strip().lower()
        if text in ('off', 'quit'):
            self.off()
        elif text == 'enable':
            if self.pulse != 1500 or self.enabled:
                raise ValueError('enable only once at neutral')
            self.enabled = True
        elif text == 'center':
            self.pulse = 1500
        elif text in ('+10', '-10', '+5', '-5'):
            if not self.enabled:
                raise ValueError('not enabled')
            proposed = self.pulse + int(text)
            if not self.MIN_US <= proposed <= self.MAX_US:
                raise ValueError('stage 1 bound: 1400..1600 us')
            self.pulse = proposed
        elif text != 'status':
            raise ValueError('use enable/off/center/+10/-10/+5/-5/status/quit')
        return text != 'quit'

    def packet(self):
        # Invert the unchanged RT1064 serial mapping to request a physical
        # pulse: output=3000-command. This is NOT a gamepad direction change.
        control = control_packet(0.0, 3000-self.pulse if self.enabled else 1500)
        meta = bytes((0x24, 0x42, 0x30, 8, 2, 1, int(self.enabled)))
        return control + meta + bytes((sum(meta) & 255,))


def run(sink, duration):
    state = Calibration()
    running = True

    def stop(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    start = last_input = last_send = time.monotonic()
    pending = b''

    def report(reason):
        print(json.dumps(dict(pulse_us=state.pulse, enabled=state.enabled,
                              reason=reason, mcu_confirmed=False)), flush=True)

    report('neutral; local KEY3 required; no gamepad input')
    try:
        while running and time.monotonic() - start < duration:
            now = time.monotonic()
            if state.enabled and (now-last_send >= .2 or now-last_input >= 120):
                state.off()
                report('scheduler/input timeout; rearm required')
            if select.select([sys.stdin], [], [], 0)[0]:
                chunk = os.read(sys.stdin.fileno(), 256)
                if not chunk:
                    break
                pending += chunk
                if len(pending) > 256:
                    raise ValueError('input too long')
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    last_input = now
                    try:
                        running = state.command(line.decode('ascii'))
                        report('operator command')
                    except (ValueError, UnicodeError) as exc:
                        state.off()
                        report(str(exc) + '; output disabled')
                    if not running:
                        break
            if sink:
                sink.send(state.packet())
            last_send = now
            time.sleep(.05)
    finally:
        state.off()
        try:
            if sink:
                sink.send(state.packet())
        finally:
            if sink:
                sink.close()
        report('stopped')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--send', action='store_true')
    ap.add_argument('--port')
    ap.add_argument('--duration', type=int, default=600)
    args = ap.parse_args()
    if not 1 <= args.duration <= 600 or (args.send and not args.port):
        ap.error('duration 1..600; --send requires --port')
    sink = SerialSink(args.port) if args.send else None
    if sink:
        sink.__enter__()
    run(sink, args.duration)


if __name__ == '__main__':
    main()
