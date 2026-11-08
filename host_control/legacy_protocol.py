"""Compatible with main-review serial.hpp: 24 42 CMD TOTAL_LEN PAYLOAD SUM8.

This module never opens a serial port. The deadman/send loop remains separate.
"""
import math
import struct


def control_packet(speed_mps, servo_pwm):
    if not math.isfinite(speed_mps) or abs(speed_mps) > 20:
        raise ValueError("speed must be finite and within +/-20 m/s wire sanity bound")
    if not isinstance(servo_pwm, int) or not 0 <= servo_pwm <= 65535:
        raise ValueError("servo field must be uint16")
    body = bytes((0x24, 0x42, 1, 11)) + struct.pack("<fH", speed_mps, servo_pwm)
    return body + bytes((sum(body) & 255,))


def main_control_packet(speed_mm_s, angle, send_k, center=1500):
    """Same conversion as main's sendPack; caller supplies calibrated mapping.

    `angle` is main's tAngle unit, not assumed to be mechanical degrees.
    Reject uint16 overflow instead of copying the old silent wraparound.
    """
    if not all(math.isfinite(x) for x in (speed_mm_s, angle, send_k, center)):
        raise ValueError("nonfinite command")
    value = center - angle * send_k
    if not 0 <= value <= 65535:
        raise ValueError("servo conversion out of range")
    return control_packet(speed_mm_s / 1000.0, int(value))
