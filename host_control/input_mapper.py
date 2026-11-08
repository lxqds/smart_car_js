"""Dry-run normalized input gate; Linux device discovery/mapping is a later step.

The adapter must provide full, currently polled snapshots, not merely change
events. A quiet joystick is not a disconnection. Do not fabricate fresh samples
after device removal or evdev SYN_DROPPED; invalidate first.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Intent:
    speed_mm_s: int = 0
    steering_mdeg: int = 0
    deadman: bool = False
    arm_request: bool = False


class InputMapper:
    def __init__(self, speed_limit=500, steering_limit=15000, deadzone=0.1, timeout_ms=200):
        # Draft simulation values, never an approved vehicle calibration.
        if (not isinstance(speed_limit, int) or not 0 < speed_limit <= 0x7fffffff or
                not isinstance(steering_limit, int) or not 0 < steering_limit <= 0x7fffffff or
                not math.isfinite(deadzone) or not 0 <= deadzone < 1 or
                not isinstance(timeout_ms, int) or not 0 < timeout_ms < 0x80000000):
            raise ValueError("invalid input limits")
        self.speed_limit, self.steering_limit = speed_limit, steering_limit
        self.deadzone, self.timeout_ms = deadzone, timeout_ms
        self.disconnect()

    def disconnect(self):
        self.ready = self.held = self.have_sample = False
        self.last_ms = 0
        self.intent = Intent()

    def _axis(self, value):
        if abs(value) <= self.deadzone:
            return 0.0
        return math.copysign((abs(value) - self.deadzone) / (1 - self.deadzone), value)

    def sample(self, throttle, steering, deadman, now_ms):
        self.current(now_ms)
        if (type(deadman) is not bool or not all(math.isfinite(x) and -1 <= x <= 1
                                                for x in (throttle, steering))):
            self.disconnect()
            return self.intent
        self.last_ms, self.have_sample = now_ms, True
        speed, angle = self._axis(throttle), self._axis(steering)
        if not deadman:
            self.ready = speed == 0 and angle == 0
            self.held = False
            self.intent = Intent()
        elif not self.held:
            if self.ready and speed == 0 and angle == 0:
                self.held = True
                self.intent = Intent(deadman=True, arm_request=True)
            else:
                self.ready = False
                self.intent = Intent()
        else:
            self.intent = Intent(round(speed * self.speed_limit),
                                 round(angle * self.steering_limit), True, False)
        return self.intent

    def current(self, now_ms):
        if self.have_sample and ((now_ms - self.last_ms) & 0xffffffff) >= self.timeout_ms:
            self.disconnect()
        return self.intent
