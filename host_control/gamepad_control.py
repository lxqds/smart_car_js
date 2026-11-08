"""Outputs-off legacy display controller. No hardware enable command exists here."""
from .input_mapper import InputMapper
from .display_probe import make_packet


def normalize(value, minimum, maximum):
    if maximum <= minimum or not minimum <= value <= maximum:
        raise ValueError("invalid axis range/value")
    return (value - minimum) * 2.0 / (maximum - minimum) - 1.0


class GamepadControl:
    def __init__(self):
        self.mapper = InputMapper(speed_limit=500, steering_limit=15000)
        self.enabled = False
        self.sent_zero = False
        self.speed = self.angle = 0

    def update(self, sample, now_ms):
        """Return one wire packet or None; sample is (throttle, steering, held)."""
        if sample is None:
            self.mapper.disconnect()
            intent = self.mapper.current(now_ms)
        else:
            intent = self.mapper.sample(*sample, now_ms)
        self.enabled = intent.deadman
        self.speed = intent.speed_mm_s
        self.angle = intent.steering_mdeg / 1000.0
        if self.enabled:
            self.sent_zero = False
            return make_packet(self.speed, self.angle)
        if not self.sent_zero:
            self.sent_zero = True
            return make_packet(0, 0)
        return None
