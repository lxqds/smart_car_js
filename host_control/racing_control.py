"""Racing speed targets. Gear limits are not validated vehicle performance."""
import math
from .legacy_protocol import main_control_packet


class RacingControl:
    limits = (500, 1000, 1500, 2000, 3000, 4000, 5000)
    reverse_limit = 500  # mm/s; gear 0 is R on the wire

    def __init__(self):
        self.sent_zero = False
        self.reset()

    def reset(self):
        self.gear = 1
        self.speed = self.angle = 0.0
        self.enabled = self.ready = False
        self.connected = False
        self.last = None
        self.y = True
        self.up = self.down = True  # held shift at connection is not an edge

    def update(self, sample, now):
        # sample: throttle/brake 0..1, steering -1..1, Y, up, down, stop
        dt = 0 if self.last is None else ((now-self.last) & 0xffffffff)
        if sample is None or dt >= 200:
            self.reset()
            return self.packet()
        t, b, s, held, up, down, stop = sample
        if (not all(math.isfinite(x) for x in (t,b,s)) or
                not 0 <= t <= 1 or not 0 <= b <= 1 or not -1 <= s <= 1 or
                any(type(x) is not bool for x in (held,up,down,stop))):
            self.reset()
            return self.packet()
        self.last = now
        self.connected = True
        neutral = t <= .05 and b <= .05 and abs(s) <= .1
        rising = held and not self.y
        if stop:
            self.ready = self.enabled = False
        elif rising:
            if self.enabled:
                self.enabled = False
                self.ready = False
            else:
                self.enabled = self.ready and neutral
                self.ready = False
        elif not held and not self.enabled:
            self.ready = neutral
        self.y = held
        if self.enabled:
            shift_neutral = t <= .05 and self.speed == 0
            if up and not self.up and not down:
                if self.gear > 0 or shift_neutral:
                    self.gear = min(7, self.gear+1)
            if down and not self.down and not up:
                if self.gear > 1 or shift_neutral:
                    self.gear = max(0, self.gear-1)
            throttle = max(0.0, (t-.05)/.95)
            target = 0 if b > .05 else throttle*(self.limits[self.gear-1] if self.gear else -self.reverse_limit)
            # Target ramp, not validated physical acceleration/braking performance.
            rate = 500 if abs(target) > abs(self.speed) else (2000 if b > .05 else 1000)
            step = rate*dt/1000
            self.speed += max(-step, min(step, target-self.speed))
            # Preserve the original MCU direction. Gamepad inversion is applied
            # by RacingDevice, not here. Reach RT1064 command thresholds with
            # asymmetric spans around 1500; these are not measured degrees.
            fraction = 0 if abs(s) <= .1 else (abs(s)-.1)/.9
            command = round(1500 - fraction*360 if s >= 0 else 1500 + fraction*500)
            self.angle = (1500-command)/3.0
        else:
            self.speed = self.angle = 0.0
        self.up, self.down = up, down
        return self.packet()

    def packet(self):
        # Gear/Y metadata also gates motor permission; never replaces control freshness.
        body = bytes((0x24,0x42,0x30,8,2,self.gear,int(self.enabled)))
        # Keep neutral control fresh for MCU local arming while Y is OFF.
        control = main_control_packet(self.speed if self.enabled else 0,
                                      self.angle if self.enabled else 0,3)
        self.sent_zero = not self.enabled
        health = bytes((0x24,0x42,0x31,6,int(self.connected)))
        return control+body+bytes((sum(body)&255,))+health+bytes((sum(health)&255,))
