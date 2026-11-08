"""Linux evdev adapter. Imports evdev lazily so pure host tests need no device."""
from .gamepad_control import normalize


class InputLost(Exception):
    pass


def inventory():
    from evdev import InputDevice, list_devices
    result = []
    for path in list_devices():
        dev = InputDevice(path)
        try:
            caps = dev.capabilities(absinfo=False)
            result.append(dict(path=path, name=dev.name, vendor=dev.info.vendor,
                               product=dev.info.product, phys=dev.phys,
                               axes={str(code): dev.absinfo(code)._asdict() for code in caps.get(3, [])},
                               keys=caps.get(1, [])))
        finally:
            dev.close()
    return result


class GamepadDevice:
    def __init__(self, profile):
        self.profile = profile
        self.device = None

    def connect(self):
        from evdev import InputDevice, list_devices
        p = self.profile
        matches = []
        try:
            for path in list_devices():
                dev = InputDevice(path)
                try:
                    caps = dev.capabilities(absinfo=False)
                    if (dev.info.vendor == p['vendor'] and dev.info.product == p['product'] and
                            dev.name == p['name'] and
                            all(x in caps.get(3, []) for x in (p['throttle_axis'], p['steering_axis'])) and
                            p['enable_button'] in caps.get(1, [])):
                        matches.append(dev)
                        dev = None
                finally:
                    if dev is not None:
                        dev.close()
            if len(matches) != 1:
                raise InputLost("expected exactly one matching gamepad, found %d" % len(matches))
            self.device = matches.pop()
        finally:
            for dev in matches:
                dev.close()

    def close(self):
        if self.device is not None:
            self.device.close()
            self.device = None

    def snapshot(self):
        if self.device is None:
            raise InputLost("not connected")
        # Drain pending changes first. On SYN_DROPPED or an excessive backlog,
        # caller closes this fd, stops and requires release/neutral after reopen.
        count = 0
        try:
            while True:
                for e in self.device.read():
                    count += 1
                    if (e.type == 0 and e.code == 3) or count > 1024:
                        raise InputLost("SYN_DROPPED/backlog; invalidate and reopen")
                    if e.type == 1 and e.code == self.profile['enable_button'] and e.value == 0:
                        # Never hide a release followed by a quick repress in one batch.
                        raise InputLost("enable released; stop and require neutral again")
        except BlockingIOError:
            pass
        p = self.profile
        throttle = self.device.absinfo(p['throttle_axis'])
        steering = self.device.absinfo(p['steering_axis'])
        a = normalize(throttle.value, throttle.min, throttle.max)
        b = normalize(steering.value, steering.min, steering.max)
        if p['invert_throttle']:
            a = -a
        if p['invert_steering']:
            b = -b
        held = p['enable_button'] in self.device.active_keys()
        return a, b, held
