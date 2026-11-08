"""Requires measured axis/button profile; no assumed Xbox/Linux key mapping."""
from .gamepad_evdev import GamepadDevice, InputLost


class RacingDevice(GamepadDevice):
    def connect(self):
        p=self.profile
        if p.get('racing_mapping_verified') is not True:
            raise ValueError('measure RT/LT/B/X/A before enabling racing profile')
        super().connect()
        caps=self.device.capabilities(absinfo=False)
        if (any(p[k] not in caps.get(3,[]) for k in ('throttle_axis','brake_axis','steering_axis')) or
            any(p[k] not in caps.get(1,[]) for k in ('enable_button','up_button','down_button','stop_button'))):
            self.close()
            raise InputLost('racing capabilities mismatch')

    def snapshot(self):
        # Releasing Y is not a stop: Y toggles on press, unlike the old L1 gate.
        pressed=set();count=0
        try:
            while True:
                for e in self.device.read():
                    count+=1
                    if (e.type==0 and e.code==3) or count>1024:
                        raise InputLost('SYN_DROPPED/backlog')
                    if e.type==1 and e.value==1:
                        pressed.add(e.code)
        except BlockingIOError:
            pass
        p=self.profile
        def axis(key, trigger=False):
            a=self.device.absinfo(p[key])
            if a.max<=a.min or not a.min<=a.value<=a.max:
                raise InputLost('invalid axis')
            x=(a.value-a.min)/(a.max-a.min)
            # Profiles use invert_steering/invert_throttle (without _axis).
            if p.get('invert_'+key[:-5],p.get('invert_'+key,False)): x=1-x
            return x if trigger else 2*x-1
        keys=set(self.device.active_keys()) | pressed
        return (axis('throttle_axis',True),axis('brake_axis',True),axis('steering_axis'),
                *(p[k] in keys for k in ('enable_button','up_button','down_button','stop_button')))
