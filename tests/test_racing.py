import unittest
from host_control.racing_control import RacingControl


class RacingTests(unittest.TestCase):
    def test_reverse_shift_and_reconnect(self):
        import struct
        c=RacingControl();now=0
        def tick(t=0,b=0,s=0,y=False,up=False,down=False):
            nonlocal now
            now+=50
            return c.update((t,b,s,y,up,down,False),now)
        tick();tick(y=True);tick()
        tick(t=1,down=True);self.assertEqual(c.gear,1) # held throttle blocks R
        for _ in range(10):tick()
        tick(down=True);self.assertEqual(c.gear,0)
        for _ in range(25): packet=tick(t=1)
        self.assertEqual(c.speed,-500)
        self.assertAlmostEqual(struct.unpack('<f',packet[4:8])[0],-.5)
        self.assertEqual(packet[16],0) # R metadata
        tick(t=1,up=True);self.assertEqual(c.gear,0) # no immediate forward reversal
        for _ in range(10):tick(b=1)
        self.assertEqual(c.speed,0)
        tick(up=True);self.assertEqual(c.gear,1)
        c.update(None,now+50);self.assertFalse(c.enabled);self.assertEqual(c.gear,1)
        tick(y=True);self.assertFalse(c.enabled) # held Y at reconnect cannot enable

    def test_profile_inversion_only_at_gamepad_input(self):
        import json
        import struct
        from pathlib import Path
        from types import SimpleNamespace
        from host_control.racing_evdev import RacingDevice
        from host_control.legacy_protocol import control_packet
        profile=json.loads((Path(__file__).parents[1]/'host_control/zikway_racing_profile.json').read_text())
        self.assertTrue(profile['invert_steering'])
        class Fake:
            steering=0
            def read(self): raise BlockingIOError()
            def active_keys(self): return []
            def absinfo(self, code):
                return SimpleNamespace(value=self.steering if code==0 else -1,min=-1,max=1)
        d=RacingDevice(profile);d.device=Fake()
        c=RacingControl();neutral=d.snapshot()
        c.update(neutral,0)
        c.update(neutral[:3]+(True,False,False,False),50)
        for t,raw in ((100,-1),(150,1)):
            d.device.steering=raw
            sample=d.snapshot()
            self.assertEqual(sample[2],-raw)
            self.assertEqual(sample[:2],(0,0))
            packet=c.update(sample,t)
            command=struct.unpack('<H',packet[8:10])[0]
            # Same direction as the verified small range, now original limits.
            self.assertEqual(command,1140 if raw<0 else 2000)
            self.assertEqual(3000-command,1860 if raw<0 else 1000)
        # Non-gamepad callers keep the original command and wire direction.
        self.assertEqual(struct.unpack('<H',control_packet(0,1470)[8:10])[0],1470)

    def test_gate_shift_ramp_brake_loss(self):
        c=RacingControl(); now=0
        def tick(t=0,b=0,s=0,h=True,u=False,d=False,stop=False):
            nonlocal now
            now+=50
            return c.update((t,b,s,h,u,d,stop),now)
        tick(t=1); self.assertFalse(c.enabled)
        self.assertEqual(len(tick(t=1)),25)  # control + Y/gear + gamepad health
        tick(h=False); tick(); self.assertTrue(c.enabled)
        tick(h=False);self.assertTrue(c.enabled)  # release does not disable
        tick();self.assertFalse(c.enabled)       # second press toggles off
        tick(h=False);tick();self.assertTrue(c.enabled)
        for gear in range(2,8):
            tick(u=True); self.assertEqual(c.gear,gear)
            tick(u=True); self.assertEqual(c.gear,gear)
            tick()
        for _ in range(220): tick(t=1,s=1)
        self.assertEqual(c.speed,5000); self.assertEqual(c.angle,120)
        packet=tick(t=1,b=1)
        self.assertEqual(c.speed,4900)
        self.assertEqual(packet[11:17],bytes((0x24,0x42,0x30,8,2,7)))
        self.assertEqual(packet[18],sum(packet[11:18])&255)
        self.assertEqual(packet[19:24],bytes((0x24,0x42,0x31,6,1)))
        self.assertEqual(packet[24],sum(packet[19:24])&255)
        tick(stop=True);self.assertEqual(c.speed,0);self.assertFalse(c.enabled)
        tick(t=1);self.assertFalse(c.enabled)
        c.update(None,now+50);self.assertEqual(c.gear,1)
        tick(t=1);self.assertFalse(c.enabled)

    def test_original_lt_gradual_braking_both_directions(self):
        import struct
        for gear,initial in ((7,3000),(0,-500)):
            c=RacingControl()
            c.update((0,0,0,False,False,False,False),0)
            c.update((0,0,0,True,False,False,False),50)
            c.gear=gear;c.speed=initial
            packet=c.update((1,1,.5,False,False,False,False),100)
            expected=2900 if gear else -400
            self.assertEqual(c.speed,expected)  # 2m/s per second, 50ms step
            self.assertAlmostEqual(struct.unpack('<f',packet[4:8])[0],expected/1000,places=5)
            self.assertTrue(c.enabled)
            self.assertNotEqual(struct.unpack('<H',packet[8:10])[0],1500)
            for n in range(1,31):
                c.update((1,1,0,False,False,False,False),100+n*50)
            self.assertEqual(c.speed,0)
            c.update((1,0,0,False,False,False,False),1650)
            self.assertEqual(c.speed,25 if gear else -25)

    def test_disabled_neutral_heartbeat(self):
        from host_control.legacy_protocol import main_control_packet
        c=RacingControl()
        for i in range(20):
            packet=c.update((1,0,1,False,False,False,False),i*50)
            self.assertEqual(packet[:11],main_control_packet(0,0,3))
            self.assertEqual(packet[17],0)
            self.assertEqual(packet[10],sum(packet[:10])&255)
        packet=c.update(None,1100)
        self.assertEqual(packet[:11],main_control_packet(0,0,3))
        self.assertEqual(packet[17],0)
        self.assertEqual(packet[23],0) # explicit disconnected health

    def test_full_legacy_steering_range(self):
        import struct
        c=RacingControl()
        c.update((0,0,0,False,False,False,False),0)
        c.update((0,0,0,True,False,False,False),50)
        commands=[]
        for i in range(2001):
            s=(i-1000)/1000
            packet=c.update((0,0,s,False,False,False,False),100+i*50)
            command=struct.unpack('<H',packet[8:10])[0]
            self.assertTrue(1140<=command<=2000)
            self.assertEqual(packet[10],sum(packet[:10])&255)
            if abs(s)<=.1: self.assertEqual(command,1500)
            commands.append(command)
        self.assertEqual((commands[0],commands[-1]),(2000,1140))
        self.assertTrue(all(a>=b for a,b in zip(commands,commands[1:])))
        packet=c.update(None,100+2001*50)
        self.assertEqual(struct.unpack('<H',packet[8:10])[0],1500)
        self.assertFalse(c.enabled)

    def test_timeout_wrap_and_invalid(self):
        c=RacingControl()
        c.update((0,0,0,False,False,False,False),0xfffffff0)
        c.update((0,0,0,True,False,False,False),34)
        self.assertTrue(c.enabled)
        c.update((1,0,0,True,False,False,False),234)
        self.assertFalse(c.enabled);self.assertEqual(c.speed,0)
        c.update((float('nan'),0,0,False,False,False,False),284)
        self.assertFalse(c.ready)


if __name__=='__main__': unittest.main()
