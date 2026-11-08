import sys
from pathlib import Path
import unittest
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from host_control.gamepad_control import GamepadControl, normalize
from host_control.gamepad_evdev import GamepadDevice, InputLost
from host_control.display_probe import make_packet


class GamepadTests(unittest.TestCase):
    def test_hold_release_loss_reconnect(self):
        c = GamepadControl()
        self.assertEqual(c.update((1, 1, False), 0), make_packet(0, 0))
        self.assertIsNone(c.update((1, 1, True), 50))
        c.update((0, 0, False), 100)
        self.assertEqual(c.update((0, 0, True), 150), make_packet(0, 0))
        self.assertEqual(c.update((1, -1, True), 200), make_packet(500, -15))
        self.assertEqual(c.update((1, -1, False), 250), make_packet(0, 0))
        self.assertIsNone(c.update((1, -1, False), 300))
        c.update((0, 0, False), 350); c.update((0, 0, True), 400)
        c.update((1, 1, True), 450)
        self.assertEqual(c.update(None, 500), make_packet(0, 0))
        self.assertIsNone(c.update((0, 0, True), 550))
        c.update((0, 0, False), 600)
        self.assertEqual(c.update((0, 0, True), 650), make_packet(0, 0))

    def test_sample_stall_and_range(self):
        c = GamepadControl();c.update((0, 0, False), 0);c.update((0, 0, True), 50)
        c.update((1, 1, True), 100)
        self.assertEqual(c.update((1, 1, True), 300), make_packet(0, 0))
        self.assertFalse(c.enabled)
        self.assertEqual(normalize(0, 0, 255), -1)
        self.assertEqual(normalize(255, 0, 255), 1)
        with self.assertRaises(ValueError): normalize(256, 0, 255)
        with self.assertRaises(ValueError): normalize(0, 0, 0)

    def test_snapshot_queries_stationary_state_and_loss(self):
        class Fake:
            def read(self): raise BlockingIOError()
            def absinfo(self, code): return SimpleNamespace(value=128,min=0,max=255)
            def active_keys(self): return [310]
        p=dict(throttle_axis=1,steering_axis=0,enable_button=310,invert_throttle=True,invert_steering=False)
        d=GamepadDevice(p);d.device=Fake()
        a,b,held=d.snapshot();self.assertTrue(held);self.assertLess(abs(a),0.01)
        d.device.read=lambda: [SimpleNamespace(type=0,code=3)]
        with self.assertRaises(InputLost): d.snapshot()
        d.device.read=lambda: [SimpleNamespace(type=1,code=310,value=0), SimpleNamespace(type=1,code=310,value=1)]
        with self.assertRaises(InputLost): d.snapshot()
        def unplug(): raise OSError('ENODEV')
        d.device.read=unplug
        with self.assertRaises(OSError): d.snapshot()


if __name__=='__main__': unittest.main()
