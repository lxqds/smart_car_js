import struct
import unittest
from host_control.servo_calibration import Calibration


class CalibrationTests(unittest.TestCase):
    def exercise_loop(self, chunks, delay=.05):
        from unittest.mock import patch, Mock
        from host_control import servo_calibration as module
        class Sink:
            def __init__(self): self.packets=[]; self.closed=False
            def send(self, p): self.packets.append(p)
            def close(self): self.closed=True
        sink=Sink();clock=[0.0]
        def sleep(_): clock[0]+=delay
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(module.time,'sleep',side_effect=sleep), \
             patch.object(module.signal,'signal'), \
             patch.object(module.select,'select',return_value=([1],[],[])), \
             patch.object(module.os,'read',side_effect=chunks), \
             patch.object(module.sys,'stdin',Mock()), patch('builtins.print'):
            module.run(sink,600)
        self.assertTrue(sink.closed)
        self.assertEqual(sink.packets[-1][17],0)
        self.assertEqual(struct.unpack('<fH',sink.packets[-1][4:10]),(0,1500))
        return sink.packets

    def test_loop_eof_and_invalid_command_stop(self):
        packets=self.exercise_loop([b'enable\n',b'+10\n',b''])
        self.assertEqual(struct.unpack('<fH',packets[1][4:10]),(0,1490))
        self.assertEqual(packets[1][17],1)
        packets=self.exercise_loop([b'enable\n',b'2000\n',b''])
        self.assertEqual(packets[1][17],0)

    def test_scheduler_gap_disables_without_auto_reenable(self):
        packets=self.exercise_loop([b'enable\n',b'status\n',b''],delay=.3)
        self.assertEqual(packets[0][17],1)
        self.assertEqual(packets[1][17],0)

    def check_packet(self, c, pulse, enabled):
        p = c.packet()
        self.assertEqual(len(p), 19)
        self.assertEqual(p[:4], bytes((0x24, 0x42, 1, 11)))
        self.assertEqual(struct.unpack('<fH', p[4:10]), (0.0, 3000-pulse))
        self.assertEqual(p[10], sum(p[:10]) & 255)
        self.assertEqual(p[11:18], bytes((0x24, 0x42, 0x30, 8, 2, 1, enabled)))
        self.assertEqual(p[18], sum(p[11:18]) & 255)

    def test_neutral_enable_steps_off(self):
        c = Calibration()
        self.check_packet(c, 1500, 0)
        with self.assertRaises(ValueError):
            c.command('+10')
        c.command('enable')
        c.command('+10')
        self.check_packet(c, 1510, 1)
        c.command('-5')
        self.check_packet(c, 1505, 1)
        c.command('center')
        self.check_packet(c, 1500, 1)
        c.command('off')
        self.check_packet(c, 1500, 0)
        self.assertFalse(c.command('quit'))

    def test_bounds_and_no_large_jump(self):
        c = Calibration()
        c.command('enable')
        for _ in range(10):
            c.command('+10')
        self.check_packet(c, 1600, 1)
        with self.assertRaises(ValueError):
            c.command('+5')
        self.assertEqual(c.pulse, 1600)
        for _ in range(20):
            c.command('-10')
        self.check_packet(c, 1400, 1)
        for command in ('-5', '2000', '+100', 'nan', 'enable'):
            with self.assertRaises(ValueError):
                c.command(command)
        c.off()
        self.check_packet(c, 1500, 0)


if __name__ == '__main__':
    unittest.main()
