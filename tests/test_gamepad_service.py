import signal
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
from host_control import gamepad_display


class ServiceTests(unittest.TestCase):
    def test_continuous_run_and_sigterm_disables(self):
        handlers = {}
        packets = []
        clock = [0.0]
        class Device:
            device = None
            count = 0
            def __init__(self, profile): pass
            def connect(self): self.device = self
            def close(self): self.device = None
            def snapshot(self):
                self.count += 1
                if self.count == 8: handlers[signal.SIGTERM](signal.SIGTERM, None)
                return (0,0,0,self.count == 2,False,False,False)
        class Sink:
            fd = None
            def __init__(self, path): pass
            def __enter__(self): self.fd=1;return self
            def send(self, packet): packets.append(packet)
            def close(self): self.fd=None
        def sleep(seconds): clock[0] += seconds
        profile=str(Path(__file__).parents[1]/'host_control/zikway_racing_profile.json')
        argv=['gamepad_display','--racing','--profile',profile,'--send','--port','fake','--duration','0']
        with patch.object(sys,'argv',argv), patch('host_control.racing_evdev.RacingDevice',Device), \
             patch.object(gamepad_display,'SerialSink',Sink), \
             patch.object(gamepad_display.signal,'signal',lambda s,h: handlers.__setitem__(s,h)), \
             patch.object(gamepad_display.time,'monotonic',lambda:clock[0]), \
             patch.object(gamepad_display.time,'sleep',sleep), patch('builtins.print'):
            gamepad_display.main()
        self.assertEqual(len(packets),9)
        self.assertEqual(packets[0][17],0)
        self.assertEqual(packets[1][17],1)
        self.assertEqual(packets[-1][17],0)
        self.assertEqual(packets[-1][4:8],b'\0'*4)

if __name__=='__main__': unittest.main()
