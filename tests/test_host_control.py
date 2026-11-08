"""No UART/USB/GPIO access. Cross-check Python against the actual C codec."""
import os
from pathlib import Path
import random
import struct
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from host_control.protocol import Frame, Parser, encode, CONTROL, HELLO
from host_control.input_mapper import InputMapper, Intent
from host_control.protocol import ARM, ACK, DISARM, ESTOP
from host_control.transaction import Transaction
from host_control.legacy_protocol import control_packet, main_control_packet
from host_control.display_probe import make_packet, write_all
from unittest.mock import patch


class ProtocolTests(unittest.TestCase):
    def test_display_probe_without_devices(self):
        self.assertEqual(make_packet(250, 10), control_packet(0.25, 1470))
        with self.assertRaises(ValueError):
            make_packet(501, 0)
        with self.assertRaises(ValueError):
            make_packet(0, float("nan"))
        with patch("host_control.display_probe.select.select", return_value=([], [7], [])), patch("host_control.display_probe.os.write", side_effect=[2, 2, 1]) as write:
            write_all(7, b"12345")
            self.assertEqual([c.args[1] for c in write.call_args_list], [b"12345", b"345", b"5"])
        with patch("host_control.display_probe.select.select", return_value=([], [], [])):
            with self.assertRaises(TimeoutError):
                write_all(7, b"x")
    def test_main_legacy_wire(self):
        self.assertEqual(control_packet(0.5, 1500).hex(), "2442010b0000003fdc0592")
        self.assertEqual(main_control_packet(500, 10, 3), control_packet(0.5, 1470))
        with self.assertRaises(ValueError):
            control_packet(float("nan"), 1500)
        with self.assertRaises(ValueError):
            main_control_packet(500, 10000, 3)
    def test_fragmentation_and_coalescing(self):
        frame = Frame(CONTROL, 0xffffffff, 0x12345678, struct.pack("<iiB", -123, 15000, 1))
        wire = encode(frame)
        for split in range(len(wire) + 1):
            p = Parser()
            self.assertEqual(p.feed(wire[:split], 0) + p.feed(wire[split:], 1), [frame])
        self.assertEqual(Parser().feed(wire * 1000, 0), [frame] * 1000)

    def test_recovery_and_timeout(self):
        frame = Frame(HELLO, 1, 42)
        wire = encode(frame)
        bad = bytearray(wire)
        bad[-1] ^= 1
        p = Parser()
        self.assertEqual(p.feed(b"noise" + bad + wire, 0), [frame])
        self.assertEqual(p.crc_errors, 1)
        p = Parser()
        p.feed(wire[:5], 0xffffffcd)
        self.assertEqual(p.feed(wire, 49), [frame])
        self.assertEqual(p.timeouts, 1)
        bad = b"\xa5\x5a\x01\x05\xff\xff"
        self.assertEqual(Parser().feed(bad + wire, 0), [frame])
        with self.assertRaises(ValueError):
            encode(Frame(HELLO, 1, 1, bytes(33)))

    def test_c_python_cross_vectors(self):
        exe = Path(os.environ["CAR_PROTOCOL_TEST_EXE"]).resolve()
        rng = random.Random(3576)
        vectors = [Frame(HELLO, 1, 0x12345678), Frame(CONTROL, 0xffffffff, 42,
                   struct.pack("<iiB", -500, -15000, 1))]
        vectors += [Frame(rng.randrange(256), rng.getrandbits(32), rng.getrandbits(32),
                          bytes(rng.randrange(256) for _ in range(n))) for n in range(33)]
        for f in vectors:
            cwire = subprocess.check_output([str(exe), "encode", str(f.command),
                       str(f.sequence), str(f.session), f.payload.hex()], text=True).strip()
            self.assertEqual(cwire, encode(f).hex())
            self.assertEqual(Parser().feed(bytes.fromhex(cwire), 0), [f])
        # Python -> C, with noise, bad CRC, oversized length, and back-to-back frames.
        bad = bytearray(encode(vectors[0])); bad[-1] ^= 1
        stream = b"junk" + bad + b"\xa5\x5a\x01\x01\xff\xff" + b"".join(map(encode, vectors))
        lines = subprocess.check_output([str(exe), "decode", stream.hex()], text=True).splitlines()
        expected = [f"{f.command} {f.sequence} {f.session} {f.payload.hex()}".rstrip() for f in vectors]
        self.assertEqual([line.rstrip() for line in lines], expected)


class InputTests(unittest.TestCase):
    def test_enable_release_disconnect_reconnect(self):
        m = InputMapper()
        self.assertEqual(m.sample(0, 0, True, 0), Intent())  # plugged in holding trigger
        self.assertEqual(m.sample(0, 0, False, 10), Intent())
        self.assertTrue(m.sample(0, 0, True, 20).arm_request)
        value = m.sample(1, -1, True, 40)
        self.assertEqual((value.speed_mm_s, value.steering_mdeg), (500, -15000))
        self.assertEqual(m.sample(1, -1, False, 60), Intent())
        self.assertEqual(m.sample(1, -1, True, 80), Intent())
        m.disconnect()
        self.assertEqual(m.sample(0, 0, True, 100), Intent())
        m.sample(0, 0, False, 110)
        self.assertTrue(m.sample(0, 0, True, 120).arm_request)

    def test_deadzone_stale_invalid_wrap(self):
        m = InputMapper()
        m.sample(0.05, -0.05, False, 0xfffffff0)
        m.sample(0, 0, True, 0xfffffff1)
        self.assertTrue(m.sample(0.1, -0.1, True, 5).deadman)
        self.assertEqual(m.current(205), Intent())
        self.assertEqual(m.sample(0, 0, True, 206), Intent())
        for bad in (float("nan"), float("inf"), -1.01, 1.01):
            m.sample(0, 0, False, 207)
            m.sample(0, 0, True, 208)
            self.assertEqual(m.sample(bad, 0, True, 209), Intent())
        with self.assertRaises(ValueError):
            InputMapper(deadzone=1)


class TransactionTests(unittest.TestCase):
    def test_stop_ack_must_confirm_stopped_state(self):
        for command, state in ((DISARM, 2), (ESTOP, 1)):
            t = Transaction(Frame(command, 1, 42), 0)
            t.poll(0)
            self.assertTrue(t.accept(Frame(ACK, 1, 42, bytes([command, 0, state, 0])), 1))
            self.assertEqual(t.state, "rejected")

    def test_lost_ack_retry_and_correlation(self):
        q = Frame(ARM, 12, 42, struct.pack("<iiB", 0, 0, 1))
        t = Transaction(q, 0)
        first = t.poll(0)
        self.assertIsNone(t.poll(99))
        self.assertEqual(t.poll(100), first)
        self.assertFalse(t.accept(Frame(ACK, 11, 42, bytes([ARM, 0, 2, 0])), 101))
        self.assertFalse(t.accept(Frame(ACK, 12, 41, bytes([ARM, 0, 2, 0])), 101))
        self.assertTrue(t.accept(Frame(ACK, 12, 42, bytes([ARM, 4, 2, 0])), 101))
        self.assertEqual(t.state, "complete")
        self.assertIsNone(t.poll(200))

    def test_finite_retries_wrap_and_no_control_retry(self):
        q = Frame(ARM, 12, 42, struct.pack("<iiB", 0, 0, 1))
        t = Transaction(q, 0xffffffce)
        self.assertIsNotNone(t.poll(0xffffffce))
        self.assertIsNotNone(t.poll(50))
        self.assertIsNotNone(t.poll(150))
        self.assertIsNone(t.poll(250))
        self.assertEqual(t.state, "timeout")
        self.assertFalse(t.accept(Frame(ACK, 12, 42, bytes([ARM, 0, 2, 0])), 251))
        self.assertIsNone(t.poll(500))
        with self.assertRaises(ValueError):
            Transaction(Frame(CONTROL, 1, 42), 0)
        t = Transaction(q, 0);t.poll(0)
        self.assertTrue(t.accept(Frame(ACK, 12, 42, bytes([ARM, 4, 1, 0])), 1))
        self.assertEqual(t.state, "rejected")


if __name__ == "__main__":
    unittest.main()
