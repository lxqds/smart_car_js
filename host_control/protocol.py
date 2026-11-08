"""Draft car v1 codec. No device access or dependencies beyond Python stdlib."""
from dataclasses import dataclass
import binascii
import struct

MAX_PAYLOAD = 32
PARTIAL_TIMEOUT_MS = 100
HELLO, CAPS, ARM, DISARM, CONTROL, ESTOP, STATUS, ACK = range(1, 9)


@dataclass(frozen=True)
class Frame:
    command: int
    sequence: int
    session: int
    payload: bytes = b""


def encode(frame):
    if len(frame.payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 32 bytes")
    body = struct.pack("<BBHII", 1, frame.command, len(frame.payload),
                       frame.sequence, frame.session) + frame.payload
    return b"\xa5\x5a" + body + struct.pack("<H", binascii.crc_hqx(body, 0xffff))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.last_ms = 0
        self.crc_errors = self.header_errors = self.timeouts = 0

    def feed(self, data, now_ms):
        """now_ms is uint32 monotonic milliseconds; bounded retained storage."""
        result = []
        for byte in data:
            if self.buffer and ((now_ms - self.last_ms) & 0xffffffff) >= PARTIAL_TIMEOUT_MS:
                self.buffer.clear()
                self.timeouts += 1
            self.last_ms = now_ms
            self.buffer.append(byte)
            while self.buffer:
                if self.buffer[0] != 0xa5 or (len(self.buffer) >= 2 and self.buffer[1] != 0x5a):
                    del self.buffer[0]
                    continue
                if len(self.buffer) < 6:
                    break
                length = int.from_bytes(self.buffer[4:6], "little")
                if self.buffer[2] != 1 or length > MAX_PAYLOAD:
                    self.header_errors += 1
                    del self.buffer[0]
                    continue
                total = length + 16
                if len(self.buffer) < total:
                    break
                wire = self.buffer[:total]
                if binascii.crc_hqx(wire[2:-2], 0xffff) != int.from_bytes(wire[-2:], "little"):
                    self.crc_errors += 1
                    del self.buffer[0]
                    continue
                _, command, _, sequence, session = struct.unpack("<BBHII", wire[2:14])
                result.append(Frame(command, sequence, session, bytes(wire[14:-2])))
                del self.buffer[:total]
        return result
