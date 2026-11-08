"""Bounded command/ACK transaction, with caller-owned clock and transport.

CONTROL must never enter this retry queue. After failure the caller disarms
locally and requires a new neutral/user-enable cycle; this class never re-arms.
"""
from .protocol import HELLO, CAPS, ARM, DISARM, ESTOP, STATUS, ACK, encode


class Transaction:
    def __init__(self, request, now_ms, retry_ms=100, max_attempts=3):
        if request.command not in (HELLO, ARM, DISARM, ESTOP, STATUS):
            raise ValueError("only bounded management commands may retry")
        if not request.session or not 0 < retry_ms < 0x80000000 or not 1 <= max_attempts <= 10:
            raise ValueError("invalid retry configuration")
        self.request = request
        self.wire = encode(request)
        self.retry_ms, self.max_attempts = retry_ms, max_attempts
        self.last_ms = now_ms
        self.attempts = 0
        self.state = "pending"
        self.reply = None

    def poll(self, now_ms):
        if self.state != "pending":
            return None
        elapsed = (now_ms - self.last_ms) & 0xffffffff
        if self.attempts and elapsed < self.retry_ms:
            return None
        if self.attempts >= self.max_attempts:
            self.state = "timeout"
            return None
        self.attempts += 1
        self.last_ms = now_ms
        return self.wire

    def accept(self, reply, now_ms):
        if self.state != "pending" or not self.attempts:
            return False
        if ((now_ms - self.last_ms) & 0xffffffff) >= self.retry_ms:
            # Late replies do not resurrect an expired attempt. Caller polls next.
            return False
        q = self.request
        if reply.session != q.session or reply.sequence != q.sequence:
            return False
        if q.command == HELLO and reply.command == CAPS:
            if (len(reply.payload) != 16 or reply.payload[0] != 1 or
                    reply.payload[1] != 32 or reply.payload[2] > 3 or reply.payload[3] != 0):
                return False
            ok = True
        elif reply.command == ACK:
            if (len(reply.payload) != 4 or reply.payload[0] != q.command or
                    reply.payload[1] > 6 or reply.payload[2] > 3 or reply.payload[3] != 0):
                return False
            result, state = reply.payload[1:3]
            ok = result == 0
            if q.command == ARM:
                # A lost ACK retry is stale at the MCU; accept only if still armed.
                ok = result in (0, 4) and state == 2
            elif q.command == DISARM:
                ok = result == 0 and state in (1, 3)
            elif q.command == ESTOP:
                ok = result == 0 and state == 3
            if q.command == HELLO:
                ok = False  # HELLO success must supply CAPS, not a bare ACK.
        else:
            return False
        self.reply = reply
        self.state = "complete" if ok else "rejected"
        return True
