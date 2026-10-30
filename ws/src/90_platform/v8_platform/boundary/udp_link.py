"""UDP command link + receiver-side health tracking (loopback only).

The sender owns one socket and streams CommandFrames at a fixed rate; the
receiver tracks sequence gaps, duplicates, inter-arrival jitter and command
age. No robot address is configured by default.
"""
import socket
import time
from collections import deque

from .protocol import CommandFrame, ProtocolError, decode, decode_telemetry


class CommandSender:
    def __init__(self, target, sock=None):
        self.target = tuple(target)
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.sent = 0

    def send(self, vx, vy, wz, mode, flags=0, t_ns=None):
        stamp = time.monotonic_ns() if t_ns is None else t_ns
        frame = CommandFrame(self.seq, stamp, vx, vy, wz, mode, flags)
        self.sock.sendto(frame.encode(), self.target)
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.sent += 1
        return frame

    def close(self):
        self.sock.close()


class CommandReceiver:
    def __init__(self, bind=('127.0.0.1', 0), period_s=0.002, window=4096):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(tuple(bind))
        self.sock.settimeout(0.2)
        self.period_s = period_s
        self.last = None
        self.last_rx = None
        self.prev_seq = None
        self._arrivals = deque(maxlen=window)
        self.stats = {'frames': 0, 'bad': 0, 'loss': 0, 'dup': 0, 'reorder': 0}

    @property
    def address(self):
        return self.sock.getsockname()

    def poll(self):
        try:
            data, _ = self.sock.recvfrom(2048)
        except (socket.timeout, BlockingIOError):
            return None
        now = time.monotonic()
        try:
            frame = decode(data)
        except ProtocolError:
            self.stats['bad'] += 1
            return None
        self.stats['frames'] += 1
        if self.prev_seq is not None:
            gap = (frame.seq - self.prev_seq) & 0xFFFFFFFF
            if gap == 0:
                self.stats['dup'] += 1
            elif gap > 1:
                if gap < 0x80000000:
                    self.stats['loss'] += gap - 1
                else:
                    self.stats['reorder'] += 1
        self.prev_seq = frame.seq
        self._arrivals.append(now)
        self.last, self.last_rx = frame, now
        return frame

    def age_s(self, now=None):
        if self.last_rx is None:
            return None
        return (time.monotonic() if now is None else now) - self.last_rx

    def jitter(self, ratio=0.99):
        gaps = sorted(b - a for a, b in zip(self._arrivals, list(self._arrivals)[1:]))
        if not gaps:
            return {'p50': 0., 'p99': 0.}
        return {'p50': gaps[len(gaps) // 2], 'p99': gaps[min(len(gaps) - 1, int(len(gaps) * ratio))]}

    def close(self):
        self.sock.close()


class TelemetrySender:
    def __init__(self, target, sock=None):
        self.target = tuple(target)
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.sent = 0

    def send(self, frame):
        self.sock.sendto(frame.encode(), self.target)
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.sent += 1
        return frame

    def close(self):
        self.sock.close()


class TelemetryReceiver:
    def __init__(self, bind=('127.0.0.1', 0), window=4096):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(tuple(bind))
        self.sock.settimeout(0.2)
        self.last = None
        self.last_rx = None
        self.prev_seq = None
        self._arrivals = deque(maxlen=window)
        self.stats = {'frames': 0, 'bad': 0, 'loss': 0, 'dup': 0, 'reorder': 0}

    @property
    def address(self):
        return self.sock.getsockname()

    def poll(self):
        try:
            data, _ = self.sock.recvfrom(4096)
        except (socket.timeout, BlockingIOError):
            return None
        now = time.monotonic()
        try:
            frame = decode_telemetry(data)
        except ProtocolError:
            self.stats['bad'] += 1
            return None
        self.stats['frames'] += 1
        if self.prev_seq is not None:
            gap = (frame.seq - self.prev_seq) & 0xFFFFFFFF
            if gap == 0:
                self.stats['dup'] += 1
            elif gap > 1:
                if gap < 0x80000000:
                    self.stats['loss'] += gap - 1
                else:
                    self.stats['reorder'] += 1
        self.prev_seq = frame.seq
        self._arrivals.append(now)
        self.last, self.last_rx = frame, now
        return frame

    def age_s(self, now=None):
        if self.last_rx is None:
            return None
        return (time.monotonic() if now is None else now) - self.last_rx

    def jitter(self, ratio=0.99):
        gaps = sorted(b - a for a, b in zip(self._arrivals, list(self._arrivals)[1:]))
        if not gaps:
            return {'p50': 0., 'p99': 0.}
        return {'p50': gaps[len(gaps) // 2], 'p99': gaps[min(len(gaps) - 1, int(len(gaps) * ratio))]}

    def close(self):
        self.sock.close()
