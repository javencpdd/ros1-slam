"""Boundary contract tests: frame codec, rate bridge, sequence health."""
import math
import socket
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from boundary import protocol
from boundary.rate_bridge import CommandBridge, Limits
from boundary.protocol import CommandFrame, ProtocolError, decode
from boundary.udp_link import CommandReceiver, CommandSender


class ProtocolTests(unittest.TestCase):
    def test_frame_round_trip(self):
        frame = CommandFrame(7, 123456789, .5, -.1, .2, protocol.MODE_ACTIVE, 1)
        back = decode(frame.encode())
        self.assertEqual((back.seq, back.t_ns, back.mode, back.flags), (7, 123456789, 1, 1))
        for got, want in ((back.vx, .5), (back.vy, -.1), (back.wz, .2)):
            self.assertAlmostEqual(got, want, places=6)
        self.assertEqual(protocol.FRAME.size, 33)

    def test_corrupt_and_short_frames_are_rejected(self):
        data = bytearray(CommandFrame(1, 2, .1, 0., 0.).encode())
        data[10] ^= 0xFF
        with self.assertRaises(ProtocolError):
            decode(bytes(data))
        with self.assertRaises(ProtocolError):
            decode(b'V8BD')
        bad_magic = bytearray(CommandFrame(1, 2, .1, 0., 0.).encode())
        bad_magic[0] = ord('X')
        with self.assertRaises(ProtocolError):
            decode(bytes(bad_magic))

    def test_non_finite_command_is_rejected(self):
        with self.assertRaises(ProtocolError):
            CommandFrame(1, 2, float('nan'), 0., 0.).encode()


class BridgeTests(unittest.TestCase):
    def test_latch_and_timeout_zero_the_stream(self):
        bridge = CommandBridge(Limits(max_accel=1e9, max_yaw_accel=1e9, command_timeout_s=.25))
        bridge.update(.5, 0., .2, 0.)
        bridge.command(0.)
        self.assertEqual(bridge.command(.1), (.5, 0., .2))
        self.assertEqual(bridge.mode, 'active')
        bridge.command(.3)
        self.assertEqual(bridge.mode, 'hold')
        self.assertEqual(bridge.command(.4), (0., 0., 0.))

    def test_envelope_and_slew(self):
        bridge = CommandBridge(Limits(max_speed=.8, max_wz=1., max_accel=1.5,
                                      max_yaw_accel=3., command_timeout_s=10.))
        bridge.update(5., 0., 5., 0.)
        bridge.command(0.)
        vx, vy, wz = bridge.command(.1)
        self.assertLessEqual(math.hypot(vx, vy), .8 + 1e-9)
        self.assertLessEqual(abs(wz), 1. + 1e-9)
        self.assertLessEqual(abs(vx), 1.5 * .1 + 1e-9)
        self.assertAlmostEqual(vy, 0.)
        vx2, _, _ = bridge.command(.2)
        self.assertAlmostEqual(vx2 - vx, 1.5 * .1, places=6)

    def test_non_finite_update_is_rejected(self):
        bridge = CommandBridge()
        with self.assertRaises(ValueError):
            bridge.update(float('inf'), 0., 0., 0.)


class LinkTests(unittest.TestCase):
    def test_sequence_gap_counts_as_loss(self):
        receiver = CommandReceiver(('127.0.0.1', 0))
        sender = CommandSender(receiver.address)
        try:
            for seq in (0, 1, 3):
                sender.seq = seq
                sender.send(.1, 0., 0., protocol.MODE_ACTIVE)
            frames = [receiver.poll() for _ in range(3)]
            self.assertTrue(all(frame is not None for frame in frames))
            self.assertEqual([f.seq for f in frames], [0, 1, 3])
            self.assertEqual(receiver.stats['loss'], 1)
            self.assertIsNotNone(receiver.age_s())
        finally:
            sender.close()
            receiver.close()

    def test_receiver_rejects_garbage(self):
        receiver = CommandReceiver(('127.0.0.1', 0))
        raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            raw.sendto(b'garbage', receiver.address)
            self.assertIsNone(receiver.poll())
            self.assertEqual(receiver.stats['bad'], 1)
        finally:
            raw.close()
            receiver.close()


if __name__ == '__main__':
    unittest.main()
