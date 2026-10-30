"""Boundary contract tests: frame codec, rate bridge, sequence health."""
import math
import socket
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from boundary import protocol
from boundary.rate_bridge import CommandBridge, Limits
from boundary.protocol import (CommandFrame, ProtocolError, TelemetryFrame,
                               decode, decode_telemetry)
from boundary.udp_link import (CommandReceiver, CommandSender, TelemetryReceiver,
                               TelemetrySender)
from boundary.recording import Recorder, load, replay, summarize
from boundary.ros_bridge import NodeCore, policy_command


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

    def test_telemetry_round_trip_and_rejections(self):
        frame = TelemetryFrame(3, 99, 1., 2., .4, .1, .2, .3, 1., 0., 0., 0.,
                               .01, .02, .03, 0., 0., 9.8,
                               joint_pos=tuple(float(i) for i in range(12)),
                               joint_vel=tuple(float(i) for i in range(12)))
        back = decode_telemetry(frame.encode())
        self.assertEqual((back.seq, back.t_ns, back.state, back.fault), (3, 99, 1, 0))
        for got, want in ((back.x, 1.), (back.vx, .1), (back.gz, .03), (back.joint_pos[7], 7.)):
            self.assertAlmostEqual(got, want, places=5)
        bad = bytearray(frame.encode())
        bad[-1] ^= 0xFF
        with self.assertRaises(ProtocolError):
            decode_telemetry(bytes(bad))
        with self.assertRaises(ProtocolError):
            TelemetryFrame(1, 2, 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
                           0., 0., 0., 0., 0., float('nan')).encode()
        with self.assertRaises(ProtocolError):
            TelemetryFrame(1, 2, 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
                           0., 0., 0., 0., 0., 0., joint_pos=(0.,)).encode()


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

    def test_telemetry_link_counts_loss(self):
        receiver = TelemetryReceiver(('127.0.0.1', 0))
        sender = TelemetrySender(receiver.address)
        try:
            for seq in (0, 1, 4):
                sender.seq = seq
                sender.send(TelemetryFrame(seq, 1, 0., 0., 0., 0., 0., 0.,
                                           1., 0., 0., 0., 0., 0., 0., 0., 0., 9.8))
            frames = [receiver.poll() for _ in range(3)]
            self.assertEqual([f.seq for f in frames], [0, 1, 4])
            self.assertEqual(receiver.stats['loss'], 2)
        finally:
            sender.close()
            receiver.close()

    def test_core_holds_without_telemetry_and_faults_when_stale(self):
        class Clock:
            now = 0.

            def __call__(self):
                return self.now

        clock = Clock()
        core = NodeCore(link_timeout_s=.5, clock=clock)
        (vx, vy, wz), mode, flags = core.tick()
        self.assertEqual((vx, vy, wz, mode), (0., 0., 0., protocol.MODE_HOLD))
        self.assertEqual(core.fault, 'telemetry_missing')
        core.on_telemetry(TelemetryFrame(1, 1, 0., 0., 0., 0., 0., 0.,
                                         1., 0., 0., 0., 0., 0., 0., 0., 0., 9.8))
        core.on_command(.3, 0., .1)
        self.assertEqual(core.tick()[1], protocol.MODE_ACTIVE)
        clock.now = 1.
        self.assertEqual(core.tick()[1], protocol.MODE_HOLD)
        self.assertEqual(core.fault, 'telemetry_stale')

    def test_core_holds_on_robot_fault_and_planner_silence(self):
        class Clock:
            now = 0.

            def __call__(self):
                return self.now

        clock = Clock()
        core = NodeCore(clock=clock, limits=Limits(command_timeout_s=.25))
        core.on_telemetry(TelemetryFrame(1, 1, 0., 0., 0., 0., 0., 0.,
                                         1., 0., 0., 0., 0., 0., 0., 0., 0., 9.8,
                                         state=protocol.TELEM_FAULT, fault=1))
        self.assertEqual(core.tick()[1], protocol.MODE_HOLD)
        self.assertEqual(core.fault, 'robot_fault')
        core.on_telemetry(TelemetryFrame(2, 2, 0., 0., 0., 0., 0., 0.,
                                         1., 0., 0., 0., 0., 0., 0., 0., 0., 9.8))
        core.on_command(.3, 0., 0.)
        self.assertEqual(core.tick()[1], protocol.MODE_ACTIVE)
        clock.now = .4
        self.assertEqual(core.tick()[1], protocol.MODE_HOLD)
        self.assertIsNone(core.fault)

    def test_policy_command_maps_lateral_velocity_to_heading(self):
        self.assertEqual(policy_command(0., 0., 0.), (0., 0.))
        speed, wz = policy_command(.2, .2, 0.)
        self.assertAlmostEqual(speed, math.hypot(.2, .2))
        self.assertEqual(wz, .5)
        speed, wz = policy_command(1., .1, 0.)
        self.assertAlmostEqual(wz, 3. * math.atan(.1), places=6)
        self.assertEqual(policy_command(0., 0., .9)[1], .5)

    def test_record_summarize_and_exact_replay(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'frames.jsonl'
            recorder = Recorder(path)
            base = 1_000_000_000
            for seq in range(3):
                stamp = base + seq * 20_000_000
                frame = TelemetryFrame(seq, stamp - 200_000, 0., 0., .3, 0., 0., 0.,
                                       1., 0., 0., 0., 0., 0., 0., 0., 0., 9.8)
                recorder.write('telemetry', frame.encode(), at_ns=stamp)
                command = CommandFrame(seq, stamp, .1, 0., 0.)
                recorder.write('command', command.encode(), at_ns=stamp)
            recorder.close()
            rows = load(path)
            self.assertEqual(len(rows), 6)
            report = summarize(rows)
            self.assertEqual(report['telemetry_frames'], 3)
            self.assertEqual(report['command_frames'], 3)
            self.assertEqual(report['telemetry_loss'], 0)
            self.assertAlmostEqual(report['telemetry_interval_p50_s'], .02, places=4)
            self.assertAlmostEqual(report['telemetry_latency_p99_s'], .0002, places=6)
            sent = []
            count = replay(rows, 'command', lambda data, _row: sent.append(data), rate_hz=10000.)
            self.assertEqual(count, 3)
            self.assertEqual([decode(data).seq for data in sent], [0, 1, 2])

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
