"""S4 frame recording and exact replay (raw bytes, no re-encode on replay)."""
import json
import time
from pathlib import Path

from .protocol import decode, decode_telemetry


class Recorder:
    def __init__(self, path):
        self.path = Path(path)
        self.lines = []
        self.rows = 0

    def write(self, kind, data, at_ns=None, **meta):
        row = {'kind': kind, 'at_ns': time.monotonic_ns() if at_ns is None else at_ns,
               'hex': bytes(data).hex()}
        row.update(meta)
        self.lines.append(json.dumps(row, sort_keys=True) + '\n')
        self.rows += 1

    def close(self):
        self.path.write_text(''.join(self.lines), encoding='utf-8')


def load(path):
    rows = []
    with Path(path).open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def replay(rows, kind, send, rate_hz=None, speed=1.0):
    """Send recorded raw frames; preserve recorded gaps when rate_hz is omitted."""
    selected = [row for row in rows if row['kind'] == kind]
    if not selected:
        return 0
    started = time.monotonic()
    base_ns = selected[0]['at_ns']
    for index, row in enumerate(selected):
        if rate_hz:
            target = index / rate_hz
        else:
            target = (row['at_ns'] - base_ns) / 1e9 / speed
        delay = target - (time.monotonic() - started)
        if delay > 0:
            time.sleep(delay)
        send(bytes.fromhex(row['hex']), row)
    return len(selected)


def summarize(rows, telemetry_period_s=.02, command_period_s=.002):
    from .protocol import ProtocolError

    telemetry, commands = [], []
    for row in rows:
        raw = bytes.fromhex(row['hex'])
        try:
            if row['kind'] == 'telemetry':
                telemetry.append((row['at_ns'], decode_telemetry(raw)))
            elif row['kind'] == 'command':
                commands.append((row['at_ns'], decode(raw)))
        except ProtocolError:
            continue
    t_at = [at for at, _ in telemetry]
    c_at = [at for at, _ in commands]
    t_gaps = sorted((b - a) / 1e9 for a, b in zip(t_at, t_at[1:]))
    c_gaps = sorted((b - a) / 1e9 for a, b in zip(c_at, c_at[1:]))
    latency = sorted((at - frame.t_ns) / 1e9 for at, frame in telemetry if at >= frame.t_ns)
    seq_gap = 0
    for (_, a), (_, b) in zip(telemetry, telemetry[1:]):
        gap = (b.seq - a.seq) & 0xFFFFFFFF
        if 1 < gap < 0x80000000:
            seq_gap += gap - 1
    def pct(values, ratio):
        return values[min(len(values) - 1, int(len(values) * ratio))] if values else 0.
    report = {
        'telemetry_frames': len(telemetry),
        'command_frames': len(commands),
        'telemetry_rate_hz': round(len(telemetry) / ((t_at[-1] - t_at[0]) / 1e9), 2)
        if len(t_at) > 1 else 0.,
        'command_rate_hz': round(len(commands) / ((c_at[-1] - c_at[0]) / 1e9), 2)
        if len(c_at) > 1 else 0.,
        'telemetry_interval_p50_s': round(pct(t_gaps, .5), 6),
        'telemetry_interval_p99_s': round(pct(t_gaps, .99), 6),
        'command_interval_p50_s': round(pct(c_gaps, .5), 6),
        'command_interval_p99_s': round(pct(c_gaps, .99), 6),
        'telemetry_loss': seq_gap,
        'telemetry_latency_p50_s': round(pct(latency, .5), 6),
        'telemetry_latency_p99_s': round(pct(latency, .99), 6),
        'latency_scope': 'endpoint t_ns -> application recv timestamp; includes receiver scheduling',
        'telemetry_fault_frames': sum(1 for _, frame in telemetry if frame.state != 1),
        'provisional_targets': {
            'rate_error_pct_le': 2.0,
            'interval_p99_le_period_x': 1.5,
            'telemetry_loss': 0,
            'local_transport_p99_le_s': .001},
        'rate_error_pct': round(abs(len(telemetry) / ((t_at[-1] - t_at[0]) / 1e9)
                                  * telemetry_period_s - 1) * 100, 3)
        if len(t_at) > 1 else None,
        'interval_p99_ratio': round(pct(t_gaps, .99) / telemetry_period_s, 3)
        if len(t_gaps) else None,
        'command_interval_p99_ratio': round(pct(c_gaps, .99) / command_period_s, 3)
        if len(c_gaps) else None}
    return report


def main(argv=None):
    import argparse
    import json
    import socket

    parser = argparse.ArgumentParser(prog='recording.py')
    sub = parser.add_subparsers(dest='command', required=True)
    report = sub.add_parser('summarize')
    report.add_argument('--file', required=True)
    report.add_argument('--telemetry-period', type=float, default=.02)
    report.add_argument('--command-period', type=float, default=.002)
    report.add_argument('--out')
    play = sub.add_parser('replay')
    play.add_argument('--file', required=True)
    play.add_argument('--kind', choices=('command', 'telemetry'), required=True)
    play.add_argument('--target', required=True)
    play.add_argument('--rate-hz', type=float)
    play.add_argument('--speed', type=float, default=1.)
    args = parser.parse_args(argv)
    rows = load(args.file)
    if args.command == 'summarize':
        value = summarize(rows, args.telemetry_period, args.command_period)
        text = json.dumps(value, indent=1)
        print(text)
        if args.out:
            Path(args.out).write_text(text + '\n', encoding='utf-8')
        return 0
    host, port = args.target.rsplit(':', 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sent = replay(rows, args.kind,
                      lambda data, _row: sock.sendto(data, (host, int(port))),
                      rate_hz=args.rate_hz, speed=args.speed)
    finally:
        sock.close()
    print(json.dumps({'replayed': sent, 'kind': args.kind, 'target': args.target}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
