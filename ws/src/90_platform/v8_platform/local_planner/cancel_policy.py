"""Wire helpers for the local planner cancel contract."""
from __future__ import annotations


def parse_cancel(text):
    """Parse "<seq>:<scope>:<reason>"; return (seq, scope, reason) or None."""
    if not isinstance(text, str):
        return None
    parts = text.split(':', 2)
    if len(parts) < 2:
        return None
    seq, scope = parts[0], parts[1]
    reason = parts[2] if len(parts) == 3 else ''
    if not seq.isdigit() or scope not in ('current', 'all'):
        return None
    if len(reason) > 200 or ';' in reason or '\n' in reason:
        return None
    return int(seq), scope, reason


def ack_text(seq, outcome):
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        raise ValueError('seq')
    if outcome not in ('cancelled', 'ack_fail', 'unacked'):
        raise ValueError('outcome')
    return '%d:%s' % (seq, outcome)
