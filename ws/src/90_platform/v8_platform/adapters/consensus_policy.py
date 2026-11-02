"""Pure agreement policy for the A6 absolute-localization consensus gate."""
import math


def distance(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def decide(candidate, consensus, radius):
    if not candidate or not consensus:
        return None, 'missing'
    try:
        gap = distance(candidate['pose'], consensus['pose'])
    except (KeyError, TypeError, ValueError, IndexError):
        return None, 'malformed'
    if gap > radius:
        return None, 'disagree'
    output = {'pose': list(consensus['pose']),
              'yaw': consensus.get('yaw', candidate.get('yaw', 0.0)),
              'score': consensus.get('score', candidate.get('score', 1.0)),
              'agreement_m': gap,
              'source': consensus.get('source', 'consensus')}
    return output, 'agreed'
