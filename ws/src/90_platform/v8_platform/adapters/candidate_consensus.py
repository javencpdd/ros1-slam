"""A6 candidate gate: require agreement across BBS resolutions.

A single BBS query can land in the wrong basin (R1075).  Candidates from
multiple resolutions within `radius` form a cluster; a cluster of at least
`min_support` members is accepted and the finest-resolution member is returned.
Isolated candidates and tied clusters are rejected.
"""
import math


class CandidateConsensus:
    def __init__(self, radius=0.6, min_support=2):
        if radius <= 0 or min_support < 2:
            raise ValueError('radius > 0 and min_support >= 2 required')
        self.radius = radius
        self.min_support = min_support

    @staticmethod
    def _distance(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def select(self, samples):
        """samples: iterable of {'source': float, 'pose': (x,y,...)}.

        A sample joins a cluster only if it is within radius of every member
        (complete link), so a chain of near points cannot bridge two basins.
        """
        clusters = []
        for sample in samples:
            pose = tuple(sample['pose'][:2])
            for cluster in clusters:
                if all(self._distance(pose, other['pose'][:2]) <= self.radius
                       for other in cluster):
                    cluster.append(dict(sample, pose=pose))
                    break
            else:
                clusters.append([dict(sample, pose=pose)])
        if not clusters:
            return None
        best = max(len(c) for c in clusters)
        winners = [c for c in clusters if len(c) == best]
        if best < self.min_support or len(winners) != 1:
            return None
        members = winners[0]
        pick = min(members, key=lambda item: item['source'])
        return {'pose': list(pick['pose']), 'source': pick['source'],
                'support': len(members),
                'members': [{'source': item['source'], 'pose': list(item['pose'])}
                            for item in members]}
