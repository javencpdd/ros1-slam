#!/usr/bin/env python3
"""Gate an A6 candidate with an independent absolute-localization candidate.

The gate forwards only when the two poses agree within `radius`; the output
uses the independent pose (the A6 candidate is the trigger, not the authority).
A missing or stale independent candidate fails closed.
"""
import argparse
import json
import time

import rospy
from std_msgs.msg import String

from consensus_policy import decide, distance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-topic', default='/a6/recovery/candidate_raw')
    parser.add_argument('--consensus-topic', default='/a6/consensus/candidate')
    parser.add_argument('--output-topic', default='/a6/recovery/candidate')
    parser.add_argument('--radius', type=float, default=.5)
    parser.add_argument('--max-age-s', type=float, default=5.0)
    args = parser.parse_args(argv)
    if args.radius <= 0 or args.max_age_s <= 0:
        raise SystemExit('radius and max-age must be positive')

    rospy.init_node('consensus_gate', anonymous=True)
    consensus = {'payload': None, 'at': 0.0}
    pending = {'payload': None, 'at': 0.0, 'done': False}
    output = rospy.Publisher(args.output_topic, String, queue_size=5)

    def parse(text):
        try:
            payload = json.loads(text)
        except (TypeError, ValueError):
            return None
        if not isinstance(payload, dict) or 'pose' not in payload:
            return None
        return payload

    def try_forward():
        if pending['payload'] is None or pending['done']:
            return
        now = time.monotonic()
        if now - pending['at'] > args.max_age_s:
            pending['done'] = True
            rospy.logwarn('consensus gate dropped stale candidate')
            return
        if consensus['payload'] is None or now - consensus['at'] > args.max_age_s:
            return
        result, reason = decide(pending['payload'], consensus['payload'], args.radius)
        pending['done'] = True
        if result is None:
            gap = None
            try:
                gap = distance(pending['payload']['pose'], consensus['payload']['pose'])
            except (KeyError, TypeError, ValueError, IndexError):
                pass
            rospy.logwarn('consensus gate dropped candidate: %s gap=%s', reason, gap)
            return
        output.publish(String(data=json.dumps(result)))
        rospy.loginfo('consensus gate accepted gap=%.3f -> %s', result['agreement_m'],
                      result['pose'])

    def on_consensus(message):
        payload = parse(message.data)
        if payload is not None:
            consensus['payload'] = payload
            consensus['at'] = time.monotonic()
            try_forward()

    def on_candidate(message):
        candidate = parse(message.data)
        if candidate is None:
            rospy.logwarn('consensus gate dropped malformed candidate')
            return
        pending.update(payload=candidate, at=time.monotonic(), done=False)
        try_forward()

    rospy.Subscriber(args.input_topic, String, on_candidate, queue_size=5)
    rospy.Subscriber(args.consensus_topic, String, on_consensus, queue_size=5)
    rospy.loginfo('consensus gate %s + %s -> %s radius=%.2f',
                  args.input_topic, args.consensus_topic, args.output_topic, args.radius)
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
