#!/usr/bin/env python3
"""Publish a simulator truth channel from Gazebo model states.

This is a verification-side oracle, not part of any algorithm graph: it reads
/gazebo/model_states and republishes one model pose on the configured truth
topic so the evaluator can compare against algorithm output.
"""
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


def main():
    rospy.init_node('v8_gazebo_truth')
    truth_topic = rospy.get_param('~truth_topic', '/v8/verify/truth')
    model_name = rospy.get_param('~model_name', '')
    rate_hz = float(rospy.get_param('~rate_hz', 10.0))
    timeout_s = float(rospy.get_param('~model_timeout_s', 60.0))
    states = rospy.wait_for_message('/gazebo/model_states', ModelStates,
                                    timeout=timeout_s)
    names = list(states.name)
    index = names.index(model_name) if model_name in names else 0
    model = names[index]
    latest = {'states': states}

    def on_states(message):
        latest['states'] = message

    publisher = rospy.Publisher(truth_topic, Odometry, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, on_states, queue_size=1)
    rate = rospy.Rate(rate_hz)
    sequence = 0
    while not rospy.is_shutdown():
        states = latest['states']
        if model not in states.name:
            rate.sleep()
            continue
        index = list(states.name).index(model)
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'world'
        message.child_frame_id = model
        message.header.seq = sequence
        message.pose.pose = states.pose[index]
        message.twist.twist = states.twist[index]
        publisher.publish(message)
        sequence += 1
        rate.sleep()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
