#!/usr/bin/env python3
"""Publish a MuJoCo truth channel for a minimal in-repo drop world.

The node owns the physics model, advances it in real time and republishes the
probe pose as nav_msgs/Odometry. It is a verification backend only: no policy,
no ROS command input, no claim about quadruped contact fidelity.
"""
import rospy
from nav_msgs.msg import Odometry
import mujoco


def main():
    rospy.init_node('v8_mujoco_truth')
    xml = rospy.get_param('~xml')
    truth_topic = rospy.get_param('~truth_topic', '/v8/verify/truth_mj')
    model_name = rospy.get_param('~model_name', 'probe')
    rate_hz = float(rospy.get_param('~rate_hz', 10.0))
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, model_name)
    if body_id < 0:
        raise ValueError('unknown MuJoCo body: %s' % model_name)
    joint_id = int(model.body_jntadr[body_id])
    dof = int(model.jnt_dofadr[joint_id]) if joint_id >= 0 else -1
    publisher = rospy.Publisher(truth_topic, Odometry, queue_size=10)
    rate = rospy.Rate(rate_hz)
    steps = max(1, int(round((1.0 / rate_hz) / model.opt.timestep)))
    sequence = 0
    while not rospy.is_shutdown():
        for _ in range(steps):
            mujoco.mj_step(model, data)
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'world'
        message.child_frame_id = model_name
        message.header.seq = sequence
        message.pose.pose.position.x = float(data.xpos[body_id][0])
        message.pose.pose.position.y = float(data.xpos[body_id][1])
        message.pose.pose.position.z = float(data.xpos[body_id][2])
        message.pose.pose.orientation.w = float(data.xquat[body_id][0])
        message.pose.pose.orientation.x = float(data.xquat[body_id][1])
        message.pose.pose.orientation.y = float(data.xquat[body_id][2])
        message.pose.pose.orientation.z = float(data.xquat[body_id][3])
        if dof >= 0:
            message.twist.twist.linear.x = float(data.qvel[dof])
            message.twist.twist.linear.y = float(data.qvel[dof + 1])
            message.twist.twist.linear.z = float(data.qvel[dof + 2])
            message.twist.twist.angular.x = float(data.qvel[dof + 3])
            message.twist.twist.angular.y = float(data.qvel[dof + 4])
            message.twist.twist.angular.z = float(data.qvel[dof + 5])
        publisher.publish(message)
        sequence += 1
        rate.sleep()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
