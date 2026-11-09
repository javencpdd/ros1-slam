"""Shadow localization-initialization readiness node for the vendor 8.2.1 gap.

The official Lite3 procedure has a third step: after navigation starts, the
robot localization must be initialized (2D Pose Estimate / /initialpose).  The
ROS1 chain in this repository has hdl_localization.launch:specify_init_pose=true
but no startup gate that requires an explicit initial pose, map readiness, an
odometry chain and localization convergence before a goal is accepted.

This node is the SHADOW counterpart.  It is deliberately inert:

* It is NOT included by any launch file.  Starting it is an explicit operator
  decision and, on a real robot, requires an L2 authorization.
* Default readiness_mode is ``off``; ``shadow`` only reports; ``assist`` emits
  a veto *recommendation* on a diagnostic topic and never touches /cmd_vel,
  the safety mux, the hold topic, ros2qnx or any chassis command.
* It publishes exactly one topic: /v8/nav/init_readiness (std_msgs/String,
  JSON).  No parameter is written, no judgement threshold is changed.
* The convergence signal has NO registered producer in this repository, so it
  stays None by design: the gate reports insufficient_data/not_ready rather
  than a false green.  Wiring a producer is a separate, L2-gated decision.

Pure-stdlib core is in vendor_nav_chain.py; this file is only the ROS bridge.
"""

from __future__ import annotations

import json

from vendor_nav_chain import (
    REQUIRED_SIGNALS,
    initial_pose_gate,
    no_backfill_report,
    resolve_mode,
)

DIAG_TOPIC = "/v8/nav/init_readiness"
INIT_POSE_TOPIC = "/initialpose"
MAP_TOPIC = "/map"
ODOM_TOPIC = "/hdl_odom"
CONVERGED_TOPIC = "/v8/nav/localization_converged"

DEFAULT_PARAMS = {
    "readiness_mode": "off",
    "init_pose_topic": INIT_POSE_TOPIC,
    "map_topic": MAP_TOPIC,
    "odom_topic": ODOM_TOPIC,
    "converged_topic": CONVERGED_TOPIC,
    "diag_topic": DIAG_TOPIC,
    "publish_rate_hz": 1.0,
}


class NavInitReadinessNode:
    """ROS-agnostic core; ``ros`` is injected so tests need no roscore."""

    def __init__(self, ros=None, params=None):
        self.ros = ros
        self.params = dict(DEFAULT_PARAMS)
        if params:
            self.params.update(params)
        self.mode = resolve_mode(self.params.get("readiness_mode", "off"))
        self.signals = {k: None for k in REQUIRED_SIGNALS}
        self.status_pub = None
        self.subscribers = []
        self.last_result = None
        self.topic_writes = []  # audit: proves no motion topic is ever written

    # -- input callbacks: each sets exactly one signal -----------------------

    def on_initial_pose(self, _msg):
        self.signals["initial_pose_received"] = True

    def on_map(self, _msg):
        self.signals["map_ready"] = True

    def on_odom(self, _msg):
        self.signals["odom_chain_ready"] = True

    def on_converged(self, msg):
        # std_msgs/Bool-like. A missing/None data field stays unknown.
        value = getattr(msg, "data", None)
        if value is None:
            return
        self.signals["localization_converged"] = bool(value)

    # -- evaluation ---------------------------------------------------------

    def evaluate(self):
        result = initial_pose_gate(self.signals, self.mode)
        result["hardware"] = False
        result["live_topic_writes"] = 0
        result["motion_topic_writes"] = 0
        result["gate_thresholds_modified"] = False
        result["produces_pass_fail"] = False
        result["veto_is_recommendation_only"] = True
        result["converged_producer_registered"] = False
        self.last_result = result
        return result

    def publish(self, result):
        if self.status_pub is None or self.mode == "off":
            return False
        payload = json.dumps(result, ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(payload)
        self.topic_writes.append(self.params["diag_topic"])
        return True

    # -- ROS wiring (only runs when explicitly started) ---------------------

    def start(self):
        """Subscribe/publish. Never called by any launch in this repository."""
        # off must be fully inert: it returns before any ROS runtime is needed.
        if self.mode == "off":
            return
        if self.ros is None:
            raise RuntimeError("ROS runtime not injected")
        r = self.ros
        self.status_pub = r.Publisher(self.params["diag_topic"], r.String,
                                      queue_size=1, latch=True)
        specs = (
            (self.params["init_pose_topic"], r.PoseWithCovarianceStamped,
             self.on_initial_pose),
            (self.params["map_topic"], r.OccupancyGrid, self.on_map),
            (self.params["odom_topic"], r.Odometry, self.on_odom),
            (self.params["converged_topic"], r.Bool, self.on_converged),
        )
        for topic, msg_type, cb in specs:
            self.subscribers.append(r.Subscriber(topic, msg_type, cb))
        r.Timer(r.Duration(1.0 / float(self.params["publish_rate_hz"])),
                self._on_timer)

    def _on_timer(self, _event):
        self.publish(self.evaluate())

    def subscribed_topics(self):
        return [
            self.params["init_pose_topic"], self.params["map_topic"],
            self.params["odom_topic"], self.params["converged_topic"],
        ]

    def written_topics(self):
        return [self.params["diag_topic"]]


def projected_settings_check(project_settings):
    """Convenience wrapper: refuse to accept any backfilled vendor value."""
    return no_backfill_report(project_settings)


def main():  # pragma: no cover - requires a live ROS master
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from std_msgs.msg import Bool, String

    rospy.init_node("nav_init_readiness")
    ros = type("RosShim", (), {
        "Publisher": staticmethod(rospy.Publisher),
        "Subscriber": staticmethod(rospy.Subscriber),
        "Timer": staticmethod(rospy.Timer),
        "Duration": staticmethod(rospy.Duration),
        "String": String,
        "Bool": Bool,
        "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
        "OccupancyGrid": OccupancyGrid,
        "Odometry": Odometry,
    })()
    node = NavInitReadinessNode(ros=ros, params={
        "readiness_mode": rospy.get_param("~readiness_mode", "off"),
    })
    node.start()
    if node.mode != "off":
        rospy.loginfo("nav_init_readiness: shadow mode=%s, topics=%s",
                      node.mode, node.written_topics())
    rospy.spin()


if __name__ == "__main__":
    main()
