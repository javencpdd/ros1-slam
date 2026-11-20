#!/usr/bin/env python3
"""Isolated ROS1 integration driver; parent owns launch, preflight and teardown.

Optional lease-loss injection pauses only this isolated stack's BT process.
The 0.35 m check is only the existing scoped ground-truth comparison, not a
new navigation acceptance threshold. All deadlines use monotonic wall time.
"""
import argparse
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from urllib.parse import urlsplit

BACKEND = "/navigation_executor/move_base"
NAV = "/v8/navigation/"
LIVE = (0, 1, 6, 7)  # PENDING, ACTIVE, PREEMPTING, RECALLING; not retained terminals.


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def twist_values(twist):
    return [getattr(vector, axis) for vector in (twist.linear, twist.angular)
            for axis in ("x", "y", "z")]


class Driver:
    def __init__(self, evidence, local_planner="teb", global_planner="global_planner"):
        self.local_planner = local_planner
        self.global_planner = global_planner
        evidence.update(local_planner=local_planner, global_planner=global_planner)
        import rosgraph
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String

        self.ros, self.Pose, self.String = rospy, PoseStamped, String
        self.cv, self.error, self.data = threading.Condition(), "", {}
        self.evidence = evidence
        evidence.update(cases=[], streams=self.data, graphs=[])
        rospy.init_node("bt_navigation_runtime_test", argv=[sys.argv[0]], anonymous=True,
                        disable_signals=True)
        require(rospy.get_param("/use_sim_time", False) is True, "simulation clock required")
        self.master = rosgraph.Master(rospy.get_name())
        self.goal = rospy.Publisher("/v8/bt/goal", PoseStamped, queue_size=1)
        self.cancel = rospy.Publisher("/v8/bt/cancel", String, queue_size=1)
        topics = {"bt": ("/v8/bt/status", String), "adapter": (NAV + "status", String),
                  "result": (NAV + "result", String), "ack": (NAV + "cancel_ack", String),
                  "cancel": (NAV + "cancel", String), "gt": ("/sim/odom_gt", Odometry),
                  "backend": (BACKEND + "/status", GoalStatusArray),
                  "cmd": ("/cmd_vel", Twist), "raw": ("/cmd_vel_raw", Twist),
                  "candidate": (NAV + "cmd_vel_raw_candidate", Twist),
                  "cloud": ("/rslidar_points", PointCloud2)}
        topics.update({k: ("/navigation_executor/" + k + "_costmap/costmap", OccupancyGrid)
                       for k in ("global", "local")})
        self.subs = [rospy.Subscriber(topic, kind, self.receive, callback_args=key,
                                     queue_size=200) for key, (topic, kind) in topics.items()]

    def receive(self, msg, key):
        with self.cv:
            try:
                if key in ("bt", "adapter"):
                    value = json.loads(msg.data)
                    fields = ("current_seq", "outcome_seq", "queue_depth") if key == "bt" else ("seq",)
                    require(all(type(value[k]) is int and value[k] >= 0 for k in fields),
                            "malformed identity/queue on " + key)
                elif key in ("cmd", "raw", "candidate"):
                    value = twist_values(msg)
                elif key == "gt":
                    p, q = msg.pose.pose.position, msg.pose.pose.orientation
                    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w] + twist_values(msg.twist.twist)
                    require(all(math.isfinite(v) for v in values), "nonfinite ground truth")
                    require(abs(sum(v * v for v in values[3:7]) - 1) < 0.01, "invalid GT quaternion")
                    value = dict(x=p.x, y=p.y, z=p.z, stamp=msg.header.stamp.to_sec(),
                                 yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                                 speed=max(abs(v) for v in values[7:]))
                elif key == "backend":
                    value = {s.goal_id.id: s.status for s in msg.status_list}
                elif key in ("global", "local"):
                    value = dict(width=msg.info.width, height=msg.info.height,
                                 resolution=msg.info.resolution, cells=len(msg.data),
                                 known=sum(v >= 0 for v in msg.data), occupied=sum(v > 0 for v in msg.data))
                elif key == "cloud":
                    value = dict(points=msg.width*msg.height, bytes=len(msg.data))
                else:
                    value = msg.data
                json.dumps(value, allow_nan=False)  # Reject NaN/Inf, including JSON numeric overflow.
                self.data.setdefault(key, []).append((time.monotonic(), value))
            except Exception as exc:
                self.error = self.error or "{}: {}".format(key, exc)
            self.cv.notify_all()

    def rows(self, key, since=0):
        with self.cv:
            return [(t, v) for t, v in self.data.get(key, []) if t >= since]

    def last(self, key):
        rows = self.rows(key)
        return rows[-1][1] if rows else None

    def fresh(self, key, age=1.5):
        rows = self.rows(key)
        return bool(rows and time.monotonic() - rows[-1][0] <= age)

    def wait(self, label, predicate, timeout=15):
        end = time.monotonic() + timeout
        with self.cv:
            while True:
                require(not self.error, self.error)
                require(not self.ros.is_shutdown(), "ROS shutdown during " + label)
                result = predicate()
                if result:
                    return result
                require(time.monotonic() < end, "wall timeout: " + label)
                self.cv.wait(min(0.1, max(0, end - time.monotonic())))

    def graph(self):
        pubs, subs, services = self.master.getSystemState()
        text = json.dumps([pubs, subs, services]).lower()
        require(not any(s in text for s in ("move_base_flex", "mbf", "v8_local_planner")),
                "forbidden MBF/custom planner in ROS graph")
        pubs, subs = dict(pubs), dict(subs)
        expected = {"/cmd_vel": "/safe_cmd_vel_mux", "/cmd_vel_raw": "/bt_navigation_executor",
                    NAV + "cmd_vel_raw_candidate": "/navigation_executor",
                    BACKEND + "/goal": "/bt_navigation_executor",
                    BACKEND + "/cancel": "/bt_navigation_executor",
                    BACKEND + "/status": "/navigation_executor",
                    BACKEND + "/result": "/navigation_executor", "/v8/bt/status": "/bt_console"}
        for topic, node in expected.items():
            allowed = {node, "/navigation_executor"} if topic == BACKEND + "/goal" else {node}
            require(not pubs.get(topic) or set(pubs[topic]) <= allowed,
                    "unexpected publisher: " + topic + " " + str(pubs.get(topic)))
        require(not pubs.get("/navigation_executor/disabled_simple_goal"),
                "disabled backend simple-goal input has a publisher")
        links = {BACKEND + "/goal": "/navigation_executor",
                 BACKEND + "/result": "/bt_navigation_executor",
                 NAV + "cmd_vel_raw_candidate": "/bt_navigation_executor",
                 "/cmd_vel_raw": "/safe_cmd_vel_mux"}
        ready = all(node in pubs.get(t, []) for t, node in expected.items()) and all(n in subs.get(t, []) for t, n in links.items())
        if ready:
            self.evidence["graphs"].append(dict(at=time.monotonic(), publishers=pubs, subscribers=subs))
        return ready

    def ready(self):
        self.wait("backend graph connections", self.graph, 60)
        planners = {
            "global_planner": "global_planner/GlobalPlanner",
            "navfn": "navfn/NavfnROS",
        }
        local = {
            "teb": "teb_local_planner/TebLocalPlannerROS",
            "dwa": "dwa_local_planner/DWAPlannerROS",
            "trajectory": "base_local_planner/TrajectoryPlannerROS",
        }
        require(self.local_planner in local and self.global_planner in planners,
                "unknown planner candidate")
        require(self.ros.get_param("/navigation_executor/base_global_planner")
                == planners[self.global_planner], "wrong base_global_planner")
        require(self.ros.get_param("/navigation_executor/base_local_planner")
                == local[self.local_planner], "wrong base_local_planner")
        def data_ready():
            if not (self.goal.get_num_connections() and self.cancel.get_num_connections()
                    and all(s.get_num_connections() for s in self.subs)):
                return False
            if not all(self.fresh(k, 5) for k in ("global", "local", "cloud", "gt", "bt", "backend", "cmd", "raw")):
                return False
            for key in ("global", "local"):
                m = self.last(key)
                if not (m["width"] > 0 and m["height"] > 0 and m["resolution"] > 0
                        and m["cells"] == m["width"] * m["height"] and m["known"] > 0):
                    return False
            return (self.last("cloud")["points"] > 0 and self.last("cloud")["bytes"] > 0
                    and self.last("gt")["stamp"] > self.rows("gt")[0][1]["stamp"])
        self.wait("nonempty costmaps, sensor data and BT", data_ready, 60)
        require(self.last("bt")["queue_depth"] == 0, "pre-existing BT queue")
        require(not any(s in LIVE for s in self.last("backend").values()), "pre-existing backend goal")
        self.quiet("initial idle", 1)

    def send(self, name, x, y, z=0):
        previous = self.last("bt")["current_seq"]
        case = dict(name=name, target=[x, y, z], previous_seq=previous,
                    previous_backend_ids=list(self.last("backend")), sent=time.monotonic())
        self.evidence["cases"].append(case)
        goal = self.Pose()
        goal.header.frame_id, goal.header.stamp = "map", self.ros.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, z
        goal.pose.orientation.w = 1.0  # Header.seq is transport-owned, NEVER our goal identity.
        self.goal.publish(goal)
        acquired = self.wait(name + " BT identity", lambda: next((v for _, v in self.rows("bt", case["sent"])
                             if v["current_seq"] > previous), None))
        case.update(seq=acquired["current_seq"], bt_acquired=acquired)
        return case

    def complete(self, case, outcome, state, reason=None):
        seq, since = case["seq"], case["sent"]
        expected = "{}:ok".format(seq) if outcome == "reached" else "{}:fail:{}".format(seq, reason)
        def matched():
            bt = [v for _, v in self.rows("bt", since) if v["current_seq"] >= seq]
            require(all(v["current_seq"] == seq for v in bt), "unexpected BT retry/foreign goal")
            results = [v for _, v in self.rows("result", since) if v.partition(":")[0] == str(seq)]
            require(all(v == expected for v in results), "wrong scoped adapter result: " + str(results))
            terminal = [v for v in bt if v["outcome_seq"] == seq and v["outcome"]]
            require(all(v["outcome"] == outcome for v in terminal), "wrong scoped BT outcome")
            adapter = [v for _, v in self.rows("adapter", since) if v["seq"] == seq and v["state"] == state
                       and (reason is None or v["reason"] == reason)]
            if not (results and terminal and adapter):
                return False
            case.update(bt_terminal=terminal[-1], adapter_results=results, adapter_terminal=adapter[-1])
            require(terminal[-1]["queue_depth"] == 0, "unexpected queued goals")
            if outcome == "reached":
                require(adapter[-1]["backend_state"] == 3, "backend did not succeed")
            return True
        self.wait(case["name"] + " scoped outcome", matched, 90 if outcome == "reached" else 20)

    def quiet(self, label, duration, since=None):
        if since is None:
            self.wait(label + " settle", lambda: all(self.fresh(k) for k in ("cmd", "raw", "gt", "backend"))
                      and max(map(abs, self.last("cmd") + self.last("raw"))) <= 1e-4
                      and self.last("gt")["speed"] <= 0.02
                      and not any(s in LIVE for s in self.last("backend").values()), 10)
            since = time.monotonic()
        def observed():
            samples = {k: self.rows(k, since) for k in ("cmd", "raw", "gt", "backend")}
            for key in ("cmd", "raw"):
                require(all(max(map(abs, v)) <= 1e-4 for _, v in samples[key]), label + ": motion command resumed")
            gt = samples["gt"]
            if gt:
                origin = gt[0][1]
                require(all(math.hypot(v["x"]-origin["x"], v["y"]-origin["y"]) <= 0.03
                            and abs(math.atan2(math.sin(v["yaw"]-origin["yaw"]), math.cos(v["yaw"]-origin["yaw"]))) <= 0.03
                            and v["speed"] <= 0.02 for _, v in gt), label + ": ground truth moved")
            require(all(not any(s in LIVE for s in v.values()) for _, v in samples["backend"]), label + ": backend resumed")
            for key, rows in samples.items():
                require(all(b[0]-a[0] <= 1.5 for a, b in zip(rows, rows[1:])), label + ": sample gap " + key)
                if len(rows) < 3 or rows[-1][0]-rows[0][0] < duration or not self.fresh(key):
                    return False
            require(gt[-1][1]["stamp"] > gt[0][1]["stamp"], "frozen simulation during " + label)
            self.evidence.setdefault("stationary", []).append(dict(label=label, since=since,
                until=time.monotonic(), duration=duration, counts={k: len(v) for k, v in samples.items()}))
            return True
        self.wait(label + " observation", observed, duration + 6)

    def success(self, case):
        self.complete(case, "reached", "succeeded")
        require(self.fresh("gt"), "stale ground truth at success")
        gt = self.last("gt")
        case["ground_truth"] = gt
        case["scoped_distance_m"] = math.hypot(gt["x"]-case["target"][0], gt["y"]-case["target"][1])
        require(case["scoped_distance_m"] <= 0.35, "existing scoped distance comparison exceeded")
        self.quiet(case["name"] + " stopped", 1)

    def lease_loss(self):
        import xmlrpc.client
        from pathlib import Path
        gt = self.last("gt")
        case = self.send("lease_loss", gt["x"] + 2.0, gt["y"])
        self.wait("lease test active motion", lambda: self.fresh("raw")
                  and max(map(abs, self.last("raw"))) > 0.01
                  and any(v["seq"] == case["seq"] and v["state"] == "active"
                          for _, v in self.rows("adapter", case["sent"])), 30)
        node = xmlrpc.client.ServerProxy(self.master.lookupNode("/bt_console"))
        code, _, pid = node.getPid(self.ros.get_name())
        require(code == 1 and type(pid) is int and pid > 1, "BT PID unavailable")
        require(b"bt_console" in Path("/proc/%d/cmdline" % pid).read_bytes(), "BT process identity mismatch")
        case["paused_pid"] = pid
        case["paused_at"] = time.monotonic()
        os.kill(pid, signal.SIGSTOP)
        try:
            self.wait("lease expiry", lambda: any(v["seq"] == case["seq"] and v["reason"] == "lease_lost"
                      for _, v in self.rows("adapter", case["paused_at"])), 8)
            self.quiet("lease loss independent stop", 3)
        finally:
            os.kill(pid, signal.SIGCONT)
        self.complete(case, "failed", "latched", "lease_lost")
        self.quiet("BT resume cannot resume motion", 3)

    def run(self, lease_loss=False):
        self.ready()
        start = self.last("gt")
        self.success(self.send("golden", start["x"] + 1, start["y"]))
        case = self.send("cancel_active", start["x"] + 1.5, start["y"])
        def active():
            states = [v for _, v in self.rows("adapter", case["sent"]) if v["seq"] == case["seq"]]
            ids = [i for i, s in self.last("backend").items() if s == 1 and i not in case["previous_backend_ids"]]
            return ids if self.fresh("backend") and states and states[-1]["state"] == "active" else False
        case["backend_active_ids"] = self.wait("new scoped backend ACTIVE", active)
        case["cancel_at"] = time.monotonic()
        self.cancel.publish(self.String(data="100:current"))
        forwarded = self.wait("BT forwarded reasonless cancel", lambda: next((json.loads(v) for _, v in self.rows("cancel", case["cancel_at"])
                              if json.loads(v).get("reason") == ""), None))
        self.cancel.publish(self.String(data="101:current:repeat"))
        require(int(forwarded["goal_seq"]) == case["seq"], "cancel targets wrong goal")
        require(bool(forwarded["session"]), "cancel lacks session identity")
        case["forwarded_cancel"] = forwarded
        ack = str(forwarded["seq"]) + ":cancelled"
        self.wait("matching cancel ack", lambda: ack in [v for _, v in self.rows("ack", case["cancel_at"])])
        case["cancel_ack"] = ack
        self.complete(case, "cancelled", "failed", "cancelled")
        case["backend_stopped"] = self.wait("all backend active states gone", lambda: next(
            (dict(at=t, statuses=v) for t, v in self.rows("backend", case["cancel_at"])
             if not any(s in LIVE for s in v.values())), None))
        self.quiet("cancel no resumption", 3)
        self.success(self.send("post_cancel_reuse", start["x"] + 1.5, start["y"]))
        gt = self.last("gt")
        case = self.send("invalid_nonzero_z", gt["x"] + 1, gt["y"], 0.5)
        self.complete(case, "failed", "rejected", "invalid_pose")
        self.quiet("invalid goal never moved", 3, since=case["sent"])
        cases = self.evidence["cases"]
        require(self.graph() and len(cases) == 4, "incomplete runtime checks")
        require({v["current_seq"] for _, v in self.rows("bt", cases[0]["sent"])
                 if v["current_seq"] > cases[0]["previous_seq"]} == {c["seq"] for c in cases},
                "unscoped/retried BT goal observed")
        for c in cases:  # Also reject contradictory late results during stationary observations.
            results = [v for _, v in self.rows("result", c["sent"]) if v.partition(":")[0] == str(c["seq"])]
            require(results and set(results) == set(c["adapter_results"]), "late contradictory adapter result")
            require(all(v["outcome"] == c["bt_terminal"]["outcome"] for _, v in self.rows("bt", c["sent"])
                        if v["outcome_seq"] == c["seq"] and v["outcome"]), "late contradictory BT outcome")
        if lease_loss:
            self.lease_loss()
        require(not self.error, self.error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="JSON evidence path; parent directory must exist")
    parser.add_argument("--approved-test-port", type=int, default=11313)
    parser.add_argument("--lease-loss", action="store_true")
    parser.add_argument("--local-planner", default="teb", choices=("teb", "dwa", "trajectory"))
    parser.add_argument("--global-planner", default="global_planner", choices=("global_planner", "navfn"))
    parser.add_argument("--timeout", type=float, default=300, help="overall wall seconds, at most 600")
    args = parser.parse_args()
    started, driver, rc = time.monotonic(), None, 1
    evidence = dict(verdict="INVALID", master=os.environ.get("ROS_MASTER_URI", ""), runtime_verified=False)
    def interrupt(signum, frame):
        raise KeyboardInterrupt("wall deadline/signal {}".format(signum))
    try:
        uri = urlsplit(evidence["master"])
        require(1024 <= args.approved_test_port <= 65535 and args.approved_test_port not in (11311, 11312)
                and uri.scheme == "http" and uri.hostname == "127.0.0.1" and uri.port == args.approved_test_port
                and not (uri.username or uri.password or uri.query or uri.fragment) and uri.path in ("", "/"),
                "REFUSED: only 127.0.0.1 on the approved isolated test port; never 11311/11312")
        require(math.isfinite(args.timeout) and 30 <= args.timeout <= 600, "invalid wall timeout")
        for sig in (signal.SIGALRM, signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, interrupt)
        signal.setitimer(signal.ITIMER_REAL, args.timeout)
        socket.setdefaulttimeout(2.0)
        driver = Driver(evidence, args.local_planner, args.global_planner)
        evidence["verdict"] = "FAIL"
        driver.run(lease_loss=args.lease_loss)
        evidence.update(verdict="PASS", runtime_verified=True)
        rc = 0
    except (Exception, KeyboardInterrupt) as exc:
        evidence["error"] = "{}: {}".format(type(exc).__name__, exc)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        if driver is not None and rc:
            try:
                driver.cancel.publish(driver.String(data="1000000:all:runtime_test_cleanup"))
            except Exception as exc:
                evidence["cleanup_error"] = str(exc)
        evidence["elapsed_wall_s"] = time.monotonic() - started
        try:
            if driver is not None:
                with driver.cv:
                    evidence = json.loads(json.dumps(evidence, allow_nan=False))
            with open(os.path.abspath(args.output), "w", encoding="utf-8") as stream:
                json.dump(evidence, stream, indent=2, allow_nan=False)
                stream.write("\n")
        except Exception as exc:
            print("Cannot write evidence: " + str(exc), file=sys.stderr)
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
