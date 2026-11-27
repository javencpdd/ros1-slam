#!/usr/bin/env python3
"""生成 Foxglove 三类诊断布局（定位 / 导航 / 任务）。

为什么单独三份（2026-11-27）：M2 出问题时看的东西完全不同——定位要看 z 与
reason/detail；导航要看全局/局部路径、障碍与速度；任务要看 BT 事件序列与取消确认。
一份"什么都放一点"的布局在故障时反而看不清。

只生成 JSON（不连桥、不改 ROS 状态）。话题名按**实际图**核过，不按默认猜。
用法：gen_foxglove_layouts.py [输出目录，默认 docker/foxglove]
"""
import json
import os
import sys

PCD_TOPICS = {"/cloud_registered": {"visible": True, "colorBy": "z", "pointShape": "circle",
                                    "pointSize": 0.04, "decayTime": 0}}


def panel_3d(name, title, topics):
    t = dict(PCD_TOPICS)
    t.update(topics)
    t["/tf"] = {"visible": True, "axisScale": 1}
    return {name: {"id": name, "title": title,
                   "config": {"cameraState": {"distance": 12, "fovy": 0.7853981852531433,
                                              "near": 0.1, "far": 5000, "perspective": True,
                                              "phi": 0.7853981852531433, "target": [0, 0, 0],
                                              "targetOffset": [0, 0, 0], "thetaOffset": 0},
                              "followMode": "follow-pose",
                              "scene": {"enableStats": False,
                                        "grid": {"color": "#248eff", "divisions": 10, "size": 40},
                                        "labelScaleFactor": 1},
                              "topics": t, "transforms": {}}}}


def panel_raw(name, title, topic):
    return {name: {"id": name, "title": title,
                   "config": {"topicName": topic, "diffEnabled": False}}}  # 只读文本


def mk(lid, lname, panels, tree):
    return {"id": lid, "name": lname, "version": 1,
            "data": {"configById": panels, "layout": json.dumps(tree, ensure_ascii=False)}}


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'docker', 'foxglove')
    os.makedirs(out_dir, exist_ok=True)

    loc = {}
    loc.update(panel_3d("3D!loc", "3D · 定位（pose/TF/点云）",
                        {"/v8/local/pose": {"visible": True},
                         "/sim/odom_gt": {"visible": True},
                         "/rslidar_points": {"visible": False}}))
    loc.update(panel_raw("RawMessages!loc_status", "local status（reason/detail/age）",
                         "/v8/local/status"))
    loc_tree = {"direction": "row", "first": "3D!loc",
                "second": "RawMessages!loc_status", "splitPercentage": 65}

    nav = {}
    nav.update(panel_3d("3D!nav", "3D · 导航（路径/障碍/地形）",
                        {"/v8/3d/global/path": {"visible": True},
                         "/v8/gait_guard/path": {"visible": True},
                         "/v8/local/trajectory": {"visible": True},
                         "/v8/local/obstacles": {"visible": True},
                         "/terrain_cloud": {"visible": False, "colorBy": "z"}}))
    nav.update(panel_raw("RawMessages!nav_status", "local status", "/v8/local/status"))
    nav_tree = {"direction": "row", "first": "3D!nav",
                "second": "RawMessages!nav_status", "splitPercentage": 70}

    task = {}
    task.update(panel_raw("RawMessages!bt", "BT status（event/seq/outcome）", "/v8/bt/status"))
    task.update(panel_raw("RawMessages!cancel", "local cancel_ack", "/v8/local/cancel_ack"))
    task_tree = {"direction": "row", "first": "RawMessages!bt",
                 "second": "RawMessages!cancel", "splitPercentage": 60}

    files = {
        'ros1_slam_sim_loc_layout.json': mk('ros1-slam-sim-loc', 'ros1-slam 仿真 · 定位诊断', loc, loc_tree),
        'ros1_slam_sim_nav_layout.json': mk('ros1-slam-sim-nav', 'ros1-slam 仿真 · 导航诊断', nav, nav_tree),
        'ros1_slam_sim_task_layout.json': mk('ros1-slam-sim-task', 'ros1-slam 仿真 · 任务生命周期', task, task_tree),
    }
    for fname, payload in files.items():
        path = os.path.join(out_dir, fname)
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=1)
        print('wrote', path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
