#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""world_load_assert.py —— 世界件在位断言（R563／批219 新建）

为什么有这件：八枚 stair_lab 世界件此前不被任何 launch 引用，只能手动 world:= 传路径；
而传相对路径时 gzserver 会**静默不加载**楼梯（无报错、/gazebo/model_states 缺模型名、
点云退化成纯地面）——照字面复跑的人会拿到一份"楼梯看不见、近场全空"的假读数。
⇒ 本节点把"世界件到底加载了没有"从"靠人记得核验"变成"链路自己拦"：断言不过就不开录。

判据：从世界件本体解析出它声明的全部 <model name="X">，逐枚比对 /gazebo/get_world_properties
      的 model_names。缺任何一枚即 rc=1（见下"两半断言"）。

★两半断言（R565／本批活体实测后改）★
  ① 世界件声明的那枚场景模型（如 stairs_easy）——它缺＝楼梯没加载，点云退化成纯地面。
  ② ★模型库代理模型 ground_plane★——八枚世界件都 include model://ground_plane 与 model://sun；
     若 GAZEBO_MODEL_PATH 没生效，gzserver 会去联网拉 model database，离线必挂、ground_plane 就不在。
  ★不能拿 sun 当代理★：活体实测 sun 作为场景光源被 gazebo 吃掉，**不出现在 model_names 里**
  （实测空白景 loaded=['ground_plane']）⇒ 把 sun 写进 extra_required 会得到一枚**恒假的红**。
  这条是本批正反控换来的：第一版 extra_required="ground_plane,sun" 一起跑就红，看着像真缺陷、其实是判据错。

出口：除了 rc，还把结论写成全局参数 /world_load_assert/ok（bool）＋/world_load_assert/{declared,loaded}，
      供采集脚本直接断言——因为 roslaunch 的 required="true" 是"节点一死就整链关"，与本节点
      "PASS 即退出 0"的正常收尾冲突，不能靠 required 当闸。

边界：只读，不写任何文件、不改任何参数（除上述三枚结论参数）、不参与判据；★不改 precheck.py 一行★。
"""
import re
import sys

import rospy
from gazebo_msgs.srv import GetWorldProperties

# 期望模型名＝世界件本体声明的那些（不含 <include><uri>model://... 引来的，那些由 gazebo 模型库提供）
MODEL_TAG = re.compile(r'<model\s+name\s*=\s*"([^"]+)"')


def wanted_models(world_path):
    with open(world_path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()
    names = MODEL_TAG.findall(text)
    # 去重保序
    seen, out = set(), []
    for n in names:
        if n not in seen:
            seen.add(n)
            out.append(n)
    return out


def main():
    rospy.init_node("world_load_assert", anonymous=False)
    rospy.set_param("/world_load_assert/ok", False)

    world_file = rospy.get_param("~world_file", "")
    timeout = float(rospy.get_param("~timeout", 30.0))
    extra = rospy.get_param("~extra_required", [])
    if isinstance(extra, str):
        extra = [x.strip() for x in extra.split(",") if x.strip()]

    if not world_file:
        rospy.logfatal("[world_load_assert] ~world_file 未给，无法断言")
        sys.exit(2)

    try:
        want = wanted_models(world_file)
    except OSError as e:
        rospy.logfatal("[world_load_assert] 读不到世界件 %s: %s", world_file, e)
        sys.exit(2)

    if not want:
        rospy.logwarn(
            "[world_load_assert] 世界件里没有 <model name=...>（空白对照景属正常），只剩模型库代理断言: %s",
            world_file,
        )

    rospy.loginfo("[world_load_assert] 世界件=%s ‖ 声明模型=%s ‖ 代理要求=%s", world_file, want, extra)

    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=timeout)
    except rospy.ROSException:
        rospy.logfatal("[world_load_assert] %ss 内等不到 /gazebo/get_world_properties —— gzserver 没起来", timeout)
        sys.exit(1)

    try:
        props = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
    except rospy.ServiceException as e:
        rospy.logfatal("[world_load_assert] 调 get_world_properties 失败: %s", e)
        sys.exit(1)

    have = list(props.model_names)
    rospy.set_param("/world_load_assert/declared", want)
    rospy.set_param("/world_load_assert/loaded", have)
    rospy.loginfo("[world_load_assert] 实际加载模型=%s", have)

    missing = [m for m in want if m not in have]
    if missing:
        rospy.logfatal(
            "[world_load_assert] ★缺 %s —— 世界件没加载上，采集不开始★。"
            "常见成因＝world 传了相对路径（gzserver 的 cwd 被 roslaunch 设为 ~/.ros/）。"
            "本 launch 已强制 $(find ...) 绝对路径，若仍缺请查世界件本体。",
            missing,
        )
        sys.exit(1)

    miss2 = [m for m in extra if m not in have]
    if miss2:
        rospy.logfatal(
            "[world_load_assert] ★缺模型库代理模型 %s★ —— GAZEBO_MODEL_PATH 没生效／本地模型库缺件；"
            "gzserver 会转去联网拉 model database，离线环境必挂。",
            miss2,
        )
        sys.exit(1)

    rospy.set_param("/world_load_assert/ok", True)
    rospy.loginfo(
        "[world_load_assert] PASS —— 声明模型 %d 枚全部在位、代理模型 %d 枚全部在位",
        len(want), len(extra),
    )


if __name__ == "__main__":
    main()
