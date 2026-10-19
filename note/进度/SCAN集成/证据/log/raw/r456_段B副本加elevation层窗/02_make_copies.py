#!/usr/bin/env python3
# 02_make_copies.py —— ★段 B 的"副本"生成器★（★在册四件一字不改、改动只发生在复制出来的副本上★）
# 用法：python3 02_make_copies.py <仓根>            （窗内跑＝/opt/ros1-slam）
# 产物（写在本件同目录）：
#   global_costmap_params_r456.yaml   ＝ 在册同名件 ＋ plugins 里多一枚 elevation StaticLayer ＋ 该层参数块（★显式钉 use_maximum: true ＋ combination_method: 1★）
#   mbf_navigation_r456.launch        ＝ 在册件，仅把 global_costmap 那一路 params 文件指向上一行那个副本（★local_costmap 两路不动、costmap_common 不复制★）
#   mode_nav_sim_r456.launch          ＝ 在册件，仅把 M4 那一路 include 指向上一行那个副本（绝对路径）
#   sim_nav_smoke_r456.sh             ＝ 在册冒烟件，仅把 setsid roslaunch 那一行指向 mode 副本
#   03_delta.txt                      ＝ 四组 `diff -u 在册 副本`（★改动面全公开、不得有第五处★）
import os
import subprocess
import sys

root = sys.argv[1] if len(sys.argv) > 1 else "/opt/ros1-slam"
D = os.path.join(root, "note/进度/SCAN集成/证据/log/raw/r456_段B副本加elevation层窗")

SRC = {
    "yaml": os.path.join(root, "ws/src/40_navigation/lite3_navigation/config/global_costmap_params.yaml"),
    "mbf": os.path.join(root, "ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch"),
    "mode": os.path.join(root, "ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch"),
    "smoke": os.path.join(root, "ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh"),
}
DST = {
    "yaml": os.path.join(D, "global_costmap_params_r456.yaml"),
    "mbf": os.path.join(D, "mbf_navigation_r456.launch"),
    "mode": os.path.join(D, "mode_nav_sim_r456.launch"),
    "smoke": os.path.join(D, "sim_nav_smoke_r456.sh"),
}

ELEV_PLUG = '  - {name: elevation_layer, type: "costmap_2d::StaticLayer"}  # R456 段 B 新增（只在副本里）\n'
ELEV_BLOCK = (
    "\n# ===== R456 段 B 副本独有（在册件无此块）=====\n"
    "# ★R445／R446 硬结论★＝两枚 StaticLayer 同栈时，第二枚★漏写或写 false★＝走覆盖、能把底层致命格抹成自由（cost 0）\n"
    "# ⇒ ★新增层必须把两把键都显式钉上★：use_maximum: true ＋ combination_method: 1\n"
    "elevation_layer:\n"
    "  map_topic: map\n"
    "  first_map_only: true\n"
    "  subscribe_to_updates: false\n"
    "  track_unknown_space: false\n"
    "  use_maximum: true\n"
    "  combination_method: 1\n"
    "  lethal_cost_threshold: 100\n"
    "  unknown_cost_value: -1\n"
    "  trinary_costmap: true\n"
)


def edit_yaml(t):
    a = '  - {name: stvl_obstacle_layer, type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}\n'
    if t.count(a) != 1:
        sys.exit("yaml 锚点1 命中 %d 次" % t.count(a))
    t = t.replace(a, ELEV_PLUG + a)
    if not t.endswith("\n"):
        t += "\n"
    return t + ELEV_BLOCK


def edit_mbf(t):
    a = 'file="$(find lite3_navigation)/config/global_costmap_params.yaml"'
    if t.count(a) != 1:
        sys.exit("mbf 锚点命中 %d 次" % t.count(a))
    return t.replace(a, 'file="%s"' % DST["yaml"])


def edit_mode(t):
    a = '<include file="$(find lite3_navigation)/launch/mbf_navigation.launch">'
    if t.count(a) != 1:
        sys.exit("mode 锚点命中 %d 次" % t.count(a))
    return t.replace(a, '<include file="%s">' % DST["mbf"])


def edit_smoke(t):
    a = "setsid roslaunch lite3_bringup mode_nav_sim.launch"
    if t.count(a) != 1:
        sys.exit("smoke 锚点命中 %d 次" % t.count(a))
    return t.replace(a, "setsid roslaunch %s" % DST["mode"])


OUT = {"yaml": edit_yaml, "mbf": edit_mbf, "mode": edit_mode, "smoke": edit_smoke}
log = []
for k in ["yaml", "mbf", "mode", "smoke"]:
    src = open(SRC[k], encoding="utf-8").read()
    dst = OUT[k](src)
    open(DST[k], "w", encoding="utf-8").write(dst)
    if k == "smoke":
        os.chmod(DST[k], 0o755)
    log.append("===== %s =====" % k)
    log.append("在册=%s（%d 行）｜副本=%s（%d 行）" % (SRC[k], src.count("\n") + 1, os.path.basename(DST[k]), dst.count("\n") + 1))
    r = subprocess.run(["diff", "-u", SRC[k], DST[k]], capture_output=True, text=True)
    log.append(r.stdout if r.stdout.strip() else "(无差异＝生成失败，必须报)")
    n = len([l for l in r.stdout.splitlines() if l.startswith("+") and not l.startswith("+++")])
    m = len([l for l in r.stdout.splitlines() if l.startswith("-") and not l.startswith("---")])
    log.append("本组 delta 行数＝+%d/-%d" % (n, m))
open(os.path.join(D, "03_delta.txt"), "w", encoding="utf-8").write("\n".join(log) + "\n")
print("副本四件已生成；delta 落 03_delta.txt")
