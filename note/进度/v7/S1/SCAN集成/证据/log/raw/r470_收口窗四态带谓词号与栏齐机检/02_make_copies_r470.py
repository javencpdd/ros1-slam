#!/usr/bin/env python3
# 02_make_copies_r470.py —— 副本生成器（★在册件一字不改★，改动只在副本）
#   A 侧＝在册 global_costmap_params.yaml 做★同形（flow）重排★（C1-B 已立的形：让 A↔C 的 diff 命中行全部以 plugins／elevation_layer 起始）
#   C 侧＝A ＋一枚 elevation_layer（StaticLayer），★三把键全钉死＝use_maximum: true ‖ combination_method: 1 ‖ lethal_cost_threshold: 100★
#        （R445／R446 硬结论＝漏写或写 false 走覆盖、能把底层致命抹成自由＝安全侧）
#   注释与空行一律★不新增★；白名单逐行验落 03_delta.txt
import os
import subprocess
import sys

root = sys.argv[1] if len(sys.argv) > 1 else "/opt/ros1-slam"
D = os.path.join(root, "note/进度/SCAN集成/证据/log/raw/r470_收口窗四态带谓词号与栏齐机检")

Y_REG = os.path.join(root, "ws/src/40_navigation/lite3_navigation/config/global_costmap_params.yaml")
B_REG = os.path.join(root, "ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch")
M_REG = os.path.join(root, "ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch")
S_REG = os.path.join(root, "ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh")

Y_A = os.path.join(D, "global_costmap_params_A_r470.yaml")
Y_C = os.path.join(D, "global_costmap_params_C_r470.yaml")
B_CP = os.path.join(D, "mbf_navigation_r470.launch")
M_CP = os.path.join(D, "mode_nav_sim_r470.launch")
S_CP = os.path.join(D, "sim_nav_smoke_r470.sh")

BLOCK = 'plugins:\n  - {name: static_layer, type: "costmap_2d::StaticLayer"}\n  - {name: stvl_obstacle_layer, type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}\n  - {name: sob_layer,  type: "sob_layer::SobLayer"}\n'
PL_A = 'plugins: [{name: static_layer, type: "costmap_2d::StaticLayer"}, {name: stvl_obstacle_layer, type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}, {name: sob_layer, type: "sob_layer::SobLayer"}]'
PL_C = 'plugins: [{name: static_layer, type: "costmap_2d::StaticLayer"}, {name: elevation_layer, type: "costmap_2d::StaticLayer"}, {name: stvl_obstacle_layer, type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}, {name: sob_layer, type: "sob_layer::SobLayer"}]'
ELEV_C = 'elevation_layer: {map_topic: /elev_map, first_map_only: true, subscribe_to_updates: false, track_unknown_space: false, use_maximum: true, combination_method: 1, lethal_cost_threshold: 100, unknown_cost_value: -1, trinary_costmap: true}'


def write(dst, txt):
    if not txt.endswith("\n"):
        txt += "\n"
    open(dst, "w", encoding="utf-8").write(txt)


t = open(Y_REG, encoding="utf-8").read()
if t.count(BLOCK) != 1:
    sys.exit("在册 plugins 块锚点命中 %d 次" % t.count(BLOCK))
a = t.replace(BLOCK, PL_A + "\n")
write(Y_A, a)
c = a.replace(PL_A, PL_C) + ELEV_C + "\n"
if c.count(PL_C) != 1 or c.count(ELEV_C) != 1:
    sys.exit("C 侧替换失败")
write(Y_C, c)

t = open(B_REG, encoding="utf-8").read()
write(B_CP, t.replace('file="$(find lite3_navigation)/config/global_costmap_params.yaml"', 'file="$(env R470_YAML)"'))
t = open(M_REG, encoding="utf-8").read()
write(M_CP, t.replace('<include file="$(find lite3_navigation)/launch/mbf_navigation.launch">', '<include file="%s">' % B_CP))
t = open(S_REG, encoding="utf-8").read()
open(S_CP, "w", encoding="utf-8").write(t.replace("setsid roslaunch lite3_bringup mode_nav_sim.launch", "setsid roslaunch %s" % M_CP))
os.chmod(S_CP, 0o755)

log = []
for tag, src, dst in (("A↔C", Y_A, Y_C), ("mbf", B_REG, B_CP), ("mode", M_REG, M_CP), ("smoke", S_REG, S_CP)):
    u = subprocess.run(["diff", "-u", src, dst], capture_output=True, text=True)
    raw = subprocess.run(["diff", src, dst], capture_output=True, text=True)
    hits = [l for l in raw.stdout.splitlines() if l.startswith("<") or l.startswith(">")]
    log.append("===== %s =====" % tag)
    log.append(u.stdout if u.stdout.strip() else "(无差异)")
    log.append("── 普通 diff（机检面）──")
    log.append(raw.stdout if raw.stdout.strip() else "(无差异)")
    log.append("★命中行数=%d★" % len(hits))
    bad = 0
    for hh in hits:
        body = hh[1:].strip()
        ok = body.startswith("plugins") or body.startswith("elevation_layer")
        if not ok:
            bad += 1
        log.append("  白名单逐行验｜%s｜起始=%s｜合＝%s" % (hh[:80], body[:20], ok))
    log.append("  ★不合字面行数=%d★" % bad)
open(os.path.join(D, "03_delta.txt"), "w", encoding="utf-8").write("\n".join(log) + "\n")
print("副本五件已生成（A／C 两枚参数面＋mbf／mode／smoke 三件副本链）；白名单逐行验落 03_delta.txt")
