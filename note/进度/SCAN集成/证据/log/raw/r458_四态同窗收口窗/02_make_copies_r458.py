#!/usr/bin/env python3
# 02_make_copies_r458.py —— ★R458 的副本生成器★（★在册件一字不改、改动只发生在副本★）
# 与 R456 那版的差别＝按 r456q 裁定的 C1 半步自解：
#   ①★不放空行、不放注释★（注释移到本件与留底 06，不进 yaml）；
#   ②★plugins 用 YAML flow 一行★ ⇒ A↔B 的 diff 命中行只剩「plugins 那一行」＋「elevation_layer 那一行」＋「被替换掉的三条旧列表项行」；
#     ★旧列表项行属于 plugins 列表本体、不是 foreign 变量★＝这一条本件当场跑在册机检式并把结果逐字落 03_delta.txt（★不合字面的行数如实报、不改判、不假装通过★）；
#   ③★B／C 两态共用同一条副本链★，被试参数面用 `$(env R458_YAML)` 选文件 ⇒ 唯一变量只剩「那枚层的图从哪来」。
import os
import subprocess
import sys

root = sys.argv[1] if len(sys.argv) > 1 else "/opt/ros1-slam"
D = os.path.join(root, "note/进度/SCAN集成/证据/log/raw/r458_四态同窗收口窗")

Y_REG = os.path.join(root, "ws/src/40_navigation/lite3_navigation/config/global_costmap_params.yaml")
B_REG = os.path.join(root, "ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch")
M_REG = os.path.join(root, "ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch")
S_REG = os.path.join(root, "ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh")

Y_B = os.path.join(D, "global_costmap_params_B_r458.yaml")
Y_C = os.path.join(D, "global_costmap_params_C_r458.yaml")
B_CP = os.path.join(D, "mbf_navigation_r458.launch")
M_CP = os.path.join(D, "mode_nav_sim_r458.launch")
S_CP = os.path.join(D, "sim_nav_smoke_r458.sh")

PLUGINS_B = 'plugins: [{name: static_layer, type: "costmap_2d::StaticLayer"}, {name: elevation_layer, type: "costmap_2d::StaticLayer"}, {name: stvl_obstacle_layer, type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}, {name: sob_layer, type: "sob_layer::SobLayer"}]\n'
ELEV_B = 'elevation_layer: {map_topic: map, first_map_only: true, subscribe_to_updates: false, track_unknown_space: false, use_maximum: true, combination_method: 1, lethal_cost_threshold: 100, unknown_cost_value: -1, trinary_costmap: true}\n'
ELEV_C = ELEV_B.replace("map_topic: map,", "map_topic: /elev_map,")


def make_yaml(dst, elev):
    t = open(Y_REG, encoding="utf-8").read()
    a = "plugins:\n  - {name: static_layer, type: \"costmap_2d::StaticLayer\"}\n  - {name: stvl_obstacle_layer, type: \"spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer\"}\n  - {name: sob_layer,  type: \"sob_layer::SobLayer\"}\n"
    if t.count(a) != 1:
        sys.exit("在册 plugins 块锚点命中 %d 次" % t.count(a))
    out = t.replace(a, PLUGINS_B)
    if not out.endswith("\n"):
        out += "\n"
    open(dst, "w", encoding="utf-8").write(out + elev)


def edit_mbf(dst):
    t = open(B_REG, encoding="utf-8").read()
    a = 'file="$(find lite3_navigation)/config/global_costmap_params.yaml"'
    if t.count(a) != 1:
        sys.exit("mbf 锚点命中 %d 次" % t.count(a))
    open(dst, "w", encoding="utf-8").write(t.replace(a, 'file="$(env R458_YAML)"'))


def edit_mode(dst):
    t = open(M_REG, encoding="utf-8").read()
    a = '<include file="$(find lite3_navigation)/launch/mbf_navigation.launch">'
    if t.count(a) != 1:
        sys.exit("mode 锚点命中 %d 次" % t.count(a))
    open(dst, "w", encoding="utf-8").write(t.replace(a, '<include file="%s">' % B_CP))


def edit_smoke(dst):
    t = open(S_REG, encoding="utf-8").read()
    a = "setsid roslaunch lite3_bringup mode_nav_sim.launch"
    if t.count(a) != 1:
        sys.exit("smoke 锚点命中 %d 次" % t.count(a))
    open(dst, "w", encoding="utf-8").write(t.replace(a, "setsid roslaunch %s" % M_CP))
    os.chmod(dst, 0o755)


make_yaml(Y_B, ELEV_B)
make_yaml(Y_C, ELEV_C)
edit_mbf(B_CP)
edit_mode(M_CP)
edit_smoke(S_CP)

log = []
for tag, src, dst in (("yaml A↔B", Y_REG, Y_B), ("yaml B↔C", Y_B, Y_C),
                      ("mbf", B_REG, B_CP), ("mode", M_REG, M_CP), ("smoke", S_REG, S_CP)):
    r = subprocess.run(["diff", "-u", src, dst], capture_output=True, text=True)
    # ★在册机检式用的是普通 diff 的 `^[<>]` 形（r448q C1 原式），不是 diff -u 的 +/- 形★
    raw = subprocess.run(["diff", src, dst], capture_output=True, text=True)
    hits = [l for l in raw.stdout.splitlines() if l.startswith("<") or l.startswith(">")]
    log.append("===== %s =====" % tag)
    log.append(r.stdout if r.stdout.strip() else "(无差异)")
    log.append("── 普通 diff（机检面）──")
    log.append(raw.stdout if raw.stdout.strip() else "(无差异)")
    log.append("★命中行数=%d★" % len(hits))
    for h in hits:
        body = h[1:].strip()
        ok = body.startswith("plugins") or body.startswith("elevation_layer")
        log.append("  白名单逐行验｜%s｜起始=%s｜以 plugins 或 elevation_layer 开头＝%s" % (h[:90], body[:22], ok))
    log.append("  ★不合字面行数=%d★" % len([h for h in hits if not (h[1:].strip().startswith("plugins") or h[1:].strip().startswith("elevation_layer"))]))
open(os.path.join(D, "03_delta.txt"), "w", encoding="utf-8").write("\n".join(log) + "\n")
print("副本五件已生成（B／C 两枚 yaml ＋ mbf／mode／smoke 各一枚共用副本链）；delta 与白名单逐行验落 03_delta.txt")
