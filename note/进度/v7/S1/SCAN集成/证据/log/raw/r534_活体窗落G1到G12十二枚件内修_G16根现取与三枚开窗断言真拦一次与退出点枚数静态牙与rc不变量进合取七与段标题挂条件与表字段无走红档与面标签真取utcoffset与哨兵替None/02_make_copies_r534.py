#!/usr/bin/env python3
# 02_make_copies_r534.py —— 副本生成器（★在册件一字不改★，改动只在副本）
#   A 侧＝在册 global_costmap_params.yaml 做★同形（flow）重排★（C1-B 已立的形：让 A↔C 的 diff 命中行全部以 plugins／elevation_layer 起始）
#   C 侧＝A ＋一枚 elevation_layer（StaticLayer），★三把键全钉死＝use_maximum: true ‖ combination_method: 1 ‖ lethal_cost_threshold: 100★
#        （R445／R446 硬结论＝漏写或写 false 走覆盖、能把底层致命抹成自由＝安全侧）
#   注释与空行一律★不新增★；白名单逐行验落 03_delta.txt
import os
import subprocess
import sys

root = sys.argv[1] if len(sys.argv) > 1 else "/opt/ros1-slam"
# ★G17/R534＝写出目录★由本件自己的位置现取★（旧形＝硬编码 raw/<窗目录名>＝目录名与件面各写一遍、改一枚即指空；
#   ★批190 事故的第二层★＝窗脚本那枚硬编码修掉了（G16），但这枚★副本生成器★仍然写死路径＝同族缺陷还剩两处＝本枚把它们一起收掉）
D = os.path.dirname(os.path.abspath(__file__))
assert os.path.basename(D).startswith("r534_"), "★G17＝写出目录不在 r534_ 窗内＝%s★" % D
assert not os.path.exists(os.path.join(D, "_sha256_manifest.txt")), "★G17＝写出目录已有清单＝已入库只读不写＝%s★" % D
print("  ★G17 自证★＝02_make_copies 写出目录＝%s（由 __file__ 现取 ‖ 仓根 argv＝%s）" % (D, root))

Y_REG = os.path.join(root, "ws/src/40_navigation/lite3_navigation/config/global_costmap_params.yaml")
B_REG = os.path.join(root, "ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch")
M_REG = os.path.join(root, "ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch")
S_REG = os.path.join(root, "ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh")

Y_A = os.path.join(D, "global_costmap_params_A_r534.yaml")
Y_C = os.path.join(D, "global_costmap_params_C_r534.yaml")
B_CP = os.path.join(D, "mbf_navigation_r534.launch")
M_CP = os.path.join(D, "mode_nav_sim_r534.launch")
S_CP = os.path.join(D, "sim_nav_smoke_r534.sh")

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
write(B_CP, t.replace('file="$(find lite3_navigation)/config/global_costmap_params.yaml"', 'file="$(env R530_YAML)"'))
t = open(M_REG, encoding="utf-8").read()
# ★G17／R534＝★生成面★也不许把窗目录绝对路径写进副本（旧形＝把 B_CP 直接嵌进 launch＝★生成器每跑一次就重播一遍硬编码★）
INC_OLD = '<include file="$(find lite3_navigation)/launch/mbf_navigation.launch">'
INC_NEW = ('  <arg name="win_root"/>\n'
           '  <include file="$(arg win_root)/mbf_navigation_r534.launch">')   # ★无 default＝没传就 roslaunch 当场报错＝fail-fast★
assert t.count(INC_OLD) == 1, "★launch 锚点命中 %d 次★" % t.count(INC_OLD)
write(M_CP, t.replace(INC_OLD, INC_NEW))
t = open(S_REG, encoding="utf-8").read()
ROLD = "setsid roslaunch lite3_bringup mode_nav_sim.launch"
RNEW = ('SELF="$(cd "$(dirname "$0")" && pwd)"; '
        '[ -f "$SELF/mode_nav_sim_r534.launch" ] || { echo "CHECK|g17_selfpath|FAIL|本件旁边没有 mode_nav_sim_r534.launch＝$SELF"; exit 1; }; '
        'echo "[smoke] ★G17 自证★＝win_root=$SELF（现取 dirname $0）"; '
        'setsid roslaunch "$SELF/mode_nav_sim_r534.launch" win_root:="$SELF"')
assert t.count(ROLD) == 1, "★smoke 锚点命中 %d 次★" % t.count(ROLD)
open(S_CP, "w", encoding="utf-8").write(t.replace(ROLD, RNEW))
# ★G17 事后复现＝★生成之后★再扫刚写出来的那两枚字节（旧形＝只在生成前看形＝生成面重播硬编码无人拦＝追加一百一十六③「当场值≠落盘后值」的★写入面★亚种）
for _p in (M_CP, S_CP):
    _x = open(_p, encoding="utf-8").read()
    NEEDLE = "log" + "/" + "raw" + "/"   # ★拼接造锚＝本文件字面里★没有★被检串（追加一百零一②「修元字符那句自己带元字符」的同族＝★检查件不许自带被检字面★）
    _hit = [q for q in (D, B_CP, M_CP, NEEDLE) if q in _x]
    if _hit:
        sys.exit("★G17 事后复现不过＝生成面把绝对路径写进了副本＝%s ‖ 命中＝%s★" % (_p, _hit))
print("  ★G17 事后复现＝生成面两件（launch ‖ smoke）现扫＝本目录路径 ‖ 两枚副本绝对路径 ‖ 那枚目录前缀字面（拼接造锚、本件不自带）＝0 命中★（被试＝刚写出的字节）")
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
