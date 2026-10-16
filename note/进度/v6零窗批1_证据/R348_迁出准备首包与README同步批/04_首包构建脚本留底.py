#!/usr/bin/env python3
# R348 首包构建脚本（★不入库为在册载体件、★不入门禁串；本留底＝可复跑凭据）
# 契约真身＝note/进度/SCAN集成/设计/av_v6仿真拆分与x86验证架构设计稿.md §2.1/§2.2/§2.3/§2.4/§2.5/§6.1
#           ＋ note/进度/SCAN集成/设计/au_x86仿真迁出交接任务单_设计稿.md §4.2/§4.3/§4.4/§4.5/§5
# 排除面（真人 2026-10-17 照准）＝ws/build·ws/devel·.git·运行态·生产面＋design/scripts 的 9 枚（修⑩）＋s1_offline_frames_smoke.py＋note/ 整目录（只放三类白名单）
import os, shutil, subprocess, sys, time

REPO = '/home/ros1-slam'
PKG  = os.path.join(REPO, 'export', 'simexp_gates3cls_20261017_01')
EXC_DIRS  = {'build', 'devel', '.git', '__pycache__', 'logs', '.ros', '.gazebo', 'server-root'}
EXC_SUFFIX = ('.pyc', '.pyo', '.swp', '.DS_Store')

def rel(p):
    return os.path.relpath(p, REPO)

def excluded_dir(name):
    return name in EXC_DIRS

def excluded_file(name):
    return name.endswith(EXC_SUFFIX)

def dst_for(src_rel, root_sub):
    return os.path.join(PKG, root_sub, src_rel)

def copy_tree(src_abs, sub, stats):
    """按原相对路径拷一棵子树，逐条守 au §4.2 四条排除。"""
    src_rel = rel(src_abs)
    for dirpath, dirnames, filenames in os.walk(src_abs):
        dirnames[:] = [d for d in dirnames if not excluded_dir(d)]
        r = rel(dirpath)
        td = os.path.join(PKG, sub, r) if sub else os.path.join(PKG, r)
        os.makedirs(td, exist_ok=True)
        try:
            os.chmod(td, 0o775)
        except Exception:
            pass
        for fn in filenames:
            if excluded_file(fn):
                continue
            s = os.path.join(dirpath, fn)
            if os.path.islink(s):
                continue
            t = os.path.join(td, fn)
            shutil.copy2(s, t)
            st = os.stat(s)
            os.chmod(t, 0o755 if (st.st_mode & 0o111) else 0o664)
            stats[0] += 1
            stats[1] += st.st_size

def copy_file(src_abs, sub, stats):
    if not os.path.isfile(src_abs):
        raise SystemExit('★盘上未找到＝' + rel(src_abs))
    t = os.path.join(PKG, sub, rel(src_abs))
    os.makedirs(os.path.dirname(t), exist_ok=True)
    shutil.copy2(src_abs, t)
    st = os.stat(src_abs)
    os.chmod(t, 0o755 if (st.st_mode & 0o111) else 0o664)
    stats[0] += 1
    stats[1] += st.st_size

stats = [0, 0]
CODE = 'code'
ENV  = 'env'

# ---- 1) code/ws/src ＝ v5.3 §10 五批白名单前四批的 18 包源码面（av §2.2）----
PKGS = [
 'ws/src/00_drivers/lslidar_msgs', 'ws/src/00_drivers/lslidar_driver',
 'ws/src/00_drivers/message_transformer', 'ws/src/00_drivers/lite3_drivers',
 'ws/src/10_slam/faster_lio', 'ws/src/10_slam/pcd2grid', 'ws/src/10_slam/lite3_slam',
 'ws/src/20_localization/ndt_omp', 'ws/src/20_localization/fast_gicp',
 'ws/src/20_localization/hdl_localization', 'ws/src/20_localization/hdl_global_localization',
 'ws/src/20_localization/lite3_localization',
 'ws/src/30_perception/lite3_perception',
 'ws/src/40_navigation/lite3_navigation', 'ws/src/50_task/lite3_task',
 'ws/src/60_safety/lite3_safety', 'ws/src/70_bringup/lite3_bringup',
 'ws/src/90_supervisor/lite3_supervisor',
]
for p in PKGS:
    copy_tree(os.path.join(REPO, p), CODE, stats)

# ---- 2) code/scripts ＝ scripts/verify 全目录（门禁件＋四张面清单＋窗回执）----
copy_tree(os.path.join(REPO, 'scripts/verify'), CODE, stats)

# ---- 3) code/tooling ＝ tooling 全树（技能栈镜像＋tooling 自带清单，原样带、不重生成）----
copy_tree(os.path.join(REPO, 'tooling'), CODE, stats)

# ---- 4) env/ ＝ §1.2 I-1…I-9 加 I-6b 的选定子集 ----
worlds = subprocess.run(['find', 'ws', '-name', '*.world'], cwd=REPO, capture_output=True, text=True).stdout.split()
for w in sorted(worlds):                                   # I-1
    copy_file(os.path.join(REPO, w), ENV, stats)
for m in ['ws/src/95_simulation/lite3_description/urdf/Lite3.urdf',        # I-2
          'ws/src/95_simulation/lite3_description/meshes/torso.STL',
          'ws/src/95_simulation/lite3_description/meshes/hip.STL',
          'ws/src/95_simulation/lite3_description/meshes/thigh.STL',
          'ws/src/95_simulation/lite3_description/meshes/shank.STL',
          'ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro',   # I-3
          'ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp',     # I-4
          'ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp.before_M2R1.bak',
          'ws/src/95_simulation/lite3_gazebo_sim/stairstudy/fixtures.py',         # I-8
          'ws/src/95_simulation/lite3_gazebo_sim/stairstudy/precheck.py',
          'ws/src/95_simulation/lite3_gazebo_sim/stairstudy/bandstudy.py',
          'ws/src/95_simulation/lite3_gazebo_sim/stairstudy/sweep.py',
          'ws/src/95_simulation/config/faster_lio_sim.yaml',                      # I-7
          'ws/src/95_simulation/config/passability/passability.yaml',
          'ws/src/00_drivers/lite3_drivers/launch/95_sim.launch',                 # I-6b 驱动面
          'ws/src/00_drivers/lite3_drivers/launch/01_sensors.launch',
          'ws/src/00_drivers/message_transformer/launch/message_transformer.launch',
          'ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh',              # I-6b 调用端逐字
          ]:
    copy_file(os.path.join(REPO, m), ENV, stats)
copy_tree(os.path.join(REPO, 'ws/src/95_simulation/Mid360_simulation_plugin/livox_laser_simulation'), ENV, stats)  # I-5
for pat, cwd_ in [('*.launch', 'ws/src/95_simulation'), ('*.launch', 'ws/src/70_bringup/lite3_bringup/launch')]:
    hits = subprocess.run(['find', cwd_, '-name', pat], cwd=REPO, capture_output=True, text=True).stdout.split()
    for h in sorted(hits):                                                        # I-6
        copy_file(os.path.join(REPO, h), ENV, stats)
for d in ['ws/src/00_drivers/livox_ros_driver2/launch_ROS1',                       # I-6b 点云入口面
          'ws/src/00_drivers/lslidar_driver/launch',
          'ws/src/40_navigation/lite3_navigation/config',
          'ws/src/90_supervisor/lite3_supervisor/launch',
          'ws/src/60_safety/lite3_safety/launch',
          'ws/src/40_navigation/lite3_navigation/launch']:
    for f in sorted(subprocess.run(['find', d, '-maxdepth', '1', '-type', 'f'], cwd=REPO,
                                   capture_output=True, text=True).stdout.split()):
        copy_file(os.path.join(REPO, f), ENV, stats)

# ---- 5) data/ ＝ av §2.3 七件 ----
for f in ['data/maps/lite3.pcd', 'data/maps/lite3.pgm', 'data/maps/lite3.yaml',
          'data/maps/sim_venue.pcd', 'data/maps/lite3_1007.pcd',
          'data/maps/lite3_1007_polished.pcd', 'data/waypoints/README.md']:
    copy_file(os.path.join(REPO, f), '', stats)

# ---- 6) note/ 面三类白名单：★当期只到两类（接口件 av·au）；★取数留底模板盘上无在册件＝不自造 ----
for f in ['note/进度/SCAN集成/设计/av_v6仿真拆分与x86验证架构设计稿.md',
          'note/进度/SCAN集成/设计/au_x86仿真迁出交接任务单_设计稿.md']:
    copy_file(os.path.join(REPO, f), '', stats)

os.makedirs(os.path.join(PKG, 'receipt'), exist_ok=True)
os.chmod(os.path.join(PKG, 'receipt'), 0o775)
print('COPIED files=%d bytes=%d' % (stats[0], stats[1]))
print('PKG=%s' % PKG)
