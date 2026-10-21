#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""矩阵W2 壳包三件修改器（小明 2026-10-07，manifest B 文 Wave2 预登记②+①执行）。
两段式：本脚本先以 ysc 写入 /tmp，再 sudo python3 执行（勿 heredoc 直通 sudo）。
纪律：每处替换断言唯一/指定计数命中，失配即中止且不写盘（零半施用）。"""
import sys, io

def apply(path, subs):
    with io.open(path, encoding='utf-8') as f:
        txt = f.read()
    for old, new, cnt in subs:
        n = txt.count(old)
        if n != cnt:
            print('ABORT %s: 期望 %d 处、实得 %d 处：\n%r' % (path, cnt, n, old[:120]))
            sys.exit(1)
        txt = txt.replace(old, new)
    with io.open(path, 'w', encoding='utf-8') as f:
        f.write(txt)
    print('OK %s（%d 组替换全命中）' % (path, len(subs)))

# ---------- ① 95_sim.launch：mockamap 参数 arg 化（默认值=maze2d 现值，向后兼容） ----------
L95 = '/home/ros1-slam/ws/src/00_drivers/lite3_drivers/launch/95_sim.launch'
args_block = '''  <arg name="extra_sim_args" default=""/>  <!-- 附加参数透传 sim_adapter（mode_nav_sim 传 no_imu 开关；默认空=G8 原行为） -->
  <!-- 矩阵W2（2026-10-07 小明，manifest B 文 Wave2.3 预登记②执行）：mockamap 参数 arg 化——默认值=maze2d 现值，
       向后兼容零行为变化；maze2d 强制正方形（非正方形 Eigen 越界实证修正①）；perlin3d/post2d 类型参数
       取 mockamap 模板 launch 原值（type=3 时 mockamap_node 不读这些键，默认路径零影响） -->
  <arg name="map_type" default="3"/>     <!-- 3=maze2d，1=perlin3d，2=post2d -->
  <arg name="map_seed" default="7"/>
  <arg name="map_x" default="12"/>       <!-- maze2d 强制正方形 -->
  <arg name="map_y" default="12"/>
  <arg name="map_z" default="3"/>
  <arg name="map_res" default="0.1"/>
  <arg name="map_traj" default="$(find lite3_drivers)/config/traj_maze.yaml"/>  <!-- sim_adapter 轨迹源（多地图换静止/定制轨迹） -->
  <arg name="perlin_complexity" default="0.07"/>   <!-- perlin3d 模板原值 -->
  <arg name="perlin_fill"       default="0.1"/>
  <arg name="perlin_fractal"    default="1"/>
  <arg name="perlin_attenuation" default="0.1"/>
  <arg name="post_width_min"    default="0.6"/>    <!-- post2d 模板原值 -->
  <arg name="post_width_max"    default="1.5"/>
  <arg name="post_number"       default="50"/>
  <arg name="post_surface_res"  default="0.03"/>'''
subs95 = [
    ('  <arg name="extra_sim_args" default=""/>  <!-- 附加参数透传 sim_adapter（mode_nav_sim 传 no_imu 开关；默认空=G8 原行为） -->',
     args_block, 1),
    ('<param name="seed"        type="int"    value="7"/>',
     '<param name="seed"        type="int"    value="$(arg map_seed)"/>', 1),
    ('<param name="resolution"  type="double" value="0.1"/>',
     '<param name="resolution"  type="double" value="$(arg map_res)"/>', 1),
    ('<param name="x_length"    type="int"    value="12"/>',
     '<param name="x_length"    type="int"    value="$(arg map_x)"/>', 1),
    ('<param name="y_length"    type="int"    value="12"/>',
     '<param name="y_length"    type="int"    value="$(arg map_y)"/>', 1),
    ('<param name="z_length"    type="int"    value="3"/>',
     '<param name="z_length"    type="int"    value="$(arg map_z)"/>', 1),
    ('<param name="type"        type="int"    value="3"/>   <!-- maze2d -->',
     '<param name="type"        type="int"    value="$(arg map_type)"/>   <!-- maze2d/perlin3d/post2d -->', 1),
    ('''<param name="maze_type"   type="int"    value="1"/>   <!-- recursive division -->
  </node>''',
     '''<param name="maze_type"   type="int"    value="1"/>   <!-- recursive division -->
    <param name="complexity"         type="double" value="$(arg perlin_complexity)"/>   <!-- perlin3d 键（type=1 用） -->
    <param name="fill"               type="double" value="$(arg perlin_fill)"/>
    <param name="fractal"            type="int"    value="$(arg perlin_fractal)"/>
    <param name="attenuation"        type="double" value="$(arg perlin_attenuation)"/>
    <param name="width_min"          type="double" value="$(arg post_width_min)"/>     <!-- post2d 键（type=2 用） -->
    <param name="width_max"          type="double" value="$(arg post_width_max)"/>
    <param name="obstacle_number"    type="int"    value="$(arg post_number)"/>
    <param name="surface_resolution" type="double" value="$(arg post_surface_res)"/>
  </node>''', 1),
    ('<param name="map/x_size"  value="12.0"/>',
     '<param name="map/x_size"  value="$(eval float(arg(\'map_x\')))"/>', 2),
    ('<param name="map/y_size"  value="12.0"/>',
     '<param name="map/y_size"  value="$(eval float(arg(\'map_y\')))"/>', 2),
    ('<param name="map/z_size"  value="3.0"/>',
     '<param name="map/z_size"  value="$(eval float(arg(\'map_z\')))"/>', 2),
    ('--traj $(find lite3_drivers)/config/traj_maze.yaml',
     '--traj $(arg map_traj)', 1),
]
# ---------- ② mode_nav_sim.launch：参数透传 + sim_map_bridge grid_size ----------
MNS = '/home/ros1-slam/ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch'
mns_args = '''  <arg name="use_gpu" default="true"/>  <!-- G8 遗产：USE_GPU=OFF 件走 CPU 渲染降级分支（吞吐~1.5Hz） -->
  <!-- 矩阵W2（2026-10-07 小明，manifest B 文 Wave2.3）：多地图矩阵透传（默认=maze2d 现值，零行为变化） -->
  <arg name="map_type" default="3"/>
  <arg name="map_seed" default="7"/>
  <arg name="map_x" default="12"/>
  <arg name="map_y" default="12"/>
  <arg name="map_z" default="3"/>
  <arg name="map_res" default="0.1"/>
  <arg name="map_traj" default="$(find lite3_drivers)/config/traj_maze.yaml"/>
  <arg name="map_grid" default="12.0"/>  <!-- sim_map_bridge ~grid_size，须 ≥ 地图最长边 -->
  <arg name="perlin_complexity" default="0.07"/>
  <arg name="perlin_fill" default="0.1"/>
  <arg name="perlin_fractal" default="1"/>
  <arg name="perlin_attenuation" default="0.1"/>
  <arg name="post_width_min" default="0.6"/>
  <arg name="post_width_max" default="1.5"/>
  <arg name="post_number" default="50"/>
  <arg name="post_surface_res" default="0.03"/>'''
subs_mns = [
    ('  <arg name="use_gpu" default="true"/>  <!-- G8 遗产：USE_GPU=OFF 件走 CPU 渲染降级分支（吞吐~1.5Hz） -->',
     mns_args, 1),
    ('''    <arg name="extra_sim_args" value="--no-imu"/>
  </include>''',
     '''    <arg name="extra_sim_args" value="--no-imu"/>
    <arg name="map_type" value="$(arg map_type)"/>
    <arg name="map_seed" value="$(arg map_seed)"/>
    <arg name="map_x" value="$(arg map_x)"/>
    <arg name="map_y" value="$(arg map_y)"/>
    <arg name="map_z" value="$(arg map_z)"/>
    <arg name="map_res" value="$(arg map_res)"/>
    <arg name="map_traj" value="$(arg map_traj)"/>
    <arg name="perlin_complexity" value="$(arg perlin_complexity)"/>
    <arg name="perlin_fill" value="$(arg perlin_fill)"/>
    <arg name="perlin_fractal" value="$(arg perlin_fractal)"/>
    <arg name="perlin_attenuation" value="$(arg perlin_attenuation)"/>
    <arg name="post_width_min" value="$(arg post_width_min)"/>
    <arg name="post_width_max" value="$(arg post_width_max)"/>
    <arg name="post_number" value="$(arg post_number)"/>
    <arg name="post_surface_res" value="$(arg post_surface_res)"/>
  </include>''', 1),
    ('<node pkg="lite3_bringup" type="sim_map_bridge.py" name="sim_map_bridge" output="screen"/>',
     '''<node pkg="lite3_bringup" type="sim_map_bridge.py" name="sim_map_bridge" output="screen">
    <param name="grid_size" type="double" value="$(arg map_grid)"/>
  </node>''', 1),
]
# ---------- ③ mbf_navigation.yaml：controllers 增 DWAPlannerROS（teb 首位默认不变） ----------
MBF = '/home/ros1-slam/ws/src/40_navigation/lite3_navigation/config/mbf_navigation.yaml'
subs_mbf = [
    ('''controllers:
  - name: "TebLocalPlannerROS"
    type: "teb_local_planner/TebLocalPlannerROS"   # 串源=teb_local_planner_plugin.xml 双注册实测（mbf_costmap_core::CostmapController）→ mbf 原生可载''',
     '''controllers:
  - name: "TebLocalPlannerROS"
    type: "teb_local_planner/TebLocalPlannerROS"   # 串源=teb_local_planner_plugin.xml 双注册实测（mbf_costmap_core::CostmapController）→ mbf 原生可载
  - name: "DWAPlannerROS"
    type: "dwa_local_planner/DWAPlannerROS"   # 矩阵W2/A4（2026-10-07 小明，manifest 预登记①执行）：dwa_local_planner 1.17.3 apt 在装（W2 预检），
                                              # blp_plugin.xml 实测仅注册 nav_core::BaseLocalPlanner → 经 mbf 内建 nav_core 包装加载
                                              # （mbf_costmap_nav 0.4.0 costmap_navigation_server.h:239 nav_core_controller_plugin_loader_ 实证）；
                                              # teb 首位默认不变；T2 对比用 ExePath controller 按名点单，零默认行为变化''', 1),
]

apply(L95, subs95)
apply(MNS, subs_mns)
apply(MBF, subs_mbf)
print('三件全部施用完成')
