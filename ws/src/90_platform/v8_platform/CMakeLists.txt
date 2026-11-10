cmake_minimum_required(VERSION 3.0.2)
project(v8_platform)

find_package(catkin REQUIRED)

# 纯 Python 平台层：没有编译目标。
#
# ★2026-11-11 更正★ 这里原先写「不生成编译目标，也不安装模块；测试与夹具按
# 源码路径运行」。但 framework/runtime.py 对 manifest 里 kind=node 的组件一律
# 生成 `rosrun <pkg> <type>`；包内不安装任何可执行 ⇒ rosrun 找不到入口，规划器
# 节点从未被真正启动过（planner/a4_runtime_smoke.py 因此 ok=false、path=null，
# 见 note/进度/v8/M5/26-11-11/x86仿真侧跑通记录.md 第 6 节）。
#
# 现在只安装**节点入口**：catkin_install_python 在 devel space 生成 wrapper，
# 运行时 exec 源文件，因此 sys.path.insert(0, HERE) 与 HERE.parent/contracts
# 这类相对依赖照旧指向源码树，不必把模块搬进安装目录。
# 测试与夹具仍然只按源码路径运行，不安装。
catkin_package()

catkin_install_python(PROGRAMS
  planner/topic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/planner)
