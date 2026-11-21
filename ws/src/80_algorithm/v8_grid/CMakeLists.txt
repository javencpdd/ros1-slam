cmake_minimum_required(VERSION 3.0.2)
project(v8_grid)
find_package(catkin REQUIRED)
catkin_package()
# 无编译目标、无 catkin_python_setup（全仓零先例，刻意不用）。
# Python 模块躺源码树（v8_grid/v8_grid/），消费者 sys.path.insert 指向本包根后
# `from v8_grid import indexing`。catkin_make 只 configure 本包、产物为零。
