cmake_minimum_required(VERSION 3.0.2)
project(v8_platform)

find_package(catkin REQUIRED)

# 纯 Python 平台层：不生成编译目标，也不安装模块；
# 测试与夹具按源码路径运行（见 README.md）。
catkin_package()
