# Framework 目录迁移独立验收

最新更新时间：2026-11-13T22:46:53.148851+08:00。

**PASS，限布局、离线入口与导入自举。**

- 清除PYTHONPATH并从/tmp启动的6项CLI/离线检查通过。
- 12个节点在独立新进程、只保留自己的脚本目录和生产自举下导入通过；ROS模块仅用惰性替身，未执行main。
- 23个真实本包node manifest路径存在且当前身份可执行；9个明确test double不作真实入口。
- 66个迁移目录Python文件通过3.8语法检查；4份README本地Markdown链接0断链。
- run_map_file shim模块路径断言正确；matrix SMOKE文件在新路径，调用使用Python，无需X位；仅生成消息替身的argparse帮助通过。

初检发现3份已修改README更新时间仍旧，root已更正，独立读回4份当前README均为2026-11-13。matrix初始X位断言是探针过严，原失败保留并纠正；host无v8_msgs的帮助失败不当作迁移失效。

本结论不签ROS加载、Docker、仿真或目标Python3.8实跑，不签本地commit。未修改生产、.ai或Git索引；仅写本/tmp目录。

主控另报framework全套296=286通过+10skip；scene为14通过+1个socket EPERM环境失败。本实例未重复这些全套，不把环境失败写成全PASS；Noetic/Docker运行NOT_RUN。独立检查后66源快照与当前工作树差异为[]。
