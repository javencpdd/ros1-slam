
## G8-sim 仿真预演批（Q14，来源 /home/test——新裁决：/home/test 同 /home/ysc 禁入沙箱，只读 tar 管道复制）
| # | 我方路径 | 来源 | 方式 | 日期 | 备注 |
|---|---|---|---|---|---|
| 18 | /home/ros1-slam/ws/src/95_simulation/mockamap | /home/test/scan_planner/src/simulator/mockamap | 原样复制 | 2026-10-04 | Q14 仿真预演 |
| 19 | /home/ros1-slam/ws/src/95_simulation/map_generator | /home/test/scan_planner/src/simulator/map_generator | 原样复制 | 2026-10-04 | Q14 仿真预演 |
| 20 | /home/ros1-slam/ws/src/95_simulation/local_sensing | /home/test/scan_planner/src/simulator/local_sensing | 原样复制 | 2026-10-04 | Q14 仿真预演（包名 local_sensing_node） |
| 21 | /home/ros1-slam/ws/src/95_simulation/lite3_description | /home/test/scan_planner/src/simulator/Utils/lite3_description | 原样复制 | 2026-10-04 | Q14 仿真预演（仅仿真可视化，不作真机外参依据） |

> 复制对账：diff -r 四包全等 + 8 关键件（package.xml/CMakeLists.txt）sha256 逐对一致，430 文件/8.7M；隔离扫描 /home/ysc 与 /home/test 双类 0 命中（95_simulation 全目录）。
