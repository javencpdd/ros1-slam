# pcd_2_gridmap

> 最新更新时间：2026-11-06 15:17:09 CST。现行规划见仓库 `note/计划书v8.1/`；本文件描述配置，不替代运行验收。
transfrom .pcd file to ros grid map 

## run

1. set your .pcd file path in config.yaml 

2. set your parameters in map.launch (for grid map)

3. roslaunch build_octomap.launch

4. rosrun map_server map_saver (to save the grid map with topic "/map")
