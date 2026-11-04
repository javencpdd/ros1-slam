# pcd_2_gridmap

> 更新时间：2026-11-04（v8 现行口径校对）｜ 权威口径：计划书 v8（note/计划书v8/）；本文件如与 v8 冲突，以 v8 为准
transfrom .pcd file to ros grid map 

## run

1. set your .pcd file path in config.yaml 

2. set your parameters in map.launch (for grid map)

3. roslaunch build_octomap.launch

4. rosrun map_server map_saver (to save the grid map with topic "/map")