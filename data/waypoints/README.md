# 航点数据目录

> 最新更新时间：2026-11-05 11:49:54 CST。本次按实际读写代码核对，未录制航点、未启动任务。

本目录目前只有本 README，没有航点 JSON。它是预留的数据目录，**当前 `pipeline_tracking` 不会自动从这里读写**。

| 入口 | 实际路径与行为 |
| --- | --- |
| [录制工具](../../ws/src/50_task/pipeline_tracking/tools/location_record.py) | PyQt5 界面读取 `map → base_link` TF，把 `<order>.json` 写入包内 `pipeline_tracking/data/` |
| [任务加载器](../../ws/src/50_task/pipeline_tracking/scripts/Task.py) | `LoadTaskpoints()` 遍历同一个包内 `data/`，按 `order` 排序；没有可用文件时抛异常 |
| [任务转移器](../../ws/src/50_task/pipeline_tracking/scripts/TaskTransfer.py) | 消费已解析的任务点，向 MBF action 发送目标；不负责发现本目录 |

旧 README 所述“录制后落在本目录并由 TaskTransfer 直接消费”与源码不符。本次只更正文档，不改第三方任务包的数据路径。

录制 JSON 的主要字段为 `order`、`robot_pose`（`pos_x/pos_y/pos_z`、`ori_x/ori_y/ori_z/ori_w`）及 `option`（三类步态布尔值）；准确格式以录制工具和 [TaskPoint](../../ws/src/50_task/pipeline_tracking/scripts/TaskPoint.py) 为准。包内加载器不会按扩展名筛选文件，说明文件不应混入它的 `data/`。

v8 应用使用另一套 task 契约与 map registry，入口见 [v8 平台](../../ws/src/90_platform/v8_platform/README.md)。本目录是否有 JSON 不表示地图、定位或真机任务已验收；当前地图准备和真机排程以[执行队列](../../note/计划书v8/23_v8任务队列.md)为准。
