# Foxglove 真实显示复验（2026-11-28）

## 结论

在本轮 T2/T3 改动后的 M2 图上，Foxglove/rosbridge 数据面 **PASS**：

- `HANDSHAKE OK`
- `TOPICS n=64 m2_ok=True`（含 `/v8/local/pose`、`/v8/bt/status`、`/v8/local/status`）
- `DATA OK n=3 topic=/v8/local/pose`
- `TF(local ROS) map->base_link OK`
- `TF(ws client) map->base_link OK edges=7`
- `VERDICT PASS`

## 关键修正

`sim` 与 `sim-viz` 是两个网络命名空间。桥在 `sim-viz` 内若用
`ROS_MASTER_URI=127.0.0.1:11313`，连到的是 sim-viz 自己的 master（4 条桥自身话题），
不是仿真的 11313；必须用 `ros1-slam-sim` 的容器 IP。实测：

| 目标 | 话题数 | 结果 |
|---|---|---|
| `ws://<viz-ip>:9090`（桥连 127.0.0.1） | 4 | FAIL（域不对） |
| `ws://<viz-ip>:9091`（桥连 `<sim-ip>:11313`） | 64 | PASS |

同时修了 `foxglove_ws_check.py` 的关闭期异常：`Client.__init__` 未调用
`super().__init__()`，autobahn 关闭时取 `is_closed` 会抛 AttributeError；
这是探针自身缺陷，不是连接失败。

## 文件

- `ws_check.log`：本次 PASS 的完整输出
- `layouts_check.txt`：`docker/foxglove/*.json` 均 JSON 可解析（不是客户端导入兼容性证明）
- `bridge_nodes.txt` / `topic_count.txt`：桥节点与 64 话题
- `nav_graph/`：9/9 装配

## 边界

- 只读观察；未发 goal/initialpose/cmd_vel。
- 客户端导入导出兼容性仍需实际客户端操作；本次只证明 JSON 可解析与数据面通。
- 性能开销未单独测量；不改变正式判据。
