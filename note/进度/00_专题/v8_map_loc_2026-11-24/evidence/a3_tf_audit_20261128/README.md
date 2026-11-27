# A3 TF 审计与隔离负控证据（2026-11-28）

## 结论

本切片完成审计器实现修正与可执行负控：

| 项 | 读数 | 判定 |
|---|---|---|
| 审计器纯负控 | 16/16 OK | PASS |
| 生产链正常 | `rslidar←base_link←body←camera_init←map` | PASS |
| 同边第二发布者 | `body->base_link` 命中 `/body_to_base_link` + `/rogue_body_base` | FAIL（符合预期） |
| 双父 | `base_link` 同时挂 `body` 与 `camera_init` | FAIL（符合预期） |
| 关闸 | `camera_init->body` stale 4.75 s、`/v8/local/localization_ok=false`、GT 仍正常 | FAIL（符合预期） |
| 缺生产 TF 边 | 关掉闸门节点后链只到 `body`、`no_root`、loc=false、GT 仍正常 | FAIL（符合预期） |
| 停钟/暂停物理 | loc=false；恢复物理与闸门后 loc 回 true | 符合预期 |
| FAR/感知静态 GT 消费面 | FAR 图唯一 GT 字样是 loc 的只读 `gt_topic`；FAR 适配器无 GT 订阅；感知 manifest/source 无 GT | 静态 PASS |
| GT TF 关闭 / GT 话题关闭 / 恢复不自动续跑 | **未在本切片执行** | 需全图任务窗，转 A5 |

## 审计器修正

`scripts/verify/tf_chain_audit.py`：

1. **时间口径分开**：`stale`/`chain_stale` 用 `time.monotonic()` 的接收墙钟；`future_stamp` 用 `rospy.Time.now()` 的 ROS 源时钟。不再拿 `time.time()` 与 ROS sim 戳混比。
2. **边级发布者**：从 `TFMessage._connection_header['callerid']` 取发布节点，写入每条边自己的 `publishers`。不再用 `/tf` 话题级多发布者推出边级多写。
3. **双写证据分级**：50 ms/xyz 压缩冲突只记为 `OBSERVE value_conflict`，不参与 FAIL；同边多个 callerid 才判 `duplicate_publisher`。
4. **停钟/源戳停滞**：动态边 `stamp_stalled`（源戳集合不前进）与 `stale`（monotonic 墙钟超龄）各判一条；静态边永不判超龄。
5. 新增负控：source-stamp stall + fresh wall、ROS/monotonic 分离、value conflict 只观察、边级 publisher 不串边。

## 定位闸门同时约束 TF 与导航位姿

- `gate_relay_node.py` 新增 `tf_parent/tf_child`。每转发一条 `/Odometry_gated`，同时从该消息发布 `camera_init→body`。
- 关闭闸门：不再转发位姿，也不再有新的生产 TF；loc 看门狗由墙钟判 `localization_ok=false`。
- `m2_mapping.launch` 把原始 LIO TF 帧改为 `lio_body_raw`，所以原始 LIO TF **不在** `map→…→rslidar` 生产链上，不能绕过闸门。
- 三张 M2 图均装配 `lio_gate`，参数 `tf_parent=camera_init`、`tf_child=body`；loc 消费 `/Odometry_gated`。
- manifest `v8.fw.lio_gate` 的 `tf_edges` 声明 `camera_init→body`。

正常活体证据（`missing_edge_and_loc.log`）：

```
camera_init->body publisher = /lio_gate_probe
lipo raw edge = camera_init->lio_body_raw (not in production chain)
loc = false when gate is off; GT gt_odom->gt_base still publishes
```

## 非法边负控

- `duplicate_publisher.log`：`/body_to_base_link` 与 `/rogue_body_base` 同写 `body->base_link` ⇒ `duplicate_publisher`，且仅作辅助的 `value_conflict` 也出现。
- `double_parent.log`：`/rogue_parent` 写 `camera_init->base_link` ⇒ `base_link` 双父 ⇒ `double_parent`。
- `missing_edge_and_loc.log`：kill `/lio_gate_probe` 后 `no_root`；loc=false；GT 分支仍可查。
- `stopped_clock_and_recovery.log`：`pause_physics` 后 loc=false；`unpause_physics` 后 loc=true；闸门重起后恢复。
- `gate_off_and_gt_normal.log`：审计采集中途关闸，动态边 stale + chain_stale 均落；loc=false；GT 正常。

## 复跑

```bash
# 隔离底座 + 生产闸门 + 定位探针
docker exec ros1-slam-sim bash -lc \
  'M2_ISOLATE_GT=1 bash /opt/ros1-slam/docker/sim.sh base m2'
docker exec -d ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; \
   exec roslaunch /opt/ros1-slam/scripts/verify/lio_gate_probe.launch'
docker exec -d ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; \
   exec roslaunch /opt/ros1-slam/scripts/verify/loc_tf_contract_probe.launch'

# 正常审计
docker exec ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; \
   python3 /opt/ros1-slam/scripts/verify/tf_chain_audit.py \
   --root map --target rslidar --gt-frames odom,gt_odom'

# 关闸 / 恢复
docker exec ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11313; \
   rosservice call /lio_gate_probe/set_enabled "data: false"'
docker exec ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11313; \
   rosservice call /lio_gate_probe/set_enabled "data: true"'
```

## 未闭合

- **GT TF 关闭**：当前 GT TF 仍由 Gazebo 插件直接发 `gt_odom→gt_base`，无法只关 GT TF 而不停 Gazebo；需 GT 评价域中继后在 A5 做独立关断。
- **GT 话题关闭**：同上，`/sim/odom_gt` 目前由插件直接发；需 raw GT + eval relay。
- **恢复不自动续跑**：需 BT/任务图运行，属 A5 任务链验收。
- **FAR/感知运行时不消费 GT**：本切片只做静态装配/源码证明；需 A5 全图运行做 `rosnode info` 与数据流证明。
- 回跳/冻结源戳的真实注入无法由当前 LIO 产生，已由纯逻辑负控覆盖；真实入口缺边/stale/双父/双写已实跑。
