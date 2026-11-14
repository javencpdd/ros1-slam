# ros1-slam-console · P3 建图/导航可视化子项目

> 2026-11-14 由同级仓 `c1848e3` 以 `git subtree` 并入 [ros1-slam](https://local/ros1-slam) 主仓的 `subprojects/console/`（历史保留）。
> host-only 只读任务台：agent `:8780` + UI，白名单启停栈、只读订阅 ROS；**无 ROS 写入、无 P1/P2 源码 import**。

## 布局

```
console/agent.py          # HTTP :8780，白名单栈、whoami、audit
console/ui/               # console.html + assets（地图库/世界/体素）
console/stacks.json       # 12 栈白名单
console/world_registry.json
console/voxel_stream.py   # /cloud_registered → data/console/voxels
console/ops/              # build_maps_index.py / build_world_scenes.py（生成 ui/assets）
console/test_*.py|js      # 契约/体素/任务事件/BT 配置
```

## 环境根（默认同级 P1 仓）

| 变量 | 默认 | 用途 |
| --- | --- | --- |
| `V8_REPO_ROOT` | `../..` | P1 主仓根：`data/`、`docker/foxglove`、P1 `ws` |
| `V8_SIM_ROOT` | `../simulation` | P2 子项目根（世界/场景） |
| `V8_CONTAINER_ROOT` | `/opt/ros1-slam` | 容器内根（`ws/devel`、`sim_ws/devel`、`data`） |
| `V8_CONSOLE_DATA` | `$V8_REPO_ROOT/data/console` | 体素/栈运行数据 |
| `V8_CONSOLE_RUN` | `/tmp/v8-console` | pid/log/备份 |

## 运行 / 测试 / 发布

```bash
./console/start_agent.sh          # :8780
python3 console/test_agent_contract.py
python3 console/test_voxel_stream.py
python3 console/test_bt_config.py
node console/test_mission_events.js
make release                      # dist/ros1-slam-console-<version>.tar.gz + manifest.json
```

## 来源

- 2026-11-14 由同级仓 `c1848e3` 以 `git subtree` 并入主仓；更早来源 `ros1-slam@2d0c32a5` 的 `scripts/console/` + `scripts/ops/{build_maps_index,build_world_scenes}.py`。
- 历史随 subtree 进入主仓；主仓 `origin` 是唯一远端。
