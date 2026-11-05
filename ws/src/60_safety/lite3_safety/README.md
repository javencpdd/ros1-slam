# lite3_safety · 传统 Twist 仲裁

> 最新核对时间：2026-11-05 11:49:54 CST。现行规划见仓库[计划书 v8](../../../../note/计划书v8/README.md)，能力状态见[进度入口](../../../../note/进度/README.md)。

本包的 `safe_cmd_vel_mux.py` 是传统 ROS 链的 `/cmd_vel` 仲裁输出。
启动入口为 `launch/60_safety.launch`，全部源表与限速规则在 `config/cmd_vel_mux.yaml`。

当前七个源按数字从小到大选优：`estop`、`teleop_joy`、`teleop_app`、`task`、
`navigation`（`/cmd_vel_raw`）、`demo`、`scan_planner`（`/scan_planner/cmd_vel`）。
源超时淘汰；无可用源或 `/lite3/supervisor/hold=true` 时输出零速，默认输出频率 20 Hz。

当前配置还包含静态线性/角速度上限，以及 `/v8/safety/speed_cap` 动态限速。
`speed_cap_required=true` 时，cap 缺失/过期退到 `blind_cap_mps`，当前值 0.1 m/s；
它是限速降级，不是“感知失效一定立即停车”的证明。具体参数以 YAML 为准。

纯速度约束函数在 `scripts/velocity_limit.py`，源/仲裁验证入口为 `verify/v_mux_sources.sh`。
本包与 v8 的 3D `cmd3d_mux` 分工不同，不能把两个输出类型或安全证据混用。
任何运动链都须保持最终 `/cmd_vel` 单一写者；本 README 不授权启动真机运动。
