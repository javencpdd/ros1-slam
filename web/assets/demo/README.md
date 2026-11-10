# Lite3 AI 策略 MuJoCo 演示（宿主）

交互式 MuJoCo 窗口：12 个厂商 ONNX 策略 + 5 段厂商 CSV 参考动作；地形为 50 m 楼梯（25 层×3 级）与 50 m 坡面（15—50°）。

## 启动

```bash
# 仓库根目录
bash web/assets/demo/start_lite3_mujoco_demo.sh --check   # 检查依赖与资产
bash web/assets/demo/start_lite3_mujoco_demo.sh           # 后台启动
bash web/assets/demo/start_lite3_mujoco_demo.sh --stop    # 停止
```

资产默认在仓外：`~/.cache/lite3-policy-staging-20261109`；venv 默认 `~/.cache/mujoco-demo-venv`。可覆盖：

```bash
LITE3_DEMO_ROOT=/path/to/staging \
LITE3_DEMO_PY=/path/to/venv/bin/python \
LITE3_DEMO_DISPLAY=:1 \
bash web/assets/demo/start_lite3_mujoco_demo.sh
```

## 操作

- 地形下拉框：平地 / 楼梯 5/10/15/18/20 cm / 斜坡 15—50°。
- 策略按钮：A-270 行走、A-270 跟随（已验证站立/行走）；B/C 10 个（实验）。
- 滑杆：vx / vy / wz，以及 B/C 的 uncertain / reflect；B/C 另有逐维命令向量编辑器。
- 键盘（3D 窗口或面板，含小键盘）：`8` 前进、`2` 后退、`4` 左移、`6` 右移、`7/9` 左转/右转、`5/0` 停车；每按一次 ±0.1，连按累加。数字键不与 MuJoCo 查看器的快捷键（Space、+/-、方向键、[]、Esc、F1–F5）冲突。
- 参考动作：厂商 CSV（前跳/高跳/前空翻/后空翻/侧空翻），按 CSV 记录的基座轨迹回放；播放完停在末帧，点「停止参考动作」回策略。
- 导入策略：把 `.onnx` 放进 `demo_policies/` 或登记到 `demo_policies.json`，点「刷新策略」。

## 依赖（宿主，不进容器）

- 宿主 X（默认 `DISPLAY=:1`）。
- Python 3.10 venv：`mujoco 2.3.7`、`onnxruntime 1.16.3`、`numpy 1.24.4`、`tkinter`、`Pillow`。
- 资产：`policies/*.onnx`（12 个有效策略）、`mjcf/Lite3.xml`＋`meshes/`、`csv/` 5 段厂商动作。

## 边界

- A-270 行走验证了站立/平地行走（R883 基线）；0.20 m 三级台阶的 clean/stable 只是交互演示观察，不是 formal。跟随是独立网络，厂商文档未给出「策略文件 ↔ App 模式」映射，人物跟随语义属推断；walk 判据通过不等于跟随能力验收。
- B/C 10 个是实验策略：零命令是蹲伏，命令向量语义为推断，且没有楼梯训练证据。
- CSV 回放是含基座轨迹的开环参考，不是策略能力、不是真机；高跳末帧按 CSV 记录的 1.2 m 落点（1 m 高台），回放时高台会显示在落点。
- 厂商三挡越障是高踏步 TVMPC 控制器（libdeepras/deeprcs），不是 ONNX，也没有 sim 后端；本演示不覆盖。
- 不是 formal、不是 Gazebo 运行面、不是真机。
