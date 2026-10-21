# E. 风险清单（接矩阵 v2 F1–F7，续编 F8–F12，v1.0）

| 编号 | 风险 | 实证/机理 | 缓解 |
|---|---|---|---|
| **F8** | **SCAN ESDF 质量**：点云稀疏（仿真雷达降采样/真机 Mid360）→ 占据图漏障或膨胀失真 → 轨迹穿墙 | SCAN grid_map 依赖 /cloud_registered_body 增量建图；smoke.sh 闭环是 SCAN 自带稠密仿真点云，换真传感器链未验证 | A7 P1 感知入先行：先静态核对占据图与 world 一致再放行 P2/P3；map 分辨率/膨胀参数预调并外化壳包 yaml |
| **F9** | **CPU 预算叠加**：平台 RTF 0.288–0.290 已贴 0.3 门，faster_lio+SCAN+teb 同机叠加可能击穿 | 矩阵 F3/F6 实证 | SCAN 与 teb 不同时使能（mux 互斥）；SCAN 档载前跑 RTF 增载探针（同 0.3 门）；批量验证串行 |
| **F10** | **跟踪精度**：closed_loop_controller 参数按 Go2 调，Lite3 planar_move 下速度/角速度响应不同 → 跟踪发散 | .bak-maxvel075/vy050 调参史佐证参数敏感 | max_vel/vy 首跑降至 0.5/0.3 保守值；dev>0.35m 即触发 T2 pure pursuit 备选（预登记不预施工） |
| **F11** | **依赖链断裂**：/home/test 只读，SCAN 源不可改；源内残留 Go2/ROS2/depth 逻辑 | 实证：go2_kinematic_sim、transfer_ros2 注释、depth 外参分支 | 双目录制：scan_planner 五包原样拷 30_3rd；一切适配进 lite3_scan 壳包（launch remap/参数外化/relay 脚本）；先 /tmp 试编译预检 |
| **F12** | **优先级冲突**：用户可能把真机链（9/11/12）排在 SCAN 仿真（3）前 | C 文可调整项标注 | 已标 9↔2/3 可换序、7/8 可后移；2（M5.1，C12 既定门）建议保位；换序由主理人裁决后更新 §16.3 |
