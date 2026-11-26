# Graph 与传统装配一致性（2026-11-27）

## graph_3d_m1_hdl_bt_far.yaml 渲染结果
name="bt"
name="carrier"
name="loc"
name="local"
name="map"
name="mux3d"
name="path_goal"
name="perception"

## include
<include file="$(find v8_platform)/launch/far_stack.launch"

## 传统装配 far_stack.launch 的节点
name="far_planner"
name="v8_global_far"

结论：Graph = 8 个 v8 组件 + include(far_stack.launch: far_planner/v8_global_far) = 10 个节点；
传统装配 far_stack.launch(2) + 同一批 8 个 v8 组件 = 10，节点名一一对应，无额外/缺失节点。
