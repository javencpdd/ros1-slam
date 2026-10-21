# attempt3 ＝ 作废窗（探针又调了库里不存在的方法，被试读数未取）

崩溃点：`AttributeError: 'SimpleActionClient' object has no attribute 'wait_for_state'`
（在 run_once 的取消路径里，line 90）⇒ 尝试① 30s 未回话后进入 cancel 分支，随即炸在**我发明的方法名**上。
`grep def` 于 `/opt/ros/noetic/lib/python3/dist-packages/actionlib/simple_action_client.py` 实证：
该类只有 `wait_for_server(66)／send_goal(83)／wait_for_result(124)／get_state(162)／cancel_goal(203)／get_result(148)`，
**没有 `wait_for_state`**（那是 `ActionClient` 一系的接口，我记串了类）。
修法＝按存在的 `get_state()` 轮询到终态；并把本轮所有用到的 API 逐个回库核对（已做完，全对上）。
红线面照旧干净：样张 sha 起前＝退后一致、foxy 4 PID 逐位一致、teardown 零残留、11313 零监听。
