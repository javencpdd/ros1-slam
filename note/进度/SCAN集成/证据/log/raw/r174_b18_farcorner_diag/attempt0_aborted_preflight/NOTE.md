# attempt0 ＝ 作废窗（未起链，无任何被试读数）

2026-10-11 12:23 首跑在本件自身 source 段当场 abort：

    /opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh: line 3: ROS_DISTRO: unbound variable

成因＝本件用了 `set -u`，而 source 前一行 `unset ROS_DISTRO ...` 正是在册 w3_t1_probe2.sh 的写法；
在册件不带 `set -u`，所以同一段话在它那里无事、在我这里拦死 ⇒ **属施工侧自制的前置失败（坑18/坑40 家族），不是 perlin3d 远角对的任何读数**。

本目录三件是那次 abort 的**前置回执**（load、foxy PID 基线、样张 sha），保留以证明「作废的是窗、不是数据」。
后续正式窗的重取在上级目录同名件里。
