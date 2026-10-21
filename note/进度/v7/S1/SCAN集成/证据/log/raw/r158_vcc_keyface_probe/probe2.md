# probe2

D 句（裸形、指针后紧跟键名）：以全局柄发 cmd_vel，经 70_scan.launch:61 remap 落到 /scan_planner/cmd_vel。

E 句（裸形、指针后无键名）：以全局柄发 cmd_vel，经 70_scan.launch:61 落到 /scan_planner/cmd_vel。

F 句（反引号、指针后紧跟键名）：经 `70_scan.launch:61` remap 落到 /scan_planner/cmd_vel。
