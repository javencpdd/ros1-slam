# current-2026-11-15 门禁读数说明

- 采集时间：2026-11-15 01:08 CST（一次性容器 `gate-once2`，容器内为 UTC，故 `gate-chain-2026-11-15.raw.txt` 头行显示 `2026-11-14 17:08`）。
- 运行方式：`ros1-slam:noetic-sim` 一次性容器，挂整仓（含 `.git`）到 `/opt/ros1-slam`，再挂 `docker_ros1_x86_sim_{build,devel,install}` 卷；`HOME=/tmp/gatehome`、`git safe.directory=/opt/ros1-slam`、source Noetic + `ws/devel`。
- 为什么不用 `ros1-slam-sim`：该容器没有 `.git`，GTC/账构在链内会因 `git log` 不可读而报 9；这是环境缺口，不是门禁通过。
- `gate-readings.txt` 为 11 件逐行摘要；完整原始输出见 `gate-chain-2026-11-15.raw.txt`。
- 判读：SYN/PYST 已因面清单改指 `subprojects` 恢复为 0；GTC 注册调用 0 但 `--check` 为 5；ISO/DC 为在册非零；VAR 为 INVALID；VCC 为在册基线 6。不得把本目录读数当作 formal/真机验收。
