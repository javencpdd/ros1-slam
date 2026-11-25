#!/usr/bin/env bash
# Xvfb 存活守卫（被 docker/sim.sh source；可单独测试）。
#
# 为什么要有它（2026-11-25 实测事故）：`sim.sh` 原先只在 `DISPLAY` 为空时才补起 Xvfb。
# 镜像里 `DISPLAY=:99` 已设、但 Xvfb 进程已死（只剩 /tmp/.X11-unix/X99 与 /tmp/.X99-lock 残留），
# 兜底被跳过 ⇒ gzserver 打不开显示 ⇒ 模型 spawn 失败、传感器不创建 ⇒ `/livox/imu` 连话题都不出现，
# 表现为"冒烟时好时坏"，且极易被误判成 URDF / 插件 / 包路径问题（这几条我都排查过，都不是）。
#
# 判据：**以显示服务是否真的应答为准**，不以环境变量是否为空为准。
#   1) /tmp/.X11-unix/X<N> 必须存在（socket）；
#   2) 优先用 `xdpyinfo` 实答；没有 xdpyinfo 时退化为"存在服务该 DISPLAY 的 Xvfb 进程"。
# 只有两条都过才算存活；否则清残留锁并新起一个。

xvfb_display_num() {  # ":99" -> "99"
  local d="${1#:}"
  printf '%s' "${d%%.*}"
}

xvfb_display_alive() {  # $1=display(:99)
  local d="${1:-:99}" n
  n=$(xvfb_display_num "$d")
  [ -S "/tmp/.X11-unix/X${n}" ] || return 1
  if command -v xdpyinfo >/dev/null 2>&1; then
    timeout 3 xdpyinfo -display "$d" >/dev/null 2>&1 || return 1
    return 0
  fi
  pgrep -f "Xvfb ${d}( |$)" >/dev/null 2>&1 || pgrep -x Xvfb >/dev/null 2>&1
}

xvfb_guard_ensure() {  # $1=display(:99)，成功则 export DISPLAY
  local d="${1:-${DISPLAY:-:99}}" n
  n=$(xvfb_display_num "$d")
  if xvfb_display_alive "$d"; then
    echo "[xvfb] ${d} 存活，复用"
    export DISPLAY="$d"
    return 0
  fi
  # 到这里说明"环境变量有值但服务不应答"或"根本没起"：先清残留锁再起，否则 Xvfb 会拒绝启动。
  rm -f "/tmp/.X${n}-lock" "/tmp/.X11-unix/X${n}" 2>/dev/null || true
  command -v Xvfb >/dev/null 2>&1 || { echo "[xvfb] 无 Xvfb 可执行文件，无法兜底" >&2; return 1; }
  echo "[xvfb] ${d} 不等于存活（进程已死或残留锁），重新起一个"
  Xvfb "$d" -screen 0 1280x1024x24 -nolisten tcp >/tmp/xvfb-guard.log 2>&1 &
  local i
  for i in $(seq 1 20); do
    sleep 0.5
    if xvfb_display_alive "$d"; then
      export DISPLAY="$d"
      echo "[xvfb] ${d} 已就绪"
      return 0
    fi
  done
  echo "[xvfb] ${d} 启动后仍不应答，见 /tmp/xvfb-guard.log" >&2
  return 1
}
