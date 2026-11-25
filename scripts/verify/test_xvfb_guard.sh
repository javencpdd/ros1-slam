#!/usr/bin/env bash
# Xvfb 守卫的三连自测：正常（复用）/ 已死（重起）/ 残留锁（清锁后重起），外加一条反控。
#
# 为什么要三连：这条守卫的失效方式很隐蔽——"看着在跑，其实显示服务不应答"，
# 后果不是报错而是 gzserver 静默起不来（模型不 spawn ⇒ /livox/imu 连话题都没有）。
# 所以必须证明：①活着的时候不会多起一个；②死了的时候真的会重起；③残留锁不会挡住重起；
# ④反控——杀掉服务后判据必须翻转成"不存活"（否则这判据恒真，等于没测）。
set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
GUARD="${GUARD:-$HERE/../../docker/xvfb_guard.sh}"
# shellcheck source=/dev/null
source "$GUARD"

pass=0; fail=0
ok()   { echo "[PASS] $*"; pass=$((pass+1)); }
bad()  { echo "[FAIL] $*"; fail=$((fail+1)); }
cleanup() { pkill -f "Xvfb :9[123]" 2>/dev/null; rm -f /tmp/.X9[123]-lock /tmp/.X11-unix/X9[123] 2>/dev/null; }
trap cleanup EXIT
cleanup; sleep 1

# ---------- 用例 1：正常存活 ⇒ 复用，不多起 ----------
Xvfb :91 -screen 0 320x240x24 -nolisten tcp >/dev/null 2>&1 &
sleep 2
before=$(pgrep -fc "Xvfb :91" || true)
DISPLAY=:91 xvfb_guard_ensure :91 >/tmp/g1.log 2>&1; rc=$?
after=$(pgrep -fc "Xvfb :91" || true)
if [ "$rc" = "0" ] && [ "$before" = "1" ] && [ "$after" = "1" ]; then
  ok "正常存活：复用现有 :91（进程数 1→1）"
else
  bad "正常存活被误判（rc=$rc，进程 $before→$after）"
fi

# ---------- 用例 2：进程已死（DISPLAY 仍指向它）⇒ 重起 ----------
pkill -f "Xvfb :92" 2>/dev/null
Xvfb :92 -screen 0 320x240x24 -nolisten tcp >/dev/null 2>&1 &
sleep 2
pkill -f "Xvfb :92"; sleep 1
alive_after_kill=0; xvfb_display_alive :92 && alive_after_kill=1
DISPLAY=:92 xvfb_guard_ensure :92 >/tmp/g2.log 2>&1; rc=$?
cnt=$(pgrep -fc "Xvfb :92" || true)
if [ "$alive_after_kill" = "0" ] && [ "$rc" = "0" ] && [ "$cnt" = "1" ]; then
  ok "已死重起：判据先翻转，再起且只起 1 个"
else
  bad "已死场景处理错误（翻转=$alive_after_kill rc=$rc 进程=$cnt）"
fi

# ---------- 用例 3：残留锁（无进程）⇒ 清锁并起 ----------
pkill -f "Xvfb :93" 2>/dev/null; sleep 1
rm -f /tmp/.X93-lock; echo "999999" > /tmp/.X93-lock   # 伪造残留锁
DISPLAY=:93 xvfb_guard_ensure :93 >/tmp/g3.log 2>&1; rc=$?
cnt=$(pgrep -fc "Xvfb :93" || true)
if [ "$rc" = "0" ] && [ "$cnt" = "1" ]; then
  ok "残留锁：已清锁并起 1 个（未被锁挡住）"
else
  bad "残留锁场景失败（rc=$rc 进程=$cnt）"; cat /tmp/g3.log
fi

# ---------- 反控：杀掉服务后判据必须翻转 ----------
pkill -f "Xvfb :93"; sleep 1
if xvfb_display_alive :93; then
  bad "反控失败：服务已杀，判据仍报存活（判据恒真＝没测）"
else
  ok "反控成立：服务被杀后判据翻转为不存活"
fi

echo "---- 结果：$pass PASS / $fail FAIL ----"
[ "$fail" = "0" ] || exit 1
