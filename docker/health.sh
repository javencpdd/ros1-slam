#!/usr/bin/env bash
# =============================================================================
# 长跑体检：在"跑之前"和"跑之后"各跑一次，对比差值就知道什么东西在涨。
#
#   bash docker/health.sh            # 打印当前基线
#   bash docker/health.sh --save b1  # 存成基线 b1（写 logs/health_b1.txt）
#   bash docker/health.sh --diff b1  # 与基线 b1 对比，报告增长项
#
# 为什么需要它：静态代码扫描（复杂度/重复）看不出资源累积。本项目实测过三类
# 长跑垃圾：① 3.4G 的 core dump（10-16 崩溃遗留）② rviz 的 Decay 设成近乎永久
# 导致 22 分钟吃 1.2G ③ 点云累积器 dict 无上限。这三样都不会报错，只会几小时后
# 把容器 OOM 掉或撑满磁盘。
# =============================================================================
set -uo pipefail

C="docker compose -f docker/docker-compose.yml --env-file docker/.env"
# ★基线落家目录，不落 logs/★：宿主 logs/ 属主可能是 nobody（容器曾以 root 写过，
# 经 uid 映射后落到 nobody），非 root 的 shell 会 Permission denied。
OUT_DIR="${HEALTH_DIR:-$HOME/.cache/ros1slam-health}"
SVC="sim-viz"
MODE="${1:-}"; ARG="${2:-}"

probe() {
    echo "### snapshot $(date -Is)"
    echo
    echo "## 磁盘（宿主侧）"
    for d in .ros logs data; do
        [ -e "$d" ] && printf "  %-8s %s\n" "$d" "$(du -sh "$d" 2>/dev/null | cut -f1)"
    done
    echo
    echo "## 容器内（$SVC）"
    $C exec -T "$SVC" bash -lc '
        printf "  %-10s %s\n" ".ros"   "$(du -sh /opt/ros1-slam/.ros 2>/dev/null | cut -f1)"
        printf "  %-10s %s\n" "logs"   "$(du -sh /opt/ros1-slam/logs 2>/dev/null | cut -f1)"
        printf "  %-10s %s\n" "/tmp"   "$(du -sh /tmp 2>/dev/null | cut -f1)"
        printf "  %-10s %s\n" "ulimit" "$(ulimit -Sn)/$(ulimit -Hn)"
        printf "  %-10s %s\n" "僵尸"   "$(ps -eo stat | grep -c "^Z" || true)"
        echo
        echo "## core dump"
        n=$(find /opt/ros1-slam/.ros/core-samples -type f 2>/dev/null | wc -l)
        if [ "$n" -gt 0 ]; then
            echo "  ★发现 $n 个 core 文件（长跑前应清理）:"
            find /opt/ros1-slam/.ros/core-samples -type f -size +10M 2>/dev/null \
                -exec ls -lh {} \; | awk "{print \"    \"\$5, \$9}" | head -5
        else
            echo "  无"
        fi
        echo
        echo "## 关键进程 RSS(MB) / fd"
        # ★取 RSS 最大的那个 PID★：pgrep -f 会同时命中 /bin/sh 包装层与我的探测 shell
        # （实测 gzserver 报成 2MB/fd=4，那是 bash 不是真身）。真身的 RSS 一定最大。
        for pat in gzserver gzclient run_mapping_online "rviz$" cloud_accumulator; do
            best=""; bestrss=0
            for p in $(pgrep -f "$pat" 2>/dev/null); do
                r=$(ps -o rss= -p "$p" 2>/dev/null | tr -d " ")
                [ -n "$r" ] && [ "$r" -gt "$bestrss" ] && { best="$p"; bestrss="$r"; }
            done
            [ -n "$best" ] && printf "  %-20s %6.0f MB  fd=%s\n" "$pat" \
                "$(echo "$bestrss" | awk "{print \$1/1024}")" "$(ls /proc/$best/fd 2>/dev/null | wc -l)"
        done
    ' 2>/dev/null
    echo
    echo "## 容器日志 rotation（运行时真值）"
    # ★用 docker inspect 的 HostConfig.LogConfig★：
    #   - 不去 du json.log：/var/lib/docker/containers/ 要 root，非 root 下恒假；
    #   - 不 grep `docker compose config`：展开后的 YAML 结构会骗过简单正则
    #     （实测把 7 个都有 rotation 的服务误判成 4 个）。
    bad=""
    for s in roscore slam slam-host rosbridge sim sim-viz web; do
        cid=$($C ps -q "$s" 2>/dev/null | head -1)
        [ -z "$cid" ] && continue
        ms=$(docker inspect --format='{{index .HostConfig.LogConfig.Config "max-size"}}' "$cid" 2>/dev/null)
        mf=$(docker inspect --format='{{index .HostConfig.LogConfig.Config "max-file"}}' "$cid" 2>/dev/null)
        if [ -z "$ms" ]; then bad="$bad $s"; fi
    done
    if [ -z "$bad" ]; then
        echo "  ✓ 全部服务均受 rotation 约束（max-size=$ms / max-file=$mf）"
        echo "    ⇒ 单容器日志上限 ≈ 10m × 3 = 30MB"
    else
        echo "  ★以下服务未配 rotation（日志会无限涨）:$bad ★"
    fi
}

case "$MODE" in
    --save)
        [ -z "$ARG" ] && { echo "用法: $0 --save <名称>" >&2; exit 2; }
        mkdir -p "$OUT_DIR" && probe > "$OUT_DIR/health_$ARG.txt" || { echo "写入失败: $OUT_DIR" >&2; exit 2; }
        echo "基线已存: $OUT_DIR/health_$ARG.txt" ;;
    --diff)
        [ -z "$ARG" ] && { echo "用法: $0 --diff <名称>" >&2; exit 2; }
        f="$OUT_DIR/health_$ARG.txt"
        [ -f "$f" ] || { echo "找不到基线: $f" >&2; exit 2; }
        probe > /tmp/health_now.$$.txt
        # ★不用进程替换 <(...)★：本机 bash 不支持（实测 diff: /dev/fd/63 没有那个文件），
        # 一律走临时文件。
        A=/tmp/hb_old.$$; B=/tmp/hb_new.$$
        sed 's/[0-9.]\+[KMG]//g' "$f" | grep -v snapshot > "$A"
        sed 's/[0-9.]\+[KMG]//g' /tmp/health_now.$$.txt | grep -v snapshot > "$B"
        echo "=== 与基线 $ARG 的结构差异 ==="
        diff "$A" "$B" | grep "^[<>]" | head -40 || echo "  （无结构变化）"
        echo
        echo "=== 数值变化 ==="
        paste "$A" "$B" | awk -F'\t' '$1!=$2 {printf "  基线: %-42s 现在: %s\n", $1, $2}' | head -20
        rm -f "$A" "$B" /tmp/health_now.$$.txt ;;
    *)
        probe ;;
esac
