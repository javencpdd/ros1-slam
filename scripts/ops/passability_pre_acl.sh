#!/usr/bin/env bash
# passability_pre_acl.sh —— C7 选「运维也标」时的属主侧前置（小明 2026-10-09；口径照 b3_pre_chown.sh 的甲案）
#
# 为什么需要它：本仓目录属主是 ros1-slam，施工侧以 ysc 身份写文件，而目录默认 ACL 只给了
#   default:user:ysc:rw- ⇒ **我新建的源标注文件，ros1-slam/运维改不了**（实测 touch 出来是 -rw-rw---- ysc:ysc，
#   且我试图 setfacl -m d:u:ros1-slam:rw- 被拒 Operation not permitted——只有目录属主或 root 能改 ACL）。
#   反向是通的（运维建的文件带 user:ysc，我能改）。这个不对称最坏的演化是：现场改不动 ⇒ 私自另存一份
#   ⇒ 两处定义（02 坑43）。本工单就是把「双方都可写」一次配好。
#
# 谁跑：改 ACL 需要目录属主或 root ⇒ **由主理人执行**；本件默认只 dry-run，绝不自己动（甲案：本仓不代跑 chown/sudo）。
#
#   bash scripts/ops/passability_pre_acl.sh              # 只列现状与将要执行的命令（零改动）
#   sudo bash scripts/ops/passability_pre_acl.sh --apply # 落默认 ACL：双方都可写
#   bash scripts/ops/passability_pre_acl.sh --verify     # 复验：默认 ACL 到位 ＋ 双向写测试
set -u
ROOT=/home/ros1-slam
DIRS=("$ROOT/ws/src/95_simulation/config/passability" "$ROOT/note/进度/仿真测试台")
# 目录必须给 rwx、文件只给 rw-：**目录少了 x 就是"能列名字但不能进入/stat"**，
#   而 setfacl 会按新条目重算 mask ⇒ 只写 rw- 的递归工单在 mode 700 的子目录上等于没落
#   （2026-10-10 主理人实跑 --apply 后 note/进度/仿真测试台/{设计稿,证据} 仍 Permission denied，即此因）。
CMD='find <目录> -type d -exec setfacl -m d:u:ros1-slam:rwx,d:u:ysc:rwx,u:ros1-slam:rwx,u:ysc:rwx {} + ; find <目录> -type f -exec setfacl -m u:ros1-slam:rw-,u:ysc:rw- {} + ; setfacl -d -m u:ros1-slam:rwx,u:ysc:rwx <目录>'

echo "=== 现状（谁可写）==="
for d in "${DIRS[@]}"; do
  printf '%s\n  属主 %s\n  默认ACL %s\n' "$d" "$(stat -c '%U:%G' "$d" 2>/dev/null || echo 缺目录)" \
    "$(getfacl -p "$d" 2>/dev/null | grep '^default:user' | tr '\n' ' ')"
done
echo
echo "=== 要落的东西（root 权限，施工侧不代跑）==="
for d in "${DIRS[@]}"; do echo "  ${CMD//<目录>/$d}"; done
echo
case "${1:-}" in
  --apply)
    [ "$(id -un)" = root ] || { echo "【拒跑】改 ACL 需 root（当前 $(id -un)）⇒ 请用 sudo bash $0 --apply"; exit 2; }
    N=0
    for d in "${DIRS[@]}"; do
      [ -d "$d" ] || { echo "  跳过缺目录 $d"; continue; }
      find "$d" -type d -exec setfacl -m d:u:ros1-slam:rwx,d:u:ysc:rwx,u:ros1-slam:rwx,u:ysc:rwx {} + \
        && find "$d" -type f -exec setfacl -m u:ros1-slam:rw-,u:ysc:rw- {} + \
        && setfacl -d -m u:ros1-slam:rwx,u:ysc:rwx "$d" && N=$((N+1))
    done
    echo "已对 $N 个目录落 ACL。注意：这只覆盖**执行时刻已存在**的文件；新建文件靠 default ACL 自动继承。"
    ;;
  --verify)
    OK=1
    for d in "${DIRS[@]}"; do
      [ -d "$d" ] || { echo "  [缺] $d"; OK=0; continue; }
      g=$(getfacl -p "$d" 2>/dev/null | grep -c '^default:user:ros1-slam:rwx')
      # 子目录可进入性也要验：只看顶层会给出假绿（本次就是这么漏的）
      for sub in "$d"/*/; do
        [ -d "$sub" ] || continue
        getfacl -p "$sub" >/dev/null 2>&1 || { echo "  [进不去] $sub ⇒ 目录缺 x，需重跑 --apply"; OK=0; }
      done
      echo "  $d 默认ACL 含 ros1-slam:$g"
      [ "$g" = "1" ] || OK=0
      t="$d/.acl_probe.$$"
      # 假阴教训（2026-10-10 实跑）：`su` 只有 root 能免密切用户 ⇒ 以 ysc 跑 --verify 时 su 必失败，
      # 旧写法把"切不了用户"报成"不可写"＝**方法自身造出假红**。现分两路：root 跑才做真写测试，
      # 非 root 只做**静态有效权限核对**（getfacl 的 #effective 标记），并明写"对方可写性需 root 或对方会话验"。
      if [ "$(id -un)" = root ]; then
        if (set -e; : > "$t" && su -s /bin/bash ros1-slam -c "test -w '$t' && echo '    ros1-slam 可写：是（实测）'" && rm -f "$t") 2>/dev/null; then :
        else echo "    ros1-slam 可写：否（实测）"; OK=0; fi
      else
        eff=$(getfacl -p "$d" 2>/dev/null | grep '^user:ros1-slam' | head -1)
        case "$eff" in
          *rwx*) echo "    ros1-slam 条目：$eff ⇒ 静态核对通过（真写测试需 root 跑本件）";;
          *)     echo "    ros1-slam 条目：${eff:-缺} ⇒ 未拿到 rwx，需重跑 --apply"; OK=0;;
        esac
        touch "$t" 2>/dev/null && { echo "    ysc 侧写测试：可写"; rm -f "$t"; } || { echo "    ysc 侧写测试：不可写"; OK=0; }
      fi
    done
    [ "$OK" = 1 ] && echo "PASSABILITY ACL VERIFY: PASS（双方可写，运维可自助改源标注）" || { echo "PASSABILITY ACL VERIFY: 未到位"; exit 5; }
    ;;
  *) echo "（dry-run，未做任何改动。确认无误后： sudo bash scripts/ops/passability_pre_acl.sh --apply）" ;;
esac
