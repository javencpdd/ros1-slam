#!/usr/bin/env bash
# x86_link_driver.sh —— R356 联调批驱动（x86 侧，host 视角；容器内经 docker exec）
# 到签依据＝真人 2026-10-18 R356：①格③补测窗按联调跑（读数/日志/bag 三样存证、自称旁证，
#   窗号登记与转正随批量走）＋预检变体跑通即视为预检 rc=0；③按 au §5.1/§6.2 出 A1 等价基准件。
# ★纪律：预检变体 rc≠0 ⇒ 不起窗（当场拒窗、不改判据、不绕门槛）；本件不 kill 非本件进程。
set -u
REPO=/home/jack/ysc-code/ros1-slam
BATCH=$REPO/note/进度/SCAN集成/证据/log/raw/r356_x86联调批
WIN=$BATCH/x2_cell3_window
A1=$BATCH/a1_baseline
PF=$BATCH/preflight_variant
CT=ros1-slam-sim
CBATCH=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r356_x86联调批
CWIN=$CBATCH/x2_cell3_window
say(){ echo "[$(date '+%F %T')] $*"; }
mkdir -p "$WIN" "$A1" "$PF"
say "== R356 联调批驱动开跑｜BATCH=$BATCH｜身份=$(whoami) =="

# ================= A1 起前（host 侧；au §6.2 五断言）=================
{
  echo "# assert1 六 unit systemd 态＋覆盖面登记（host 现跑 $(date '+%F %T')）"
  for u in lite3-monitor lite3-ros-bridge transfer_ros2 whoopsie apport-autoreport.path apport-forward.socket; do
    echo "  $u is-enabled=$(systemctl is-enabled "$u" 2>&1 | head -1) is-active=$(systemctl is-active "$u" 2>&1 | head -1)"
  done
  echo "# ★登记（au §6.2 断言①）：**覆盖面变化：异机无此六 unit 对象**——★不记通过（六 unit 属主理人机 root 面）"
  echo "# 等价物＝本机常驻大负载源榜首式（au §6.2：权威清单等主理人给、未给前逐窗现跑）："
  echo "# ★视图限制照实报：本驱动跑在沙箱内，ps 只见沙箱自身进程树（现跑仅 $(ps -e 2>/dev/null | wc -l) 个）⇒"
  echo "#   host 级榜首式须机主/主理人终端补跑：ps -eo pid,user,etimes,pcpu,args | sort -k3 -nr | head -3"
  echo "# 沙箱可见面榜首（仅证沙箱自身、★不得当宿主常驻源读）："
  ps -eo pid,user,etimes,pcpu,args --sort=-pcpu 2>/dev/null | head -4
} > "$A1/assert1_unit_coverage.txt"

{
  echo "# assert2 foxy 残留零进程（host 起前 $(date '+%F %T')；★唯一可靠式＝pgrep -x）"
  echo "  pgrep -x run_mapping_online = [$(pgrep -x run_mapping_online | tr '\n' ' ')]"
  echo "  pgrep -x livox_ros_driver  = [$(pgrep -x livox_ros_driver | tr '\n' ' ')]"
  echo "  pgrep -x ros2qnx            = [$(pgrep -x ros2qnx | tr '\n' ' ')]"
  echo "  ls -d /opt/ros/foxy = $(ls -d /opt/ros/foxy 2>&1)"
} > "$A1/assert2_foxy_before.txt"

{
  echo "# assert3 样张 lite3.pcd sha256（host 起前 $(date '+%F %T')）"
  sha256sum "$REPO/data/maps/lite3.pcd" 2>&1
  echo "# 锚串（au §4.4 在册）＝aaa09362c5f1aef93d2cbbb36f4b71f42ce32a5c98083f6c379c39cb27146f34"
} > "$A1/assert3_pcd_before.txt"

{
  echo "# assert4 端口零监听（host 起前 $(date '+%F %T')；TCP 与 UDP 分查；域＝1131x）"
  echo "  TCP 1131x 计数 = $(ss -lnt 2>/dev/null | grep -cE ':1131[1-9]')"
  echo "  UDP 1131x 计数 = $(ss -lnu 2>/dev/null | grep -cE ':1131[1-9]')"
  echo "  TCP 明细："; ss -lnt 2>/dev/null | grep -E ':1131[1-9]' | sed 's/^/    /' || true
  echo "  UDP 明细："; ss -lnu 2>/dev/null | grep -E ':1131[1-9]' | sed 's/^/    /' || true
  echo "  （★起前态预期：sim 容器 11313 映射在听属既起容器面、非本窗引入；判据看起前＝退后同形）"
} > "$A1/assert4_ports_before.txt"

say "A1 起前（host 四件）已落 $A1"

# ================= 容器内：预检变体当刻 → 格③窗 → bag → 退净 =================
say "容器内开跑（$CT）"
timeout 280 docker exec "$CT" bash -c "
set -u
say(){ echo \"[container \$(date '+%F %T')] \$*\"; }
CBATCH=$CBATCH; CWIN=$CWIN; CPF=$CBATCH/preflight_variant
mkdir -p \"\$CWIN\" \"\$CPF\"
# ★R356 实录修：容器 ENV 的 PATH 不含 /opt/ros/noetic/bin ⇒ 未 source 的外壳里 rosnode/rosbag
#   一律 command not found（驱动轮询曾因此空转 45s 假报\"roscore 未起\"，而探针自带 source、2s 即成）。
#   本外壳先 source（set +u 照在册规矩：noetic setup.bash 在 -u 下会炸）。
set +u; . /opt/ros/noetic/setup.bash; set -u
say '== ①预检变体当刻（到签①：跑通 rc=0 即视为预检 rc=0）=='
bash scripts/verify/v_window_preflight_x86_variant.sh > \"\$CPF/preflight_variant_real.txt\" 2>&1
PRC=\$?
say \"预检变体 rc=\$PRC\"
if [ \"\$PRC\" != 0 ]; then say '★预检未过 ⇒ 按纪律不起窗、当场拒窗'; exit 7; fi

{
  echo '# assert4 端口（container 起前 ' \$(date '+%F %T') '；TCP/UDP 分查）'
  echo \"  TCP 1131x = \$(ss -lnt 2>/dev/null | grep -cE ':1131[1-9]')\"
  echo \"  UDP 1131x = \$(ss -lnu 2>/dev/null | grep -cE ':1131[1-9]')\"
} > \"\$CBATCH/a1_baseline/assert4_ports_container_before.txt\"

say '== ②格③补测窗（X2_SEGS=x2_g6 四插件段；探针在册件原样跑）=='
X2_RUN=\"\$CWIN\" X2_SEGS='x2_g6' bash note/进度/SCAN集成/设计/scripts/probe_x2_fourcells.sh > \"\$CWIN/probe_stdout.txt\" 2>&1 &
P=\$!
for i in \$(seq 1 45); do rosnode list >/dev/null 2>&1 && break; sleep 1; done
rosnode list >/dev/null 2>&1 || { say '★roscore 45s 未起 ⇒ 前置缺失、不起 bag'; wait \$P; say \"探针自行收尾 rc=\$?\"; exit 9; }
say 'master 在位 ⇒ 起 bag（读数三样之 bag）'
rosbag record -O \"\$CWIN/x2_g6_window.bag\" /x2_g6/costmap/costmap /x2_g6/costmap/costmap_updates /pmap_a /pmap_b /rslidar_points /tf /tf_static > \"\$CWIN/rosbag_record.log\" 2>&1 &
B=\$!
sleep 22
say '== assert5 窗中读数（master 在位）=='
{
  echo '# assert5 ros2qnx/qnx 零出现（container 窗中 ' \$(date '+%F %T') '；★跨机照搬无豁免）'
  echo \"  rosnode list | grep -c qnx = \$(rosnode list 2>/dev/null | grep -c qnx)\"
  echo \"  pgrep -x ros2qnx = [\$(pgrep -x ros2qnx | tr '\n' ' ')]\"
  echo '  rosnode list 全表（留底）：'
  rosnode list 2>/dev/null | sed 's/^/    /'
} > \"\$CBATCH/a1_baseline/assert5_qnx_during.txt\"
wait \$P; PRC=\$?
say \"探针退出 rc=\$PRC\"
kill -INT \$B 2>/dev/null; wait \$B 2>/dev/null
say '== 退净读数（探针 trap 已执行）=='
{
  echo '# 退净形状（container 退后 ' \$(date '+%F %T') '；au §6.2 原句形状）'
  echo \"  pgrep -x gzserver=\$(pgrep -x gzserver | wc -l) gzclient=\$(pgrep -x gzclient | wc -l) roslaunch=\$(pgrep -x roslaunch | wc -l) roscore=\$(pgrep -x roscore | wc -l) rosmaster=\$(pgrep -x rosmaster | wc -l)\"
  echo \"  pgrep -x run_mapping_online=[\$(pgrep -x run_mapping_online | tr '\n' ' ')] pgrep -x livox_ros_driver=[\$(pgrep -x livox_ros_driver | tr '\n' ' ')] pgrep -x ros2qnx=[\$(pgrep -x ros2qnx | tr '\n' ' ')]\"
  echo \"  TCP 1131x = \$(ss -lnt 2>/dev/null | grep -cE ':1131[1-9]')｜UDP 1131x = \$(ss -lnu 2>/dev/null | grep -cE ':1131[1-9]')\"
  echo \"  ★本窗探针自带 teardown 行见 \$CWIN/run.log 末段\"
} > \"\$CBATCH/a1_baseline/exitclean_after.txt\"
say \"联调窗段结束（探针 rc=\$PRC；bag＝\$CWIN/x2_g6_window.bag）\"
exit \$PRC
" 2>&1 | tee "$BATCH/window_container_run.log"
WRC=${PIPESTATUS[0]}
say "容器段 rc=$WRC"

# ================= A1 退后（host 侧）=================
{
  echo "# assert2 foxy 残留零进程（host 退后 $(date '+%F %T')）"
  echo "  pgrep -x run_mapping_online = [$(pgrep -x run_mapping_online | tr '\n' ' ')]"
  echo "  pgrep -x livox_ros_driver  = [$(pgrep -x livox_ros_driver | tr '\n' ' ')]"
  echo "  pgrep -x ros2qnx            = [$(pgrep -x ros2qnx | tr '\n' ' ')]"
} > "$A1/assert2_foxy_after.txt"
{
  echo "# assert3 样张 lite3.pcd sha256（host 退后 $(date '+%F %T')）"
  sha256sum "$REPO/data/maps/lite3.pcd" 2>&1
  echo "# 锚串＝aaa09362c5f1aef93d2cbbb36f4b71f42ce32a5c98083f6c379c39cb27146f34"
} > "$A1/assert3_pcd_after.txt"
{
  echo "# assert4 端口（host 退后 $(date '+%F %T')；TCP/UDP 分查）"
  echo "  TCP 1131x 计数 = $(ss -lnt 2>/dev/null | grep -cE ':1131[1-9]')"
  echo "  UDP 1131x 计数 = $(ss -lnu 2>/dev/null | grep -cE ':1131[1-9]')"
  echo "  TCP 明细："; ss -lnt 2>/dev/null | grep -E ':1131[1-9]' | sed 's/^/    /' || true
  echo "  UDP 明细："; ss -lnu 2>/dev/null | grep -E ':1131[1-9]' | sed 's/^/    /' || true
} > "$A1/assert4_ports_after.txt"
say "A1 退后（host 三件）已落 $A1"

# ================= 清单最后建（追加四十：最后生成、生成后再追加即作废）=================
# ★子目录各自收；批根只收根层文件（driver＋容器段日志；★index/判定件等后置件按"最后建"原则
#   在全部落盘后由收尾步骤重建本根清单——追加即作废，不许留旧清单装新目录）。
for d in "$A1" "$WIN" "$PF"; do
  ( cd "$d" && find . -type f ! -name '_sha256_manifest.txt' -print0 | sort -z | xargs -0 -r sha256sum > _sha256_manifest.txt )
  say "清单已建（最后）：$d/_sha256_manifest.txt（$(grep -c '' "$d/_sha256_manifest.txt") 条）"
done
( cd "$BATCH" && find . -maxdepth 1 -type f ! -name '_sha256_manifest.txt' -print0 | sort -z | xargs -0 -r sha256sum > _sha256_manifest.txt )
say "清单已建（最后，仅根层）：$BATCH/_sha256_manifest.txt（$(grep -c '' "$BATCH/_sha256_manifest.txt") 条）"
# 属主/可编面：容器以 root 写 ⇒ chmod 让宿主用户可编辑（chown 在本机 EINVAL 不可用）
chmod -R a+rw "$BATCH" 2>/dev/null || true
say "== 驱动结束｜容器段 rc=$WRC｜证据根＝$BATCH =="
exit 0
