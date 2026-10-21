#!/usr/bin/env bash
# R280 负面对照三例（小美会签条件：改后的 W5 联合判必须还能真红）
# 每例：动盘 → 跑契约件 → 立刻还原 → 验残留。任何一步没还原＝本批作废。
set -u
ROOT=/home/ros1-slam
C="$ROOT/ws/src/95_simulation/lite3_gazebo_sim/verify/v_w3_terrain_design_contract.sh"
NODE="$ROOT/ws/src/30_perception/lite3_perception/scripts/w3_elevation_node.py"
YAML="$ROOT/ws/src/70_bringup/lite3_bringup/config/elevation_costmap.yaml"
HOLD=/tmp/r280_hold
mkdir -p "$HOLD"
run_case() { # 名字 期望关键词
  local name="$1" want="$2" out rc
  out=$(bash "$C" 2>&1); rc=$?
  echo "== $name：rc=$rc"
  echo "$out" | grep -E "^  \[FAIL\]|VERDICT" | sed 's/^/   /'
  if [ "$rc" = 1 ] && echo "$out" | grep -q "$want"; then echo "   ⇒ 该例真红且命中期望句「$want」"; else echo "   ⇒ ★该例未达期望（无分辨力），记为缺陷"; fi
}
echo "### N1 撤规范位 yaml（有节点无 yaml）⇒ 期望「半件状态」红"
mv "$YAML" "$HOLD/elevation_costmap.yaml"; run_case "N1" "半件状态"; mv "$HOLD/elevation_costmap.yaml" "$YAML"
echo
echo "### N2 节点挪出落点、放到他包（落点缺件而全盘>0）⇒ 期望「落错包」红"
ALT="$ROOT/ws/src/40_navigation/lite3_scan/scripts"
mkdir -p "$ALT"
mv "$NODE" "$HOLD/w3_elevation_node.py"; cp "$HOLD/w3_elevation_node.py" "$ALT/w3_elevation_node.py"
run_case "N2" "落错包"
rm -f "$ALT/w3_elevation_node.py"; mv "$HOLD/w3_elevation_node.py" "$NODE"
echo
echo "### N3 elevation yaml 落在他包、规范位为 0 ⇒ 期望「不在规范位」红"
STRAY="$ROOT/ws/src/30_perception/lite3_perception/config/elevation_probe.yaml"
mv "$YAML" "$HOLD/elevation_costmap.yaml"
printf 'elevation:\n  resolution: 0.1  # 探针件，跑完即删\n' > "$STRAY"
run_case "N3" "不在规范位"
rm -f "$STRAY"; mv "$HOLD/elevation_costmap.yaml" "$YAML"
echo
echo "### 还原核验（三项须各回原位；多余件须 0）"
echo -n "  全盘节点命中＝"; find "$ROOT/ws/src" -name 'w3_elevation_node*' | wc -l
echo -n "  规范位 yaml 命中＝"; find "$ROOT/ws/src/70_bringup/lite3_bringup/config" -name '*elevation*' | wc -l
echo -n "  残留探针件（须 0）＝"; find "$ROOT/ws/src" -name 'elevation_probe.yaml' -o -path '*lite3_scan/scripts/w3_elevation_node.py' | wc -l
echo -n "  /tmp 暂存目录剩余（须 0）＝"; find "$HOLD" -type f | wc -l
echo
echo "### 正跑复归（三例后本体须回 rc=0）"
bash "$C" 2>&1 | tail -2
