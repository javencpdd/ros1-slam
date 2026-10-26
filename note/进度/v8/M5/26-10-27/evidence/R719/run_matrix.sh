#!/usr/bin/env bash
# 停车矩阵：0.3/0.4 × 三相 × 三重复；低摩擦/坡/阶各 3。结果逐行打印。
set +u
FLAT=/work/Lite3_rl_deploy-main/third_party/deep_robotics_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml
run() { echo "=== seed=$1 extra=$2 v=$3 xml=$4 fric=$5"; bash /work/run_stop_chain.sh "$1" "$2" "$3" "$4" "$5" 2>&1 | tail -1; }
# 0.3 flat phase C (3)
for s in 6 7 8; do run $s 0.14 0.3 "$FLAT" ""; done
# 0.4 flat phases A/B/C (9)
for s in 9 10 11; do run $s 0.00 0.4 "$FLAT" ""; done
for s in 12 13 14; do run $s 0.07 0.4 "$FLAT" ""; done
for s in 15 16 17; do run $s 0.14 0.4 "$FLAT" ""; done
# low friction 0.35 at 0.3 (3)
for s in 18 19 20; do run $s 0.00 0.3 "$FLAT" 0.35; done
# ramp 10deg at 0.3 (3)
for s in 21 22 23; do run $s 0.00 0.3 /work/Lite3_ramp.xml ""; done
# step 0.18m at 0.3 (3)
for s in 24 25 26; do run $s 0.00 0.3 /work/Lite3_step.xml ""; done
echo MATRIX_DONE
