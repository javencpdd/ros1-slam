#!/usr/bin/env bash
set +u
FLAT=/work/Lite3_rl_deploy-main/third_party/deep_robotics_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml
run() { echo "=== seed=$1 extra=$2 v=$3 xml=$4 fric=$5"; bash /work/run_stop_chain.sh "$1" "$2" "$3" "$4" "$5" 2>&1 | tail -1; }
for s in 18 19 20; do run $s 0.00 0.3 "$FLAT" 0.35; done
for s in 21 22 23; do run $s 0.00 0.3 /work/Lite3_ramp.xml ""; done
for s in 24 25 26; do run $s 0.00 0.3 /work/Lite3_step.xml ""; done
echo MATRIX2_DONE
