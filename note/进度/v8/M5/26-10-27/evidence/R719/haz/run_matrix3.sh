#!/usr/bin/env bash
set +u
run() { echo "=== seed=$1 extra=$2 v=$3 xml=$4 fric=$5"; bash /work/run_stop_chain.sh "$1" "$2" "$3" "$4" "$5" 2>&1 | tail -1; }
for s in 27 28 29; do run $s 0.00 0.3 /work/Lite3_step10.xml ""; done
echo MATRIX3_DONE
