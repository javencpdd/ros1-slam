#!/usr/bin/env bash
set +u
for s in 0 1 2; do
  for r in a b; do
    echo "=== seed=$s rep=$r"
    bash /work/run_haz_rep.sh "$s" "${s}_${r}" 2>&1 | tail -1
  done
done
echo HAZ_REPS_DONE
