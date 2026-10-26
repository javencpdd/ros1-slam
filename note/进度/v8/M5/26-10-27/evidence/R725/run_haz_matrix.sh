#!/usr/bin/env bash
set +u
for s in 0 1 2 3 4 5 6 7 8; do
  echo "=== seed=$s"
  bash /work/run_haz_chain.sh "$s" 2>&1 | tail -1
done
echo HAZ_MATRIX_DONE
