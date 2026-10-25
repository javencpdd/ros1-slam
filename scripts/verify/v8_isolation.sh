#!/usr/bin/env bash
set -eu
HERE=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
if [ "$#" -ne 0 ]; then
  printf '%s\n' 'usage: v8_isolation.sh (full check only, no arguments)'
  exit 9
fi
exec python3 -B "$HERE/v8_isolation.py"
