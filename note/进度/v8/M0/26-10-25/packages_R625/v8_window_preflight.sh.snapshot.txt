#!/usr/bin/env bash
set -eu
HERE=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
if [ "$#" -ne 0 ]; then
  printf '%s\n' 'usage: v8_window_preflight.sh (no arguments; offline tests are separate)'
  exit 9
fi
export ISO_TARGET="$HERE/v8_isolation.sh"
exec bash "$HERE/v_window_preflight_x86_variant.sh"
