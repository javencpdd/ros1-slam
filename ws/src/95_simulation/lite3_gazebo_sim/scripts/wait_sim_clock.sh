#!/usr/bin/env bash
until rostopic echo -n 1 /clock >/dev/null 2>&1; do sleep 0.5; done
exec "$@"
