#!/usr/bin/env bash
source "$(dirname "$0")/m_env.sh"
exec roslaunch lite3_drivers 01_sensors.launch
