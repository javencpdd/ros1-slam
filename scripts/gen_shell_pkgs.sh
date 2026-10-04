#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/ros1-slam/ws/src
declare -A PKGS=(
  [00_drivers/lite3_drivers]="M0"
  [10_slam/lite3_slam]="M1"
  [20_localization/lite3_localization]="M2"
  [30_perception/lite3_perception]="M3"
  [40_navigation/lite3_navigation]="M4"
  [50_task/lite3_task]="M5"
  [60_safety/lite3_safety]="M6"
  [70_bringup/lite3_bringup]="TOP"
  [90_supervisor/lite3_supervisor]="SUP"
)
for rel in "${!PKGS[@]}"; do
  name=$(basename "$rel"); d="$ROOT/$rel"
  mkdir -p "$d"/{launch/stub,launch/sink,config,scripts,verify}
  cat > "$d/package.xml" <<EOF
<?xml version="1.0"?>
<package format="2">
  <name>$name</name>
  <version>0.1.0</version>
  <description>lite3 shell package (${PKGS[$rel]}) - self-owned launch/config/scripts only</description>
  <maintainer email="ros1-slam@lite3.local">ysc</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
EOF
  printf 'cmake_minimum_required(VERSION 3.0.2)\nproject(%s)\n\ncatkin_package()\n' "$name" > "$d/CMakeLists.txt"
  echo "# $name（模块 ${PKGS[$rel]}）" > "$d/README.md"
done
