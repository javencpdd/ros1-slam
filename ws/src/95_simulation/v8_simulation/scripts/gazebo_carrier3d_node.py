#!/usr/bin/env python3
"""Simulation package entry for the existing Gazebo carrier."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _source_entry import run_source


if __name__ == '__main__':
    run_source('framework/gazebo_carrier3d_node.py')
