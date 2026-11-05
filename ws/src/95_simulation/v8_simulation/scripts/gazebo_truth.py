#!/usr/bin/env python3
"""Simulation package entry for the existing Gazebo truth publisher."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _source_entry import run_source


if __name__ == '__main__':
    run_source('gazebo_truth.py')
