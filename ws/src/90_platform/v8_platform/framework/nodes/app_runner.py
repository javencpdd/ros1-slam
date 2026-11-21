#!/usr/bin/env python3
"""One-command 3D navigation app runner (façade; M9 split).

External importers keep using ``app_runner.<name>``; implementation lives in
``app_runner_prepare`` / ``app_runner_execute``.
"""
import sys

from app_runner_prepare import (  # noqa: F401
    resolve_sim_launch, _prepare_app, prepare_execution, dry_run)
from app_runner_execute import (  # noqa: F401
    _terminate, initialize_ros_node, path_summary, quiesce_before_stop,
    trace_observer, assembly_failed, execute, main)


if __name__ == '__main__':
    sys.exit(main())
