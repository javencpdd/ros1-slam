"""Delegate simulation entries to the retained v8 source implementation."""
import runpy
import sys
from pathlib import Path


def _platform_root():
    import rospkg
    return Path(rospkg.RosPack().get_path('v8_platform')).resolve()


def run_source(relative_script):
    """Preserve ROS argv and sibling imports without copying implementations.

    v8_platform currently does not install its Python sources. An install-only
    workspace must fail explicitly instead of resolving a different repository.
    """
    script = _platform_root() / relative_script
    if not script.is_file():
        raise RuntimeError(
            'v8_simulation requires the v8_platform source/devel workspace; '
            'source implementation is missing: %s' % script)
    previous_path = sys.path[:]
    try:
        sys.path.insert(0, str(script.parent))
        return runpy.run_path(str(script), run_name='__main__')
    finally:
        sys.path[:] = previous_path
