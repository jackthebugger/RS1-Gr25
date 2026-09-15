#!/usr/bin/env python3
"""Workspace-root shim for basic_autonomy_demo.py.

Lets you run from ~/git/RS1-Gr25 after sourcing:

    python3 scripts/basic_autonomy_demo.py --attach --goal -4.5 -4.5 0

Prefer the installed entry point when possible:

    ros2 run 41068_ignition_bringup basic_autonomy_demo.py --attach --goal -4.5 -4.5 0
"""

from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path


def _resolve_script() -> Path:
    # Prefer the sourced install tree so edits still work with --symlink-install.
    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    for prefix in ament_prefix.split(os.pathsep):
        candidate = Path(prefix) / 'lib' / '41068_ignition_bringup' / 'basic_autonomy_demo.py'
        if candidate.is_file():
            return candidate

    here = Path(__file__).resolve()
    # RS1-Gr25/scripts -> RS1-Gr25/src/.../scripts/basic_autonomy_demo.py
    src_candidate = (
        here.parent.parent
        / 'src'
        / '41068_ignition_bringup_v1'
        / '41068_ignition_bringup'
        / 'scripts'
        / 'basic_autonomy_demo.py'
    )
    if src_candidate.is_file():
        return src_candidate

    raise SystemExit(
        'basic_autonomy_demo.py not found.\n'
        'Source the workspace first:\n'
        '  source /opt/ros/humble/setup.bash\n'
        '  source ~/git/RS1-Gr25/install/setup.bash\n'
        'Then run:\n'
        '  ros2 run 41068_ignition_bringup basic_autonomy_demo.py --attach --goal -4.5 -4.5 0'
    )


if __name__ == '__main__':
    script = _resolve_script()
    sys.argv[0] = str(script)
    runpy.run_path(str(script), run_name='__main__')
