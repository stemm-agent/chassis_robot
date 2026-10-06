#!/usr/bin/env python3
"""Enable the nav2_rviz_plugins displays in config/vision_nav.rviz.

Adds the AMCL particle cloud display and the Nav2 goal tool.  Idempotent:
running it twice changes nothing.  Keeps a timestamped .bak next to the file.

Install the plugin package first:
    sudo apt-get install -y ros-humble-nav2-rviz-plugins

Usage:
    python3 scripts/enable_nav2_rviz_displays.py [--file PATH]
"""
import argparse
import io
import os
import shutil
import sys
import time

PARTICLES = """    - Alpha: 1
      Class: nav2_rviz_plugins/ParticleCloud
      Color: 0; 180; 0
      Enabled: true
      Max Arrow Length: 0.3
      Min Arrow Length: 0.02
      Name: AMCL particles /particle_cloud
      Shape: Arrow (Flat)
      Topic:
        Value: /particle_cloud
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Best Effort
        History Policy: Keep Last
"""
ANCHOR = ("    - Class: rviz_default_plugins/PointCloud2\n"
          "      Name: Vision cloud /vision/live_cloud (RGB)\n")
TOOL_OLD = ("    # Nav2 goal tool: uncomment after installing ros-humble-nav2-rviz-plugins\n"
            "    #- Class: nav2_rviz_plugins/GoalTool\n")
TOOL_NEW = "    - Class: nav2_rviz_plugins/GoalTool\n"


def main():
    default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           os.pardir, 'config', 'vision_nav.rviz')
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=default)
    args = ap.parse_args()
    path = os.path.abspath(args.file)
    if not os.path.isfile(path):
        sys.exit('not found: %s' % path)
    src = io.open(path, encoding='utf-8').read()

    changed = []
    if 'nav2_rviz_plugins/ParticleCloud' not in src:
        if src.count(ANCHOR) != 1:
            sys.exit('anchor for the particle display not found exactly once')
        src = src.replace(ANCHOR, PARTICLES + ANCHOR, 1)
        changed.append('AMCL particle cloud display')
    if '\n    - Class: nav2_rviz_plugins/GoalTool\n' not in src:
        if src.count(TOOL_OLD) != 1:
            sys.exit('goal tool comment block not found exactly once')
        src = src.replace(TOOL_OLD, TOOL_NEW, 1)
        changed.append('Nav2 goal tool')
    if not changed:
        print('already enabled, nothing to do: %s' % path)
        return
    backup = path + '.bak-' + time.strftime('%Y%m%d-%H%M%S')
    shutil.copy2(path, backup)
    io.open(path, 'w', encoding='utf-8').write(src)
    print('enabled: %s' % ', '.join(changed))
    print('backup : %s' % backup)


if __name__ == '__main__':
    main()
