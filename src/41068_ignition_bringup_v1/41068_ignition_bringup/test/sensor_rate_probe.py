#!/usr/bin/env python3
"""Measure the achieved sensor rates for a set of camera/lidar settings.

Ignition renders every camera and lidar on one thread, so camera cost is paid
out of the lidar's budget. Navigation depends on the lidar, so this probe
measures what each configuration actually delivers rather than what it requests.

    python3 test/sensor_rate_probe.py
"""

import sys
import time

from nav_test_lib import bringup, log, ros2, sweep_orphans


def measure_rate(topic: str, window: float = 14.0, expect: float = 1.0) -> float:
    """Return the observed publication rate of `topic`, or 0.0 if silent."""
    code, out = ros2(f'topic hz {topic} --window 40', timeout=window)
    rates = []
    for line in out.splitlines():
        line = line.strip()
        if line.startswith('average rate:'):
            try:
                rates.append(float(line.split(':', 1)[1]))
            except ValueError:
                pass
    if not rates:
        return 0.0
    # The last reading has the largest sample window, so trust it most.
    return rates[-1]


CASES = (
    ('original (720x480 cam @3, lidar @3)',
     {'enable_camera': 'true', 'camera_width': '720', 'camera_height': '480',
      'camera_update_rate': '3', 'lidar_update_rate': '3'}),
    ('small cam (320x240 @2, lidar @10)',
     {'enable_camera': 'true', 'camera_width': '320', 'camera_height': '240',
      'camera_update_rate': '2', 'lidar_update_rate': '10'}),
    ('camera off (lidar @10)',
     {'enable_camera': 'false', 'lidar_update_rate': '10'}),
)


def main() -> int:
    results = []
    for label, settings in CASES:
        log('=' * 68)
        log(f'CASE: {label}')
        sup = bringup(
            nav2=False, rviz=False, gui=False,
            max_runtime=240.0,
            log_path=f'/tmp/sensor_rate_{len(results)}.log',
            extra=settings,
        )
        with sup:
            # Give Gazebo time to load the world, spawn the robot and bring the
            # sensor render targets up before measuring.
            time.sleep(35.0)

            _code, stats = ros2('topic echo /clock --once --field clock.sec', timeout=12.0)
            sim_sec = stats.strip().splitlines()[0] if stats.strip() else 'none'

            scan_hz = measure_rate('/husky1/scan')
            odom_hz = measure_rate('/husky1/odometry', window=10.0)
            image_hz = measure_rate('/husky1/camera/image', window=10.0)

            log(f'  sim clock      : {sim_sec}')
            log(f'  /husky1/scan   : {scan_hz:.2f} Hz')
            log(f'  /husky1/odometry: {odom_hz:.2f} Hz')
            log(f'  /husky1/camera/image: {image_hz:.2f} Hz')
            results.append((label, scan_hz, odom_hz, image_hz))

    sweep_orphans('final')
    log('=' * 68)
    log(f'{"case":42} {"scan":>8} {"odom":>8} {"image":>8}')
    for label, scan_hz, odom_hz, image_hz in results:
        log(f'{label:42} {scan_hz:8.2f} {odom_hz:8.2f} {image_hz:8.2f}')
    log('=' * 68)
    return 0


if __name__ == '__main__':
    sys.exit(main())
