#!/usr/bin/env python3
"""K1b: Stop under FollowPath load + sole /cmd_vel publisher check.

Self-bringup (nav_test_lib / path_bank pattern): headless bush_trail + path_bank.
Measures:
  1. Publishers on /husky1/cmd_vel while NAV authority active (expect gate only)
  2. StopMission during FOLLOWING_PATH → command_authority NONE + odom ~0

    python3 test/k1b_stop_under_load_test.py
"""

from __future__ import annotations

import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..'))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from nav_test_lib import await_simulation, bringup, log as nav_log, print_diagnostics
from rs1_interfaces.srv import StopMission
from rs1_nav.nav_observer import NavObserver, init_ros


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

START = (-18.0, 3.0, 0.0)
GOAL = (16.0, -0.1, 0.0)  # on-trail (same as path_bank_nav)
WORLD = 'bush_trail_world'


def log(msg: str) -> None:
    print(msg, flush=True)


class _CmdVelProbe(Node):
    def __init__(self, robot: str = 'husky1'):
        super().__init__('k1b_cmd_vel_probe', namespace=f'/{robot}')
        self.robot = robot
        self.last_cmd = Twist()
        self.cmd_count = 0
        self.last_odom_v = 0.0
        self.last_odom_w = 0.0
        self.authority = ''
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 20)
        self.create_subscription(Odometry, 'odometry/filtered', self._on_odom, 20)
        self.create_subscription(
            String, 'mission_owner/command_authority', self._on_auth, LATCHED)

    def _on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.cmd_count += 1

    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom_v = float(msg.twist.twist.linear.x)
        self.last_odom_w = float(msg.twist.twist.angular.z)

    def _on_auth(self, msg: String) -> None:
        self.authority = msg.data.strip()


def _spin(nodes, dt: float = 0.1) -> None:
    end = time.monotonic() + dt
    while time.monotonic() < end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.05)


def _sole_publisher_info(node: Node, topic: str) -> list:
    """Return publisher GIDs/names via graph API when available."""
    try:
        infos = node.get_publishers_info_by_topic(topic)
        return [f'{i.node_name}|{i.node_namespace}' for i in infos]
    except Exception as exc:  # noqa: BLE001
        return [f'error:{exc}']


def run_k1b(observer: NavObserver, probe: _CmdVelProbe) -> bool:
    ok = True
    topic = f'/{observer.robot}/cmd_vel'

    # Arm mission via StartMission.
    if not observer.nav2_ready(timeout=60.0):
        log('FAIL  K1b nav/StartMission not ready')
        return False
    if not observer.send_goal(GOAL[0], GOAL[1], GOAL[2]):
        log('FAIL  K1b StartMission/NavigateToPose rejected')
        return False

    # Wait until we see non-trivial cmd or authority NAV.
    deadline = time.monotonic() + 45.0
    saw_nav = False
    while time.monotonic() < deadline:
        _spin([observer, probe], 0.2)
        if probe.authority == 'NAV':
            saw_nav = True
        speed = math.hypot(probe.last_cmd.linear.x, probe.last_cmd.angular.z)
        if saw_nav and (speed > 0.05 or probe.cmd_count > 5):
            break
    else:
        log(f'FAIL  K1b no NAV motion (auth={probe.authority!r} cmds={probe.cmd_count})')
        return False

    pubs = _sole_publisher_info(probe, topic)
    log(f'  cmd_vel publishers under load (raw): {pubs}')
    # Ignore harness nodes that may advertise Twist for open-loop helpers.
    harness = ('k1b_observer', 'k1b_cmd_vel_probe', 'nav_observer', 'sensor_only')
    prod = [p for p in pubs if not any(h in p for h in harness)]
    # Dedupe identical node|ns entries (behavior plugins can register many).
    prod_unique = sorted(set(prod))
    log(f'  cmd_vel production publishers: {prod_unique}')
    gate_hits = [p for p in prod_unique if 'rs1_cmd_vel_gate' in p]
    if len(prod_unique) != 1 or not gate_hits:
        log(
            f'FAIL  K1b sole publisher expected rs1_cmd_vel_gate, '
            f'got {prod_unique}'
        )
        ok = False
    else:
        log('PASS  K1b sole /cmd_vel publisher is rs1_cmd_vel_gate')

    # StopMission under load.
    cli = observer.create_client(StopMission, 'mission_owner/stop_mission')
    if not cli.wait_for_service(timeout_sec=5.0):
        log('FAIL  K1b StopMission service missing')
        return False
    fut = cli.call_async(StopMission.Request())
    stop_deadline = time.monotonic() + 10.0
    while time.monotonic() < stop_deadline and not fut.done():
        _spin([observer, probe], 0.1)
    resp = fut.result() if fut.done() else None
    if resp is None or not resp.success:
        log(f'FAIL  K1b StopMission rejected: {resp}')
        return False
    log(f'  StopMission ok state={resp.state} reason={resp.stop_reason}')

    # Physical stop: odom and cmd near zero, authority NONE.
    settle_deadline = time.monotonic() + 8.0
    stopped = False
    while time.monotonic() < settle_deadline:
        _spin([observer, probe], 0.1)
        v = abs(probe.last_odom_v)
        w = abs(probe.last_odom_w)
        cv = abs(probe.last_cmd.linear.x)
        if probe.authority == 'NONE' and v < 0.05 and w < 0.08 and cv < 0.05:
            stopped = True
            break
    log(
        f'  after Stop: auth={probe.authority!r} odom_v={probe.last_odom_v:.3f} '
        f'odom_w={probe.last_odom_w:.3f} cmd_v={probe.last_cmd.linear.x:.3f}'
    )
    if not stopped:
        log('FAIL  K1b physical stop not observed after StopMission')
        ok = False
    else:
        log('PASS  K1b Stop under load → NONE + odom≈0')
    return ok


def main() -> int:
    """Self-bringup bush_trail + path_bank; assert gate-only cmd_vel + Stop under load."""
    bringup_log = '/tmp/rs1_k1b_bringup.log'
    sup = bringup(
        world=WORLD,
        nav2=True,
        rviz=False,
        gui=False,
        husky_x=START[0],
        husky_y=START[1],
        husky_yaw=START[2],
        max_runtime=900.0,
        log_path=bringup_log,
        extra={
            'husky_z': '0.50',
            'use_prior_map': 'true',
            'prior_map_file': 'bush_trail_world.yaml',
            'path_bank': 'true',
            'path_bank_switch_resend': 'true',
            'fire_avoidance': 'false',
            'test_speed': 'true',
        },
    )

    passed = False
    with sup:
        init_ros('husky1')
        ok, _ = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup, topics=['/husky1/prior_map', '/husky1/plan'])
            log('FAIL K1b stack start')
            return 1

        observer = NavObserver('husky1', node_name='k1b_observer')
        probe = _CmdVelProbe('husky1')
        try:
            if not observer.nav2_ready(timeout=180.0):
                print_diagnostics(sup)
                log('FAIL K1b StartMission/Nav2 not ready')
                return 1
            nav_log('K1b Nav2/StartMission ready; arming under-load stop')
            passed = run_k1b(observer, probe)
        finally:
            try:
                observer.cancel_goal()
            except Exception:  # noqa: BLE001
                pass
            observer.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()

    log('K1b SUMMARY: ' + ('PASS' if passed else 'FAIL'))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
