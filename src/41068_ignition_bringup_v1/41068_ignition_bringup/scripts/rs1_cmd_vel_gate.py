#!/usr/bin/env python3
"""Sole publisher of /cmd_vel under mission-owner command_authority.

Fail-closed: missing/stale authority → NONE (forced zero). Never spawn a
secondary zero publisher — faults stay inside this process.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

VALID = frozenset({'NONE', 'NAV', 'BT_MANUAL', 'TELEOP'})


class Rs1CmdVelGate(Node):
    def __init__(self) -> None:
        super().__init__('rs1_cmd_vel_gate')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('authority_stale_s', 1.0)
        self.declare_parameter('nav_topic', 'cmd_vel_nav_smooth')
        self.declare_parameter('teleop_topic', 'cmd_vel_teleop')
        self.declare_parameter('authority_topic', 'mission_owner/command_authority')
        self.declare_parameter('forward_only_topic', 'path_bank/forward_only')
        self.declare_parameter('out_topic', 'cmd_vel')

        rate = float(self.get_parameter('publish_rate').value)
        self._stale_s = float(self.get_parameter('authority_stale_s').value)

        self._authority = 'NONE'
        self._authority_stamp = 0.0  # monotonic; 0 = never received → NONE
        self._nav = Twist()
        self._teleop = Twist()
        self._fault = False
        self._forward_only = False

        self.create_subscription(
            String, self.get_parameter('authority_topic').value,
            self._on_authority, LATCHED)
        self.create_subscription(
            Twist, self.get_parameter('nav_topic').value, self._on_nav, 10)
        self.create_subscription(
            Twist, self.get_parameter('teleop_topic').value, self._on_teleop, 10)
        self.create_subscription(
            Bool, self.get_parameter('forward_only_topic').value,
            self._on_forward_only, LATCHED)

        self._pub = self.create_publisher(
            Twist, self.get_parameter('out_topic').value, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        self.get_logger().info(
            'rs1_cmd_vel_gate ready (fail-closed NONE until authority; '
            'forward_only clamps NAV reverse during path switch)'
        )

    def _on_authority(self, msg: String) -> None:
        value = (msg.data or '').strip().upper()
        if value not in VALID:
            self.get_logger().warn(f'Ignoring invalid authority={msg.data!r}')
            return
        self._authority = value
        self._authority_stamp = time.monotonic()
        self._fault = False

    def _on_nav(self, msg: Twist) -> None:
        self._nav = msg

    def _on_teleop(self, msg: Twist) -> None:
        self._teleop = msg

    def _on_forward_only(self, msg: Bool) -> None:
        self._forward_only = bool(msg.data)

    def _effective_authority(self) -> str:
        if self._fault:
            return 'NONE'
        if self._authority_stamp <= 0.0:
            return 'NONE'
        age = time.monotonic() - self._authority_stamp
        if age > self._stale_s and self._authority != 'NONE':
            # Stale non-NONE → fail closed.
            return 'NONE'
        return self._authority

    def _tick(self) -> None:
        auth = self._effective_authority()
        out = Twist()
        if auth == 'NAV' or auth == 'BT_MANUAL':
            out = self._nav
            # Route-switch policy: rotate/drive forward — no sustained reverse.
            if self._forward_only and out.linear.x < 0.0:
                out = Twist()
                out.linear.x = 0.0
                out.angular.z = self._nav.angular.z
        elif auth == 'TELEOP':
            out = self._teleop
        # NONE → forced zero (already default Twist)
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = Rs1CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
