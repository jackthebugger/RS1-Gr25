#!/usr/bin/env python3
"""WASD keyboard teleop for a differential-drive robot.

Publishes geometry_msgs/Twist on cmd_vel (remap to /husky1/cmd_vel).

Controls (press a key; motion sticks until you press another key or Space):
  W / S   forward / reverse
  A / D   turn left / right
  Q / E   decrease / increase linear speed
  Z / C   decrease / increase angular speed
  Space   stop
  Ctrl-C  quit
"""

from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
WASD teleop - click this terminal, then drive:
  W/S  forward/back          A/D  turn left/right
  Q/E  slower/faster linear  Z/C  slower/faster turn
  Space  stop                Ctrl-C  quit
"""


class WasdTeleop(Node):
    def __init__(self) -> None:
        super().__init__('wasd_teleop')
        self.declare_parameter('linear_speed', 0.6)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('publish_rate', 20.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._target = Twist()
        self.create_timer(1.0 / max(rate, 1.0), self._publish)

    def handle_key(self, key: str) -> None:
        twist = Twist()
        if key == 'w':
            twist.linear.x = self.linear_speed
        elif key == 's':
            twist.linear.x = -self.linear_speed
        elif key == 'a':
            twist.angular.z = self.angular_speed
        elif key == 'd':
            twist.angular.z = -self.angular_speed
        elif key == ' ':
            pass  # zero twist
        elif key == 'q':
            self.linear_speed = max(0.1, self.linear_speed * 0.9)
            self._print_speeds()
            return
        elif key == 'e':
            self.linear_speed = min(2.0, self.linear_speed * 1.1)
            self._print_speeds()
            return
        elif key == 'z':
            self.angular_speed = max(0.1, self.angular_speed * 0.9)
            self._print_speeds()
            return
        elif key == 'c':
            self.angular_speed = min(2.5, self.angular_speed * 1.1)
            self._print_speeds()
            return
        else:
            return

        # Diagonal: keep previous axis if the new key only sets the other.
        if key in ('w', 's') and abs(self._target.angular.z) > 0.0:
            twist.angular.z = self._target.angular.z
        if key in ('a', 'd') and abs(self._target.linear.x) > 0.0:
            twist.linear.x = self._target.linear.x
        if key == ' ':
            twist = Twist()

        self._target = twist

    def _print_speeds(self) -> None:
        print(
            f'\rlinear={self.linear_speed:.2f} m/s  '
            f'angular={self.angular_speed:.2f} rad/s   ',
            end='',
            flush=True,
        )

    def _publish(self) -> None:
        self.pub.publish(self._target)


def _read_key(timeout: float = 0.05) -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def main() -> None:
    if not sys.stdin.isatty():
        print('wasd_teleop needs an interactive terminal (stdin TTY).', file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = WasdTeleop()
    print(HELP)
    node._print_speeds()
    print()

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key(0.05)
            if key is None:
                continue
            if key == '\x03':  # Ctrl-C
                break
            node.handle_key(key.lower() if key != ' ' else ' ')
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nwasd_teleop stopped.')


if __name__ == '__main__':
    main()
