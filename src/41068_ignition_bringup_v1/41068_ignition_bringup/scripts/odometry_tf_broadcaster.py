#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdometryTfBroadcaster(Node):
    """Broadcast odom->base_link TF from incoming odometry and republish odom."""

    def __init__(self):
        super().__init__('odometry_tf_broadcaster')

        self.declare_parameter('odom_topic', 'odometry')
        self.declare_parameter('output_odom_topic', 'odom')
        self.declare_parameter('odom_frame', '')
        self.declare_parameter('base_frame', '')

        odom_topic = self.get_parameter('odom_topic').value
        output_odom_topic = self.get_parameter('output_odom_topic').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, output_odom_topic, 20)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 50)

    def _odom_cb(self, msg: Odometry) -> None:
        odom_frame = self.odom_frame if self.odom_frame else msg.header.frame_id
        base_frame = self.base_frame if self.base_frame else msg.child_frame_id

        if not odom_frame or not base_frame:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = odom_frame
        tf_msg.child_frame_id = base_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        # Republish odometry to a stable topic expected by Nav2 config.
        odom_out = deepcopy(msg)
        odom_out.header.frame_id = odom_frame
        odom_out.child_frame_id = base_frame
        self.odom_pub.publish(odom_out)


def main() -> None:
    rclpy.init()
    node = OdometryTfBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        # Normal shutdown path when launch sends SIGINT/SIGTERM.
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
