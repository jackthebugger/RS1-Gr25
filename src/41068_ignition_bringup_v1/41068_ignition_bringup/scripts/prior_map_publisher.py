#!/usr/bin/env python3
"""Publish a saved occupancy map for navigation priors.

Publishes:
1. ``prior_map`` — latched OccupancyGrid (RViz / operators).
2. ``prior_obstacles`` — PointCloud2 of occupied cells for Nav2 ObstacleLayer.

StaticLayer + rolling NavFn was shown to hang the planner (Entry 002/003), so
prior knowledge enters planning through the same ObstacleLayer path as lidar
and fire hazards.
"""

from __future__ import annotations

import os
import struct
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import yaml


def _load_pgm(path: str) -> np.ndarray:
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic not in (b'P5', b'P2'):
            raise RuntimeError(f'Unsupported PGM magic {magic!r} in {path}')
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = [int(x) for x in line.split()]
        maxval = int(f.readline().split()[0])
        if magic == b'P5':
            raw = f.read(width * height)
            if maxval > 255:
                arr = np.frombuffer(raw, dtype='>u2').reshape((height, width))
            else:
                arr = np.frombuffer(raw, dtype=np.uint8).reshape((height, width))
        else:
            data = []
            while len(data) < width * height:
                data.extend(int(x) for x in f.readline().split())
            arr = np.asarray(data[: width * height], dtype=np.uint8).reshape((height, width))
    return arr


def _pgm_to_occupancy(
    img: np.ndarray,
    *,
    negate: bool,
    occupied_thresh: float,
    free_thresh: float,
) -> np.ndarray:
    if img.dtype != np.float32:
        img_f = img.astype(np.float32)
    else:
        img_f = img
    if img_f.max() > 1.0:
        img_f = img_f / 255.0
    if negate:
        img_f = 1.0 - img_f
    occ = np.full(img_f.shape, -1, dtype=np.int8)
    prob = 1.0 - img_f
    occ[prob >= occupied_thresh] = 100
    occ[prob <= free_thresh] = 0
    return occ


def _pack_xyz_cloud(header: Header, points: List[Tuple[float, float, float]]) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    buf = bytearray()
    for x, y, z in points:
        buf += struct.pack('<fff', float(x), float(y), float(z))
    msg.data = bytes(buf)
    return msg


def _occupied_cloud(
    occ: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    stride: int,
) -> List[Tuple[float, float, float]]:
    """Downsample occupied cells to a PointCloud (prefer free-space edges)."""
    occupied = occ >= 50
    # Prefer perimeter cells so the cloud stays small but walls remain solid.
    from numpy.lib.stride_tricks import as_strided  # local import keeps top clean
    padded = np.pad(occupied.astype(np.uint8), 1, mode='constant')
    # 3x3 neighbourhood sum; edge cells have fewer occupied neighbours.
    windows = as_strided(
        padded,
        shape=occupied.shape + (3, 3),
        strides=padded.strides + padded.strides,
    )
    neighbour_sum = windows.sum(axis=(2, 3))
    edge = occupied & (neighbour_sum < 9)
    ys, xs = np.where(edge)
    if stride > 1:
        ys = ys[::stride]
        xs = xs[::stride]
    points = [
        (origin_x + (int(x) + 0.5) * resolution,
         origin_y + (int(y) + 0.5) * resolution,
         0.0)
        for y, x in zip(ys, xs)
    ]
    return points


class PriorMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__('prior_map_publisher')
        self.declare_parameter('yaml_filename', '')
        self.declare_parameter('topic_name', 'prior_map')
        self.declare_parameter('obstacle_topic', 'prior_obstacles')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('robot_name', 'husky1')
        self.declare_parameter('publish_period_s', 2.0)
        self.declare_parameter('cloud_stride', 2)

        yaml_filename = str(self.get_parameter('yaml_filename').value)
        if not yaml_filename or not os.path.isfile(yaml_filename):
            raise FileNotFoundError(f'prior map yaml not found: {yaml_filename!r}')

        robot = str(self.get_parameter('robot_name').value).strip().strip('/') or 'husky1'
        frame_id = str(self.get_parameter('frame_id').value).strip() or f'{robot}_map'

        with open(yaml_filename, 'r', encoding='utf-8') as f:
            meta = yaml.safe_load(f)

        image_name = meta['image']
        image_path = image_name if os.path.isabs(image_name) else os.path.join(
            os.path.dirname(yaml_filename), image_name
        )
        resolution = float(meta['resolution'])
        origin = meta['origin']
        negate = bool(int(meta.get('negate', 0)))
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        free_thresh = float(meta.get('free_thresh', 0.25))
        img = _load_pgm(image_path)
        occ = _pgm_to_occupancy(
            img,
            negate=negate,
            occupied_thresh=occupied_thresh,
            free_thresh=free_thresh,
        )

        self._msg = OccupancyGrid()
        self._msg.header = Header()
        self._msg.header.frame_id = frame_id
        info = MapMetaData()
        info.map_load_time = self.get_clock().now().to_msg()
        info.resolution = resolution
        info.width = int(occ.shape[1])
        info.height = int(occ.shape[0])
        info.origin = Pose()
        info.origin.position.x = float(origin[0])
        info.origin.position.y = float(origin[1])
        info.origin.position.z = float(origin[2]) if len(origin) > 2 else 0.0
        info.origin.orientation.w = 1.0
        self._msg.info = info
        self._msg.data = occ.flatten().tolist()

        stride = max(1, int(self.get_parameter('cloud_stride').value))
        self._cloud_points = _occupied_cloud(
            occ,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            resolution=resolution,
            stride=stride,
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        topic = str(self.get_parameter('topic_name').value)
        obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        self.pub = self.create_publisher(OccupancyGrid, topic, qos)
        self.cloud_pub = self.create_publisher(PointCloud2, obstacle_topic, 10)
        period = float(self.get_parameter('publish_period_s').value)
        self.create_timer(period, self._publish)
        self._publish()
        self.get_logger().info(
            f'Publishing prior map {yaml_filename} ({info.width}x{info.height} @ '
            f'{resolution} m) on {topic}; {len(self._cloud_points)} obstacle '
            f'points on {obstacle_topic}; frame={frame_id}'
        )

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._msg.header.stamp = stamp
        self.pub.publish(self._msg)
        header = Header()
        header.stamp = stamp
        header.frame_id = self._msg.header.frame_id
        self.cloud_pub.publish(_pack_xyz_cloud(header, self._cloud_points))


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[PriorMapPublisher] = None
    try:
        node = PriorMapPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
