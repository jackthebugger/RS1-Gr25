#!/usr/bin/env python3
"""Publish sensor-derived KeepoutFilter mask for global obstacle memory.

Authority
---------
* Live ObstacleLayer still marks/clears from scan for reactive local behaviour.
* This node remembers previously observed hazards in a typed OccupancyGrid mask
  consumed by ``nav2_costmap_2d::KeepoutFilter`` on the global costmap.
* Cells clear only with spatially valid lidar free-space evidence (LIDAR) or
  consecutive in-FOV thermal negatives after hold expiry (FIRE).
* Absence of messages, Gazebo GT, or lidar rays through FIRE cells never clear.

Topics (robot namespace, e.g. husky1)
------------------------------------
* Publishes TransientLocal:
    obstacle_memory/filter_info  (nav2_msgs/CostmapFilterInfo)
    obstacle_memory/mask         (nav_msgs/OccupancyGrid)
* Subscribes: scan, fire_hazards, fire_detected, prior_map (metadata)
"""

from __future__ import annotations

import math
import struct
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, String
import tf2_ros

from rs1_nav.obstacle_memory import (
    CellSource,
    ClearResult,
    MapMeta,
    ObstacleMemory,
    ObstacleMemoryConfig,
)


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def _unpack_xy(cloud: PointCloud2) -> List[Tuple[float, float]]:
    """Extract XY from a dense xyz float32 PointCloud2."""
    if cloud.point_step < 8 or cloud.width == 0:
        return []
    pts: List[Tuple[float, float]] = []
    raw = bytes(cloud.data)
    step = cloud.point_step
    for i in range(cloud.width * max(1, cloud.height)):
        off = i * step
        if off + 8 > len(raw):
            break
        x, y = struct.unpack_from('<ff', raw, off)
        if math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
    return pts


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class ObstacleMemoryManager(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_memory_manager')

        self.declare_parameter('robot_name', 'husky1')
        self.declare_parameter('enabled', True)
        # When False, lidar/fire still clear but new marks are suppressed
        # (T5 observation teleport without sealing the map via SLAM jumps).
        self.declare_parameter('marking_enabled', True)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('fire_hazards_topic', 'fire_hazards')
        self.declare_parameter('fire_detected_topic', 'fire_detected')
        self.declare_parameter('prior_map_topic', 'prior_map')
        self.declare_parameter('filter_info_topic', 'obstacle_memory/filter_info')
        self.declare_parameter('mask_topic', 'obstacle_memory/mask')
        self.declare_parameter('publish_hz', 2.0)
        # KeepoutFilter resets its mask subscription on every new filter_info
        # (Humble keepout_filter.cpp → WARN + mask_sub_.reset()). Publish info
        # once at start, then rarely: each refresh re-latches TransientLocal
        # mask and sets has_updated_data_ so keepouts re-apply after
        # clear_entirely / resetLayers (T15). Do NOT refresh at publish_hz.
        self.declare_parameter('filter_info_refresh_s', 12.0)
        # 0 = publish mask only when occupied set changes (preferred).
        # Periodic force (>0) causes KeepoutFilter WARN on every republish
        # ("Updating old filter mask") without improving Steady-state
        # persistence; TransientLocal + rare filter_info refresh is enough.
        self.declare_parameter('mask_force_republish_s', 0.0)
        # Cap dirty-driven mask publish rate (KeepoutFilter full-mask rebuild).
        self.declare_parameter('mask_min_publish_period_s', 1.0)
        self.declare_parameter('scan_frame', '')  # default <robot>_base_scan
        self.declare_parameter('max_scan_age_s', 0.5)
        # Clear range must not exceed practical observation range used for
        # marking. Otherwise a robot 12 m away with corridor LOS can free-ray
        # clear Path A keepouts while still "off-FOV" for marking (T3).
        self.declare_parameter('clear_max_range_m', 10.0)
        self.declare_parameter('raytrace_max_m', 12.0)
        # 0 = hit cell only; neighbours bleed into free trail and can accumulate.
        self.declare_parameter('mark_neighbour_radius_cells', 0)
        self.declare_parameter('prior_edge_margin_cells', 6)
        self.declare_parameter('obstacle_max_range_m', 8.0)
        # Entry 023: skip marks inside Husky footprint halo (keepout trap).
        self.declare_parameter('mark_exclude_radius_m', 0.65)
        self.declare_parameter('fire_clear_confirm_scans', 3)
        self.declare_parameter('thermal_cone_half_angle', 0.6)
        self.declare_parameter('thermal_max_range', 8.0)
        self.declare_parameter('fire_hold_s', 8.0)
        # Fallback map meta if prior_map not yet received (bush_trail defaults).
        self.declare_parameter('default_resolution', 0.05)
        self.declare_parameter('default_width', 1200)
        self.declare_parameter('default_height', 1200)
        self.declare_parameter('default_origin_x', -30.0)
        self.declare_parameter('default_origin_y', -30.0)

        robot = str(self.get_parameter('robot_name').value).strip('/') or 'husky1'
        self._robot = robot
        self._map_frame = f'{robot}_map'
        self._scan_frame = (
            str(self.get_parameter('scan_frame').value).strip()
            or f'{robot}_base_scan'
        )
        self._enabled = _as_bool(self.get_parameter('enabled').value)
        self._marking_enabled = _as_bool(
            self.get_parameter('marking_enabled').value)
        self._fire_hold_s = float(self.get_parameter('fire_hold_s').value)
        self._fire_detected = False
        self._fire_true_until = 0.0
        self._last_fire_msg_time: Optional[float] = None
        self._latest_scan: Optional[LaserScan] = None
        self._scan_wall_time: Optional[float] = None
        self._dirty = True
        self._last_accept_log = 0.0
        self._last_info_pub = 0.0
        self._last_mask_force = 0.0
        self._last_mask_pub = 0.0
        # Occupancy fingerprint so mark/clear churn with the same cell set does
        # not republish (KeepoutFilter rebuilds the full mask on every message).
        self._last_mask_fp: Optional[Tuple[int, Tuple[Tuple[int, int], ...]]] = None
        self._info_refresh_s = float(self.get_parameter('filter_info_refresh_s').value)
        self._mask_force_s = float(self.get_parameter('mask_force_republish_s').value)
        self._mask_min_period = float(
            self.get_parameter('mask_min_publish_period_s').value)
        self._prior_free: Optional[object] = None  # numpy bool HxW when prior known

        cfg = ObstacleMemoryConfig(
            max_scan_age_s=float(self.get_parameter('max_scan_age_s').value),
            clear_max_range_m=float(self.get_parameter('clear_max_range_m').value),
            raytrace_max_m=float(self.get_parameter('raytrace_max_m').value),
            mark_neighbour_radius_cells=int(
                self.get_parameter('mark_neighbour_radius_cells').value),
            prior_edge_margin_cells=int(
                self.get_parameter('prior_edge_margin_cells').value),
            obstacle_max_range_m=float(
                self.get_parameter('obstacle_max_range_m').value),
            mark_exclude_radius_m=float(
                self.get_parameter('mark_exclude_radius_m').value),
            fire_clear_confirm_scans=int(
                self.get_parameter('fire_clear_confirm_scans').value),
            thermal_cone_half_angle_rad=float(
                self.get_parameter('thermal_cone_half_angle').value),
            thermal_max_range_m=float(self.get_parameter('thermal_max_range').value),
        )
        meta = MapMeta(
            origin_x=float(self.get_parameter('default_origin_x').value),
            origin_y=float(self.get_parameter('default_origin_y').value),
            resolution=float(self.get_parameter('default_resolution').value),
            width=int(self.get_parameter('default_width').value),
            height=int(self.get_parameter('default_height').value),
            frame_id=self._map_frame,
        )
        self._memory = ObstacleMemory(meta=meta, cfg=cfg)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._on_scan,
            SENSOR_QOS,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter('fire_hazards_topic').value,
            self._on_fire_hazards,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter('fire_detected_topic').value,
            self._on_fire_detected,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('prior_map_topic').value,
            self._on_prior,
            LATCHED,
        )

        info_topic = str(self.get_parameter('filter_info_topic').value)
        mask_topic = str(self.get_parameter('mask_topic').value)
        # Absolute names so KeepoutFilter (configured with /husky1/...) finds them
        # regardless of node namespace remaps.
        if not info_topic.startswith('/'):
            info_topic = f'/{robot}/{info_topic}'
        if not mask_topic.startswith('/'):
            mask_topic = f'/{robot}/{mask_topic}'
        self._info_topic = info_topic
        self._mask_topic = mask_topic
        # Relative publish under namespace; filter_info points at absolute mask.
        self._info_pub = self.create_publisher(CostmapFilterInfo, 'obstacle_memory/filter_info', LATCHED)
        self._mask_pub = self.create_publisher(OccupancyGrid, 'obstacle_memory/mask', LATCHED)
        self._diag_pub = self.create_publisher(String, 'obstacle_memory/status', 10)

        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / max(hz, 0.2), self._on_timer)

        self.get_logger().info(
            f'obstacle_memory_manager ready (robot={robot}, enabled={self._enabled}, '
            f'mask={mask_topic}, cells={self._memory.occupied_count()})'
        )
        # Publish empty mask + info once so KeepoutFilter can latch.
        self._maybe_publish_filter_info(force=True)
        self._publish_mask()
        self._dirty = False

    # -- callbacks ---------------------------------------------------------

    def _on_prior(self, msg: OccupancyGrid) -> None:
        import numpy as np

        info = msg.info
        new_meta = MapMeta(
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            resolution=float(info.resolution),
            width=int(info.width),
            height=int(info.height),
            frame_id=msg.header.frame_id or self._map_frame,
        )
        data = np.asarray(msg.data, dtype=np.int16).reshape((info.height, info.width))
        # Only true free (0). Unknown must NOT be markable — otherwise off-trail
        # lidar returns accumulate forever in the keepout mask.
        new_free = (data == 0)
        if self._prior_free is not None and new_free.shape == self._prior_free.shape:
            if bool(np.array_equal(new_free, self._prior_free)):
                return  # identical prior republish — ignore
        self._prior_free = new_free
        free_n = int(self._prior_free.sum())
        self.get_logger().info(
            f'prior_map free cells={free_n}/{data.size} '
            f'({100.0 * free_n / max(1, data.size):.1f}%)'
        )

        old = self._memory.meta
        if (
            new_meta.width != old.width
            or new_meta.height != old.height
            or abs(new_meta.resolution - old.resolution) > 1e-9
            or abs(new_meta.origin_x - old.origin_x) > 1e-6
            or abs(new_meta.origin_y - old.origin_y) > 1e-6
        ):
            self.get_logger().info(
                f'Syncing memory mask meta to prior_map '
                f'{new_meta.width}x{new_meta.height} @ {new_meta.resolution}m '
                f'origin=({new_meta.origin_x:.2f},{new_meta.origin_y:.2f})'
            )
            remapped = {}
            for cell in self._memory.cells.values():
                wx, wy = old.grid_to_world(cell.ix, cell.iy)
                idx = new_meta.world_to_grid(wx, wy)
                if idx is not None:
                    cell.ix, cell.iy = idx
                    remapped[idx] = cell
            self._memory.meta = new_meta
            self._memory.cells = remapped
            self._dirty = True

        # Always prune to interior-free prior cells (also on first prior receipt).
        from rs1_nav.obstacle_memory import _is_interior_free
        margin = self._memory.cfg.prior_edge_margin_cells
        pruned = {
            k: v for k, v in self._memory.cells.items()
            if v.source == CellSource.FIRE
            or (
                k[1] < self._prior_free.shape[0]
                and k[0] < self._prior_free.shape[1]
                and bool(self._prior_free[k[1], k[0]])
                and _is_interior_free(self._prior_free, k[0], k[1], margin)
            )
        }
        if len(pruned) != len(self._memory.cells):
            self.get_logger().info(
                f'Pruned memory {len(self._memory.cells)} → {len(pruned)} '
                '(interior-free prior only)'
            )
            self._memory.cells = pruned
            self._dirty = True

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        self._scan_wall_time = time.monotonic()
        if not self._enabled:
            return
        # Do not mark until prior_map free mask exists — otherwise every wall
        # return is persisted and seals the trail before StaticLayer sync.
        if self._prior_free is None:
            return
        tf = self._lookup_scan()
        if tf is None:
            return
        ox = tf.transform.translation.x
        oy = tf.transform.translation.y
        yaw = _yaw_from_quat(tf.transform.rotation)
        now = time.monotonic()
        # Re-read marking gate so tests can disable marks without killing clear.
        try:
            self._marking_enabled = _as_bool(
                self.get_parameter('marking_enabled').value)
        except Exception:
            pass
        fp_before = self._occupancy_fingerprint()
        if self._marking_enabled:
            robot_xy = self._robot_xy_map()
            self._memory.mark_lidar_scan(
                ranges=list(msg.ranges),
                angle_min=float(msg.angle_min),
                angle_increment=float(msg.angle_increment),
                range_min=float(msg.range_min),
                range_max=float(msg.range_max),
                origin_xy=(ox, oy),
                origin_yaw=yaw,
                now=now,
                prior_free=self._prior_free,
                robot_xy=robot_xy,
            )
        age = 0.0
        accepted, rejected = self._memory.clear_lidar_with_scan(
            ranges=list(msg.ranges),
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            origin_xy=(ox, oy),
            origin_yaw=yaw,
            scan_age_s=age,
            tf_ok=True,
        )
        fp_after = self._occupancy_fingerprint()
        # Only republish when the occupied cell *set* changes. Count-only
        # checks miss swap churn and over-trigger when count oscillates.
        if fp_after != fp_before:
            self._dirty = True
        if accepted and (now - self._last_accept_log) > 5.0:
            self._last_accept_log = now
            self.get_logger().info(
                f'CLEARING_OBSERVATION_ACCEPTED lidar cells={accepted} '
                f'(rejected={rejected}, memory={fp_after[0]})'
            )

    def _on_fire_hazards(self, msg: PointCloud2) -> None:
        if not self._enabled:
            return
        now = time.monotonic()
        pts = _unpack_xy(msg)
        if not pts:
            return
        fp_before = self._occupancy_fingerprint()
        robot_xy = self._robot_xy_map()
        for x, y in pts:
            self._memory.mark_fire_hit(
                x, y, now, prior_free=self._prior_free, robot_xy=robot_xy)
        if self._occupancy_fingerprint() != fp_before:
            self._dirty = True

    def _on_fire_detected(self, msg: Bool) -> None:
        self._last_fire_msg_time = time.monotonic()
        if bool(msg.data):
            self._fire_detected = True
            self._fire_true_until = time.monotonic() + max(0.0, self._fire_hold_s)
        elif time.monotonic() >= self._fire_true_until:
            self._fire_detected = False

    def _on_timer(self) -> None:
        if not self._enabled:
            self._maybe_publish_filter_info(force=False)
            if self._dirty:
                self._publish_mask()
                self._dirty = False
            return

        # Fire clear pass (uses base_link pose for thermal cone).
        tf_base = self._lookup_base()
        if tf_base is not None and self._last_fire_msg_time is not None:
            ox = tf_base.transform.translation.x
            oy = tf_base.transform.translation.y
            yaw = _yaw_from_quat(tf_base.transform.rotation)
            hold_active = time.monotonic() < self._fire_true_until
            fresh = (time.monotonic() - self._last_fire_msg_time) < 1.0
            n = self._memory.clear_fire_with_observation(
                fire_detected=self._fire_detected,
                hold_active=hold_active,
                origin_xy=(ox, oy),
                origin_yaw=yaw,
                observation_fresh=fresh,
            )
            if n:
                self._dirty = True
                self.get_logger().info(
                    f'CLEARING_OBSERVATION_ACCEPTED fire cells={n} '
                    f'(memory={self._memory.occupied_count()})'
                )

        # filter_info: rare refresh only. KeepoutFilter resets mask_sub and
        # re-latches TransientLocal (has_updated_data_ full bounds) — do NOT
        # also republish the mask here (duplicate WARN + rebuild).
        self._maybe_publish_filter_info(force=False)
        # Mask: publish-on-change (rate-limited). Optional force interval (>0)
        # retained for experiments; launch sets 0 (Entry 023).
        now = time.monotonic()
        force_mask = (
            self._mask_force_s > 0.0
            and (now - self._last_mask_force) >= self._mask_force_s
            and self._memory.occupied_count() > 0
        )
        due_dirty = (
            self._dirty
            and (now - self._last_mask_pub) >= self._mask_min_period
        )
        if due_dirty or force_mask:
            self._publish_mask()
            self._dirty = False
            self._last_mask_pub = now
            if force_mask:
                self._last_mask_force = now
        self._publish_status()

    def _maybe_publish_filter_info(self, *, force: bool) -> bool:
        """Publish filter_info when due. Returns True if a message was sent."""
        now = time.monotonic()
        if force or (now - self._last_info_pub) >= self._info_refresh_s:
            self._publish_filter_info()
            self._last_info_pub = now
            return True
        return False

    # -- TF / publish ------------------------------------------------------

    def _lookup_scan(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                self._map_frame, self._scan_frame, rclpy.time.Time()
            )
        except Exception:
            return None

    def _lookup_base(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                self._map_frame, f'{self._robot}_base_link', rclpy.time.Time()
            )
        except Exception:
            return None

    def _robot_xy_map(self) -> Optional[Tuple[float, float]]:
        """Robot base_link XY in map frame for footprint mark exclusion."""
        tf = self._lookup_base()
        if tf is None:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _publish_filter_info(self) -> None:
        msg = CostmapFilterInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.type = 0  # KEEPOUT_FILTER
        msg.filter_mask_topic = self._mask_topic
        # KeepoutFilter requires default base/multiplier (OccupancyGrid→Costmap
        # conversion maps 100→LETHAL inside Costmap2D).
        msg.base = 0.0
        msg.multiplier = 1.0
        self._info_pub.publish(msg)

    def _occupancy_fingerprint(self) -> Tuple[int, Tuple[Tuple[int, int], ...]]:
        """Stable (count, sorted cell keys) for change detection."""
        keys = tuple(sorted(self._memory.cells.keys()))
        return (len(keys), keys)

    def _publish_mask(self) -> None:
        meta = self._memory.meta
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = meta.frame_id or self._map_frame
        msg.info.resolution = meta.resolution
        msg.info.width = meta.width
        msg.info.height = meta.height
        msg.info.origin.position.x = meta.origin_x
        msg.info.origin.position.y = meta.origin_y
        msg.info.origin.orientation.w = 1.0
        data = self._memory.build_occupancy_data()
        msg.data = data.tolist()
        self._mask_pub.publish(msg)
        self._last_mask_fp = self._occupancy_fingerprint()
        self._last_mask_pub = time.monotonic()

    def _publish_status(self) -> None:
        lidar_n = sum(
            1 for c in self._memory.cells.values() if c.source == CellSource.LIDAR)
        fire_n = sum(
            1 for c in self._memory.cells.values() if c.source == CellSource.FIRE)
        msg = String()
        msg.data = (
            f'cells={self._memory.occupied_count()} lidar={lidar_n} fire={fire_n} '
            f'fire_detected={self._fire_detected}'
        )
        self._diag_pub.publish(msg)


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMemoryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
