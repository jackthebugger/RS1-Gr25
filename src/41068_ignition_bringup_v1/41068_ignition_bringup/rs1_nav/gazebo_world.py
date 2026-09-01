"""Insert and remove real obstacles in a running Ignition Gazebo world.

This is the mechanism behind the dynamic-replanning demonstration. An obstacle
is inserted as a genuine Gazebo model, so it is seen by the simulated lidar
exactly like any other object: the scan shortens, the Nav2 costmaps mark the
cells, the current path becomes invalid and Nav2 replans. Nothing about the
navigation stack is told that an obstacle was added.

Ignition is driven through its `create` / `remove` services. Two details matter:

* The request is Gazebo Transport protobuf *text* format, in which a quoted
  string cannot span lines. The SDF is therefore emitted as a single line.
* The obstacle must be tall enough to intersect the lidar plane. The Husky's
  lidar sits 0.845 m above the ground, so a 0.5 m tall box would be completely
  invisible no matter how solid it is. `ObstacleSpec` defaults to 1.5 m and
  `warns_below_lidar` exposes the check.
"""

import math
import shutil
import subprocess
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

# Height of husky1_base_scan above the ground: scan_joint z=0.68 in
# husky.urdf.xacro plus the 0.1651 m wheel radius that lifts base_link.
HUSKY_LIDAR_HEIGHT = 0.845


@dataclass
class ObstacleSpec:
    """A box obstacle to insert into the world."""

    name: str = 'dynamic_barrier'
    x: float = 0.0
    y: float = 0.0
    size_x: float = 0.4
    size_y: float = 3.0
    size_z: float = 1.5
    yaw: float = 0.0

    @property
    def z(self) -> float:
        """Centre height that leaves the box resting on the ground."""
        return 0.5 * self.size_z

    @property
    def top_height(self) -> float:
        return self.size_z

    def warns_below_lidar(self) -> Optional[str]:
        """Return a warning if the lidar would scan straight over this box."""
        if self.top_height <= HUSKY_LIDAR_HEIGHT:
            return (
                f'obstacle "{self.name}" is {self.top_height:.2f} m tall but the '
                f'Husky lidar plane is at {HUSKY_LIDAR_HEIGHT:.2f} m, so it would '
                f'never be detected'
            )
        return None

    def to_sdf(self) -> str:
        """Single-line SDF. Static, so it cannot be shoved aside by the robot."""
        size = f'{self.size_x} {self.size_y} {self.size_z}'
        return (
            '<?xml version="1.0" ?>'
            f'<sdf version="1.8"><model name="{self.name}"><static>true</static>'
            '<link name="link">'
            f'<collision name="collision"><geometry><box><size>{size}'
            '</size></box></geometry></collision>'
            f'<visual name="visual"><geometry><box><size>{size}'
            '</size></box></geometry>'
            '<material><ambient>0.9 0.3 0.1 1</ambient>'
            '<diffuse>0.9 0.3 0.1 1</diffuse></material></visual>'
            '</link></model></sdf>'
        )


class GazeboWorld:
    """Thin, timeout-bounded wrapper around the Ignition world services."""

    def __init__(
        self,
        world_name: str,
        *,
        logger: Optional[Callable[[str], None]] = None,
        service_timeout: float = 8.0,
    ):
        self.world_name = world_name
        self.service_timeout = service_timeout
        self._log = logger or (lambda message: print(message, flush=True))
        # Gazebo Fortress ships `ign`; later releases use `gz`.
        self.command = 'ign' if shutil.which('ign') else 'gz'
        self.msg_prefix = 'ignition.msgs' if self.command == 'ign' else 'gz.msgs'

    # -- plumbing ---------------------------------------------------------

    def _service(self, service: str, request: str, reqtype: str) -> bool:
        argv = [
            self.command, 'service',
            '-s', f'/world/{self.world_name}/{service}',
            '--reqtype', f'{self.msg_prefix}.{reqtype}',
            '--reptype', f'{self.msg_prefix}.Boolean',
            '--timeout', str(int(self.service_timeout * 1000)),
            '--req', request,
        ]
        try:
            done = subprocess.run(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=self.service_timeout + 5.0,
                check=False,
            )
        except subprocess.TimeoutExpired:
            self._log(f'Gazebo {service} timed out after {self.service_timeout:.0f}s')
            return False
        except FileNotFoundError:
            self._log(f'Gazebo command "{self.command}" not found')
            return False

        output = (done.stdout or '').strip()
        # Ignition reports success in the reply body, not the exit code.
        if 'data: true' in output:
            return True
        self._log(f'Gazebo {service} failed: {output[:400]}')
        return False

    # -- public API -------------------------------------------------------

    def is_available(self) -> bool:
        """True when the world's service interface answers."""
        try:
            done = subprocess.run(
                [self.command, 'service', '-l'],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, timeout=10.0, check=False,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False
        return f'/world/{self.world_name}/create' in (done.stdout or '')

    def spawn_obstacle(self, spec: ObstacleSpec) -> bool:
        """Insert `spec` into the running world. Returns True on success."""
        warning = spec.warns_below_lidar()
        if warning:
            self._log(f'WARNING: {warning}')

        sdf = spec.to_sdf().replace('\\', '\\\\').replace("'", "\\'")
        qz = math.sin(0.5 * spec.yaw)
        qw = math.cos(0.5 * spec.yaw)
        request = (
            f"sdf: '{sdf}' "
            f'pose {{ '
            f'position {{ x: {spec.x:.4f} y: {spec.y:.4f} z: {spec.z:.4f} }} '
            f'orientation {{ x: 0 y: 0 z: {qz:.6f} w: {qw:.6f} }} '
            f'}} '
            f"name: '{spec.name}' allow_renaming: false"
        )
        ok = self._service('create', request, 'EntityFactory')
        if ok:
            self._log(
                f'Inserted obstacle "{spec.name}" at ({spec.x:.2f}, {spec.y:.2f}) '
                f'yaw {spec.yaw:.2f} rad, '
                f'{spec.size_x:.2f} x {spec.size_y:.2f} x {spec.size_z:.2f} m'
            )
        return ok

    def remove_model(self, name: str) -> bool:
        """Remove a model by name. Safe to call when it does not exist."""
        return self._service('remove', f"name: '{name}' type: MODEL", 'Entity')

    def wait_until_available(self, max_wait: float, poll_interval: float = 2.0) -> bool:
        """Bounded wait for the world services to appear."""
        deadline = time.monotonic() + max_wait
        while time.monotonic() < deadline:
            if self.is_available():
                return True
            time.sleep(poll_interval)
        return False


def barrier_across_heading(
    *,
    name: str,
    at: Tuple[float, float],
    heading: float,
    width: float = 4.0,
    thickness: float = 0.4,
    height: float = 1.5,
) -> ObstacleSpec:
    """A wall centred at `at` and lying across a path travelling at `heading`.

    Oriented perpendicular to the direction of travel and only `width` metres
    long, so it blocks the current route while leaving open ground on both
    sides. That matters for the replanning test: an obstacle with no way around
    it proves nothing, because failing to reach the goal is then correct.
    """
    return ObstacleSpec(
        name=name,
        x=at[0],
        y=at[1],
        size_x=thickness,
        size_y=width,
        size_z=height,
        yaw=heading,
    )


class PathBlocker:
    """Drop a real Gazebo barrier onto the current Nav2 path once the robot moves.

    The barrier is a genuine model, so the lidar, costmaps and planner see it
    the same way they would see a tree. Nothing in Nav2 is told that an
    obstacle was added -- that is the whole point of the replan demonstration.
    """

    def __init__(
        self,
        observer,
        world: GazeboWorld,
        *,
        min_travel: float = 1.0,
        look_ahead: float = 2.5,
        width: float = 2.5,
        thickness: float = 0.4,
        height: float = 1.5,
        name: str = 'replan_barrier',
        logger=None,
    ):
        self.observer = observer
        self.world = world
        self.min_travel = min_travel
        self.look_ahead = look_ahead
        self.width = width
        self.thickness = thickness
        self.height = height
        self.name = name
        self._log = logger or (lambda message: print(message, flush=True))
        self.injected = False
        self.spec: Optional[ObstacleSpec] = None
        self.injection_xy: Optional[Tuple[float, float]] = None

    def maybe_inject(self, _elapsed: float, report) -> bool:
        """Insert the barrier once the robot is moving along a plan. Idempotent."""
        if self.injected:
            return True
        if report.distance_travelled < self.min_travel:
            return False
        if report.plans_received < 1:
            return False
        path = self.observer.latest_path()
        pose = self.observer.robot_pose()
        if path is None or pose is None:
            return False

        from .geometry import point_ahead_on_path

        ahead = point_ahead_on_path(path.points, (pose[0], pose[1]), self.look_ahead)
        if ahead is None:
            return False
        (x, y), heading = ahead
        spec = barrier_across_heading(
            name=self.name,
            at=(x, y),
            heading=heading,
            width=self.width,
            thickness=self.thickness,
            height=self.height,
        )
        if not self.world.spawn_obstacle(spec):
            self._log('failed to insert replan barrier')
            return False
        self.spec = spec
        self.injection_xy = (x, y)
        self.injected = True
        self._log(
            f'injected barrier "{self.name}" at ({x:.2f}, {y:.2f}) '
            f'after {report.distance_travelled:.2f} m of travel, '
            f'{self.look_ahead:.2f} m ahead on the current plan'
        )
        return True
