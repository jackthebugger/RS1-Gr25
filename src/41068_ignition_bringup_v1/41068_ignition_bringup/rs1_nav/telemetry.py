"""Time-aligned navigation telemetry for route-switch / stall diagnosis (Entry 028).

Two parts:

* Pure functions (``classify_motion``, ``parse_launch_log_line``) with no ROS
  dependency so the stall classifier is unit-testable.
* ``TelemetryRecorder`` — attaches to a ``NavObserver`` and records odometry
  velocity, raw (``cmd_vel_nav``) and smoothed (``cmd_vel``) commands, plan
  publications, behaviour-tree node status changes, path-bank state, memory
  status/events, and Nav2 launch-log events, all stamped with sim time so the
  ROS log timestamps and our samples share one time base.

Stall definition (justified in Entry 028): controller + local costmap run at
10 Hz, planner at 1 Hz, memory mask ≤ 1.5 s. Any legitimate transient settles
well inside 2 s, so a *stall* is ``|v| < 0.05 m/s and |w| < 0.10 rad/s`` for
≥ 2.0 s while the path bank reports ``FOLLOWING_PATH`` and the goal is > 0.5 m
away. A *rotate* interval is ``|w| ≥ 0.15`` with ``|v| < 0.05`` (legitimate
rotate-to-heading, reported separately, never counted as a stall).
"""

from __future__ import annotations

import csv
import math
import re
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


# ---------------------------------------------------------------------------
# Pure classification
# ---------------------------------------------------------------------------

@dataclass
class MotionSample:
    t: float            # sim time (s)
    v: float            # |odom linear| m/s
    w: float            # odom angular z rad/s
    v_nav: float = float('nan')   # raw controller cmd (cmd_vel_nav)
    v_cmd: float = float('nan')   # smoothed cmd (cmd_vel)
    w_cmd: float = float('nan')
    x: float = float('nan')
    y: float = float('nan')
    yaw: float = float('nan')
    bank_state: str = ''
    goal_dist: float = float('nan')


@dataclass
class Interval:
    kind: str
    start: float
    end: float

    @property
    def duration(self) -> float:
        return self.end - self.start


def classify_motion(
    samples: Sequence[MotionSample],
    *,
    t0: float,
    t1: float,
    stall_v: float = 0.05,
    stall_w: float = 0.10,
    rot_w: float = 0.15,
    min_stall_s: float = 2.0,
    min_rot_s: float = 0.5,
    state_filter: Optional[str] = 'FOLLOWING_PATH',
    min_goal_dist: float = 0.5,
) -> Dict[str, List[Interval]]:
    """Return ``{'stall': [...], 'rotate': [...], 'zero_cmd': [...]}``.

    ``zero_cmd`` marks runs where the smoothed command magnitude is zero while
    the robot is still > ``min_goal_dist`` from the goal (upstream zero
    velocity), independent of the state filter — used to attribute stalls.
    """
    out: Dict[str, List[Interval]] = {'stall': [], 'rotate': [], 'zero_cmd': []}
    win = [s for s in samples if t0 <= s.t <= t1]
    if not win:
        return out

    def _runs(pred, min_s: float, kind: str) -> List[Interval]:
        res: List[Interval] = []
        start: Optional[float] = None
        last_t: Optional[float] = None
        for s in win:
            ok = pred(s)
            if ok and start is None:
                start = s.t
            if not ok and start is not None:
                if last_t is not None and (last_t - start) >= min_s:
                    res.append(Interval(kind, start, last_t))
                start = None
            last_t = s.t
        if start is not None and last_t is not None and (last_t - start) >= min_s:
            res.append(Interval(kind, start, last_t))
        return res

    def _in_scope(s: MotionSample) -> bool:
        if state_filter and s.bank_state and s.bank_state != state_filter:
            return False
        if not math.isnan(s.goal_dist) and s.goal_dist <= min_goal_dist:
            return False
        return True

    out['stall'] = _runs(
        lambda s: _in_scope(s) and abs(s.v) < stall_v and abs(s.w) < stall_w,
        min_stall_s, 'stall')
    out['rotate'] = _runs(
        lambda s: _in_scope(s) and abs(s.v) < stall_v and abs(s.w) >= rot_w,
        min_rot_s, 'rotate')
    out['zero_cmd'] = _runs(
        lambda s: (
            (not math.isnan(s.goal_dist) and s.goal_dist > min_goal_dist)
            and not math.isnan(s.v_cmd) and abs(s.v_cmd) < 1e-3
            and not math.isnan(s.w_cmd) and abs(s.w_cmd) < 1e-3
        ),
        min_stall_s, 'zero_cmd')
    return out


_LOG_STAMP = re.compile(r'\[(INFO|WARN|ERROR|DEBUG)\]\s+\[(\d+)\.(\d+)\]\s+\[([^\]]+)\]:\s*(.*)$')

# (regex, short label) — Nav2 Humble + project node messages that explain
# why progress stops or a replan/preempt happens.
LAUNCH_LOG_PATTERNS: Tuple[Tuple[re.Pattern, str], ...] = tuple(
    (re.compile(p), label) for p, label in (
        (r'Received a goal, begin computing control effort', 'FOLLOW_PATH_START'),
        (r'Passing new path to controller', 'NEW_PATH_TO_CONTROLLER'),
        (r'detected collision ahead', 'RPP_COLLISION'),
        (r'Controller patience exceeded', 'CONTROLLER_PATIENCE'),
        (r'Failed to make progress', 'PROGRESS_FAIL'),
        (r'Received goal preemption request', 'BT_PREEMPT'),
        (r'Running (spin|backup|wait|drive_on_heading)', 'BEHAVIOR_RUN'),
        (r'(spin|backup|wait|drive_on_heading) completed successfully', 'BEHAVIOR_DONE'),
        (r'Received request to clear entirely the (\w+)', 'COSTMAP_CLEAR'),
        (r'Goal (succeeded|failed|canceled)', 'BT_GOAL_END'),
        (r'failed to create plan|Failed to create plan|Failed to create a plan', 'PLAN_FAIL'),
        (r'ROUTE_MARKED_BLOCKED', 'BANK_BLOCKED'),
        (r'Switching to Path', 'BANK_SWITCH'),
        (r'ROUTE_MARKED_AVAILABLE', 'BANK_AVAILABLE'),
        (r'Cleared local costmap after path-bank switch', 'BANK_LOCAL_CLEAR'),
        (r'Ignoring superseded NavigateToPose result', 'BANK_IGNORE_RESULT'),
        (r'resending goal once', 'BANK_ABORT_RESEND'),
        (r'suppressing abort-resend', 'BANK_RESEND_SUPPRESSED'),
        (r'CLEARING_OBSERVATION_ACCEPTED fire', 'MEM_FIRE_CLEAR'),
        (r'CLEARING_OBSERVATION_ACCEPTED lidar', 'MEM_LIDAR_CLEAR'),
        (r'THERMAL_FIRE_MARK', 'BRIDGE_MARK'),
        (r'Thermal positive without usable LiDAR range', 'BRIDGE_NO_RANGE'),
        (r'FIRE DETECTED AHEAD', 'THERMAL_FIRE_ON'),
        (r'Fire cleared', 'THERMAL_FIRE_OFF'),
        (r'Timed out waiting for transform|extrapolation into the (past|future)', 'TF_ERROR'),
        (r'Updating old filter mask', 'KEEPOUT_MASK_UPDATE'),
        (r'Failed to get result for follow_path in node halt', 'BT_HALT_FOLLOW'),
        (r'Received request to clear.*global_costmap', 'GLOBAL_CLEAR'),
    )
)


def parse_launch_log_line(line: str) -> Optional[Tuple[float, str, str, str]]:
    """Return ``(sim_time, node, label, message)`` when the line matches a known event."""
    m = _LOG_STAMP.search(line)
    if not m:
        return None
    stamp = float(m.group(2)) + float('0.' + m.group(3))
    node = m.group(4)
    msg = m.group(5)
    for pat, label in LAUNCH_LOG_PATTERNS:
        if pat.search(msg):
            return stamp, node, label, msg.strip()
    return None


# ---------------------------------------------------------------------------
# ROS recorder
# ---------------------------------------------------------------------------

class TelemetryRecorder:
    """Attach to a NavObserver; record samples/events; write CSV + event log."""

    def __init__(self, observer, *, launch_log_path: Optional[str] = None,
                 record_bt_log: bool = True):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry, Path, OccupancyGrid
        from nav2_msgs.msg import Costmap
        from std_msgs.msg import String
        from sensor_msgs.msg import Image

        self.node = observer
        self.samples: List[MotionSample] = []
        self.events: List[Tuple[float, str, str, str]] = []  # (t, source, label, text)
        self._launch_log_path = launch_log_path
        self._launch_log_pos = 0
        self._launch_seen = 0
        self._clock_pairs: List[Tuple[float, float]] = []

        self._odom = None
        self._cmd_nav = None
        self._cmd = None
        self._bank_state = ''
        self._mem_status = ''
        self._last_plan_stamp: Optional[float] = None
        self.plan_count = 0
        self.mask_count = 0
        self.costmap_count = 0
        self.thermal_count = 0
        self._thermal_first: Optional[float] = None
        self._thermal_last: Optional[float] = None
        self._bt_last: Dict[str, str] = {}

        n = self.node
        n.create_subscription(Odometry, 'odom', self._on_odom, 10)
        n.create_subscription(Twist, 'cmd_vel_nav', self._on_cmd_nav, 10)
        n.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        n.create_subscription(Path, 'plan', self._on_plan, 5)
        n.create_subscription(String, 'path_bank/state', self._on_bank_state, 10)
        n.create_subscription(String, 'obstacle_memory/status', self._on_mem_status, 10)
        n.create_subscription(String, 'obstacle_memory/events', self._on_mem_event, 50)
        n.create_subscription(Costmap, 'global_costmap/costmap_raw', self._on_costmap, 2)
        n.create_subscription(Image, 'thermal/image', self._on_thermal, 5)
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        n.create_subscription(OccupancyGrid, 'obstacle_memory/mask', self._on_mask, latched)
        if record_bt_log:
            try:
                from nav2_msgs.msg import BehaviorTreeLog
                n.create_subscription(
                    BehaviorTreeLog, 'behavior_tree_log', self._on_bt_log, 20)
            except ImportError:
                pass

    # -- time ---------------------------------------------------------------

    def now(self) -> float:
        """Sim time (s). Also records a (wall, sim) pair for log alignment."""
        sim = self.node.get_clock().now().nanoseconds * 1e-9
        wall = time.time()
        pairs = self._clock_pairs
        if not pairs or wall - pairs[-1][0] >= 0.5:
            pairs.append((wall, sim))
            if len(pairs) > 20000:
                del pairs[:10000]
        return sim

    def wall_to_sim(self, wall: float) -> float:
        """Map an rcutils (wall-clock) log stamp onto the sim-time axis.

        rcutils logging always stamps with system time even under
        ``use_sim_time``; samples are sim-stamped. Piecewise-linear
        interpolation over recorded pairs keeps both on one axis even if the
        real-time factor drifts.
        """
        pairs = self._clock_pairs
        if not pairs:
            return wall
        if wall <= pairs[0][0]:
            return pairs[0][1] - (pairs[0][0] - wall)
        if wall >= pairs[-1][0]:
            return pairs[-1][1] + (wall - pairs[-1][0])
        lo, hi = 0, len(pairs) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if pairs[mid][0] <= wall:
                lo = mid
            else:
                hi = mid
        (w0, s0), (w1, s1) = pairs[lo], pairs[hi]
        if w1 <= w0:
            return s0
        return s0 + (s1 - s0) * (wall - w0) / (w1 - w0)

    # -- callbacks ----------------------------------------------------------

    def _on_odom(self, msg) -> None:
        self._odom = msg

    def _on_cmd_nav(self, msg) -> None:
        self._cmd_nav = msg

    def _on_cmd(self, msg) -> None:
        self._cmd = msg

    def _on_plan(self, msg) -> None:
        self.plan_count += 1
        t = self.now()
        gap = None if self._last_plan_stamp is None else t - self._last_plan_stamp
        self._last_plan_stamp = t
        self.events.append((t, 'plan', 'PLAN_PUBLISHED',
                            f'poses={len(msg.poses)} gap={gap if gap is None else round(gap, 2)}'))

    def _on_bank_state(self, msg) -> None:
        if msg.data != self._bank_state:
            self.events.append((self.now(), 'path_bank', 'BANK_STATE', msg.data))
        self._bank_state = msg.data

    def _on_mem_status(self, msg) -> None:
        self._mem_status = msg.data

    def _on_mem_event(self, msg) -> None:
        label = msg.data.split(' ', 1)[0]
        self.events.append((self.now(), 'obstacle_memory', label, msg.data))

    def _on_costmap(self, _msg) -> None:
        self.costmap_count += 1

    def _on_mask(self, _msg) -> None:
        self.mask_count += 1
        self.events.append((self.now(), 'obstacle_memory', 'MASK_PUBLISHED', ''))

    def _on_thermal(self, _msg) -> None:
        self.thermal_count += 1
        t = time.monotonic()
        if self._thermal_first is None:
            self._thermal_first = t
        self._thermal_last = t

    def _on_bt_log(self, msg) -> None:
        # BehaviorTreeLog stamps are taken from the bt_navigator clock (sim).
        for ev in msg.event_log:
            name = ev.node_name
            status = ev.current_status
            if self._bt_last.get(name) == status:
                continue
            self._bt_last[name] = status
            t = ev.timestamp.sec + ev.timestamp.nanosec * 1e-9
            self.events.append((t, 'bt', 'BT_NODE', f'{name} {ev.previous_status}->{status}'))

    # -- sampling -----------------------------------------------------------

    def thermal_rate_hz(self) -> float:
        if self._thermal_first is None or self._thermal_last is None:
            return 0.0
        span = self._thermal_last - self._thermal_first
        if span <= 0.0:
            return 0.0
        return (self.thermal_count - 1) / span

    def reset_thermal_rate(self) -> None:
        self.thermal_count = 0
        self._thermal_first = None
        self._thermal_last = None

    def mark(self, label: str, text: str = '') -> float:
        t = self.now()
        self.events.append((t, 'test', label, text))
        return t

    def sample(self, pose: Optional[Tuple[float, float, float]],
               goal_xy: Optional[Tuple[float, float]]) -> MotionSample:
        v = w = float('nan')
        if self._odom is not None:
            tw = self._odom.twist.twist
            v = math.hypot(tw.linear.x, tw.linear.y)
            w = tw.angular.z
        v_nav = float('nan') if self._cmd_nav is None else float(self._cmd_nav.linear.x)
        v_cmd = float('nan') if self._cmd is None else float(self._cmd.linear.x)
        w_cmd = float('nan') if self._cmd is None else float(self._cmd.angular.z)
        x = y = yaw = float('nan')
        gd = float('nan')
        if pose is not None:
            x, y, yaw = pose
            if goal_xy is not None:
                gd = math.hypot(goal_xy[0] - x, goal_xy[1] - y)
        s = MotionSample(
            t=self.now(), v=v, w=w, v_nav=v_nav, v_cmd=v_cmd, w_cmd=w_cmd,
            x=x, y=y, yaw=yaw, bank_state=self._bank_state, goal_dist=gd)
        self.samples.append(s)
        return s

    def poll_launch_log(self) -> int:
        """Ingest new launch-log lines; returns number of new events."""
        if not self._launch_log_path:
            return 0
        added = 0
        try:
            with open(self._launch_log_path, 'r', errors='replace') as fh:
                fh.seek(self._launch_log_pos)
                for line in fh:
                    parsed = parse_launch_log_line(line)
                    if parsed is not None:
                        t_wall, node, label, msg = parsed
                        self.events.append((self.wall_to_sim(t_wall), node, label, msg))
                        added += 1
                self._launch_log_pos = fh.tell()
        except OSError:
            return 0
        return added

    # -- analysis / output --------------------------------------------------

    def classify(self, t0: float, t1: float, **kw) -> Dict[str, List[Interval]]:
        return classify_motion(self.samples, t0=t0, t1=t1, **kw)

    def events_between(self, t0: float, t1: float,
                       labels: Optional[Iterable[str]] = None) -> List[Tuple[float, str, str, str]]:
        want = None if labels is None else set(labels)
        return sorted(
            [e for e in self.events if t0 <= e[0] <= t1 and (want is None or e[2] in want)],
            key=lambda e: e[0],
        )

    def count(self, label: str, t0: float, t1: float) -> int:
        return len(self.events_between(t0, t1, [label]))

    def write(self, prefix: str) -> Tuple[str, str]:
        csv_path = f'{prefix}_samples.csv'
        ev_path = f'{prefix}_events.txt'
        with open(csv_path, 'w', newline='') as fh:
            wr = csv.writer(fh)
            wr.writerow(['t', 'v_odom', 'w_odom', 'v_nav', 'v_cmd', 'w_cmd',
                         'x', 'y', 'yaw', 'bank_state', 'goal_dist'])
            for s in self.samples:
                wr.writerow([f'{s.t:.3f}', f'{s.v:.3f}', f'{s.w:.3f}', f'{s.v_nav:.3f}',
                             f'{s.v_cmd:.3f}', f'{s.w_cmd:.3f}', f'{s.x:.3f}', f'{s.y:.3f}',
                             f'{s.yaw:.3f}', s.bank_state, f'{s.goal_dist:.2f}'])
        with open(ev_path, 'w') as fh:
            for t, src, label, text in sorted(self.events, key=lambda e: e[0]):
                fh.write(f'{t:.3f} [{src}] {label} {text}\n')
        return csv_path, ev_path

    def summarize(self, t0: float, t1: float) -> str:
        cls = self.classify(t0, t1)
        lines = [f'window sim {t0:.1f}..{t1:.1f} ({t1 - t0:.1f}s)']
        for kind in ('stall', 'rotate', 'zero_cmd'):
            ivs = cls[kind]
            total = sum(i.duration for i in ivs)
            lines.append(
                f'  {kind}: n={len(ivs)} total={total:.1f}s '
                + ' '.join(f'[{i.start - t0:+.1f}s..{i.end - t0:+.1f}s]' for i in ivs[:8])
            )
        counts: Dict[str, int] = {}
        for e in self.events_between(t0, t1):
            counts[e[2]] = counts.get(e[2], 0) + 1
        lines.append('  events: ' + ', '.join(
            f'{k}={v}' for k, v in sorted(counts.items())))
        return '\n'.join(lines)
