#!/usr/bin/env python3
"""Mission layer: drive the robot to a goal and report what the stack did.

This is the top of the autonomy loop. It does not steer the robot and it does
not plan: Nav2 owns both. Its job is to give Nav2 a goal, then watch the stack
closely enough to say afterwards *why* the robot did what it did -- which plan
it was following, when that plan changed, whether an obstacle appeared, and
whether the robot genuinely arrived.

    send goal -> Nav2 plans -> DWB follows -> lidar updates costmaps
        -> plan becomes invalid -> Nav2 replans -> RPP follows -> goal reached

Everything is bounded. `run` takes a deadline and returns a MissionReport; it
cannot block indefinitely, and it stops the instant the outcome is decided
rather than waiting out a timeout.

The report is the evidence the tests assert on, so the same code path that a
user runs from scripts/basic_autonomy_demo.py is the one under test.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import rclpy

from .geometry import path_closest_approach
from .nav_observer import NavObserver, PathSnapshot

# How far a new plan must depart from the previous one to count as a genuine
# reroute rather than the same route republished.
#
# Nav2 replans every second for the whole mission, so most new plans are simply
# the remaining suffix of the old one: the length drops by roughly a second of
# travel each time while the geometry is unchanged. Judging by length would
# therefore report a "replan" continuously and mean nothing -- measured here,
# normal following shortens the plan by 1.2 m per cycle while diverging from the
# old route by only 0.04-0.16 m. Sideways divergence is the signal that the
# route itself changed; 1.5 m is well above the observed noise and well below
# the several metres a detour around an obstacle produces.
REPLAN_DIVERGENCE = 1.5  # metres

# Slightly looser than the controller's 0.25 m xy_goal_tolerance, so arriving
# within tolerance is never failed by measurement noise.
ARRIVAL_TOLERANCE = 0.45


@dataclass
class ReplanEvent:
    """A plan change large enough to count as a genuine reroute."""

    at_seconds: float
    robot_xy: Tuple[float, float]
    old_length: float
    new_length: float
    max_divergence: float

    def describe(self) -> str:
        return (
            f'replan at t={self.at_seconds:.1f}s, robot at '
            f'({self.robot_xy[0]:.2f}, {self.robot_xy[1]:.2f}): path length '
            f'{self.old_length:.2f} m -> {self.new_length:.2f} m, '
            f'diverging up to {self.max_divergence:.2f} m from the old route'
        )


@dataclass
class MissionReport:
    """Outcome and evidence for one goal."""

    goal: Tuple[float, float, float]
    reached: bool = False
    nav2_status: str = 'NOT_STARTED'
    duration: float = 0.0
    straight_line: float = 0.0
    distance_to_goal: Optional[float] = None
    distance_travelled: float = 0.0
    first_plan_length: Optional[float] = None
    final_plan_length: Optional[float] = None
    plans_received: int = 0
    replans: List[ReplanEvent] = field(default_factory=list)
    failure: str = ''

    def summary(self) -> str:
        verdict = 'REACHED' if self.reached else f'NOT REACHED ({self.nav2_status})'
        parts = [
            f'goal ({self.goal[0]:.2f}, {self.goal[1]:.2f}): {verdict}',
            f'took {self.duration:.1f}s',
            f'drove {self.distance_travelled:.2f} m',
        ]
        if self.distance_to_goal is not None:
            parts.append(f'stopped {self.distance_to_goal:.2f} m from the goal')
        if self.first_plan_length is not None:
            parts.append(f'first plan {self.first_plan_length:.2f} m')
        parts.append(f'{len(self.replans)} replan(s) from {self.plans_received} plans')
        if self.failure:
            parts.append(f'failure: {self.failure}')
        return ', '.join(parts)


def plan_divergence(old: PathSnapshot, new: PathSnapshot) -> float:
    """Farthest any point of the new plan sits from the old plan.

    Measured point-to-segment against the whole old path, so a route that goes
    around the other side of an obstacle scores high while a slightly
    resampled version of the same route scores near zero.
    """
    if not old.points or not new.points:
        return 0.0
    worst = 0.0
    # Sampling every few points keeps this cheap; plans have hundreds of poses
    # and are smooth, so nothing meaningful hides between samples.
    step = max(1, len(new.points) // 60)
    for point in new.points[::step]:
        distance = path_closest_approach(old.points, point)
        if distance is not None:
            worst = max(worst, distance)
    return worst


class MissionRunner:
    """Runs bounded NavigateToPose missions on top of a NavObserver."""

    def __init__(
        self,
        observer: NavObserver,
        *,
        logger: Optional[Callable[[str], None]] = None,
    ):
        self.obs = observer
        self._log = logger or (lambda message: print(message, flush=True))

    def log(self, message: str) -> None:
        self._log(message)

    # -- startup ----------------------------------------------------------

    def wait_until_ready(self, *, timeout: float = 180.0) -> bool:
        """Bounded check that every layer the mission depends on is live.

        Ordered cheapest-first so a broken stack reports the lowest layer that
        failed, which is nearly always the actual cause.
        """
        stages = (
            ('lidar scans arriving',
             lambda: self.obs.scan is not None, 60.0),
            ('EKF odometry arriving',
             lambda: self.obs.odom is not None, 40.0),
            ('TF husky map -> base_link resolving',
             lambda: self.obs.robot_pose() is not None, 60.0),
            ('SLAM map published',
             lambda: self.obs.map is not None, 90.0),
            ('Nav2 global costmap published',
             lambda: self.obs.global_costmap is not None, 60.0),
            ('Nav2 navigate_to_pose server available',
             lambda: self.obs.nav2_ready(timeout=0.5), 90.0),
        )
        deadline = time.monotonic() + timeout
        for description, predicate, stage_timeout in stages:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.log(f'FAIL  overall startup budget of {timeout:.0f}s exhausted '
                         f'before "{description}"')
                return False
            if not self.obs.wait_for(description, predicate,
                                     timeout=min(stage_timeout, remaining)):
                return False
        return True

    # -- one goal ---------------------------------------------------------

    def run(
        self,
        goal: Tuple[float, float, float],
        *,
        timeout: float,
        on_tick: Optional[Callable[[float, MissionReport], None]] = None,
        tick_interval: float = 1.0,
    ) -> MissionReport:
        """Navigate to `goal` and return what happened.

        `on_tick(elapsed, report)` is called about once a second while the robot
        drives. The replanning test uses it to inject an obstacle mid-mission;
        the demo uses it to print progress.
        """
        report = MissionReport(goal=goal)

        start_pose = self.obs.robot_pose()
        if start_pose is None:
            report.failure = 'no robot pose available, cannot start'
            report.nav2_status = 'NO_POSE'
            return report
        report.straight_line = math.hypot(goal[0] - start_pose[0],
                                          goal[1] - start_pose[1])
        self.log(f'GOAL  ({goal[0]:.2f}, {goal[1]:.2f}, yaw {goal[2]:.2f}) '
                 f'from ({start_pose[0]:.2f}, {start_pose[1]:.2f}); '
                 f'straight-line {report.straight_line:.2f} m, '
                 f'budget {timeout:.0f}s')

        # Only plans published after the goal is sent belong to this mission.
        plans_before = len(self.obs.paths)
        if not self.obs.send_goal(*goal):
            report.failure = 'Nav2 did not accept the goal'
            report.nav2_status = 'REJECTED'
            return report

        started = time.monotonic()
        last_pose = start_pose
        last_tick = 0.0
        reference_plan: Optional[PathSnapshot] = None

        while True:
            elapsed = time.monotonic() - started
            rclpy.spin_once(self.obs, timeout_sec=0.1)

            pose = self.obs.robot_pose()
            if pose is not None:
                step = math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1])
                # Ignore TF explosions (Gazebo/SLAM glitches publish kilometre jumps).
                if step < 3.0:
                    report.distance_travelled += step
                last_pose = pose

            # Track plans and decide which changes are real reroutes.
            mission_plans = self.obs.paths[plans_before:]
            report.plans_received = len(mission_plans)
            if mission_plans:
                newest = mission_plans[-1]
                if reference_plan is None:
                    reference_plan = newest
                    report.first_plan_length = newest.length()
                    self.log(f'  plan received: {newest.length():.2f} m, '
                             f'{len(newest.points)} poses')
                else:
                    divergence = plan_divergence(reference_plan, newest)
                    if divergence >= REPLAN_DIVERGENCE:
                        event = ReplanEvent(
                            at_seconds=elapsed,
                            robot_xy=(last_pose[0], last_pose[1]),
                            old_length=reference_plan.length(),
                            new_length=newest.length(),
                            max_divergence=divergence,
                        )
                        report.replans.append(event)
                        self.log(f'  {event.describe()}')
                        reference_plan = newest
                report.final_plan_length = newest.length()

            if self.obs.goal_finished():
                report.nav2_status = self.obs.goal_status_text()
                break

            if elapsed >= timeout:
                report.nav2_status = 'TIMEOUT'
                report.failure = f'goal not reached within {timeout:.0f}s'
                self.log(f'  giving up after {elapsed:.1f}s; cancelling the goal')
                self.obs.cancel_goal()
                break

            if on_tick is not None and elapsed - last_tick >= tick_interval:
                last_tick = elapsed
                on_tick(elapsed, report)

        report.duration = time.monotonic() - started

        # Trust geometry, not just the action result: confirm from TF that the
        # robot is actually where it was asked to go.
        self.obs.spin_for(0.5)
        final = self.obs.robot_pose()
        if final is not None:
            report.distance_to_goal = math.hypot(goal[0] - final[0], goal[1] - final[1])
        report.reached = (
            self.obs.goal_succeeded()
            and report.distance_to_goal is not None
            and report.distance_to_goal <= ARRIVAL_TOLERANCE
        )
        if self.obs.goal_succeeded() and not report.reached:
            report.failure = (
                f'Nav2 reported success but the robot is '
                f'{report.distance_to_goal:.2f} m from the goal'
            )
        self.log(f'  {report.summary()}')
        return report
