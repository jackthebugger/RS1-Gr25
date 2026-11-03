"""Mission-owner axes: phase, latch, inhibits, command_authority, generation.

Pure helpers used by path_bank_manager. Keep ROS I/O in the node; keep
policy decisions here so unit tests can exercise §C without spinning.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable, Optional, Set


class MissionPhase(str, Enum):
    IDLE = 'IDLE'
    STOPPING = 'STOPPING'
    STOPPED = 'STOPPED'
    NAVIGATING = 'NAVIGATING'
    SWITCHING = 'SWITCHING'
    RETURNING_TO_START = 'RETURNING_TO_START'
    COSTMAP_REBUILD = 'COSTMAP_REBUILD'
    HOLDING = 'HOLDING'
    MANUAL_NAV = 'MANUAL_NAV'


class CommandAuthority(str, Enum):
    NONE = 'NONE'
    NAV = 'NAV'
    BT_MANUAL = 'BT_MANUAL'
    TELEOP = 'TELEOP'


class SafetyInhibit(str, Enum):
    THERMAL_FIRE_PENDING = 'THERMAL_FIRE_PENDING'
    PENDING_FIRE_UNRESOLVED = 'PENDING_FIRE_UNRESOLVED'
    HOLD_REBUILD_TIMEOUT = 'HOLD_REBUILD_TIMEOUT'
    HOLD_PERSISTENT_SAFETY = 'HOLD_PERSISTENT_SAFETY'
    ALL_BLOCKED_NO_HOME = 'ALL_BLOCKED_NO_HOME'
    SENSOR_AUTHORITY_FAULT = 'SENSOR_AUTHORITY_FAULT'


# Terminal inhibits leave STOPPED rather than HOLDING after raise settles.
TERMINAL_INHIBITS = frozenset({
    SafetyInhibit.PENDING_FIRE_UNRESOLVED,
    SafetyInhibit.HOLD_REBUILD_TIMEOUT,
    SafetyInhibit.HOLD_PERSISTENT_SAFETY,
    SafetyInhibit.ALL_BLOCKED_NO_HOME,
    SafetyInhibit.SENSOR_AUTHORITY_FAULT,
})

ALWAYS_REJECT_START = frozenset({
    SafetyInhibit.THERMAL_FIRE_PENDING,
    SafetyInhibit.PENDING_FIRE_UNRESOLVED,
    SafetyInhibit.SENSOR_AUTHORITY_FAULT,
    # Phase Two §3.5: ALL_BLOCKED_NO_HOME / NO_SAFE_ROUTE always rejects
    # until StopMission (or explicit clear) removes it — no Start exemption.
    SafetyInhibit.ALL_BLOCKED_NO_HOME,
})

MOTION_PHASES = frozenset({
    MissionPhase.STOPPING,
    MissionPhase.NAVIGATING,
    MissionPhase.SWITCHING,
    MissionPhase.RETURNING_TO_START,
    MissionPhase.HOLDING,
    MissionPhase.MANUAL_NAV,
    MissionPhase.COSTMAP_REBUILD,
})

ABORT_RESEND_BLOCKED_PHASES = frozenset({
    MissionPhase.STOPPING,
    MissionPhase.STOPPED,
    MissionPhase.HOLDING,
    MissionPhase.COSTMAP_REBUILD,
})


@dataclass
class MissionOwnerState:
    phase: MissionPhase = MissionPhase.IDLE
    operator_stop_latch: bool = False
    active_inhibits: Set[SafetyInhibit] = field(default_factory=set)
    command_authority: CommandAuthority = CommandAuthority.NONE
    stop_reason: str = ''
    detail: str = ''
    mission_id: int = 0
    generation: int = 0

    def bump_generation(self) -> int:
        self.generation += 1
        return self.generation

    def new_mission_id(self) -> int:
        self.mission_id += 1
        return self.mission_id

    def inhibit_codes(self) -> list[str]:
        return sorted(i.value for i in self.active_inhibits)

    def has_blocking_inhibit(self) -> bool:
        return bool(self.active_inhibits)

    def abort_resend_forbidden(self) -> bool:
        """Entry 023/024 hard gate — §C3."""
        if self.operator_stop_latch:
            return True
        if self.command_authority == CommandAuthority.NONE:
            return True
        if self.phase in ABORT_RESEND_BLOCKED_PHASES:
            return True
        return False

    def raise_inhibit(self, code: SafetyInhibit) -> None:
        self.active_inhibits.add(code)
        self.command_authority = CommandAuthority.NONE
        self.bump_generation()
        if code in TERMINAL_INHIBITS:
            self.phase = MissionPhase.STOPPED
            self.stop_reason = code.value
        else:
            self.phase = MissionPhase.HOLDING
            self.stop_reason = code.value

    def clear_inhibit(self, code: SafetyInhibit) -> bool:
        """Clear one inhibit via §S. Returns True if set changed."""
        if code not in self.active_inhibits:
            return False
        self.active_inhibits.discard(code)
        if not self.active_inhibits and self.phase == MissionPhase.HOLDING:
            # §C1: last inhibit cleared → STOPPED (latch) or IDLE.
            self.phase = (
                MissionPhase.STOPPED if self.operator_stop_latch else MissionPhase.IDLE
            )
            if not self.operator_stop_latch:
                self.stop_reason = ''
            # Gate stays NONE until explicit Start/ManualNav/Teleop.
            self.command_authority = CommandAuthority.NONE
        return True

    def clear_inhibits(self, codes: Iterable[SafetyInhibit]) -> None:
        for code in list(codes):
            self.clear_inhibit(code)

    def begin_operator_stop(self) -> None:
        self.operator_stop_latch = True
        self.stop_reason = 'OPERATOR_STOP'
        self.command_authority = CommandAuthority.NONE
        self.bump_generation()
        self.phase = MissionPhase.STOPPING

    def finish_operator_stop(self) -> None:
        self.phase = MissionPhase.STOPPED
        self.command_authority = CommandAuthority.NONE

    def start_reject_code(self) -> Optional[str]:
        """Return reject_code if StartMission must be refused, else None."""
        if self.phase in MOTION_PHASES:
            return f'PHASE_{self.phase.value}'
        if self.phase not in (MissionPhase.IDLE, MissionPhase.STOPPED):
            return f'PHASE_{self.phase.value}'
        for code in ALWAYS_REJECT_START:
            if code in self.active_inhibits:
                return code.value
        # Rebuild-recovery is allowed for HOLD_REBUILD_TIMEOUT alone — caller
        # handles that exception. HOLD_PERSISTENT_SAFETY is also blocked by
        # StartMission service (defense in depth). ALL_BLOCKED_NO_HOME is in
        # ALWAYS_REJECT_START (no longer exempted here).
        remaining = self.active_inhibits - {SafetyInhibit.HOLD_REBUILD_TIMEOUT}
        remaining -= {SafetyInhibit.HOLD_PERSISTENT_SAFETY}
        if remaining:
            return sorted(i.value for i in remaining)[0]
        return None

    def clear_all_blocked_on_stop(self) -> None:
        """StopMission rearm: drop ALL_BLOCKED_NO_HOME so Start can proceed.

        Phase Two §3.5 — today Stop did not clear this inhibit (permanent
        Start lockout). Discard only; phase/auth follow begin/finish stop.
        """
        self.active_inhibits.discard(SafetyInhibit.ALL_BLOCKED_NO_HOME)

    def accept_normal_start(self) -> None:
        self.new_mission_id()
        self.bump_generation()
        self.operator_stop_latch = False
        self.stop_reason = ''
        self.command_authority = CommandAuthority.NAV
        self.phase = MissionPhase.NAVIGATING

    def accept_manual_nav(self) -> None:
        self.bump_generation()
        self.operator_stop_latch = False
        self.stop_reason = ''
        self.command_authority = CommandAuthority.BT_MANUAL
        self.phase = MissionPhase.MANUAL_NAV

    def accept_teleop(self) -> None:
        self.bump_generation()
        self.command_authority = CommandAuthority.TELEOP
        # Phase stays IDLE or STOPPED.

    def disable_teleop(self) -> None:
        if self.command_authority == CommandAuthority.TELEOP:
            self.command_authority = CommandAuthority.NONE

    def disable_manual_nav(self) -> None:
        self.command_authority = CommandAuthority.NONE
        self.phase = (
            MissionPhase.STOPPED if self.operator_stop_latch else MissionPhase.IDLE
        )


# ---------------------------------------------------------------------------
# Hang-bound ladder helpers (Phase Two Wave 0) — keep ladder EXPLICIT.
#
#   Mission stuck N → attempt fresh home once → (plan fail | home N | T₂) → §3.2
#   T₁ (PATH_BLOCKED + command_authority NONE) / T_seed → §3.2 direct (no home)
#
# Soft-stop (cmd≈0 while FOLLOWING_PATH): treat as no-progress → recoverable
# stuck ticks, then terminal. Must not remain FOLLOWING_PATH on terminal.
# ---------------------------------------------------------------------------


def mission_stuck_action(
    *,
    recoveries: int,
    budget_n: int,
    home_already_attempted: bool,
    home_available: bool,
) -> str:
    """Decide next step after a mission stuck tick.

    Returns one of: ``'recover'``, ``'attempt_home'``, ``'terminal'``.
    """
    if recoveries < budget_n:
        return 'recover'
    if home_available and not home_already_attempted:
        return 'attempt_home'
    return 'terminal'


def home_stuck_action(
    *,
    recoveries: int,
    budget_n: int,
    elapsed_home_s: float,
    t2_s: float,
) -> str:
    """Home-mode stuck / wall-clock decision.

    Returns ``'recover'`` or ``'terminal'`` (never attempt another home).
    """
    if elapsed_home_s >= t2_s:
        return 'terminal'
    if recoveries >= budget_n:
        return 'terminal'
    return 'recover'


def path_blocked_none_timed_out(*, elapsed_s: float, t1_s: float) -> bool:
    """T₁: PATH_BLOCKED while command_authority=NONE → §3.2 direct."""
    return elapsed_s >= t1_s


def seed_timed_out(*, elapsed_s: float, t_seed_s: float) -> bool:
    """T_seed: deferred StartMission bank seed → §3.2 direct (no home)."""
    return elapsed_s >= t_seed_s
