import math
import sys
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / '41068_ignition_bringup_v1' / '41068_ignition_bringup' / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / '41068_ignition_bringup_v1' / '41068_ignition_bringup'))

import robot_status_gui as gui
from rs1_nav.forest_obstacle_manager import (
    PATH_A,
    PATH_B,
    ForestGapConfig,
    ForestObstacleManager,
    GapSpec,
    ObstacleState,
    default_custom_world_gaps,
    load_forest_gap_config,
)


class _FakeImage:
    """Minimal sensor_msgs/Image stand-in for conversion unit tests."""

    def __init__(self, encoding, width, height, data, step=None):
        self.encoding = encoding
        self.width = width
        self.height = height
        self.data = data
        channels = 1 if encoding == 'mono8' else 2 if encoding == 'mono16' else 3
        if encoding in ('rgba8', 'bgra8'):
            channels = 4
        self.step = step if step is not None else width * channels


def test_format_speed_rounds_cleanly():
    assert gui.format_speed(1.234) == '1.23 m/s'
    assert gui.format_speed(0.0) == '0.00 m/s'


def test_rgb_image_to_ppm_rgb8():
    # 2x1 rgb8: red, green
    data = bytes([255, 0, 0, 0, 255, 0])
    img = _FakeImage('rgb8', 2, 1, data)
    ppm, err = gui.rgb_image_to_ppm(img)
    assert err is None and ppm is not None
    assert ppm.startswith(b'P6\n2 1\n255\n')
    assert ppm.endswith(data)


def test_rgb_image_to_ppm_rejects_unknown():
    img = _FakeImage('yuv422', 1, 1, b'\x00\x00')
    ppm, err = gui.rgb_image_to_ppm(img)
    assert ppm is None and 'unsupported' in err


def test_thermal_image_to_ppm_mono16():
    # 2x1 mono16 little-endian: cold then hot
    data = bytes([10, 0, 200, 0])
    img = _FakeImage('mono16', 2, 1, data)
    ppm, err = gui.thermal_image_to_ppm(img)
    assert err is None and ppm is not None
    assert ppm.startswith(b'P6\n2 1\n255\n')
    assert len(ppm) == len(b'P6\n2 1\n255\n') + 2 * 3


def test_camera_display_hz_defaults_at_or_below_sensor():
    assert gui.DEFAULT_CAMERA_DISPLAY_HZ <= 3.0
    assert gui.DEFAULT_THERMAL_DISPLAY_HZ <= 10.0


def test_parse_path_bank_active_id():
    status = (
        'state=FOLLOWING_PATH\n'
        'switches=1\n'
        'nav_mode=bank\n'
        'active_id=2\n'
        'all_blocked=False\n'
    )
    assert gui.parse_path_bank_active_id(status) == 2
    assert gui.parse_path_bank_active_id('active_id=None') is None
    assert gui.parse_path_bank_active_id('') is None


def test_mission_status_from_path_bank_mapping():
    assert gui.mission_status_from_path_bank('FOLLOWING_PATH', 1) == (
        'navigating to goal (path 1 active)'
    )
    assert gui.mission_status_from_path_bank('SWITCHING_PATH', None) == (
        'navigating to goal'
    )
    assert gui.mission_status_from_path_bank('RETURNING_TO_START') == (
        'all paths blocked - returning to home'
    )
    assert gui.mission_status_from_path_bank('NO_PATH_AVAILABLE') == (
        'all paths blocked'
    )
    assert gui.mission_status_from_path_bank('RETURNED_TO_START') == (
        'returned to start'
    )
    assert gui.mission_status_from_path_bank('ARRIVED') == 'complete'
    assert gui.mission_status_from_path_bank('FAILED') == 'failed'
    assert gui.mission_status_from_path_bank('IDLE') is None
    assert gui.mission_status_from_path_bank('NO_SAFE_ROUTE') == 'no safe route'
    assert gui.mission_status_from_path_bank('ALL_BLOCKED_NO_HOME') == (
        'all blocked - no home route'
    )


def test_mission_banner_maps_no_safe_route_inhibits():
    assert gui.mission_banner_from_owner(
        'HOLDING',
        active_inhibits=['ALL_BLOCKED_NO_HOME'],
    ) == 'HOLDING: all blocked - no home route'
    assert gui.mission_banner_from_owner(
        'STOPPED',
        active_inhibits=['NO_SAFE_ROUTE'],
    ) == 'STOPPED: no safe route'


def test_boot_mission_banner_is_idle_not_stopped():
    win = gui.RobotStatusWindow('husky1')
    win.update_idletasks()
    assert win.mission_var.get() == 'Mission: IDLE'
    assert 'STOPPED' not in win.mission_var.get()
    win.destroy()


def test_mission_banner_from_owner_idle_vs_stopped_vs_inhibit():
    assert gui.mission_banner_from_owner('IDLE') == 'IDLE'
    assert gui.mission_banner_from_owner(
        'STOPPED', operator_stop_latch=True, stop_reason='OPERATOR_STOP'
    ) == 'STOPPED'
    assert gui.mission_banner_from_owner(
        'HOLDING',
        active_inhibits=['THERMAL_FIRE_PENDING'],
    ) == 'HOLDING: thermal fire pending - holding'
    assert gui.mission_banner_from_owner(
        'STOPPED',
        active_inhibits=['PENDING_FIRE_UNRESOLVED'],
    ) == 'STOPPED: pending fire unresolved - stopped'
    assert gui.mission_banner_from_owner(
        'NAVIGATING', mission_running=True, command_authority='NAV'
    ) == 'running'


def test_start_mission_action_never_silent_ntp_by_default():
    assert gui.start_mission_action(
        client_available=True, service_ready=True, allow_ntp_fallback=False
    ) == 'start_mission'
    assert gui.start_mission_action(
        client_available=True, service_ready=False, allow_ntp_fallback=False
    ) == 'wait_mission_owner'
    assert gui.start_mission_action(
        client_available=False, service_ready=False, allow_ntp_fallback=False
    ) == 'fail_closed'
    assert gui.start_mission_action(
        client_available=False, service_ready=False, allow_ntp_fallback=True
    ) == 'ntp_fallback'


def test_start_mission_unavailable_banner_string():
    """M2a: client None → exact STARTMISSION_UNAVAILABLE banner (not TIMEOUT)."""
    win = gui.RobotStatusWindow('husky1')
    win.update_idletasks()
    win.set_mission_running(True)
    win.set_mission_status('rejected: STARTMISSION_UNAVAILABLE')
    win.set_mission_running(False)
    assert 'STARTMISSION_UNAVAILABLE' in win.mission_var.get()
    assert 'TIMEOUT' not in win.mission_var.get()
    assert win.mission_button.cget('text') == 'START MISSION'
    win.destroy()


def test_start_mission_timeout_banner_distinct_from_unavailable():
    """M2b: owner-down timeout uses TIMEOUT_WAITING_MISSION_OWNER."""
    win = gui.RobotStatusWindow('husky1')
    win.update_idletasks()
    win.set_mission_status('rejected: TIMEOUT_WAITING_MISSION_OWNER')
    assert 'TIMEOUT_WAITING_MISSION_OWNER' in win.mission_var.get()
    assert 'STARTMISSION_UNAVAILABLE' not in win.mission_var.get()
    win.set_mission_status('waiting for mission owner')
    assert 'WAITING FOR MISSION OWNER' in win.mission_var.get()
    win.destroy()


def test_set_mission_status_shows_reject_code_on_banner():
    win = gui.RobotStatusWindow('husky1')
    win.update_idletasks()
    win.set_mission_status('rejected: PHASE_HOLDING')
    assert 'REJECTED: PHASE_HOLDING' in win.mission_var.get()
    assert win.mission_label.cget('bg') == '#dc2626'
    win.set_mission_status('rejected: TIMEOUT_WAITING_MISSION_OWNER')
    assert 'TIMEOUT_WAITING_MISSION_OWNER' in win.mission_var.get()
    win.set_mission_status('idle')
    assert win.mission_var.get() == 'Mission: IDLE'
    assert win.mission_label.cget('bg') == '#64748b'
    win.destroy()


def test_set_mission_status_return_home_colors():
    win = gui.RobotStatusWindow('husky1')
    win.update_idletasks()
    win.set_mission_status('all paths blocked - returning to home')
    assert 'RETURNING TO HOME' in win.mission_var.get()
    assert win.mission_label.cget('bg') == '#d97706'
    win.set_mission_status('returned to start')
    assert 'RETURNED TO START' in win.mission_var.get()
    assert win.mission_label.cget('bg') == '#16a34a'
    win.set_mission_status('navigating to goal (path 2 active)')
    assert 'PATH 2 ACTIVE' in win.mission_var.get()
    assert win.mission_label.cget('bg') == '#16a34a'
    win.destroy()


def test_fire_status_label_is_thermal_not_obstacle():
    """Fire Detected line must reflect thermal bool, not forest obstacle state."""
    win = gui.RobotStatusWindow('husky1')
    win.update_idletasks()
    status = gui.RobotStatus(fire_detected=True)
    win.set_status(status)
    assert win.fire_var.get() == 'Fire detected: yes'
    status.fire_detected = False
    win.set_status(status)
    assert win.fire_var.get() == 'Fire detected: no'
    # Forest-gap helper must not be required for the fire line.
    win.set_dyn_obstacle_status('Obstacle: ACTIVE (Path A)')
    assert win.fire_var.get() == 'Fire detected: no'
    win.destroy()


def test_detect_obstacle_uses_threshold():
    assert gui.detect_obstacle([3.0, 3.0, 0.25, 3.0], threshold=0.5) is True
    assert gui.detect_obstacle([3.0, 2.0, 1.5, 3.0], threshold=0.5) is False
    assert gui.detect_obstacle([], threshold=0.5) is False


def test_parse_coordinate_rejects_invalid():
    assert gui.parse_coordinate('', 'X')[1] is not None
    assert gui.parse_coordinate('abc', 'X')[1] is not None
    value, err = gui.parse_coordinate('18.0', 'X')
    assert err is None and value == 18.0


def test_validate_goal_bounds():
    assert gui.validate_goal(16.0, -0.1, 0.0) is None
    assert gui.validate_goal(18.0, 0.0, 0.0) is None  # still valid coords (forest)
    assert gui.validate_goal(100.0, 0.0, 0.0) is not None
    assert gui.validate_goal(0.0, 50.0, 0.0) is not None
    assert gui.validate_goal(0.0, 0.0, 20.0) is not None
    # bush_trail 60×60 allows |Y| up to ~29
    assert gui.validate_goal(0.0, 21.0, 0.0) is None


def test_default_gaps_match_world_geometry():
    gaps = default_custom_world_gaps()
    assert PATH_A in gaps and PATH_B in gaps
    assert gaps[PATH_A].x == 8.0 and math.isclose(gaps[PATH_A].y, 5.5)
    assert gaps[PATH_B].x == 8.0 and math.isclose(gaps[PATH_B].y, -9.5)
    assert gaps[PATH_A].label == 'Path A'
    assert gaps[PATH_B].label == 'Path B'


def test_load_forest_gap_config_from_source_tree():
    """Single source of truth: forest_gaps_custom_world_1.yaml (trigger 7.0)."""
    cfg = load_forest_gap_config(world_name='custom_world_1')
    assert cfg.world_name == 'custom_world_1'
    assert cfg.trigger_distance_m == 7.0
    assert set(cfg.gaps.keys()) == {PATH_A, PATH_B}
    assert cfg.gaps[PATH_A].label == 'Path A'


def test_load_bush_trail_block_config():
    cfg = load_forest_gap_config(world_name='bush_trail_world')
    assert cfg.world_name == 'bush_trail_world'
    assert set(cfg.gaps.keys()) == {PATH_A, PATH_B}
    assert cfg.gaps[PATH_A].y < 0.0  # southern corridor Path A
    assert cfg.gaps[PATH_B].y > 0.0  # northern corridor Path B
    assert cfg.size_z >= 1.5


def test_obstacle_manager_path_a_5m_trigger_and_clear():
    world = MagicMock()
    world.is_available.return_value = True
    world.spawn_obstacle.return_value = True
    world.remove_model.return_value = True

    cfg = ForestGapConfig(
        trigger_distance_m=5.0,
        gaps=default_custom_world_gaps(),
    )
    mgr = ForestObstacleManager(cfg, world=world)

    ok, _ = mgr.request_path_a()
    assert ok and mgr.state == ObstacleState.WAITING
    assert mgr.selected_gap.key == PATH_A

    mgr.tick((-18.0, 3.0))
    assert mgr.state == ObstacleState.WAITING
    world.spawn_obstacle.assert_not_called()

    status = mgr.tick((8.0, 2.0))  # within 5 m of Path A
    assert mgr.state == ObstacleState.ACTIVE
    assert status is not None and 'ACTIVE' in status
    spec = world.spawn_obstacle.call_args[0][0]
    assert spec.x == 8.0 and spec.y == 5.5

    # Dual-block (G5): a different gap may be armed while A is ACTIVE.
    ok2, msg2 = mgr.request_path_b()
    assert ok2 and 'Path B' in msg2
    assert any(g.key == PATH_B for g in mgr.pending_gaps)
    # Same gap cannot be re-armed.
    ok_same, msg_same = mgr.request_path_a()
    assert not ok_same and 'already' in msg_same.lower()

    ok3, _ = mgr.clear_obstacles()
    assert ok3 and mgr.state == ObstacleState.IDLE
    assert mgr.active_names == []
    assert mgr.pending_gaps == [] and mgr.active_gaps == []


def test_obstacle_manager_selected_path_trigger_not_region():
    """Approaching Path A must not spawn a pending Path B obstacle."""
    world = MagicMock()
    world.is_available.return_value = True
    world.spawn_obstacle.return_value = True
    world.remove_model.return_value = True
    cfg = ForestGapConfig(trigger_distance_m=5.0, gaps=default_custom_world_gaps())
    mgr = ForestObstacleManager(cfg, world=world)
    mgr.request_path_b()
    mgr.tick((8.0, 3.0))  # near Path A, ~12.5 m from Path B
    assert mgr.state == ObstacleState.WAITING
    world.spawn_obstacle.assert_not_called()
    mgr.tick((8.0, -9.0))  # within 5 m of Path B
    assert mgr.state == ObstacleState.ACTIVE
    assert world.spawn_obstacle.call_args[0][0].y == -9.5


def test_obstacle_manager_can_block_both_paths_separately():
    world = MagicMock()
    world.is_available.return_value = True
    world.spawn_obstacle.return_value = True
    world.remove_model.return_value = True
    cfg = ForestGapConfig(
        trigger_distance_m=5.0,
        gaps=default_custom_world_gaps(),
    )
    mgr = ForestObstacleManager(cfg, world=world)

    ok, message = mgr.request_both()
    assert ok and 'Path A' in message and 'Path B' in message

    first = mgr.tick((8.0, 5.5))
    assert mgr.state == ObstacleState.ACTIVE
    assert first is not None and 'Path A' in first
    assert len(mgr.active_names) == 1

    second = mgr.tick((8.0, -9.5))
    assert second is not None and 'Path B' in second
    assert len(mgr.active_names) == 2
    assert len(world.spawn_obstacle.call_args_list) == 2


def test_random_gap_selection_covers_both():
    world = MagicMock()
    world.is_available.return_value = True
    world.spawn_obstacle.return_value = True
    world.remove_model.return_value = True
    cfg = ForestGapConfig(gaps=default_custom_world_gaps())

    seen = set()
    for seed in range(40):
        mgr = ForestObstacleManager(cfg, world=world, seed=seed)
        mgr.request_random()
        seen.add(mgr.selected_gap.key)
        mgr.clear_obstacles()
        if seen == {PATH_A, PATH_B}:
            break
    assert seen == {PATH_A, PATH_B}


def test_ui_has_unified_mission_and_four_obstacle_buttons():
    win = gui.RobotStatusWindow('husky1')
    win.update_idletasks()
    assert win.mission_button.cget('text') == 'START MISSION'
    assert not hasattr(win, 'go_button') or getattr(win, 'go_button', None) is None
    assert not hasattr(win, 'start_button')
    assert not hasattr(win, 'stop_button')
    assert win.block_path_a_button.cget('text') == 'BLOCK FIRE A'
    assert win.block_path_b_button.cget('text') == 'BLOCK FIRE B'
    assert win.random_block_button.cget('text') == 'RANDOM FIRE'
    assert win.clear_obstacle_button.cget('text') == 'CLEAR FIRE'
    assert win.goal_x_var.get() == '16'
    assert win.goal_y_var.get() in ('-0.1', '-0.10')
    assert win.goal_yaw_var.get() == '0'
    win.set_mission_running(True)
    assert win.mission_button.cget('text') == 'STOP MISSION'
    win.set_mission_running(False)
    assert win.mission_button.cget('text') == 'START MISSION'
    win.destroy()


def test_entry063_speed_presets_honest_maximum():
    """Wave-1 Fast/MAXIMUM = 0.50/1.2; maps to vx_max + smoother (not RPP)."""
    assert gui.SPEED_PRESETS == {'slow': 0.15, 'medium': 0.30, 'fast': 0.50}
    assert gui.SPEED_ANGULAR_MAX == 1.2
    writes = gui.speed_param_writes('fast')
    assert writes['FollowPath.vx_max'] == 0.50
    assert writes['FollowPath.wz_max'] == 1.2
    assert writes['max_velocity'] == [0.50, 0.0, 1.2]
    assert writes['min_velocity'] == [-0.50, 0.0, -1.2]
    assert 'desired_linear_vel' not in writes
    slow = gui.speed_param_writes('slow')
    assert slow['FollowPath.vx_max'] == 0.15
    assert slow['max_velocity'][0] == 0.15


def test_entry063_speed_radio_labels_match_applied():
    win = gui.RobotStatusWindow('husky1')
    win.update_idletasks()
    assert win.speed_buttons['fast'].cget('text') == 'Fast 0.50'
    assert win.speed_buttons['medium'].cget('text') == 'Medium 0.30'
    assert win.speed_buttons['slow'].cget('text') == 'Slow 0.15'
    win.destroy()
