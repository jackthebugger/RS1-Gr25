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


def test_format_speed_rounds_cleanly():
    assert gui.format_speed(1.234) == '1.23 m/s'
    assert gui.format_speed(0.0) == '0.00 m/s'


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
    assert gui.validate_goal(18.0, 0.0, 0.0) is None
    assert gui.validate_goal(100.0, 0.0, 0.0) is not None
    assert gui.validate_goal(0.0, 50.0, 0.0) is not None
    assert gui.validate_goal(0.0, 0.0, 20.0) is not None


def test_default_gaps_match_world_geometry():
    gaps = default_custom_world_gaps()
    assert PATH_A in gaps and PATH_B in gaps
    assert gaps[PATH_A].x == 8.0 and math.isclose(gaps[PATH_A].y, 5.5)
    assert gaps[PATH_B].x == 8.0 and math.isclose(gaps[PATH_B].y, -9.5)
    assert gaps[PATH_A].label == 'Path A'
    assert gaps[PATH_B].label == 'Path B'


def test_load_forest_gap_config_from_source_tree():
    cfg = load_forest_gap_config()
    assert cfg.trigger_distance_m == 5.0
    assert set(cfg.gaps.keys()) == {PATH_A, PATH_B}
    assert cfg.gaps[PATH_A].label == 'Path A'


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

    ok2, msg2 = mgr.request_path_b()
    assert not ok2 and 'already' in msg2.lower()

    ok3, _ = mgr.clear_obstacles()
    assert ok3 and mgr.state == ObstacleState.IDLE
    assert mgr.active_names == []


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
    assert win.block_path_a_button.cget('text') == 'BLOCK PATH A'
    assert win.block_path_b_button.cget('text') == 'BLOCK PATH B'
    assert win.random_block_button.cget('text') == 'RANDOM BLOCK'
    assert win.clear_obstacle_button.cget('text') == 'CLEAR OBSTACLES'
    assert win.goal_x_var.get() == '18'
    assert win.goal_y_var.get() == '0'
    assert win.goal_yaw_var.get() == '0'
    win.set_mission_running(True)
    assert win.mission_button.cget('text') == 'STOP MISSION'
    win.set_mission_running(False)
    assert win.mission_button.cget('text') == 'START MISSION'
    win.destroy()
