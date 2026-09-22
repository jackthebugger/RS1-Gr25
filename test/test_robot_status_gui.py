import math
import sys
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / '41068_ignition_bringup_v1' / '41068_ignition_bringup' / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / '41068_ignition_bringup_v1' / '41068_ignition_bringup'))

import robot_status_gui as gui
from rs1_nav.forest_obstacle_manager import (
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
    assert 'gap_a' in gaps and 'gap_b' in gaps
    assert gaps['gap_a'].x == 8.0 and math.isclose(gaps['gap_a'].y, 5.5)
    assert gaps['gap_b'].x == 8.0 and math.isclose(gaps['gap_b'].y, -9.5)


def test_load_forest_gap_config_from_source_tree():
    cfg = load_forest_gap_config()
    assert cfg.trigger_distance_m == 5.0
    assert set(cfg.gaps.keys()) == {'gap_a', 'gap_b'}


def test_obstacle_manager_5m_trigger_and_clear():
    world = MagicMock()
    world.is_available.return_value = True
    world.spawn_obstacle.return_value = True
    world.remove_model.return_value = True

    cfg = ForestGapConfig(
        trigger_distance_m=5.0,
        gaps={
            'gap_a': GapSpec('gap_a', 'Gap A (north)', 8.0, 5.5),
            'gap_b': GapSpec('gap_b', 'Gap B (south)', 8.0, -9.5),
        },
    )
    mgr = ForestObstacleManager(cfg, world=world, force_gap='gap_a', seed=1)

    ok, _ = mgr.request_obstacle()
    assert ok and mgr.state == ObstacleState.WAITING
    assert mgr.selected_gap.key == 'gap_a'

    # Too far — still waiting
    mgr.tick((-18.0, 3.0))
    assert mgr.state == ObstacleState.WAITING
    world.spawn_obstacle.assert_not_called()

    # Within 5 m of gap A centre (8, 5.5) — region proximity
    status = mgr.tick((8.0, 2.0))
    assert mgr.state == ObstacleState.ACTIVE
    assert status is not None and 'ACTIVE' in status
    world.spawn_obstacle.assert_called_once()
    spec = world.spawn_obstacle.call_args[0][0]
    assert spec.x == 8.0 and spec.y == 5.5
    assert spec.size_y >= 5.68

    # Second request rejected while active
    ok2, msg2 = mgr.request_obstacle()
    assert not ok2 and 'already' in msg2.lower()

    ok3, _ = mgr.clear_obstacles()
    assert ok3 and mgr.state == ObstacleState.IDLE
    assert mgr.active_names == []
    world.remove_model.assert_called()


def test_obstacle_manager_region_trigger_spawns_selected_gap_b():
    """Approaching Gap A still activates a pending Gap B spawn."""
    world = MagicMock()
    world.is_available.return_value = True
    world.spawn_obstacle.return_value = True
    world.remove_model.return_value = True
    cfg = ForestGapConfig(
        trigger_distance_m=5.0,
        gaps=default_custom_world_gaps(),
    )
    mgr = ForestObstacleManager(cfg, world=world, force_gap='gap_b')
    mgr.request_obstacle()
    mgr.tick((8.0, 3.0))  # near gap A, far from gap B
    assert mgr.state == ObstacleState.ACTIVE
    spec = world.spawn_obstacle.call_args[0][0]
    assert spec.y == -9.5


def test_random_gap_selection_covers_both():
    world = MagicMock()
    world.is_available.return_value = True
    world.spawn_obstacle.return_value = True
    world.remove_model.return_value = True
    cfg = ForestGapConfig(gaps=default_custom_world_gaps())

    seen = set()
    for seed in range(40):
        mgr = ForestObstacleManager(cfg, world=world, seed=seed)
        mgr.request_obstacle()
        seen.add(mgr.selected_gap.key)
        mgr.clear_obstacles()
        if seen == {'gap_a', 'gap_b'}:
            break
    assert seen == {'gap_a', 'gap_b'}
