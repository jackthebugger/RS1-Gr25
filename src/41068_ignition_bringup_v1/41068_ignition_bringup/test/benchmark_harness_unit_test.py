#!/usr/bin/env python3
"""Unit tests for Wave A benchmark manifest + stall timeline helpers."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from rs1_nav.benchmark_manifest import (  # noqa: E402
    MIN_DIST_HOME_AT_RETURN_ARM_M,
    MIN_PROGRESS_BEFORE_B_BLOCK_M,
    TrialManifest,
    plugins_match,
)
from rs1_nav.stall_timeline import (  # noqa: E402
    StallTimeline,
    TimelineRow,
    classify_sample,
)


class BenchmarkManifestTest(unittest.TestCase):
    def test_thresholds_match_approved_plan(self):
        self.assertEqual(MIN_DIST_HOME_AT_RETURN_ARM_M, 5.0)
        self.assertEqual(MIN_PROGRESS_BEFORE_B_BLOCK_M, 8.0)

    def test_write_roundtrip(self):
        m = TrialManifest(
            manifest_id='P-NAVFN_R0_t01',
            scenario_id='R0',
            plugins_expected={
                'planner': 'nav2_navfn_planner/NavfnPlanner',
                'controller': (
                    'nav2_regulated_pure_pursuit_controller::'
                    'RegulatedPurePursuitController'),
            },
            status='NOT_VERIFIED',
        )
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'trial.json')
            m.write(path)
            with open(path, encoding='utf-8') as fh:
                data = json.load(fh)
        self.assertEqual(data['manifest_id'], 'P-NAVFN_R0_t01')
        self.assertEqual(data['world_name'], 'bush_trail_world')
        self.assertEqual(data['start']['x'], -18.0)
        self.assertEqual(data['goal']['x'], 16.0)

    def test_plugins_match(self):
        expected = {'planner': 'nav2_navfn_planner/NavfnPlanner'}
        self.assertTrue(plugins_match(
            {'planner': 'nav2_navfn_planner/NavfnPlanner', 'controller': 'x'},
            expected))
        self.assertFalse(plugins_match({'planner': None}, expected))
        self.assertFalse(plugins_match(
            {'planner': 'nav2_smac_planner/SmacPlanner2D'}, expected))


class StallTimelineTest(unittest.TestCase):
    def test_classify_gate_stop(self):
        self.assertEqual(
            classify_sample(
                bank_state='STOPPED',
                command_authority='NONE',
                active_route_valid=True,
                odom_v=0.0,
                cmd_v=0.0,
                dist_goal_m=10.0,
                saw_switch=False,
                home_requested=False,
                plan_existed=True,
                following=False,
            ),
            'GATE_STOP_OK',
        )

    def test_classify_b_not_selected(self):
        self.assertEqual(
            classify_sample(
                bank_state='PATH_BLOCKED',
                command_authority='NAV',
                active_route_valid=False,
                odom_v=0.0,
                cmd_v=0.0,
                dist_goal_m=10.0,
                saw_switch=False,
                home_requested=False,
                plan_existed=False,
                following=False,
            ),
            'B_NOT_SELECTED',
        )

    def test_write_csv(self):
        tl = StallTimeline()
        tl.append(TimelineRow(
            wall_mono_s=1.0, sim_time_s=2.0, bank_state='FOLLOWING_PATH',
            stall_class='OK_PROGRESS'))
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'tl.csv')
            tl.write_csv(path)
            with open(path, encoding='utf-8') as fh:
                text = fh.read()
        self.assertIn('stall_class', text)
        self.assertIn('OK_PROGRESS', text)


if __name__ == '__main__':
    unittest.main()
