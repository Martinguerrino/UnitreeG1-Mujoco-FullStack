"""Regression for turn pulses blocked by diagonal obstacles near a wall."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from g1_backend.navigation_control import TurnAssist

ROOT = Path(__file__).resolve().parents[1]


def test_diagonal_obstacle_blocks_translation_without_losing_turn_request():
    cfg = yaml.safe_load((ROOT / 'configs/navigation/nav2_tuning.yaml').read_text())
    assist = TurnAssist(cfg['turn_assist'])
    scan = [20.] * 360
    # Observed in the blocked run: 43 cm at -128 degrees, outside the old cone.
    scan[52] = .43
    assert assist._sector_clear(scan, False, .5)
    assert assist._sector_clear(scan, True, .5)
    command = np.array([0., 0., .5])
    for tick in range(151):
        result = assist._assist_turn(command, {'time': tick * .02, 'scan': scan})
        assert result[0] == 0.
    assert result[2] > 0.


def test_open_space_keeps_both_turn_phases_and_stop_cancels_them():
    cfg = yaml.safe_load((ROOT / 'configs/navigation/nav2_tuning.yaml').read_text())
    assist = TurnAssist(cfg['turn_assist'])
    outputs = [assist._assist_turn(np.array([0., 0., .5]),
               {'time': tick * .02, 'scan': [20.] * 360})[0] for tick in range(151)]
    assert min(outputs) < -.2
    assert max(outputs) > .2
    np.testing.assert_array_equal(assist._assist_turn(np.zeros(3),
                                  {'time': 3.02, 'scan': [20.] * 360}), np.zeros(3))


def test_obstacle_outside_swept_width_does_not_block_translation():
    cfg = yaml.safe_load((ROOT / 'configs/navigation/nav2_tuning.yaml').read_text())
    assist = TurnAssist(cfg['turn_assist'])
    scan = [20.] * 360
    scan[90] = .7  # Lateral obstacle outside the padded swept footprint.
    assert assist._swept_footprint_clear(scan, True)
    assert assist._swept_footprint_clear(scan, False)


def test_wall_recovery_backs_up_before_spin_with_collision_checks():
    tree = ET.parse(ROOT / 'configs/navigation/navigate_to_pose_recovery.xml')
    actions = tree.find('.//RoundRobin')
    tags = [node.tag for node in actions]
    assert tags.index('BackUp') < tags.index('Spin')
    assert math.isclose(float(actions.find('BackUp').get('backup_speed')), .1)
    assert all(node.get('disable_collision_checks', 'false') == 'false'
               for node in actions if node.tag in ('BackUp', 'Spin'))
