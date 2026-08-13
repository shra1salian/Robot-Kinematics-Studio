"""
test_robot_builder.py

Unit tests for robot/robot_model.py's Phase 2 additions: JointConfig
validation and RobotModel.from_config(). These test the GUI-independent
validation/construction logic that gui/robot_builder.py relies on, so
invalid configurations are guaranteed to be caught before a RobotModel
is ever built (Section 5 / 23 of the design spec).
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from robot.joint import JointType, JointAxis
from robot.robot_model import RobotModel, JointConfig, RobotModelError


def make_valid_configs(n: int = 3):
    return [
        JointConfig(
            name=f"J{i + 1}",
            joint_type=JointType.REVOLUTE,
            axis=JointAxis.Z,
            link_length=0.3,
            minimum_limit=-np.pi,
            maximum_limit=np.pi,
        )
        for i in range(n)
    ]


def test_valid_configs_produce_no_errors():
    configs = make_valid_configs(4)
    errors = RobotModel.validate_configs(configs)
    assert errors == []


def test_from_config_builds_correct_dof():
    configs = make_valid_configs(5)
    robot = RobotModel.from_config("Test Robot", configs)
    assert robot.dof == 5
    assert robot.name == "Test Robot"
    assert len(robot.links) == 5


def test_from_config_mixed_joint_types():
    configs = [
        JointConfig("J1", JointType.REVOLUTE, JointAxis.Z, 0.4, -np.pi, np.pi),
        JointConfig("J2", JointType.PRISMATIC, JointAxis.X, 0.0, 0.0, 0.5),
    ]
    robot = RobotModel.from_config("Hybrid Robot", configs)
    assert robot.joints[0].is_revolute()
    assert robot.joints[1].is_prismatic()


def test_empty_config_list_is_invalid():
    errors = RobotModel.validate_configs([])
    assert len(errors) == 1
    assert "at least 1 degree of freedom" in errors[0]


def test_negative_link_length_is_invalid():
    configs = make_valid_configs(1)
    configs[0].link_length = -0.1
    errors = RobotModel.validate_configs(configs)
    assert any("link length" in e for e in errors)


def test_min_exceeds_max_is_invalid():
    configs = make_valid_configs(1)
    configs[0].minimum_limit = 1.0
    configs[0].maximum_limit = -1.0
    errors = RobotModel.validate_configs(configs)
    assert any("exceeds maximum limit" in e for e in errors)


def test_empty_name_is_invalid():
    configs = make_valid_configs(1)
    configs[0].name = "   "
    errors = RobotModel.validate_configs(configs)
    assert any("name must not be empty" in e for e in errors)


def test_duplicate_names_are_invalid():
    configs = make_valid_configs(2)
    configs[1].name = configs[0].name
    errors = RobotModel.validate_configs(configs)
    assert any("unique" in e for e in errors)


def test_revolute_limits_far_out_of_range_are_flagged():
    configs = make_valid_configs(1)
    # 10 radians is way beyond +/- 2*pi -- looks like a units mistake.
    configs[0].minimum_limit = -10.0
    configs[0].maximum_limit = 10.0
    errors = RobotModel.validate_configs(configs)
    assert any("out of range" in e for e in errors)


def test_from_config_raises_on_invalid_input():
    with pytest.raises(RobotModelError):
        RobotModel.from_config("Bad Robot", [])


def test_from_config_joint_values_start_at_zero_or_clamped():
    configs = [
        JointConfig("J1", JointType.PRISMATIC, JointAxis.X, 0.2, 0.1, 0.5),
    ]
    robot = RobotModel.from_config("Prismatic Robot", configs)
    # 0.0 is below the minimum limit (0.1), so it must be clamped, not
    # silently allowed to sit at an out-of-range default.
    assert robot.joints[0].joint_value == pytest.approx(0.1)


def test_eight_dof_robot_builds_and_computes_frames():
    configs = make_valid_configs(8)
    robot = RobotModel.from_config("8DOF Robot", configs)
    frames = robot.compute_frames()
    # Base + 8 joints + End Effector
    assert len(frames) == 10
