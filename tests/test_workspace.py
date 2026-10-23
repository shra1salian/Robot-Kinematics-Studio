"""
test_workspace.py

Tests for analysis/workspace.py (Phase 7). Validates sample counts,
that every sampled position is a genuinely reachable FK result bounded
by the robot's actual maximum reach, reproducibility with a fixed
seed, correct handling of degenerate (planar) point clouds for the
volume estimate, and input validation.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from robot.robot_model import create_demo_2dof_robot, RobotModel, JointConfig
from robot.joint import JointType, JointAxis
from analysis.workspace import sample_workspace


def test_sample_count_matches_request():
    robot = create_demo_2dof_robot()
    result = sample_workspace(robot.joints, robot.links, num_samples=500, seed=42)
    assert result.num_samples == 500
    assert result.positions.shape == (500, 3)


def test_positions_are_within_true_maximum_reach():
    robot = create_demo_2dof_robot()
    max_possible_reach = robot.links[0].length + robot.links[1].length
    result = sample_workspace(robot.joints, robot.links, num_samples=2000, seed=1)

    distances = np.linalg.norm(result.positions, axis=1)
    # Every sample is a real FK result, so none can exceed the true
    # geometric maximum reach (allow a tiny floating-point margin).
    assert np.all(distances <= max_possible_reach + 1e-9)


def test_estimated_max_reach_is_plausible():
    robot = create_demo_2dof_robot()
    max_possible_reach = robot.links[0].length + robot.links[1].length
    result = sample_workspace(robot.joints, robot.links, num_samples=5000, seed=2)

    # With enough samples, the estimate should get reasonably close to
    # (but never exceed) the true maximum.
    assert result.estimated_max_reach <= max_possible_reach + 1e-9
    assert result.estimated_max_reach > max_possible_reach * 0.8


def test_reproducible_with_seed():
    robot = create_demo_2dof_robot()
    result_a = sample_workspace(robot.joints, robot.links, num_samples=200, seed=7)
    result_b = sample_workspace(robot.joints, robot.links, num_samples=200, seed=7)
    assert np.allclose(result_a.positions, result_b.positions)


def test_different_seeds_give_different_samples():
    robot = create_demo_2dof_robot()
    result_a = sample_workspace(robot.joints, robot.links, num_samples=200, seed=1)
    result_b = sample_workspace(robot.joints, robot.links, num_samples=200, seed=2)
    assert not np.allclose(result_a.positions, result_b.positions)


def test_restricted_joint_limits_shrink_the_workspace():
    """A robot with a tiny joint-1 range should sample a much smaller
    workspace than one with the full +/-180 degree range."""
    wide_configs = [
        JointConfig("J1", JointType.REVOLUTE, JointAxis.Z, 0.4, -np.pi, np.pi),
        JointConfig("J2", JointType.REVOLUTE, JointAxis.Z, 0.3, -np.pi, np.pi),
    ]
    narrow_configs = [
        JointConfig("J1", JointType.REVOLUTE, JointAxis.Z, 0.4, -0.05, 0.05),
        JointConfig("J2", JointType.REVOLUTE, JointAxis.Z, 0.3, -0.05, 0.05),
    ]
    wide_robot = RobotModel.from_config("Wide", wide_configs)
    narrow_robot = RobotModel.from_config("Narrow", narrow_configs)

    wide_result = sample_workspace(wide_robot.joints, wide_robot.links, num_samples=2000, seed=3)
    narrow_result = sample_workspace(narrow_robot.joints, narrow_robot.links, num_samples=2000, seed=3)

    wide_spread = np.ptp(wide_result.positions, axis=0).sum()
    narrow_spread = np.ptp(narrow_result.positions, axis=0).sum()
    assert narrow_spread < wide_spread


def test_planar_robot_volume_is_none_or_near_zero():
    """A purely planar (Z-axis-only) robot's point cloud is degenerate
    in 3D (z is always 0), so ConvexHull either fails (None) or, if it
    succeeds, must report an essentially zero volume."""
    robot = create_demo_2dof_robot()
    result = sample_workspace(robot.joints, robot.links, num_samples=500, seed=5)
    assert result.estimated_volume is None or result.estimated_volume < 1e-6


def test_spatial_robot_can_have_nonzero_volume():
    configs = [
        JointConfig("J1", JointType.REVOLUTE, JointAxis.Z, 0.3, -np.pi, np.pi),
        JointConfig("J2", JointType.REVOLUTE, JointAxis.Y, 0.25, -np.pi, np.pi),
        JointConfig("J3", JointType.REVOLUTE, JointAxis.X, 0.2, -np.pi, np.pi),
    ]
    robot = RobotModel.from_config("3DOF Spatial", configs)
    result = sample_workspace(robot.joints, robot.links, num_samples=3000, seed=6)
    assert result.estimated_volume is not None
    assert result.estimated_volume > 0.0


def test_is_estimated_flag_is_always_true():
    robot = create_demo_2dof_robot()
    result = sample_workspace(robot.joints, robot.links, num_samples=100, seed=8)
    assert result.is_estimated is True


def test_zero_or_negative_samples_raises():
    robot = create_demo_2dof_robot()
    with pytest.raises(ValueError):
        sample_workspace(robot.joints, robot.links, num_samples=0)
    with pytest.raises(ValueError):
        sample_workspace(robot.joints, robot.links, num_samples=-5)


def test_mismatched_joints_and_links_raises():
    robot = create_demo_2dof_robot()
    with pytest.raises(ValueError):
        sample_workspace(robot.joints, robot.links[:1], num_samples=10)


def test_prismatic_joint_workspace_bounded_by_its_range():
    configs = [JointConfig("P1", JointType.PRISMATIC, JointAxis.X, 0.0, 0.0, 0.5)]
    robot = RobotModel.from_config("Prismatic Test", configs)
    result = sample_workspace(robot.joints, robot.links, num_samples=500, seed=9)
    assert np.all(result.positions[:, 0] >= -1e-9)
    assert np.all(result.positions[:, 0] <= 0.5 + 1e-9)
    assert np.all(np.abs(result.positions[:, 1]) < 1e-9)
    assert np.all(np.abs(result.positions[:, 2]) < 1e-9)
