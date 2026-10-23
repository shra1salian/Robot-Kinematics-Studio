"""
test_singularity.py

Tests for analysis/singularity.py (Phase 6). Validates the linear vs
angular split, the status classification thresholds, and specifically
the design decision documented in SingularityAnalysisResult.status:
overall status tracks the LINEAR block only, since structural angular
rank deficiency (e.g. two parallel-axis revolute joints, as in the
project's own 2-link planar demo robot) is a fixed property of the
robot's design and would otherwise falsely flag every configuration as
"Singular" regardless of pose.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from robot.robot_model import create_demo_2dof_robot, RobotModel, JointConfig
from robot.joint import JointType, JointAxis
from analysis.singularity import analyze_singularity, ManipulabilityStatus


def test_generic_configuration_is_good():
    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.3, 0.5])
    result = analyze_singularity(robot.joints, robot.links, robot.compute_fk())
    assert result.status == ManipulabilityStatus.GOOD


def test_fully_extended_2link_arm_is_singular():
    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.5, 0.0])
    result = analyze_singularity(robot.joints, robot.links, robot.compute_fk())
    assert result.status == ManipulabilityStatus.SINGULAR
    assert result.linear.is_rank_deficient
    assert np.isinf(result.linear.condition_number)


def test_fully_folded_2link_arm_is_singular():
    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.5, np.pi])
    result = analyze_singularity(robot.joints, robot.links, robot.compute_fk())
    assert result.status == ManipulabilityStatus.SINGULAR


def test_near_singular_configuration_is_approaching_singularity():
    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.5, np.radians(3)])
    result = analyze_singularity(robot.joints, robot.links, robot.compute_fk())
    assert result.status in (
        ManipulabilityStatus.APPROACHING_SINGULARITY,
        ManipulabilityStatus.SINGULAR,
    )


def test_status_does_not_escalate_due_to_structural_angular_rank_deficiency():
    """The demo robot's angular block is rank-1 EVERYWHERE (both
    joints rotate about parallel Z axes) -- this must never by itself
    push the overall status to SINGULAR at a well-conditioned pose."""
    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.3, np.pi / 2])  # a well-conditioned pose
    result = analyze_singularity(robot.joints, robot.links, robot.compute_fk())

    assert result.angular.is_rank_deficient  # structurally always true here
    assert result.status == ManipulabilityStatus.GOOD  # but status ignores it


def test_spatial_3dof_robot_angular_block_can_be_full_rank():
    """With non-parallel axes (Z, Y, X), the angular block CAN reach
    full rank, unlike the demo robot's parallel-axis case."""
    configs = [
        JointConfig("J1", JointType.REVOLUTE, JointAxis.Z, 0.3, -np.pi, np.pi),
        JointConfig("J2", JointType.REVOLUTE, JointAxis.Y, 0.25, -np.pi, np.pi),
        JointConfig("J3", JointType.REVOLUTE, JointAxis.X, 0.2, -np.pi, np.pi),
    ]
    robot = RobotModel.from_config("3DOF Spatial", configs)
    robot.set_joint_values([0.4, 0.6, -0.2])
    result = analyze_singularity(robot.joints, robot.links, robot.compute_fk())
    assert result.angular.rank == 3


def test_jacobian_field_is_full_6xn_matrix():
    robot = create_demo_2dof_robot()
    result = analyze_singularity(robot.joints, robot.links, robot.compute_fk())
    assert result.jacobian.shape == (6, 2)


def test_prismatic_only_robot_analysis_runs_without_error():
    configs = [
        JointConfig("P1", JointType.PRISMATIC, JointAxis.X, 0.0, 0.0, 0.5),
        JointConfig("P2", JointType.PRISMATIC, JointAxis.Y, 0.0, 0.0, 0.5),
    ]
    robot = RobotModel.from_config("2DOF Prismatic", configs)
    result = analyze_singularity(robot.joints, robot.links, robot.compute_fk())
    # Two independent, non-parallel prismatic axes -> full rank, never singular.
    assert result.status == ManipulabilityStatus.GOOD
    assert result.linear.rank == 2
