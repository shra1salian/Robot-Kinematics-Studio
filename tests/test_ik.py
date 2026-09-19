"""
test_ik.py

Tests for kinematics/inverse_kinematics.py. Validates both IK methods
converge on reachable targets (checked by first running FK from a
known configuration to get a guaranteed-reachable target, then solving
IK back from a different starting guess -- Section 24's "known
analytical examples" approach adapted to IK), and that unreachable
targets are correctly reported as failures rather than silently
returning a wrong pose (Section 12).
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from robot.robot_model import create_demo_2dof_robot, RobotModel, JointConfig
from robot.joint import JointType, JointAxis
from kinematics.inverse_kinematics import (
    solve_ik,
    IKMethod,
    IKFailureReason,
)


def _reachable_target_from_known_config(robot, q):
    """Run FK at a known configuration to produce a target that is
    guaranteed reachable (exactly at q), without leaking q to the solver."""
    robot.set_joint_values(q)
    fk = robot.compute_fk()
    return fk.end_effector_position(), fk.end_effector_rotation()


@pytest.mark.parametrize("method", [IKMethod.PSEUDOINVERSE, IKMethod.DAMPED_LEAST_SQUARES])
def test_position_only_ik_converges_on_reachable_target(method):
    robot = create_demo_2dof_robot()
    target_pos, _ = _reachable_target_from_known_config(robot, [0.6, -0.4])

    result = solve_ik(
        robot.joints,
        robot.links,
        initial_q=np.array([0.0, 0.0]),
        target_position=target_pos,
        method=method,
        max_iterations=200,
    )

    assert result.converged, result.status_message()
    assert result.position_error < 1e-3


def test_ik_result_matches_fk_of_solution():
    """The whole point of IK: FK(IK(target)) should reproduce target."""
    robot = create_demo_2dof_robot()
    target_pos, _ = _reachable_target_from_known_config(robot, [0.9, 0.5])

    result = solve_ik(
        robot.joints,
        robot.links,
        initial_q=np.array([-0.3, 0.2]),
        target_position=target_pos,
    )
    assert result.converged

    robot.set_joint_values(result.joint_values)
    achieved = robot.compute_fk().end_effector_position()
    assert np.allclose(achieved, target_pos, atol=1e-3)


def test_position_and_orientation_ik_on_spatial_robot():
    configs = [
        JointConfig("J1", JointType.REVOLUTE, JointAxis.Z, 0.3, -np.pi, np.pi),
        JointConfig("J2", JointType.REVOLUTE, JointAxis.Y, 0.25, -np.pi, np.pi),
        JointConfig("J3", JointType.REVOLUTE, JointAxis.X, 0.2, -np.pi, np.pi),
    ]
    robot = RobotModel.from_config("3DOF Spatial", configs)

    target_pos, target_rot = _reachable_target_from_known_config(robot, [0.4, 0.5, -0.3])

    result = solve_ik(
        robot.joints,
        robot.links,
        initial_q=np.array([0.0, 0.0, 0.0]),
        target_position=target_pos,
        target_orientation=target_rot,
        max_iterations=300,
    )

    assert result.converged, result.status_message()
    assert result.position_error < 1e-3
    assert result.orientation_error < 1e-2


def test_unreachable_target_is_reported_not_silently_wrong():
    robot = create_demo_2dof_robot()
    max_reach = robot.links[0].length + robot.links[1].length

    far_target = np.array([max_reach * 10, 0.0, 0.0])
    result = solve_ik(
        robot.joints,
        robot.links,
        initial_q=np.array([0.0, 0.0]),
        target_position=far_target,
    )

    assert not result.converged
    assert result.failure_reason == IKFailureReason.TARGET_UNREACHABLE


def test_ik_respects_joint_limits_during_iteration():
    """A heavily restricted joint should never leave its limits, even
    mid-solve, whether or not the solve ultimately converges."""
    configs = [
        JointConfig("J1", JointType.REVOLUTE, JointAxis.Z, 0.4, -0.1, 0.1),
        JointConfig("J2", JointType.REVOLUTE, JointAxis.Z, 0.3, -np.pi, np.pi),
    ]
    robot = RobotModel.from_config("Restricted", configs)

    result = solve_ik(
        robot.joints,
        robot.links,
        initial_q=np.array([0.0, 0.0]),
        target_position=np.array([-0.5, 0.5, 0.0]),
        respect_joint_limits=True,
        max_iterations=150,
    )

    assert -0.1 - 1e-9 <= result.joint_values[0] <= 0.1 + 1e-9


def test_ik_never_applies_a_non_converged_result_to_the_robot():
    robot = create_demo_2dof_robot()
    original_values = robot.get_joint_values().copy()
    max_reach = robot.links[0].length + robot.links[1].length

    ik_result = robot.solve_ik(
        target_position=np.array([max_reach * 5, 0.0, 0.0]),
        apply_result=True,
    )

    assert not ik_result.converged
    assert np.allclose(robot.get_joint_values(), original_values)


def test_ik_applies_converged_result_when_requested():
    robot = create_demo_2dof_robot()
    target_pos, _ = _reachable_target_from_known_config(robot, [0.5, 0.5])
    robot.set_joint_values([0.0, 0.0])

    ik_result = robot.solve_ik(target_position=target_pos, apply_result=True)

    assert ik_result.converged
    assert np.allclose(robot.get_joint_values(), ik_result.joint_values)


def test_solve_ik_raises_on_mismatched_initial_guess_length():
    robot = create_demo_2dof_robot()
    with pytest.raises(ValueError):
        solve_ik(
            robot.joints,
            robot.links,
            initial_q=np.array([0.0, 0.0, 0.0]),  # wrong length
            target_position=np.array([0.5, 0.0, 0.0]),
        )
