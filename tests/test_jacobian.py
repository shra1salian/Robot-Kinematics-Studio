"""
test_jacobian.py

Tests for kinematics/jacobian.py. Per Section 24 of the design spec,
the geometric Jacobian must be validated against the known analytical
Jacobian of a 2-link planar manipulator:

    J = [ -L1*sin(q1) - L2*sin(q1+q2),   -L2*sin(q1+q2) ]
        [  L1*cos(q1) + L2*cos(q1+q2),    L2*cos(q1+q2) ]

(top two rows of J_v, since the robot is planar and both joints
rotate about Z).
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from robot.robot_model import create_demo_2dof_robot, RobotModel, JointConfig
from robot.joint import JointType, JointAxis
from kinematics.forward_kinematics import compute_forward_kinematics
from kinematics.jacobian import (
    compute_geometric_jacobian,
    jacobian_rank,
    jacobian_condition_number,
)


def analytical_2link_jacobian(L1, L2, q1, q2):
    J = np.zeros((2, 2))
    J[0, 0] = -L1 * np.sin(q1) - L2 * np.sin(q1 + q2)
    J[0, 1] = -L2 * np.sin(q1 + q2)
    J[1, 0] = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    J[1, 1] = L2 * np.cos(q1 + q2)
    return J


def test_geometric_jacobian_matches_analytical_2link_planar():
    robot = create_demo_2dof_robot()
    L1, L2 = robot.links[0].length, robot.links[1].length

    for q1, q2 in [(0.0, 0.0), (0.3, 0.6), (-0.8, 1.1), (np.pi / 4, -np.pi / 3)]:
        robot.set_joint_values([q1, q2])
        fk_result = robot.compute_fk()
        J_full = compute_geometric_jacobian(robot.joints, robot.links, fk_result)

        J_expected = analytical_2link_jacobian(L1, L2, q1, q2)
        J_position_xy = J_full[0:2, :]  # linear velocity rows, X and Y only

        assert np.allclose(J_position_xy, J_expected, atol=1e-8)


def test_jacobian_shape_is_6xn():
    robot = create_demo_2dof_robot()
    fk_result = robot.compute_fk()
    J = compute_geometric_jacobian(robot.joints, robot.links, fk_result)
    assert J.shape == (6, robot.dof)


def test_prismatic_jacobian_column():
    configs = [JointConfig("P1", JointType.PRISMATIC, JointAxis.X, 0.0, 0.0, 1.0)]
    robot = RobotModel.from_config("Prismatic Test", configs)
    fk_result = robot.compute_fk()
    J = compute_geometric_jacobian(robot.joints, robot.links, fk_result)
    # Prismatic along X: J_v = [1, 0, 0], J_w = [0, 0, 0]
    assert np.allclose(J[0:3, 0], [1.0, 0.0, 0.0])
    assert np.allclose(J[3:6, 0], [0.0, 0.0, 0.0])


def test_full_rank_jacobian_at_generic_configuration():
    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.4, 0.7])
    fk_result = robot.compute_fk()
    J = compute_geometric_jacobian(robot.joints, robot.links, fk_result)
    # Position-only (x,y) sub-Jacobian should be full rank away from
    # the q2=0/pi singularities.
    assert jacobian_rank(J[0:2, :]) == 2


def test_singular_configuration_has_low_rank_and_high_condition_number():
    robot = create_demo_2dof_robot()
    # q2 = 0: the arm is fully extended, a classic 2-link singularity.
    robot.set_joint_values([0.5, 0.0])
    fk_result = robot.compute_fk()
    J = compute_geometric_jacobian(robot.joints, robot.links, fk_result)
    J_xy = J[0:2, :]
    assert jacobian_rank(J_xy, tol=1e-6) < 2
    cond = jacobian_condition_number(J_xy)
    assert cond > 1e6 or np.isinf(cond)


def test_condition_number_of_well_conditioned_jacobian_is_finite_and_reasonable():
    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.3, np.pi / 2])
    fk_result = robot.compute_fk()
    J = compute_geometric_jacobian(robot.joints, robot.links, fk_result)
    cond = jacobian_condition_number(J[0:2, :])
    assert np.isfinite(cond)
    assert cond > 0


# ---------------------------------------------------------------------
# Phase 6: manipulability measure tests
# ---------------------------------------------------------------------

def test_manipulability_zero_at_singularity():
    from kinematics.jacobian import manipulability_measure

    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.5, 0.0])  # fully extended
    fk_result = robot.compute_fk()
    J = compute_geometric_jacobian(robot.joints, robot.links, fk_result)
    w = manipulability_measure(J[0:2, :])
    assert np.isclose(w, 0.0, atol=1e-9)


def test_manipulability_positive_away_from_singularity():
    from kinematics.jacobian import manipulability_measure

    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.3, np.pi / 2])
    fk_result = robot.compute_fk()
    J = compute_geometric_jacobian(robot.joints, robot.links, fk_result)
    w = manipulability_measure(J[0:2, :])
    assert w > 0.0


def test_manipulability_equals_singular_value_product():
    from kinematics.jacobian import manipulability_measure, jacobian_singular_values

    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.4, 0.6])
    fk_result = robot.compute_fk()
    J = compute_geometric_jacobian(robot.joints, robot.links, fk_result)
    J_xy = J[0:2, :]
    w = manipulability_measure(J_xy)
    sv = jacobian_singular_values(J_xy)
    assert np.isclose(w, np.prod(sv), atol=1e-9)
