"""
test_fk.py

Phase 1 validation: the hard-coded 2-DOF planar demo robot's
end-effector position (computed via RobotModel.compute_frames) must
match the known analytical 2-link planar manipulator equations:

    x = L1*cos(q1) + L2*cos(q1+q2)
    y = L1*sin(q1) + L2*sin(q1+q2)

This is the "important mathematical validation" required by Section
24 of the design spec, applied at the smallest possible scope for
Phase 1. The full generic FK engine will get its own, broader test
suite in Phase 3.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from robot.robot_model import create_demo_2dof_robot, RobotModel, JointConfig
from robot.joint import JointType, JointAxis
from kinematics import forward_kinematics as fk
from kinematics import transformations as tf


def analytical_2link_position(L1: float, L2: float, q1: float, q2: float):
    x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
    return np.array([x, y, 0.0])


def test_demo_robot_matches_analytical_planar_equations():
    robot = create_demo_2dof_robot()
    L1 = robot.links[0].length
    L2 = robot.links[1].length

    test_configs = [
        (0.0, 0.0),
        (0.3, 0.5),
        (np.pi / 4, -np.pi / 6),
        (-1.2, 0.9),
        (np.pi, np.pi / 2),
    ]

    for q1, q2 in test_configs:
        robot.set_joint_values([q1, q2])
        ee_pose = robot.end_effector_pose()
        ee_pos = ee_pose[:3, 3]

        expected = analytical_2link_position(L1, L2, q1, q2)
        assert np.allclose(ee_pos, expected, atol=1e-9), (
            f"Mismatch at q1={q1}, q2={q2}: got {ee_pos}, expected {expected}"
        )


def test_joint_limits_are_respected():
    robot = create_demo_2dof_robot()
    robot.set_joint_value(0, 100.0)  # far beyond +pi limit
    assert robot.joints[0].joint_value <= np.pi + 1e-9

    robot.set_joint_value(0, -100.0)
    assert robot.joints[0].joint_value >= -np.pi - 1e-9


def test_dof_matches_joint_count():
    robot = create_demo_2dof_robot()
    assert robot.dof == 2
    assert len(robot.get_joint_values()) == 2


# ---------------------------------------------------------------------
# Phase 3: generic forward-kinematics engine tests
# ---------------------------------------------------------------------

def test_compute_fk_retains_all_intermediate_transforms():
    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.2, -0.4])
    result = robot.compute_fk()

    assert len(result.joint_transforms) == robot.dof
    for T in result.joint_transforms:
        assert T.shape == (4, 4)
        assert tf.is_valid_transform(T)
    assert tf.is_valid_transform(result.end_effector_transform)


def test_compute_fk_end_effector_matches_analytical_equations():
    robot = create_demo_2dof_robot()
    L1, L2 = robot.links[0].length, robot.links[1].length

    for q1, q2 in [(0.0, 0.0), (0.4, -0.9), (np.pi / 3, np.pi / 5)]:
        robot.set_joint_values([q1, q2])
        result = robot.compute_fk()
        expected = analytical_2link_position(L1, L2, q1, q2)
        assert np.allclose(result.end_effector_position(), expected, atol=1e-9)


def test_compute_fk_matches_compute_frames():
    """The Frame-wrapper convenience method must agree exactly with the
    underlying generic FK engine -- they should never drift apart."""
    robot = create_demo_2dof_robot()
    robot.set_joint_values([0.5, -0.3])

    frames = robot.compute_frames()
    fk_result = robot.compute_fk()

    # frames = [Base, J1, J2, EndEffector]
    assert np.allclose(frames[0].transform, fk_result.base_transform)
    for i, T in enumerate(fk_result.joint_transforms):
        assert np.allclose(frames[i + 1].transform, T)
    assert np.allclose(frames[-1].transform, fk_result.end_effector_transform)


def test_forward_kinematics_rejects_mismatched_lengths():
    robot = create_demo_2dof_robot()
    try:
        fk.compute_forward_kinematics(robot.joints, robot.links[:1])
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_prismatic_joint_fk():
    configs = [
        JointConfig("J1", JointType.PRISMATIC, JointAxis.X, 0.0, 0.0, 1.0),
    ]
    robot = RobotModel.from_config("Prismatic Test", configs)
    robot.set_joint_values([0.5])
    result = robot.compute_fk()
    assert np.allclose(result.end_effector_position(), [0.5, 0.0, 0.0], atol=1e-9)


def test_three_dof_spatial_robot_fk_is_valid():
    configs = [
        JointConfig("J1", JointType.REVOLUTE, JointAxis.Z, 0.3, -np.pi, np.pi),
        JointConfig("J2", JointType.REVOLUTE, JointAxis.Y, 0.25, -np.pi, np.pi),
        JointConfig("J3", JointType.REVOLUTE, JointAxis.X, 0.2, -np.pi, np.pi),
    ]
    robot = RobotModel.from_config("3DOF Spatial", configs)
    robot.set_joint_values([0.4, 0.6, -0.2])
    result = robot.compute_fk()
    assert tf.is_valid_transform(result.end_effector_transform)
    # Sanity: end effector shouldn't be at the origin for a non-trivial pose.
    assert np.linalg.norm(result.end_effector_position()) > 1e-6


# ---------------------------------------------------------------------
# Phase 3: orientation representation tests
# ---------------------------------------------------------------------

def test_euler_xyz_round_trip_for_known_angles():
    for roll, pitch, yaw in [(0.0, 0.0, 0.0), (0.3, -0.5, 1.1), (-1.0, 0.2, 2.5)]:
        R = tf.rotation_z(yaw) @ tf.rotation_y(pitch) @ tf.rotation_x(roll)
        recovered = tf.rotation_matrix_to_euler_xyz(R)
        R_recovered = (
            tf.rotation_z(recovered[2])
            @ tf.rotation_y(recovered[1])
            @ tf.rotation_x(recovered[0])
        )
        assert np.allclose(R, R_recovered, atol=1e-8)


def test_quaternion_from_identity_is_unit_w():
    q = tf.rotation_matrix_to_quaternion(np.eye(3))
    assert np.allclose(q, [1.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_quaternion_is_unit_length_for_various_rotations():
    for R in [tf.rotation_x(0.7), tf.rotation_y(-1.2), tf.rotation_z(2.4),
              tf.rotation_about_axis([1, 1, 1], 0.9)]:
        q = tf.rotation_matrix_to_quaternion(R)
        assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-9)


def test_quaternion_reconstructs_rotation_matrix():
    for R in [tf.rotation_x(0.4), tf.rotation_y(1.5), tf.rotation_z(-0.9)]:
        w, x, y, z = tf.rotation_matrix_to_quaternion(R)
        R_rebuilt = np.array([
            [1 - 2*(y**2+z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2+z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2+y**2)],
        ])
        assert np.allclose(R, R_rebuilt, atol=1e-8)
