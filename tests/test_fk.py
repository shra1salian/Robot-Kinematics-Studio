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

from robot.robot_model import create_demo_2dof_robot


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
