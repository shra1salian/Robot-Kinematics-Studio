"""
jacobian.py

The geometric Jacobian (Section 13 of the design spec). Only the core
computation lives here in Phase 5 -- it's a hard dependency of the
numerical IK solver in kinematics/inverse_kinematics.py. Rank,
singular values, condition number, manipulability, and the GUI display
of all of this arrive in Phase 6; this module already exposes
everything they'll need (a plain 6xn numpy array) without requiring
changes here later.

For revolute joint i:   J_v_i = z_i x (p_e - p_i),  J_w_i = z_i
For prismatic joint i:  J_v_i = z_i,                J_w_i = 0

where z_i is the joint's motion axis expressed in world coordinates
and p_i is the joint's world-frame origin (both already generalized to
arbitrary joint axes, not just local Z, via
forward_kinematics.joint_motion_axis_world).
"""

from __future__ import annotations

from typing import List

import numpy as np

from robot.joint import Joint
from robot.link import Link
from kinematics.forward_kinematics import FKResult, joint_motion_axis_world


def compute_geometric_jacobian(
    joints: List[Joint],
    links: List[Link],
    fk_result: FKResult,
) -> np.ndarray:
    """Compute the 6xn geometric Jacobian for the given configuration.

    Args:
        joints: Ordered list of joints, base to end-effector.
        links: Ordered list of links (same length as `joints`).
        fk_result: An FKResult already computed for the current
            configuration (see forward_kinematics.compute_forward_kinematics).

    Returns:
        6xn numpy array. Rows 0-2 are the linear-velocity part (J_v),
        rows 3-5 are the angular-velocity part (J_w). Column i
        corresponds to joints[i].
    """
    n = len(joints)
    J = np.zeros((6, n))

    p_e = fk_result.end_effector_position()

    for i, joint in enumerate(joints):
        z_i = joint_motion_axis_world(fk_result, joint, i)
        p_i = fk_result.joint_position(i)

        if joint.is_revolute():
            J[0:3, i] = np.cross(z_i, p_e - p_i)
            J[3:6, i] = z_i
        else:  # prismatic
            J[0:3, i] = z_i
            J[3:6, i] = 0.0

    return J


def jacobian_rank(J: np.ndarray, tol: float = 1e-6) -> int:
    """Return the numerical rank of a Jacobian (or any matrix)."""
    return int(np.linalg.matrix_rank(J, tol=tol))


def jacobian_condition_number(J: np.ndarray) -> float:
    """Return the condition number of a Jacobian via its singular
    values (ratio of largest to smallest singular value). Returns
    `np.inf` if the smallest singular value is (numerically) zero,
    i.e. the Jacobian is singular.
    """
    singular_values = np.linalg.svd(J, compute_uv=False)
    smallest = singular_values[-1]
    largest = singular_values[0]
    if smallest < 1e-12:
        return float("inf")
    return float(largest / smallest)
