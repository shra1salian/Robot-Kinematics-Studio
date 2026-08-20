"""
forward_kinematics.py

The generic, chain-agnostic forward-kinematics engine (Section 10 of
the design spec). This is the single source of truth for "given a
list of joints/links and a set of joint values, where is everything in
space" -- both RobotModel (for the 3D view) and, later, the Jacobian
and IK modules build on this rather than recomputing FK themselves.

The engine works on plain Joint/Link objects rather than a RobotModel
so it has no dependency on the higher-level model class -- avoiding a
circular import and keeping the math layer independent of the data
model that happens to own it (Section 3 of the design spec).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from robot.joint import Joint
from robot.link import Link
from kinematics import transformations as tf


@dataclass
class FKResult:
    """Result of a forward-kinematics computation for one robot
    configuration.

    Attributes:
        base_transform: 4x4 pose of the base frame in world coordinates.
        joint_transforms: List of 4x4 transforms, one per joint, giving
            the world pose of each joint's origin frame (i.e. T_0_1,
            T_0_2, ..., T_0_n). This is exactly the "retained
            intermediate transforms" required by Section 10 for later
            Jacobian and visualization use.
        end_effector_transform: 4x4 world pose of the end-effector
            (after the last link's extension).
    """

    base_transform: np.ndarray
    joint_transforms: List[np.ndarray] = field(default_factory=list)
    end_effector_transform: np.ndarray = field(default_factory=lambda: np.eye(4))

    def joint_position(self, index: int) -> np.ndarray:
        """World-frame (x, y, z) origin of the joint at `index` (0-based)."""
        return self.joint_transforms[index][:3, 3].copy()

    def end_effector_position(self) -> np.ndarray:
        return self.end_effector_transform[:3, 3].copy()

    def end_effector_rotation(self) -> np.ndarray:
        return self.end_effector_transform[:3, :3].copy()


def joint_local_transform(joint: Joint) -> np.ndarray:
    """Compute the local transform contributed by a single joint's
    current value: a rotation about its axis for revolute joints, or a
    translation along its axis for prismatic joints.

    Args:
        joint: The joint to evaluate (uses `joint.effective_value()`,
            i.e. joint_value + offset).

    Returns:
        4x4 homogeneous transform.
    """
    value = joint.effective_value()
    axis_vec = joint.axis.as_vector()

    if joint.is_revolute():
        R = tf.rotation_about_axis(axis_vec, value)
        return tf.homogeneous_transform(R, np.zeros(3))
    else:  # prismatic
        p = axis_vec * value
        return tf.homogeneous_transform(np.eye(3), p)


def link_transform(link: Link) -> np.ndarray:
    """Return the fixed transform representing a link's extension: a
    translation of `link.length` along the local X axis of the
    preceding joint frame.

    Args:
        link: The link to evaluate.

    Returns:
        4x4 homogeneous transform.
    """
    return tf.translation(link.length, 0.0, 0.0)


def joint_motion_axis_world(
    fk_result: FKResult, joint: Joint, index: int
) -> np.ndarray:
    """Return the joint's rotation/sliding axis expressed in world
    coordinates at the given joint's frame. This is the `z_i` used by
    the Jacobian formulas in Section 13, generalized to arbitrary
    joint axes rather than assuming local Z.

    Args:
        fk_result: An FKResult already computed for the current
            configuration.
        joint: The joint whose motion axis is requested.
        index: The joint's index (0-based) into fk_result.joint_transforms.

    Returns:
        3-element unit vector: the joint's axis direction in world frame.
    """
    R = fk_result.joint_transforms[index][:3, :3]
    axis_local = joint.axis.as_vector()
    axis_world = R @ axis_local
    norm = np.linalg.norm(axis_world)
    return axis_world / norm if norm > 1e-12 else axis_world


def compute_forward_kinematics(
    joints: List[Joint],
    links: List[Link],
    base_transform: Optional[np.ndarray] = None,
) -> FKResult:
    """Compute forward kinematics for a full serial chain.

    Args:
        joints: Ordered list of joints, base to end-effector.
        links: Ordered list of links, one per joint (link i follows
            joint i). Must be the same length as `joints`.
        base_transform: 4x4 pose of the base frame in world
            coordinates. Defaults to identity.

    Returns:
        FKResult with the base transform, every intermediate joint
        transform (T_0_1 ... T_0_n), and the end-effector transform.

    Raises:
        ValueError: if `joints` and `links` have different lengths.
    """
    if len(joints) != len(links):
        raise ValueError(
            f"joints ({len(joints)}) and links ({len(links)}) must have "
            f"the same length."
        )

    if base_transform is None:
        base_transform = np.eye(4)

    T = base_transform.copy()
    joint_transforms: List[np.ndarray] = []

    for joint, link in zip(joints, links):
        T = T @ joint_local_transform(joint)
        joint_transforms.append(T.copy())
        T = T @ link_transform(link)

    return FKResult(
        base_transform=base_transform.copy(),
        joint_transforms=joint_transforms,
        end_effector_transform=T.copy(),
    )
