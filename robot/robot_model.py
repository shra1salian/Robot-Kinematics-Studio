"""
robot_model.py

Defines RobotModel: a serial chain of alternating joints and links.
This is the central data structure that the GUI, kinematics engine,
and visualization layer all operate on. It intentionally contains NO
GUI code and NO rendering code -- see Section 3 of the design spec.

Phase 1 scope: the model can hold an arbitrary number of joints/links,
but the FK math is only exercised here through a simple convenience
method used for the hard-coded 2-DOF demo robot. The generic FK engine
lives in kinematics/forward_kinematics.py and will be wired in during
Phase 3.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np

from robot.joint import Joint, JointType, JointAxis
from robot.link import Link
from robot.frame import Frame


class RobotModelError(Exception):
    """Raised when a robot model is constructed or modified inconsistently."""


@dataclass
class RobotModel:
    """A serial-chain robot manipulator.

    Attributes:
        name: Human-readable robot name.
        joints: Ordered list of Joint objects, base to end-effector.
        links: Ordered list of Link objects, one per joint (link i
            follows joint i).
        base_frame: Pose of the robot base relative to world origin.
    """

    name: str
    joints: List[Joint] = field(default_factory=list)
    links: List[Link] = field(default_factory=list)
    base_frame: Frame = field(default_factory=lambda: Frame("Base"))

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        """Ensure the model is internally consistent.

        Raises:
            RobotModelError: if the number of joints and links differ,
                or if the model has zero degrees of freedom.
        """
        if len(self.joints) != len(self.links):
            raise RobotModelError(
                f"Robot '{self.name}': joint count ({len(self.joints)}) "
                f"must equal link count ({len(self.links)})."
            )
        if len(self.joints) == 0:
            raise RobotModelError(
                f"Robot '{self.name}': must have at least 1 degree of freedom."
            )
        ids = [j.id for j in self.joints]
        if ids != list(range(len(ids))):
            raise RobotModelError(
                f"Robot '{self.name}': joint ids must be sequential "
                f"starting at 0, got {ids}."
            )

    @property
    def dof(self) -> int:
        """Number of degrees of freedom (= number of joints)."""
        return len(self.joints)

    def get_joint_values(self) -> np.ndarray:
        """Return the current joint values as a numpy array."""
        return np.array([j.joint_value for j in self.joints], dtype=float)

    def set_joint_values(self, values: np.ndarray) -> None:
        """Set all joint values at once (each clamped to its own limits).

        Args:
            values: Array-like of length `dof`.

        Raises:
            RobotModelError: if the length of `values` does not match dof.
        """
        values = np.asarray(values, dtype=float).flatten()
        if len(values) != self.dof:
            raise RobotModelError(
                f"Expected {self.dof} joint values, got {len(values)}."
            )
        for joint, v in zip(self.joints, values):
            joint.set_value(v)

    def set_joint_value(self, joint_id: int, value: float) -> None:
        """Set a single joint's value by id."""
        if not (0 <= joint_id < self.dof):
            raise RobotModelError(f"Invalid joint id: {joint_id}")
        self.joints[joint_id].set_value(value)

    def reset(self) -> None:
        """Reset all joints to zero (clamped to limits if 0 is out of range)."""
        for joint in self.joints:
            joint.set_value(0.0)

    def joint_local_transform(self, joint: Joint) -> np.ndarray:
        """Compute the local transform contributed by a single joint's
        current value (rotation for revolute, translation for prismatic).

        This is a Phase-1 convenience used by the demo robot; the full
        generic FK engine (Phase 3) will build on the same primitives
        from kinematics.transformations.
        """
        from kinematics import transformations as tf

        value = joint.effective_value()
        axis_vec = joint.axis.as_vector()

        if joint.is_revolute():
            R = tf.rotation_about_axis(axis_vec, value)
            return tf.homogeneous_transform(R, np.zeros(3))
        else:  # prismatic
            p = axis_vec * value
            return tf.homogeneous_transform(np.eye(3), p)

    def link_transform(self, link: Link) -> np.ndarray:
        """Return the fixed transform representing a link's extension
        (translation of `length` along the local X axis)."""
        from kinematics import transformations as tf

        return tf.translation(link.length, 0.0, 0.0)

    def compute_frames(self) -> List[Frame]:
        """Compute and return the world-frame pose of every joint frame,
        in order, using the current joint values. This is a Phase-1
        convenience method for the hard-coded demo robot and simple
        visualization; it will be superseded by the generic FK engine
        in Phase 3.

        Returns:
            List of Frame objects: [Base, J1, J2, ..., Jn, EndEffector]
        """
        frames = [Frame(self.base_frame.name, self.base_frame.transform.copy())]
        T = self.base_frame.transform.copy()

        for joint, link in zip(self.joints, self.links):
            T = T @ self.joint_local_transform(joint)
            frames.append(Frame(joint.name, T.copy()))
            T = T @ self.link_transform(link)

        frames.append(Frame("End Effector", T.copy()))
        return frames

    def end_effector_pose(self) -> np.ndarray:
        """Return the 4x4 end-effector pose for the current joint values."""
        return self.compute_frames()[-1].transform


def create_demo_2dof_robot() -> RobotModel:
    """Build the hard-coded 2-DOF planar revolute robot used in Phase 1.

    Both joints rotate about Z; links extend along local X. This
    matches the classic 2-link planar manipulator used later (Section
    24) to validate the generic FK/Jacobian implementation against the
    known analytical equations:

        x = L1*cos(q1) + L2*cos(q1+q2)
        y = L1*sin(q1) + L2*sin(q1+q2)

    Returns:
        A ready-to-use RobotModel instance.
    """
    joint1 = Joint(
        id=0,
        name="J1",
        joint_type=JointType.REVOLUTE,
        axis=JointAxis.Z,
        joint_value=0.3,
        minimum_limit=-np.pi,
        maximum_limit=np.pi,
    )
    joint2 = Joint(
        id=1,
        name="J2",
        joint_type=JointType.REVOLUTE,
        axis=JointAxis.Z,
        joint_value=0.5,
        minimum_limit=-np.pi,
        maximum_limit=np.pi,
    )
    link1 = Link(id=0, name="Link 1", length=0.4, radius=0.02)
    link2 = Link(id=1, name="Link 2", length=0.3, radius=0.02)

    return RobotModel(
        name="Demo 2DOF Planar Robot",
        joints=[joint1, joint2],
        links=[link1, link2],
    )
