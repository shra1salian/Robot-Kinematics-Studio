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
class JointConfig:
    """A single row of Robot Builder input: everything needed to
    construct one (Joint, Link) pair.

    This is the boundary object between the GUI (which works in
    user-friendly units: degrees for revolute limits) and the internal
    model (which is always SI: radians). The Robot Builder panel is
    responsible for the degrees->radians conversion before building a
    JointConfig; this class itself just validates internal
    consistency and does not know about degrees.

    Attributes:
        name: Human-readable joint name, e.g. "J1".
        joint_type: REVOLUTE or PRISMATIC.
        axis: Joint axis (X, Y, or Z for Phase 2).
        link_length: Length in meters of the link following this joint.
        minimum_limit: Lower joint limit (radians for revolute, meters
            for prismatic).
        maximum_limit: Upper joint limit (radians for revolute, meters
            for prismatic).
    """

    name: str
    joint_type: JointType
    axis: JointAxis
    link_length: float
    minimum_limit: float
    maximum_limit: float

    def validate(self, row_label: str | None = None) -> List[str]:
        """Validate this row in isolation and return a list of
        human-readable error messages (empty list = valid).

        Args:
            row_label: Optional label used to prefix error messages,
                e.g. "Joint 3". Defaults to this config's name.
        """
        label = row_label or self.name
        errors: List[str] = []

        if not self.name or not self.name.strip():
            errors.append(f"{label}: name must not be empty.")

        if self.link_length < 0:
            errors.append(
                f"{label}: link length must be non-negative "
                f"(got {self.link_length})."
            )

        if self.minimum_limit > self.maximum_limit:
            errors.append(
                f"{label}: minimum limit ({self.minimum_limit:.4f}) "
                f"exceeds maximum limit ({self.maximum_limit:.4f})."
            )

        if self.joint_type is JointType.REVOLUTE:
            # Sanity bound: reject absurd revolute limits beyond +/- 360 deg
            # (2*pi rad) since anything larger is almost certainly a unit
            # mistake (e.g. entering radians where degrees were expected).
            two_pi = 2 * np.pi
            if abs(self.minimum_limit) > two_pi or abs(self.maximum_limit) > two_pi:
                errors.append(
                    f"{label}: revolute limits look out of range "
                    f"({np.degrees(self.minimum_limit):.1f} deg to "
                    f"{np.degrees(self.maximum_limit):.1f} deg). "
                    f"Expected within +/-360 deg."
                )

        return errors


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

        Thin wrapper around the generic FK engine
        (kinematics.forward_kinematics) so there is exactly one
        implementation of this math in the whole project.
        """
        from kinematics import forward_kinematics as fk

        return fk.joint_local_transform(joint)

    def link_transform(self, link: Link) -> np.ndarray:
        """Return the fixed transform representing a link's extension
        (translation of `length` along the local X axis)."""
        from kinematics import forward_kinematics as fk

        return fk.link_transform(link)

    def compute_fk(self):
        """Run the generic forward-kinematics engine for the robot's
        current joint values.

        Returns:
            kinematics.forward_kinematics.FKResult, including every
            intermediate joint transform (T_0_1 ... T_0_n) needed by
            the Jacobian (Phase 6) and IK (Phase 5) modules.
        """
        from kinematics import forward_kinematics as fk

        return fk.compute_forward_kinematics(
            self.joints, self.links, base_transform=self.base_frame.transform
        )

    def compute_frames(self) -> List[Frame]:
        """Compute and return the world-frame pose of every joint frame,
        in order, using the current joint values. This is a thin
        Frame-naming wrapper around `compute_fk()` / the generic FK
        engine, used by the visualization layer.

        Returns:
            List of Frame objects: [Base, J1, J2, ..., Jn, EndEffector]
        """
        fk_result = self.compute_fk()

        frames = [Frame(self.base_frame.name, fk_result.base_transform.copy())]
        for joint, T in zip(self.joints, fk_result.joint_transforms):
            frames.append(Frame(joint.name, T.copy()))
        frames.append(Frame("End Effector", fk_result.end_effector_transform.copy()))
        return frames

    def end_effector_pose(self) -> np.ndarray:
        """Return the 4x4 end-effector pose for the current joint values."""
        return self.compute_fk().end_effector_transform

    def solve_ik(
        self,
        target_position: np.ndarray,
        target_orientation: "np.ndarray | None" = None,
        apply_result: bool = False,
        **kwargs,
    ):
        """Convenience wrapper around kinematics.inverse_kinematics.solve_ik
        using this robot's current joint values as the initial guess.

        Args:
            target_position: Desired end-effector position (3-vector, m).
            target_orientation: Desired end-effector orientation (3x3
                rotation matrix), or None for a position-only solve.
            apply_result: If True and the solve converges, the robot's
                joint values are updated to the solution (via
                `set_joint_values`, which re-clamps to limits). A
                non-converged result is NEVER applied, even if
                requested, since Section 12 requires that IK never
                silently return an incorrect solution.
            **kwargs: Forwarded to
                kinematics.inverse_kinematics.solve_ik (method,
                max_iterations, position_tolerance,
                orientation_tolerance, step_size, damping,
                respect_joint_limits).

        Returns:
            kinematics.inverse_kinematics.IKResult
        """
        from kinematics import inverse_kinematics as ik

        result = ik.solve_ik(
            self.joints,
            self.links,
            self.get_joint_values(),
            target_position,
            target_orientation=target_orientation,
            base_transform=self.base_frame.transform,
            **kwargs,
        )
        if apply_result and result.converged:
            self.set_joint_values(result.joint_values)
        return result

    @staticmethod
    def validate_configs(configs: List[JointConfig]) -> List[str]:
        """Validate a full list of JointConfig rows as a whole, in
        addition to each row's own validate().

        This is the function the Robot Builder panel should call
        BEFORE attempting to construct a RobotModel, so invalid
        configurations are caught and reported clearly instead of
        silently propagating into the model (Section 5 of the design
        spec).

        Args:
            configs: Ordered list of JointConfig, one per joint.

        Returns:
            List of human-readable error strings. Empty means valid.
        """
        errors: List[str] = []

        if len(configs) == 0:
            errors.append("Robot must have at least 1 degree of freedom.")
            return errors

        for i, cfg in enumerate(configs):
            errors.extend(cfg.validate(row_label=f"Joint {i + 1} ({cfg.name})"))

        names = [c.name.strip() for c in configs]
        duplicates = {n for n in names if names.count(n) > 1 and n}
        if duplicates:
            errors.append(
                f"Joint names must be unique. Duplicated: {', '.join(sorted(duplicates))}."
            )

        return errors

    @classmethod
    def from_config(
        cls,
        name: str,
        configs: List[JointConfig],
    ) -> "RobotModel":
        """Construct a RobotModel from a list of JointConfig rows, e.g.
        as produced by the Robot Builder panel.

        Args:
            name: Name for the resulting robot.
            configs: Ordered list of JointConfig, base to end-effector.

        Returns:
            A new RobotModel instance.

        Raises:
            RobotModelError: if `configs` fails validation. Callers
                that want per-field error messages ahead of time
                should call `RobotModel.validate_configs(configs)`
                first (e.g. to show them in the GUI) rather than
                relying solely on this exception.
        """
        errors = cls.validate_configs(configs)
        if errors:
            raise RobotModelError(
                "Cannot build robot, invalid configuration:\n- "
                + "\n- ".join(errors)
            )

        joints: List[Joint] = []
        links: List[Link] = []
        for i, cfg in enumerate(configs):
            joints.append(
                Joint(
                    id=i,
                    name=cfg.name,
                    joint_type=cfg.joint_type,
                    axis=cfg.axis,
                    joint_value=0.0,
                    minimum_limit=cfg.minimum_limit,
                    maximum_limit=cfg.maximum_limit,
                )
            )
            links.append(
                Link(id=i, name=f"Link {i + 1}", length=cfg.link_length)
            )

        return cls(name=name, joints=joints, links=links)


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
