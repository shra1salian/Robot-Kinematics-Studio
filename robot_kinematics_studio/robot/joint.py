"""
joint.py

Defines the Joint abstraction used by the robot model. A Joint stores
its type, axis, kinematic parameters and current value. It does NOT
know how to compute forward kinematics itself -- that responsibility
belongs to the kinematics package. This keeps the data model and the
math cleanly separated (see Section 3 / 23 of the design spec).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np


class JointType(Enum):
    """Supported joint types.

    Only REVOLUTE and PRISMATIC are supported in Phase 1. The enum
    exists (rather than a raw string) so invalid types are caught
    immediately instead of silently propagating.
    """

    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"


class JointAxis(Enum):
    """Supported joint axes for Phase 1.

    Only the principal X/Y/Z axes are exposed in the Phase 1 UI, but
    the underlying math (see kinematics.transformations.rotation_about_axis)
    already supports arbitrary axes, so this enum can be extended to a
    free 3-vector later without touching the kinematics engine.
    """

    X = "x"
    Y = "y"
    Z = "z"

    def as_vector(self) -> np.ndarray:
        """Return this axis as a unit vector in the joint's local frame."""
        mapping = {
            JointAxis.X: np.array([1.0, 0.0, 0.0]),
            JointAxis.Y: np.array([0.0, 1.0, 0.0]),
            JointAxis.Z: np.array([0.0, 0.0, 1.0]),
        }
        return mapping[self].copy()


@dataclass
class Joint:
    """Represents a single robot joint.

    Attributes:
        id: Zero-based index of this joint within the chain.
        name: Human-readable name, e.g. "J1".
        joint_type: REVOLUTE or PRISMATIC.
        axis: Local axis of rotation/translation.
        joint_value: Current joint value (radians for revolute,
            meters for prismatic).
        minimum_limit: Lower joint limit (radians or meters).
        maximum_limit: Upper joint limit (radians or meters).
        offset: Constant offset applied to joint_value before use in FK.
    """

    id: int
    name: str
    joint_type: JointType
    axis: JointAxis = JointAxis.Z
    joint_value: float = 0.0
    minimum_limit: float = -np.pi
    maximum_limit: float = np.pi
    offset: float = 0.0

    def __post_init__(self) -> None:
        self._validate_limits()
        self.set_value(self.joint_value)

    def _validate_limits(self) -> None:
        """Ensure joint limits are well formed."""
        if self.minimum_limit > self.maximum_limit:
            raise ValueError(
                f"Joint '{self.name}': minimum_limit "
                f"({self.minimum_limit}) exceeds maximum_limit "
                f"({self.maximum_limit})."
            )

    def set_value(self, value: float) -> None:
        """Set the joint value, clamping it to [minimum_limit, maximum_limit].

        Args:
            value: Desired joint value (radians for revolute, meters for
                prismatic).
        """
        clamped = min(max(value, self.minimum_limit), self.maximum_limit)
        self.joint_value = clamped

    def effective_value(self) -> float:
        """Return the joint value including its constant offset."""
        return self.joint_value + self.offset

    def is_revolute(self) -> bool:
        return self.joint_type is JointType.REVOLUTE

    def is_prismatic(self) -> bool:
        return self.joint_type is JointType.PRISMATIC
