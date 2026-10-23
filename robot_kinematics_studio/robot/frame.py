"""
frame.py

Lightweight container for a named coordinate frame -- a 4x4 pose plus
a label. Used both internally (e.g. base frame, end-effector frame)
and by the visualization layer to draw frame triads.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Frame:
    """A named coordinate frame in 3D space.

    Attributes:
        name: Human-readable label, e.g. "Base", "J1", "End Effector".
        transform: 4x4 homogeneous transform expressing this frame
            relative to the world/base frame.
    """

    name: str
    transform: np.ndarray = field(default_factory=lambda: np.eye(4))

    def position(self) -> np.ndarray:
        """Return the (x, y, z) origin of this frame."""
        return self.transform[:3, 3].copy()

    def rotation(self) -> np.ndarray:
        """Return the 3x3 orientation matrix of this frame."""
        return self.transform[:3, :3].copy()

    def x_axis(self) -> np.ndarray:
        return self.rotation()[:, 0]

    def y_axis(self) -> np.ndarray:
        return self.rotation()[:, 1]

    def z_axis(self) -> np.ndarray:
        return self.rotation()[:, 2]
