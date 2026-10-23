"""
link.py

Defines the Link abstraction: the rigid segment that connects two
joints. In Phase 1 a link is modeled simply as a length along the
parent joint's local Z-ish direction (an extension in +X of the joint
frame after rotation), rendered as a cylinder. Later phases can extend
this with full DH parameters (a, alpha, d, theta) without breaking
this interface.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Link:
    """Represents a single rigid link in the serial chain.

    Attributes:
        id: Zero-based index of this link.
        name: Human-readable name, e.g. "Link 1".
        length: Link length in meters.
        radius: Visual radius in meters (rendering only, no physics).
    """

    id: int
    name: str
    length: float
    radius: float = 0.02

    def __post_init__(self) -> None:
        if self.length < 0:
            raise ValueError(
                f"Link '{self.name}': length must be non-negative, "
                f"got {self.length}."
            )
        if self.radius <= 0:
            raise ValueError(
                f"Link '{self.name}': radius must be positive, "
                f"got {self.radius}."
            )

    def set_length(self, length: float) -> None:
        """Update the link length.

        Args:
            length: New link length in meters. Must be non-negative.
        """
        if length < 0:
            raise ValueError("Link length must be non-negative.")
        self.length = length
