"""
transformations.py

Reusable homogeneous-transformation utilities used throughout Robot
Kinematics Studio. All angles are expected in RADIANS and all lengths
in METERS (SI units). Conversion to/from degrees or millimeters should
happen only at the GUI boundary.

A pose is represented as a 4x4 homogeneous transformation matrix:

    T = [ R  p ]
        [ 0  1 ]

where R is a 3x3 rotation matrix and p is a 3x1 translation vector.
"""

from __future__ import annotations

import numpy as np


def rotation_x(angle: float) -> np.ndarray:
    """Return the 3x3 rotation matrix for a rotation about the X axis.

    Args:
        angle: Rotation angle in radians.

    Returns:
        3x3 numpy array representing the rotation matrix.
    """
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ], dtype=float)


def rotation_y(angle: float) -> np.ndarray:
    """Return the 3x3 rotation matrix for a rotation about the Y axis.

    Args:
        angle: Rotation angle in radians.

    Returns:
        3x3 numpy array representing the rotation matrix.
    """
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ], dtype=float)


def rotation_z(angle: float) -> np.ndarray:
    """Return the 3x3 rotation matrix for a rotation about the Z axis.

    Args:
        angle: Rotation angle in radians.

    Returns:
        3x3 numpy array representing the rotation matrix.
    """
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ], dtype=float)


def rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """Return the rotation matrix for a rotation about an arbitrary axis.

    Uses Rodrigues' rotation formula. This is what allows the system to
    later support arbitrary (non X/Y/Z) joint axes without changing the
    core math.

    Args:
        axis: 3-element array-like, need not be normalized.
        angle: Rotation angle in radians.

    Returns:
        3x3 numpy array representing the rotation matrix.
    """
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        raise ValueError("Rotation axis must be non-zero.")
    axis = axis / norm
    kx, ky, kz = axis
    K = np.array([
        [0, -kz, ky],
        [kz, 0, -kx],
        [-ky, kx, 0],
    ], dtype=float)
    c, s = np.cos(angle), np.sin(angle)
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def translation(x: float, y: float, z: float) -> np.ndarray:
    """Return a 4x4 pure-translation homogeneous transform.

    Args:
        x: Translation along X in meters.
        y: Translation along Y in meters.
        z: Translation along Z in meters.

    Returns:
        4x4 numpy array.
    """
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def homogeneous_transform(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Combine a 3x3 rotation matrix and a 3-element translation vector
    into a single 4x4 homogeneous transformation matrix.

    Args:
        R: 3x3 rotation matrix.
        p: 3-element translation vector.

    Returns:
        4x4 numpy array.
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p, dtype=float).flatten()
    return T


def extract_position(T: np.ndarray) -> np.ndarray:
    """Extract the translation (position) component from a 4x4 transform."""
    return T[:3, 3].copy()


def extract_rotation(T: np.ndarray) -> np.ndarray:
    """Extract the 3x3 rotation component from a 4x4 transform."""
    return T[:3, :3].copy()


def is_valid_transform(T: np.ndarray, tol: float = 1e-6) -> bool:
    """Check that a matrix is a valid 4x4 homogeneous transform.

    Verifies shape, bottom row, and that the rotation block is
    (approximately) orthonormal with determinant +1.
    """
    if T.shape != (4, 4):
        return False
    if not np.allclose(T[3, :], [0, 0, 0, 1], atol=tol):
        return False
    R = T[:3, :3]
    should_be_identity = R @ R.T
    if not np.allclose(should_be_identity, np.eye(3), atol=1e-4):
        return False
    if not np.isclose(np.linalg.det(R), 1.0, atol=1e-3):
        return False
    return True
