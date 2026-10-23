"""
workspace.py

Workspace analysis (Section 15 of the design spec). Samples the
robot's joint space (uniformly at random, within each joint's own
limits), runs forward kinematics for every sample, and collects the
resulting end-effector positions into a point cloud approximating the
robot's reachable workspace.

No GUI or rendering code lives here -- this module only produces data
(a WorkspaceResult); visualization/workspace_renderer.py turns that
into PyVista geometry, and gui/workspace_panel.py drives it from a
background thread so large sample counts don't freeze the GUI
(Section 25).

IMPORTANT: any statistic derived from sampling (maximum reach,
workspace volume) is an ESTIMATE, not an exact geometric result --
Section 15 explicitly requires this to be labeled, not presented as
exact. WorkspaceResult marks this via `is_estimated = True` throughout.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from robot.joint import Joint
from robot.link import Link
from kinematics.forward_kinematics import compute_forward_kinematics


@dataclass
class WorkspaceResult:
    """Result of a workspace sampling run.

    Attributes:
        positions: (N, 3) array of sampled end-effector positions.
        num_samples: Number of samples actually generated (== N).
        estimated_max_reach: Largest observed distance from the base
            to any sampled end-effector position. An ESTIMATE -- the
            true maximum reach could exceed this if the sampling
            missed the true extremal configuration, though it can
            never be an overestimate (every sample is a real,
            reachable configuration).
        estimated_volume: Convex-hull volume of the sampled point
            cloud, in cubic meters, or None if it could not be
            computed (e.g. degenerate/planar point clouds, where a 3D
            hull is undefined). Always an ESTIMATE -- both because it's
            derived from a finite sample and because the true
            workspace is not generally convex, so the convex hull
            volume is itself only an upper-bound-ish approximation.
        is_estimated: Always True for this dataclass; kept as an
            explicit field (rather than relying on a docstring) so any
            code displaying these numbers is forced to acknowledge it
            handles estimated data.
    """

    positions: np.ndarray
    num_samples: int
    estimated_max_reach: float
    estimated_volume: Optional[float]
    is_estimated: bool = True


def sample_workspace(
    joints: List[Joint],
    links: List[Link],
    num_samples: int,
    base_transform: Optional[np.ndarray] = None,
    seed: Optional[int] = None,
) -> WorkspaceResult:
    """Sample the robot's reachable workspace via uniform random
    joint-space sampling.

    Args:
        joints: Ordered list of joints, base to end-effector. Each
            joint's own [minimum_limit, maximum_limit] bounds the
            sampling range for that joint.
        links: Ordered list of links (same length as `joints`).
        num_samples: Number of random configurations to sample. Must
            be positive.
        base_transform: Optional 4x4 base pose; defaults to identity.
        seed: Optional random seed for reproducibility (e.g. in tests).

    Returns:
        WorkspaceResult with one end-effector position per sample.

    Raises:
        ValueError: if `num_samples` is not positive, or `joints`/
            `links` have mismatched lengths.
    """
    if len(joints) != len(links):
        raise ValueError(
            f"joints ({len(joints)}) and links ({len(links)}) must have "
            f"the same length."
        )
    if num_samples <= 0:
        raise ValueError(f"num_samples must be positive, got {num_samples}.")

    rng = np.random.default_rng(seed)
    n = len(joints)

    lows = np.array([j.minimum_limit for j in joints])
    highs = np.array([j.maximum_limit for j in joints])

    positions = np.empty((num_samples, 3))
    base_pos = (
        base_transform[:3, 3] if base_transform is not None else np.zeros(3)
    )

    for i in range(num_samples):
        q = rng.uniform(lows, highs)
        sampled_joints = _joints_with_values(joints, q)
        fk_result = compute_forward_kinematics(sampled_joints, links, base_transform)
        positions[i] = fk_result.end_effector_position()

    distances = np.linalg.norm(positions - base_pos, axis=1)
    estimated_max_reach = float(np.max(distances)) if num_samples > 0 else 0.0

    estimated_volume = _estimate_convex_hull_volume(positions)

    return WorkspaceResult(
        positions=positions,
        num_samples=num_samples,
        estimated_max_reach=estimated_max_reach,
        estimated_volume=estimated_volume,
    )


def _joints_with_values(joints: List[Joint], q: np.ndarray) -> List[Joint]:
    """Shallow-copy joints with `joint_value` set from `q`, without
    mutating the originals (mirrors
    kinematics.inverse_kinematics.joints_with_values)."""
    import copy

    result = []
    for joint, value in zip(joints, q):
        j = copy.copy(joint)
        j.joint_value = float(value)
        result.append(j)
    return result


def _estimate_convex_hull_volume(positions: np.ndarray) -> Optional[float]:
    """Estimate workspace volume as the convex hull volume of the
    sampled points. Returns None if the hull can't be computed (too
    few points, or a degenerate/planar point cloud -- e.g. any purely
    2D/planar robot, whose "volume" is legitimately undefined in 3D).
    """
    if positions.shape[0] < 4:
        return None
    try:
        from scipy.spatial import ConvexHull

        hull = ConvexHull(positions)
        return float(hull.volume)
    except Exception:
        # QhullError on degenerate/coplanar input, or scipy unavailable.
        return None
