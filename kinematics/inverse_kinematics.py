"""
inverse_kinematics.py

Numerical inverse kinematics (Section 12 of the design spec). Only
numerical IK is attempted -- no analytical IK for arbitrary robots.

Implements:
    - Method 1: Jacobian pseudoinverse    q_{k+1} = q_k + alpha * J^+ * e
    - Method 2: Damped least squares       J^+ = J^T (J J^T + lambda^2 I)^-1

Supports position-only and position+orientation targets, configurable
max iterations / tolerance / step size / damping, and respects joint
limits by clamping every candidate configuration during iteration (not
just the final result). Never silently returns an incorrect solution:
IKResult.converged is False whenever the solver did not actually reach
the target, with a human-readable reason.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import numpy as np

from robot.joint import Joint
from robot.link import Link
from kinematics.forward_kinematics import compute_forward_kinematics
from kinematics.jacobian import compute_geometric_jacobian, jacobian_condition_number

# Defaults (Section 12)
DEFAULT_MAX_ITERATIONS = 200
DEFAULT_POSITION_TOLERANCE = 1e-4      # meters
DEFAULT_ORIENTATION_TOLERANCE = 1e-3   # radians
DEFAULT_STEP_SIZE = 1.0
DEFAULT_DAMPING = 0.05

# Thresholds used to classify *why* a non-converged solve failed.
SINGULARITY_CONDITION_THRESHOLD = 1.0e4
JOINT_LIMIT_HIT_FRACTION_THRESHOLD = 0.5  # >=50% of iterations clamped


class IKMethod(Enum):
    """Supported numerical IK methods."""

    PSEUDOINVERSE = "pseudoinverse"
    DAMPED_LEAST_SQUARES = "damped_least_squares"


class IKFailureReason(Enum):
    """Why an IK solve did not converge (Section 12's required
    explicit failure reasons)."""

    NONE = "converged"
    TARGET_UNREACHABLE = "Target unreachable"
    MAX_ITERATIONS_EXCEEDED = "Maximum iterations exceeded"
    SINGULARITY_DETECTED = "Singularity detected"
    JOINT_LIMIT_PREVENTED_CONVERGENCE = "Joint limit prevented convergence"


@dataclass
class IKResult:
    """Result of a numerical IK solve.

    Attributes:
        converged: True only if both position (and orientation, if
            requested) errors are within tolerance.
        joint_values: The final joint configuration reached (radians /
            meters), regardless of whether it converged -- callers
            decide what to do with a non-converged result, but it is
            NEVER silently presented as a success.
        iterations: Number of iterations actually performed.
        position_error: Final Euclidean position error, in meters.
        orientation_error: Final orientation error, in radians (0.0 if
            orientation was not part of the target).
        failure_reason: IKFailureReason.NONE if converged, otherwise
            the best-guess explanation for the failure.
        final_condition_number: Condition number of the Jacobian at
            the final iterate (useful for diagnosing near-singular
            failures even on a successful solve).
    """

    converged: bool
    joint_values: np.ndarray
    iterations: int
    position_error: float
    orientation_error: float
    failure_reason: IKFailureReason
    final_condition_number: float

    def status_message(self) -> str:
        """Human-readable one-line summary, e.g. for the GUI status label."""
        if self.converged:
            return (
                f"Converged in {self.iterations} iterations. "
                f"Position error: {self.position_error * 1000:.4f} mm, "
                f"Orientation error: {np.degrees(self.orientation_error):.4f} deg"
            )
        return (
            f"Failed to converge after {self.iterations} iterations: "
            f"{self.failure_reason.value}. "
            f"Position error: {self.position_error * 1000:.4f} mm, "
            f"Orientation error: {np.degrees(self.orientation_error):.4f} deg"
        )


def _orientation_error_vector(R_current: np.ndarray, R_target: np.ndarray) -> np.ndarray:
    """Compute a small-angle axis-angle orientation error vector
    between the current and target rotation matrices, suitable for use
    directly in the 6-vector pose error fed to the Jacobian.

    Uses the standard skew-symmetric-part extraction of
    R_err = R_target @ R_current.T, which is exact for small errors and
    a stable descent direction even for larger ones (a common approach
    for iterative pose IK).
    """
    R_err = R_target @ R_current.T
    skew = 0.5 * (R_err - R_err.T)
    return np.array([skew[2, 1], skew[0, 2], skew[1, 0]])


def _orientation_error_angle(R_current: np.ndarray, R_target: np.ndarray) -> float:
    """Return the scalar rotation-angle error (radians) between two
    rotation matrices, via the trace formula. Always non-negative."""
    R_err = R_target @ R_current.T
    cos_angle = (np.trace(R_err) - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    return float(np.arccos(cos_angle))


def _estimate_max_reach(links: List[Link]) -> float:
    """Conservative upper bound on the robot's reach: sum of all link
    lengths. Used only for an early, approximate reachability check --
    real reach also depends on joint limits and axis configuration, so
    this is deliberately a loose bound, not an exact workspace test.
    """
    return sum(link.length for link in links)


def _clamp_to_limits(q: np.ndarray, joints: List[Joint]):
    """Clamp each joint value to its own limits.

    Returns:
        (clamped_q, was_clamped): the clamped array and a bool array
        indicating which entries were actually out of range.
    """
    clamped = q.copy()
    was_clamped = np.zeros(len(joints), dtype=bool)
    for i, joint in enumerate(joints):
        lo, hi = joint.minimum_limit, joint.maximum_limit
        if clamped[i] < lo:
            clamped[i] = lo
            was_clamped[i] = True
        elif clamped[i] > hi:
            clamped[i] = hi
            was_clamped[i] = True
    return clamped, was_clamped


def joints_with_values(joints: List[Joint], q: np.ndarray) -> List[Joint]:
    """Return shallow-copied joints with `joint_value` set to the
    corresponding entry of `q`, WITHOUT mutating the originals or
    re-clamping (the caller clamps separately via `_clamp_to_limits`).

    This keeps the IK solver from mutating the live RobotModel while it
    searches -- only a converged (or intentionally applied) result
    should ever be written back via RobotModel.set_joint_values().
    """
    result = []
    for joint, value in zip(joints, q):
        j = copy.copy(joint)
        j.joint_value = float(value)
        result.append(j)
    return result


def solve_ik(
    joints: List[Joint],
    links: List[Link],
    initial_q: np.ndarray,
    target_position: np.ndarray,
    target_orientation: Optional[np.ndarray] = None,
    method: IKMethod = IKMethod.DAMPED_LEAST_SQUARES,
    max_iterations: int = DEFAULT_MAX_ITERATIONS,
    position_tolerance: float = DEFAULT_POSITION_TOLERANCE,
    orientation_tolerance: float = DEFAULT_ORIENTATION_TOLERANCE,
    step_size: float = DEFAULT_STEP_SIZE,
    damping: float = DEFAULT_DAMPING,
    respect_joint_limits: bool = True,
    base_transform: Optional[np.ndarray] = None,
) -> IKResult:
    """Solve numerical inverse kinematics for a serial chain.

    Args:
        joints: Ordered list of joints, base to end-effector.
        links: Ordered list of links (same length as `joints`).
        initial_q: Starting joint configuration (radians/meters).
        target_position: Desired end-effector position, 3-vector (m).
        target_orientation: Desired end-effector orientation as a 3x3
            rotation matrix. If None, this is a position-only solve.
        method: IKMethod.PSEUDOINVERSE or IKMethod.DAMPED_LEAST_SQUARES.
        max_iterations: Hard iteration cap.
        position_tolerance: Convergence threshold for position error (m).
        orientation_tolerance: Convergence threshold for orientation
            error (radians). Ignored for position-only solves.
        step_size: Update scale factor (alpha in Section 12's formula).
        damping: Damping factor (lambda) used only by the damped
            least-squares method.
        respect_joint_limits: If True, every candidate configuration is
            clamped to each joint's [minimum_limit, maximum_limit]
            during iteration, not just at the end.
        base_transform: Optional 4x4 base pose; defaults to identity.

    Returns:
        IKResult. `converged` is only True if the solver actually
        reached tolerance; a non-converged result still reports the
        best configuration found, but callers must not treat it as a
        valid solution -- see `IKResult.failure_reason`.
    """
    n = len(joints)
    q = np.array(initial_q, dtype=float).flatten()
    if len(q) != n:
        raise ValueError(f"initial_q has length {len(q)}, expected {n}.")

    position_only = target_orientation is None
    clamp_hits = 0

    # Early, approximate reachability check (Section 12: report
    # "Target unreachable" rather than grinding through max_iterations
    # on an obviously-impossible target).
    fk0 = compute_forward_kinematics(joints, links, base_transform)
    base_pos = fk0.base_transform[:3, 3]
    target_distance = np.linalg.norm(np.asarray(target_position) - base_pos)
    max_reach = _estimate_max_reach(links)
    if target_distance > max_reach * 1.02:  # small tolerance for edge cases
        pos_err = float(np.linalg.norm(np.asarray(target_position) - fk0.end_effector_position()))
        orient_err = (
            0.0
            if position_only
            else _orientation_error_angle(fk0.end_effector_rotation(), target_orientation)
        )
        return IKResult(
            converged=False,
            joint_values=q,
            iterations=0,
            position_error=pos_err,
            orientation_error=float(orient_err),
            failure_reason=IKFailureReason.TARGET_UNREACHABLE,
            final_condition_number=float("nan"),
        )

    last_condition_number = float("nan")
    iterations_run = 0
    target_position = np.asarray(target_position, dtype=float)

    for iteration in range(1, max_iterations + 1):
        iterations_run = iteration
        current_joints = joints_with_values(joints, q)
        fk_result = compute_forward_kinematics(current_joints, links, base_transform)

        pos_error = target_position - fk_result.end_effector_position()

        if position_only:
            error = pos_error
            orient_err_scalar = 0.0
        else:
            R_current = fk_result.end_effector_rotation()
            orient_error_vec = _orientation_error_vector(R_current, target_orientation)
            orient_err_scalar = _orientation_error_angle(R_current, target_orientation)
            error = np.concatenate([pos_error, orient_error_vec])

        pos_err_norm = float(np.linalg.norm(pos_error))

        converged = pos_err_norm < position_tolerance and (
            position_only or orient_err_scalar < orientation_tolerance
        )
        if converged:
            return IKResult(
                converged=True,
                joint_values=q,
                iterations=iteration,
                position_error=pos_err_norm,
                orientation_error=float(orient_err_scalar),
                failure_reason=IKFailureReason.NONE,
                final_condition_number=last_condition_number,
            )

        J_full = compute_geometric_jacobian(current_joints, links, fk_result)
        J = J_full if not position_only else J_full[0:3, :]

        last_condition_number = jacobian_condition_number(J)

        if method is IKMethod.DAMPED_LEAST_SQUARES:
            J_JT = J @ J.T
            damped = J_JT + (damping ** 2) * np.eye(J.shape[0])
            J_pinv = J.T @ np.linalg.inv(damped)
        else:  # pseudoinverse
            J_pinv = np.linalg.pinv(J)

        dq = step_size * (J_pinv @ error)
        q = q + dq

        if respect_joint_limits:
            q, was_clamped = _clamp_to_limits(q, joints)
            if np.any(was_clamped):
                clamp_hits += 1

    # Did not converge within max_iterations -- classify why.
    final_joints = joints_with_values(joints, q)
    fk_final = compute_forward_kinematics(final_joints, links, base_transform)
    pos_err_final = float(np.linalg.norm(target_position - fk_final.end_effector_position()))
    orient_err_final = (
        0.0
        if position_only
        else _orientation_error_angle(fk_final.end_effector_rotation(), target_orientation)
    )

    if respect_joint_limits and (clamp_hits / max_iterations) >= JOINT_LIMIT_HIT_FRACTION_THRESHOLD:
        reason = IKFailureReason.JOINT_LIMIT_PREVENTED_CONVERGENCE
    elif last_condition_number > SINGULARITY_CONDITION_THRESHOLD:
        reason = IKFailureReason.SINGULARITY_DETECTED
    else:
        reason = IKFailureReason.MAX_ITERATIONS_EXCEEDED

    return IKResult(
        converged=False,
        joint_values=q,
        iterations=iterations_run,
        position_error=pos_err_final,
        orientation_error=float(orient_err_final),
        failure_reason=reason,
        final_condition_number=last_condition_number,
    )
