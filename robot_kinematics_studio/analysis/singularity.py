"""
singularity.py

Singularity analysis (Section 14 of the design spec). Packages the
Jacobian, its rank, condition number, and manipulability measure into
a single display-ready result, plus a qualitative status classification
("Good" / "Approaching singularity" / "Singular"). This is the module
the GUI's Jacobian & Singularity panel reads from -- it contains no Qt
code itself, only the analysis.

IMPORTANT DESIGN NOTE -- linear/angular split:
Combining position (meters) and orientation (radians) rows into a
single 6xn Jacobian and computing one rank/condition-number/
manipulability from it is dimensionally inconsistent (it mixes units),
and for common cases (e.g. all-revolute-about-Z planar arms) it can
mask the classic "arm fully extended" singularity entirely: the
orientation rows, which are identical in direction across joints that
share an axis, can keep the *combined* matrix full rank even though
the *position* sub-Jacobian is exactly singular. This was caught by
testing against the project's own 2-link planar demo robot.

To stay both mathematically sound and practically useful, this module
computes rank / condition number / manipulability separately for the
linear-velocity Jacobian (J_v, top 3 rows) and the angular-velocity
Jacobian (J_w, bottom 3 rows), and reports the more severe of the two
as the overall status. The full 6xn Jacobian is still returned and
displayed in the GUI for reference (Section 13's requirement), but the
analysis that drives the status banner is done on the split matrices.

The underlying math (compute_geometric_jacobian, jacobian_rank,
jacobian_condition_number, manipulability_measure) lives in
kinematics/jacobian.py; this module adds interpretation on top.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List

import numpy as np

from robot.joint import Joint
from robot.link import Link
from kinematics.forward_kinematics import FKResult
from kinematics.jacobian import (
    compute_geometric_jacobian,
    jacobian_rank,
    jacobian_condition_number,
    jacobian_singular_values,
    manipulability_measure,
)

# Condition-number thresholds used to classify manipulability.
# These are heuristic (Section 14 does not mandate specific numbers)
# but chosen to give useful, non-alarmist feedback: most everyday
# configurations of a well-designed robot fall in "Good".
CONDITION_NUMBER_WARNING_THRESHOLD = 20.0
CONDITION_NUMBER_SINGULAR_THRESHOLD = 1.0e4


class ManipulabilityStatus(Enum):
    """Qualitative singularity status for display."""

    GOOD = "Good"
    APPROACHING_SINGULARITY = "Approaching singularity"
    SINGULAR = "Singular"


def _classify(rank: int, full_rank: int, condition_number: float) -> ManipulabilityStatus:
    if rank < full_rank or condition_number >= CONDITION_NUMBER_SINGULAR_THRESHOLD:
        return ManipulabilityStatus.SINGULAR
    if condition_number >= CONDITION_NUMBER_WARNING_THRESHOLD:
        return ManipulabilityStatus.APPROACHING_SINGULARITY
    return ManipulabilityStatus.GOOD


@dataclass
class SubJacobianAnalysis:
    """Analysis of one 3xn block (linear or angular) of the Jacobian."""

    rank: int
    full_rank: int
    condition_number: float
    singular_values: np.ndarray
    manipulability: float
    status: ManipulabilityStatus

    @property
    def is_rank_deficient(self) -> bool:
        return self.rank < self.full_rank


@dataclass
class SingularityAnalysisResult:
    """Full singularity/manipulability analysis for one configuration.

    Attributes:
        jacobian: The full 6xn geometric Jacobian (J_v stacked on J_w),
            shown in the GUI's matrix table for reference.
        linear: Analysis of the linear-velocity block (J_v, top 3 rows) --
            this is what flags classic position singularities such as a
            fully-extended 2-link arm.
        angular: Analysis of the angular-velocity block (J_w, bottom 3
            rows). Shown for reference only -- see `status` below for
            why it does not drive the overall status.
        status: The overall status shown in the GUI. Deliberately
            equal to `linear.status`, NOT a combination with
            `angular.status`. Many common robots (e.g. any chain where
            two or more joints share a parallel rotation axis, as in
            this project's own 2-link planar demo robot) are
            *structurally* angular-rank-deficient at every single
            configuration -- that's a fixed property of the robot's
            design, not a configuration-dependent event worth an
            "approaching singularity" warning. Driving the live status
            off the angular block would make such robots permanently
            flag as singular regardless of pose, which is true but not
            useful as a moving warning indicator. The linear block,
            by contrast, genuinely varies with configuration (e.g. a
            2-link arm is only rank-deficient in position when fully
            extended or fully folded) and is what Section 14's example
            and Section 24's validation are about.
    """

    jacobian: np.ndarray
    linear: SubJacobianAnalysis
    angular: SubJacobianAnalysis
    status: ManipulabilityStatus


def _analyze_block(J_block: np.ndarray, full_rank: int) -> SubJacobianAnalysis:
    rank = jacobian_rank(J_block)
    condition_number = jacobian_condition_number(J_block)
    singular_values = jacobian_singular_values(J_block)
    manipulability = manipulability_measure(J_block)
    status = _classify(rank, full_rank, condition_number)
    return SubJacobianAnalysis(
        rank=rank,
        full_rank=full_rank,
        condition_number=condition_number,
        singular_values=singular_values,
        manipulability=manipulability,
        status=status,
    )


def analyze_singularity(
    joints: List[Joint],
    links: List[Link],
    fk_result: FKResult,
) -> SingularityAnalysisResult:
    """Run a full singularity/manipulability analysis for the given
    configuration.

    Args:
        joints: Ordered list of joints, base to end-effector.
        links: Ordered list of links (same length as `joints`).
        fk_result: An FKResult already computed for the current
            configuration.

    Returns:
        SingularityAnalysisResult.
    """
    J = compute_geometric_jacobian(joints, links, fk_result)
    n = len(joints)
    full_rank = min(3, n)

    linear = _analyze_block(J[0:3, :], full_rank)
    angular = _analyze_block(J[3:6, :], full_rank)

    return SingularityAnalysisResult(
        jacobian=J,
        linear=linear,
        angular=angular,
        status=linear.status,
    )
