"""
manipulability.py

Placeholder module. The manipulability *measure* itself
(sqrt(det(J J^T)), computed via singular values) already exists in
kinematics/jacobian.py (`manipulability_measure`) and is surfaced
through analysis/singularity.py's `SingularityAnalysisResult`, since
Phase 6's GUI needs it alongside rank/condition number in one place.

This module is reserved for later, more elaborate manipulability
analysis that doesn't fit singularity.py's scope -- e.g. manipulability
ellipsoids, task-space-direction-specific manipulability, or
manipulability optimization for redundant robots -- none of which are
part of the current phase plan.
"""
