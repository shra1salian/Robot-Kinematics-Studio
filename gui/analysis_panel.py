"""
analysis_panel.py

Phase 6: the Jacobian & Singularity Analysis panel (Section 14). Shows
the live numeric Jacobian matrix, plus rank/condition-number/
manipulability computed separately for the linear (J_v) and angular
(J_w) blocks -- see analysis/singularity.py for why the split matters
-- and a qualitative status ("Good" / "Approaching singularity" /
"Singular"). Everything updates as the user moves joints or applies an
IK solution, not just when a new robot is built.

No robotics math lives here -- everything is read from
analysis.singularity.analyze_singularity(), which itself builds on
kinematics/jacobian.py.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QLabel,
    QGroupBox,
    QGridLayout,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
)
from PySide6.QtGui import QFont

from robot.robot_model import RobotModel
from analysis.singularity import (
    analyze_singularity,
    ManipulabilityStatus,
    SubJacobianAnalysis,
)

_STATUS_COLORS = {
    ManipulabilityStatus.GOOD: "#2E7D32",                      # green
    ManipulabilityStatus.APPROACHING_SINGULARITY: "#D9822B",   # amber
    ManipulabilityStatus.SINGULAR: "#C0392B",                  # red
}

_STATUS_ICONS = {
    ManipulabilityStatus.GOOD: "\u2713",                # check mark
    ManipulabilityStatus.APPROACHING_SINGULARITY: "\u26a0",  # warning triangle
    ManipulabilityStatus.SINGULAR: "\u2715",            # cross mark
}


class _SubJacobianWidgets:
    """Holds the labels for one block's (linear or angular) metrics."""

    def __init__(self, layout: QGridLayout, row_offset: int) -> None:
        self.rank_label = QLabel("--")
        self.condition_label = QLabel("--")
        self.manipulability_label = QLabel("--")

        layout.addWidget(QLabel("Rank:"), row_offset, 0)
        layout.addWidget(self.rank_label, row_offset, 1)
        layout.addWidget(QLabel("Condition #:"), row_offset + 1, 0)
        layout.addWidget(self.condition_label, row_offset + 1, 1)
        layout.addWidget(QLabel("Manipulability:"), row_offset + 2, 0)
        layout.addWidget(self.manipulability_label, row_offset + 2, 1)

    def update(self, analysis: SubJacobianAnalysis, mono_font: QFont) -> None:
        for label in (self.rank_label, self.condition_label, self.manipulability_label):
            label.setFont(mono_font)

        self.rank_label.setText(f"{analysis.rank} / {analysis.full_rank}")
        if np.isinf(analysis.condition_number):
            self.condition_label.setText("inf (singular)")
        else:
            self.condition_label.setText(f"{analysis.condition_number:.2f}")
        self.manipulability_label.setText(f"{analysis.manipulability:.6f}")


class AnalysisPanel(QWidget):
    """Read-only panel showing live Jacobian and singularity analysis."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.robot: Optional[RobotModel] = None

        self._mono_font = QFont("Consolas")
        if not self._mono_font.exactMatch():
            self._mono_font = QFont("Courier New")
        self._mono_font.setStyleHint(QFont.Monospace)

        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title = QLabel("Jacobian && Singularity Analysis")
        title.setStyleSheet("font-weight: 600; font-size: 14px;")
        layout.addWidget(title)

        # --- Status banner --------------------------------------------
        self.status_label = QLabel("--")
        self.status_label.setStyleSheet("font-weight: 600; font-size: 13px;")
        layout.addWidget(self.status_label)

        # --- Linear (J_v) metrics -----------------------------------------
        linear_box = QGroupBox("Linear (Position) Manipulability")
        linear_layout = QGridLayout(linear_box)
        self.linear_widgets = _SubJacobianWidgets(linear_layout, 0)
        layout.addWidget(linear_box)

        # --- Angular (J_w) metrics -----------------------------------------
        angular_box = QGroupBox("Angular (Orientation) Manipulability")
        angular_layout = QGridLayout(angular_box)
        self.angular_widgets = _SubJacobianWidgets(angular_layout, 0)
        layout.addWidget(angular_box)

        # --- Full Jacobian matrix ------------------------------------------
        jacobian_box = QGroupBox("Jacobian J (J_v over J_w)")
        jacobian_layout = QVBoxLayout(jacobian_box)
        self.jacobian_table = QTableWidget(6, 0)
        self.jacobian_table.setVerticalHeaderLabels(
            ["vx", "vy", "vz", "wx", "wy", "wz"]
        )
        self.jacobian_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.jacobian_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.jacobian_table.setMinimumHeight(190)
        jacobian_layout.addWidget(self.jacobian_table)
        layout.addWidget(jacobian_box)

        note = QLabel(
            "Linear and angular manipulability are reported separately "
            "since position (m) and orientation (rad) have different "
            "units \u2014 combining them into one measure would be "
            "dimensionally inconsistent."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #777777; font-size: 10px;")
        layout.addWidget(note)

        layout.addStretch(1)

    def set_robot(self, robot: RobotModel) -> None:
        """Point this panel at a (possibly new) robot and refresh."""
        self.robot = robot
        self.jacobian_table.setColumnCount(robot.dof)
        self.jacobian_table.setHorizontalHeaderLabels(
            [j.name for j in robot.joints]
        )
        self.update_analysis()

    def update_analysis(self) -> None:
        """Recompute the Jacobian/singularity analysis for the current
        robot state and refresh every field. Call this whenever the
        robot's joint values change (slider move, IK apply, etc.)."""
        if self.robot is None:
            return

        fk_result = self.robot.compute_fk()
        result = analyze_singularity(self.robot.joints, self.robot.links, fk_result)

        color = _STATUS_COLORS[result.status]
        icon = _STATUS_ICONS[result.status]
        self.status_label.setText(f"{icon} {result.status.value}")
        self.status_label.setStyleSheet(f"font-weight: 600; font-size: 13px; color: {color};")

        self.linear_widgets.update(result.linear, self._mono_font)
        self.angular_widgets.update(result.angular, self._mono_font)

        self._fill_jacobian_table(result.jacobian)

    def _fill_jacobian_table(self, J: np.ndarray) -> None:
        rows, cols = J.shape
        if self.jacobian_table.columnCount() != cols:
            self.jacobian_table.setColumnCount(cols)
        for r in range(rows):
            for c in range(cols):
                item = QTableWidgetItem(f"{J[r, c]:.4f}")
                item.setFont(self._mono_font)
                self.jacobian_table.setItem(r, c, item)
