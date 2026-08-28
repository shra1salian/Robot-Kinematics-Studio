"""
joint_panel.py

Phase 4: the interactive joint-control panel (Section 8 of the design
spec). Shows one slider per joint, in user-friendly units (degrees for
revolute, millimeters for prismatic), each respecting that joint's own
limits. Moving a slider updates the RobotModel's joint value directly
and emits a signal so the rest of the GUI (3D viewport, Forward
Kinematics tab) can refresh in real time.

This module is GUI-only: it converts between display units and the
robot's internal SI units (radians / meters) at this boundary, exactly
like gui/robot_builder.py does. No robotics math beyond that unit
conversion lives here.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from PySide6.QtCore import Signal, Qt
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QPushButton,
    QScrollArea,
    QSizePolicy,
)

from robot.robot_model import RobotModel

# Internal slider resolution: QSlider only works with ints, so we map
# each joint's [minimum_limit, maximum_limit] onto this many discrete
# steps for smooth-feeling dragging regardless of the joint's range.
SLIDER_STEPS = 1000


class _JointSliderRow(QWidget):
    """One row: joint name, slider, and a live numeric readout."""

    def __init__(self, joint_index: int, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.joint_index = joint_index
        self.is_revolute = True  # updated by configure()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)
        layout.setSpacing(2)

        header = QHBoxLayout()
        self.name_label = QLabel()
        self.name_label.setStyleSheet("font-weight: 600;")
        self.value_label = QLabel()
        self.value_label.setAlignment(Qt.AlignRight)
        header.addWidget(self.name_label)
        header.addStretch(1)
        header.addWidget(self.value_label)
        layout.addLayout(header)

        slider_row = QHBoxLayout()
        self.min_label = QLabel()
        self.min_label.setStyleSheet("color: #888888; font-size: 10px;")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.max_label = QLabel()
        self.max_label.setStyleSheet("color: #888888; font-size: 10px;")
        slider_row.addWidget(self.min_label)
        slider_row.addWidget(self.slider, stretch=1)
        slider_row.addWidget(self.max_label)
        layout.addLayout(slider_row)

    def configure(self, name: str, is_revolute: bool, min_limit: float, max_limit: float) -> None:
        """Set up this row's static labels for a (possibly new) joint."""
        self.is_revolute = is_revolute
        self.name_label.setText(f"{name} ({'Revolute' if is_revolute else 'Prismatic'})")
        unit = "deg" if is_revolute else "mm"
        display_min = np.degrees(min_limit) if is_revolute else min_limit * 1000.0
        display_max = np.degrees(max_limit) if is_revolute else max_limit * 1000.0
        self.min_label.setText(f"{display_min:.0f}{unit}")
        self.max_label.setText(f"{display_max:.0f}{unit}")

    def set_value_silent(self, value: float, min_limit: float, max_limit: float) -> None:
        """Update slider position and readout to reflect `value`
        (internal units: radians/meters) without emitting valueChanged.
        """
        span = max_limit - min_limit
        frac = 0.5 if span <= 1e-12 else (value - min_limit) / span
        frac = min(max(frac, 0.0), 1.0)
        raw = int(round(frac * SLIDER_STEPS))

        self.slider.blockSignals(True)
        self.slider.setValue(raw)
        self.slider.blockSignals(False)
        self._update_readout(value)

    def value_from_raw(self, raw: int, min_limit: float, max_limit: float) -> float:
        """Convert a raw slider position back to an internal-unit value."""
        frac = raw / SLIDER_STEPS
        return min_limit + frac * (max_limit - min_limit)

    def _update_readout(self, value: float) -> None:
        if self.is_revolute:
            self.value_label.setText(f"{np.degrees(value):.1f} deg")
        else:
            self.value_label.setText(f"{value * 1000.0:.1f} mm")


class JointControlPanel(QWidget):
    """Panel showing one slider per joint of the current robot.

    Signals:
        robot_changed: emitted (no payload) whenever a slider moves or
            "Reset All" is pressed, after the underlying RobotModel's
            joint value has already been updated. Listeners should
            re-read the robot (e.g. re-render, recompute FK).
    """

    robot_changed = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.robot: Optional[RobotModel] = None
        self._rows: list[_JointSliderRow] = []
        self._build_ui()

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(8)

        title = QLabel("Joint Control")
        title.setStyleSheet("font-weight: 600; font-size: 14px;")
        outer.addWidget(title)

        self.empty_label = QLabel("No robot loaded.")
        self.empty_label.setStyleSheet("color: #888888;")
        outer.addWidget(self.empty_label)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        self.rows_container = QWidget()
        self.rows_layout = QVBoxLayout(self.rows_container)
        self.rows_layout.setContentsMargins(0, 0, 0, 0)
        self.rows_layout.addStretch(1)
        scroll.setWidget(self.rows_container)
        outer.addWidget(scroll, stretch=1)

        self.reset_btn = QPushButton("Reset All Joints")
        self.reset_btn.clicked.connect(self._on_reset_clicked)
        outer.addWidget(self.reset_btn)

    def set_robot(self, robot: RobotModel) -> None:
        """Rebuild the slider rows for a new/changed robot and sync
        each slider to that robot's current joint values.

        Args:
            robot: The RobotModel to control. Rows are recreated only
                when the DOF count differs from the current row count;
                otherwise existing rows are reconfigured in place.
        """
        self.robot = robot
        self.empty_label.setVisible(False)

        if len(self._rows) != robot.dof:
            self._rebuild_rows(robot.dof)

        for i, joint in enumerate(robot.joints):
            row = self._rows[i]
            row.configure(joint.name, joint.is_revolute(), joint.minimum_limit, joint.maximum_limit)
            row.set_value_silent(joint.joint_value, joint.minimum_limit, joint.maximum_limit)

    def _rebuild_rows(self, dof: int) -> None:
        # Remove old rows
        for row in self._rows:
            row.setParent(None)
            row.deleteLater()
        self._rows = []

        # Remove the trailing stretch so we can re-add it after new rows
        while self.rows_layout.count():
            item = self.rows_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        for i in range(dof):
            row = _JointSliderRow(i)
            row.slider.valueChanged.connect(
                lambda raw, idx=i: self._on_slider_moved(idx, raw)
            )
            self.rows_layout.addWidget(row)
            self._rows.append(row)

        self.rows_layout.addStretch(1)

    def _on_slider_moved(self, joint_index: int, raw: int) -> None:
        if self.robot is None:
            return
        joint = self.robot.joints[joint_index]
        row = self._rows[joint_index]
        value = row.value_from_raw(raw, joint.minimum_limit, joint.maximum_limit)
        joint.set_value(value)
        row._update_readout(joint.joint_value)
        self.robot_changed.emit()

    def _on_reset_clicked(self) -> None:
        if self.robot is None:
            return
        self.robot.reset()
        for i, joint in enumerate(self.robot.joints):
            self._rows[i].set_value_silent(joint.joint_value, joint.minimum_limit, joint.maximum_limit)
        self.robot_changed.emit()
