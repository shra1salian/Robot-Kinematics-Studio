"""
ik_panel.py

Phase 5: the interactive Inverse Kinematics panel. Lets the user enter
a target end-effector position (and, optionally, orientation via
Roll/Pitch/Yaw), choose a solver method, and run numerical IK against
the currently loaded robot. Displays convergence status, iteration
count, and position/orientation error -- and, on success, offers to
apply the solution to the robot (updating the sliders/3D view/FK tab).

This module is GUI-only: unit conversion (degrees -> radians for the
orientation target) happens here at the boundary; all solving is
delegated to RobotModel.solve_ik() / kinematics.inverse_kinematics.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QLabel,
    QDoubleSpinBox,
    QComboBox,
    QCheckBox,
    QPushButton,
    QGroupBox,
)

from robot.robot_model import RobotModel
from kinematics import transformations as tf
from kinematics.inverse_kinematics import IKMethod, DEFAULT_MAX_ITERATIONS


class IKPanel(QWidget):
    """Panel for configuring and running numerical inverse kinematics.

    Signals:
        solution_applied: emitted (no payload) when a converged IK
            solution is applied to the robot, so the rest of the GUI
            (3D view, joint sliders, FK tab) can refresh.
    """

    solution_applied = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.robot: Optional[RobotModel] = None
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title = QLabel("Inverse Kinematics")
        title.setStyleSheet("font-weight: 600; font-size: 14px;")
        layout.addWidget(title)

        # --- Target position -------------------------------------------
        pos_box = QGroupBox("Target Position (m)")
        pos_form = QFormLayout(pos_box)
        self.x_spin = self._make_pos_spin()
        self.y_spin = self._make_pos_spin()
        self.z_spin = self._make_pos_spin()
        pos_form.addRow("X:", self.x_spin)
        pos_form.addRow("Y:", self.y_spin)
        pos_form.addRow("Z:", self.z_spin)
        layout.addWidget(pos_box)

        # --- Target orientation (optional) ------------------------------
        orient_box = QGroupBox("Target Orientation (optional)")
        orient_layout = QVBoxLayout(orient_box)
        self.use_orientation_check = QCheckBox("Include orientation in target")
        self.use_orientation_check.toggled.connect(self._on_orientation_toggle)
        orient_layout.addWidget(self.use_orientation_check)

        orient_form = QFormLayout()
        self.roll_spin = self._make_angle_spin()
        self.pitch_spin = self._make_angle_spin()
        self.yaw_spin = self._make_angle_spin()
        orient_form.addRow("Roll (deg):", self.roll_spin)
        orient_form.addRow("Pitch (deg):", self.pitch_spin)
        orient_form.addRow("Yaw (deg):", self.yaw_spin)
        orient_layout.addLayout(orient_form)
        layout.addWidget(orient_box)
        self._on_orientation_toggle(False)

        # --- Solver options -----------------------------------------------
        solver_box = QGroupBox("Solver")
        solver_form = QFormLayout(solver_box)

        self.method_combo = QComboBox()
        self.method_combo.addItems(["Damped Least Squares", "Pseudoinverse"])
        solver_form.addRow("Method:", self.method_combo)

        self.max_iter_spin = QDoubleSpinBox()
        self.max_iter_spin.setDecimals(0)
        self.max_iter_spin.setRange(10, 2000)
        self.max_iter_spin.setValue(DEFAULT_MAX_ITERATIONS)
        solver_form.addRow("Max iterations:", self.max_iter_spin)

        layout.addWidget(solver_box)

        # --- Actions ------------------------------------------------------
        button_row = QHBoxLayout()
        self.solve_btn = QPushButton("Solve IK")
        self.solve_btn.setStyleSheet("font-weight: 600;")
        self.solve_btn.clicked.connect(self._on_solve_clicked)
        button_row.addWidget(self.solve_btn)

        self.apply_btn = QPushButton("Apply Solution")
        self.apply_btn.setEnabled(False)
        self.apply_btn.clicked.connect(self._on_apply_clicked)
        button_row.addWidget(self.apply_btn)
        layout.addLayout(button_row)

        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        layout.addStretch(1)

        self._last_result = None

    def _make_pos_spin(self) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(-10.0, 10.0)
        spin.setDecimals(4)
        spin.setSingleStep(0.01)
        return spin

    def _make_angle_spin(self) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(-180.0, 180.0)
        spin.setDecimals(2)
        spin.setSingleStep(1.0)
        return spin

    def _on_orientation_toggle(self, checked: bool) -> None:
        for spin in (self.roll_spin, self.pitch_spin, self.yaw_spin):
            spin.setEnabled(checked)

    def set_robot(self, robot: RobotModel) -> None:
        """Point this panel at a (possibly new) robot. Pre-fills the
        target position with the robot's current end-effector position
        so the fields start somewhere reasonable.
        """
        self.robot = robot
        fk = robot.compute_fk()
        pos = fk.end_effector_position()
        self.x_spin.setValue(pos[0])
        self.y_spin.setValue(pos[1])
        self.z_spin.setValue(pos[2])
        self.apply_btn.setEnabled(False)
        self._last_result = None
        self.status_label.setText("")

    def _on_solve_clicked(self) -> None:
        if self.robot is None:
            self.status_label.setText("No robot loaded.")
            return

        target_position = np.array(
            [self.x_spin.value(), self.y_spin.value(), self.z_spin.value()]
        )

        target_orientation = None
        if self.use_orientation_check.isChecked():
            roll = np.radians(self.roll_spin.value())
            pitch = np.radians(self.pitch_spin.value())
            yaw = np.radians(self.yaw_spin.value())
            target_orientation = (
                tf.rotation_z(yaw) @ tf.rotation_y(pitch) @ tf.rotation_x(roll)
            )

        method = (
            IKMethod.DAMPED_LEAST_SQUARES
            if self.method_combo.currentText() == "Damped Least Squares"
            else IKMethod.PSEUDOINVERSE
        )

        result = self.robot.solve_ik(
            target_position=target_position,
            target_orientation=target_orientation,
            apply_result=False,
            method=method,
            max_iterations=int(self.max_iter_spin.value()),
        )

        self._last_result = result
        self.apply_btn.setEnabled(result.converged)

        color = "#2E7D32" if result.converged else "#C0392B"
        self.status_label.setText(result.status_message())
        self.status_label.setStyleSheet(f"color: {color};")

    def _on_apply_clicked(self) -> None:
        if self.robot is None or self._last_result is None or not self._last_result.converged:
            return
        self.robot.set_joint_values(self._last_result.joint_values)
        self.solution_applied.emit()
