"""
kinematics_panel.py

Phase 3: displays forward-kinematics results for the current robot
configuration -- end-effector position, orientation (rotation matrix,
with Euler/RPY and quaternion also shown), and the full 4x4
homogeneous transform. This panel is read-only in Phase 3; it simply
reflects whatever robot/configuration it's given via `update_robot()`.

No robotics math lives here -- everything is read from
RobotModel.compute_fk() (kinematics/forward_kinematics.py) and
kinematics/transformations.py's orientation converters.
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
from kinematics import transformations as tf


class KinematicsPanel(QWidget):
    """Read-only panel showing live forward-kinematics results."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._mono_font = QFont("Consolas")
        if not self._mono_font.exactMatch():
            self._mono_font = QFont("Courier New")
        self._mono_font.setStyleHint(QFont.Monospace)

        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title = QLabel("Forward Kinematics")
        title.setStyleSheet("font-weight: 600; font-size: 14px;")
        layout.addWidget(title)

        # --- Position ---------------------------------------------------
        pos_box = QGroupBox("End-Effector Position")
        pos_layout = QGridLayout(pos_box)
        self.x_label = self._make_value_label()
        self.y_label = self._make_value_label()
        self.z_label = self._make_value_label()
        pos_layout.addWidget(QLabel("X:"), 0, 0)
        pos_layout.addWidget(self.x_label, 0, 1)
        pos_layout.addWidget(QLabel("Y:"), 1, 0)
        pos_layout.addWidget(self.y_label, 1, 1)
        pos_layout.addWidget(QLabel("Z:"), 2, 0)
        pos_layout.addWidget(self.z_label, 2, 1)
        layout.addWidget(pos_box)

        # --- Orientation --------------------------------------------------
        orient_box = QGroupBox("End-Effector Orientation")
        orient_layout = QVBoxLayout(orient_box)

        self.rotation_table = QTableWidget(3, 3)
        self.rotation_table.setFixedHeight(96)
        self.rotation_table.horizontalHeader().setVisible(False)
        self.rotation_table.verticalHeader().setVisible(False)
        self.rotation_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.rotation_table.setEditTriggers(QTableWidget.NoEditTriggers)
        orient_layout.addWidget(QLabel("Rotation matrix R:"))
        orient_layout.addWidget(self.rotation_table)

        self.rpy_label = self._make_value_label()
        self.quat_label = self._make_value_label()
        rpy_row = QGridLayout()
        rpy_row.addWidget(QLabel("Roll / Pitch / Yaw (deg):"), 0, 0)
        rpy_row.addWidget(self.rpy_label, 0, 1)
        rpy_row.addWidget(QLabel("Quaternion [w, x, y, z]:"), 1, 0)
        rpy_row.addWidget(self.quat_label, 1, 1)
        orient_layout.addLayout(rpy_row)

        layout.addWidget(orient_box)

        # --- Full homogeneous transform -----------------------------------
        transform_box = QGroupBox("Homogeneous Transform T (base -> end-effector)")
        transform_layout = QVBoxLayout(transform_box)
        self.transform_table = QTableWidget(4, 4)
        self.transform_table.setFixedHeight(120)
        self.transform_table.horizontalHeader().setVisible(False)
        self.transform_table.verticalHeader().setVisible(False)
        self.transform_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.transform_table.setEditTriggers(QTableWidget.NoEditTriggers)
        transform_layout.addWidget(self.transform_table)
        layout.addWidget(transform_box)

        layout.addStretch(1)

    def _make_value_label(self) -> QLabel:
        label = QLabel("--")
        label.setFont(self._mono_font)
        return label

    def update_robot(self, robot: RobotModel) -> None:
        """Recompute FK for `robot`'s current joint values and refresh
        every field in the panel.

        Args:
            robot: The RobotModel to display FK results for.
        """
        fk_result = robot.compute_fk()
        T = fk_result.end_effector_transform
        position = fk_result.end_effector_position()
        R = fk_result.end_effector_rotation()

        self.x_label.setText(f"{position[0]:.4f} m")
        self.y_label.setText(f"{position[1]:.4f} m")
        self.z_label.setText(f"{position[2]:.4f} m")

        self._fill_matrix_table(self.rotation_table, R)
        self._fill_matrix_table(self.transform_table, T)

        rpy = np.degrees(tf.rotation_matrix_to_euler_xyz(R))
        self.rpy_label.setText(f"[{rpy[0]:7.2f}, {rpy[1]:7.2f}, {rpy[2]:7.2f}]")

        quat = tf.rotation_matrix_to_quaternion(R)
        self.quat_label.setText(
            f"[{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]"
        )

    def _fill_matrix_table(self, table: QTableWidget, matrix: np.ndarray) -> None:
        rows, cols = matrix.shape
        for r in range(rows):
            for c in range(cols):
                item = QTableWidgetItem(f"{matrix[r, c]:.4f}")
                item.setFont(self._mono_font)
                table.setItem(r, c, item)
