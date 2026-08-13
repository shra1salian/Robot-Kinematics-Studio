"""
robot_builder.py

Phase 2: the interactive Robot Builder panel (Section 5 of the design
spec). Lets the user pick a number of degrees of freedom, generates a
configuration table (type, axis, link length, limits) for each joint,
validates the input, and emits a fully-built RobotModel.

This module is GUI-only: it reads user input, converts units at the
GUI boundary (degrees -> radians for revolute limits), and delegates
all validation/construction to robot.robot_model.RobotModel /
JointConfig. No robotics math lives here.
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np
from PySide6.QtCore import Signal, Qt
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QLabel,
    QSpinBox,
    QDoubleSpinBox,
    QComboBox,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QMessageBox,
    QGroupBox,
    QHeaderView,
)

from robot.joint import JointType, JointAxis
from robot.robot_model import RobotModel, JointConfig, RobotModelError

# Table column indices
COL_NAME = 0
COL_TYPE = 1
COL_AXIS = 2
COL_LINK_LENGTH = 3
COL_MIN_LIMIT = 4
COL_MAX_LIMIT = 5
COLUMN_COUNT = 6

# Sensible defaults for newly generated rows
DEFAULT_LINK_LENGTH_M = 0.30
DEFAULT_REVOLUTE_MIN_DEG = -180.0
DEFAULT_REVOLUTE_MAX_DEG = 180.0
DEFAULT_PRISMATIC_MIN_M = 0.0
DEFAULT_PRISMATIC_MAX_M = 0.50

MIN_DOF = 1
MAX_DOF = 8


class RobotBuilderPanel(QWidget):
    """Panel allowing the user to construct an arbitrary serial robot.

    Signals:
        robot_built: emitted with a valid RobotModel once the user
            clicks "Build Robot" and validation succeeds.
    """

    robot_built = Signal(object)  # emits a RobotModel

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._build_ui()
        self._generate_table(3)  # sensible starting point

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title = QLabel("Robot Builder")
        title.setStyleSheet("font-weight: 600; font-size: 14px;")
        layout.addWidget(title)

        # --- DOF selection -------------------------------------------------
        dof_box = QGroupBox("Configuration")
        dof_form = QFormLayout(dof_box)

        self.robot_name_edit = QLineEdit("Custom Robot")
        dof_form.addRow("Robot name:", self.robot_name_edit)

        self.dof_spin = QSpinBox()
        self.dof_spin.setRange(MIN_DOF, MAX_DOF)
        self.dof_spin.setValue(3)
        dof_form.addRow("Degrees of freedom:", self.dof_spin)

        self.generate_btn = QPushButton("Generate Joint Table")
        self.generate_btn.clicked.connect(self._on_generate_clicked)
        dof_form.addRow(self.generate_btn)

        layout.addWidget(dof_box)

        # --- Joint configuration table --------------------------------------
        table_label = QLabel("Joint Configuration")
        table_label.setStyleSheet("font-weight: 600; margin-top: 4px;")
        layout.addWidget(table_label)

        self.table = QTableWidget(0, COLUMN_COUNT)
        self.table.setHorizontalHeaderLabels(
            ["Joint", "Type", "Axis", "Link Length (m)", "Min Limit", "Max Limit"]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        layout.addWidget(self.table, stretch=1)

        self.limit_hint_label = QLabel(
            "Limits: degrees for Revolute joints, meters for Prismatic joints."
        )
        self.limit_hint_label.setWordWrap(True)
        self.limit_hint_label.setStyleSheet("color: #777777; font-size: 11px;")
        layout.addWidget(self.limit_hint_label)

        # --- Build action ----------------------------------------------------
        button_row = QHBoxLayout()
        self.build_btn = QPushButton("Build Robot")
        self.build_btn.setStyleSheet("font-weight: 600;")
        self.build_btn.clicked.connect(self._on_build_clicked)
        button_row.addWidget(self.build_btn)
        layout.addLayout(button_row)

        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

    # ------------------------------------------------------------------
    # Table generation
    # ------------------------------------------------------------------
    def _on_generate_clicked(self) -> None:
        self._generate_table(self.dof_spin.value())

    def _generate_table(self, dof: int) -> None:
        """(Re)build the table with `dof` rows of default values,
        preserving values from existing rows where an index still
        exists, so tweaking DOF doesn't discard everything the user
        already entered.
        """
        previous_rows = self._read_raw_rows(ignore_errors=True)

        self.table.setRowCount(0)
        self.table.setRowCount(dof)

        for row in range(dof):
            defaults = previous_rows[row] if row < len(previous_rows) else None
            self._populate_row(row, defaults)

        self.status_label.setText(f"Generated {dof} joint row(s). Edit as needed, then Build Robot.")
        self.status_label.setStyleSheet("color: #777777;")

    def _populate_row(self, row: int, defaults: Optional[dict]) -> None:
        name = defaults["name"] if defaults else f"J{row + 1}"
        joint_type = defaults["joint_type"] if defaults else JointType.REVOLUTE
        axis = defaults["axis"] if defaults else JointAxis.Z
        link_length = defaults["link_length"] if defaults else DEFAULT_LINK_LENGTH_M
        min_limit_display = defaults["min_display"] if defaults else DEFAULT_REVOLUTE_MIN_DEG
        max_limit_display = defaults["max_display"] if defaults else DEFAULT_REVOLUTE_MAX_DEG

        # Name
        name_item = QTableWidgetItem(name)
        self.table.setItem(row, COL_NAME, name_item)

        # Type combo
        type_combo = QComboBox()
        type_combo.addItems(["Revolute", "Prismatic"])
        type_combo.setCurrentText("Revolute" if joint_type is JointType.REVOLUTE else "Prismatic")
        type_combo.currentTextChanged.connect(lambda _text, r=row: self._on_type_changed(r))
        self.table.setCellWidget(row, COL_TYPE, type_combo)

        # Axis combo
        axis_combo = QComboBox()
        axis_combo.addItems(["X", "Y", "Z"])
        axis_combo.setCurrentText(axis.value.upper())
        self.table.setCellWidget(row, COL_AXIS, axis_combo)

        # Link length
        length_spin = QDoubleSpinBox()
        length_spin.setRange(0.0, 10.0)
        length_spin.setDecimals(3)
        length_spin.setSingleStep(0.01)
        length_spin.setValue(link_length)
        self.table.setCellWidget(row, COL_LINK_LENGTH, length_spin)

        # Min / max limit (range depends on joint type)
        min_spin = QDoubleSpinBox()
        max_spin = QDoubleSpinBox()
        for spin in (min_spin, max_spin):
            spin.setDecimals(3)
            spin.setSingleStep(1.0)
        self.table.setCellWidget(row, COL_MIN_LIMIT, min_spin)
        self.table.setCellWidget(row, COL_MAX_LIMIT, max_spin)

        self._apply_limit_range_for_type(row, joint_type)
        min_spin.setValue(min_limit_display)
        max_spin.setValue(max_limit_display)

    def _on_type_changed(self, row: int) -> None:
        """When a row's joint type changes, reset its limit spin boxes
        to sensible defaults/ranges for the new type (degrees vs
        meters) so stale values from the previous type don't linger.
        """
        type_combo = self.table.cellWidget(row, COL_TYPE)
        if type_combo is None:
            return
        new_type = (
            JointType.REVOLUTE if type_combo.currentText() == "Revolute" else JointType.PRISMATIC
        )
        self._apply_limit_range_for_type(row, new_type)

        min_spin = self.table.cellWidget(row, COL_MIN_LIMIT)
        max_spin = self.table.cellWidget(row, COL_MAX_LIMIT)
        if new_type is JointType.REVOLUTE:
            min_spin.setValue(DEFAULT_REVOLUTE_MIN_DEG)
            max_spin.setValue(DEFAULT_REVOLUTE_MAX_DEG)
        else:
            min_spin.setValue(DEFAULT_PRISMATIC_MIN_M)
            max_spin.setValue(DEFAULT_PRISMATIC_MAX_M)

    def _apply_limit_range_for_type(self, row: int, joint_type: JointType) -> None:
        min_spin = self.table.cellWidget(row, COL_MIN_LIMIT)
        max_spin = self.table.cellWidget(row, COL_MAX_LIMIT)
        if joint_type is JointType.REVOLUTE:
            min_spin.setRange(-360.0, 360.0)
            max_spin.setRange(-360.0, 360.0)
            min_spin.setSuffix(" deg")
            max_spin.setSuffix(" deg")
        else:
            min_spin.setRange(-5.0, 5.0)
            max_spin.setRange(-5.0, 5.0)
            min_spin.setSuffix(" m")
            max_spin.setSuffix(" m")

    # ------------------------------------------------------------------
    # Reading table state
    # ------------------------------------------------------------------
    def _read_raw_rows(self, ignore_errors: bool = False) -> List[dict]:
        """Read the table into a list of plain dicts (GUI-unit values,
        NOT yet converted to radians / validated). Used both for
        preserving values across a DOF change and as the first step of
        `_build_joint_configs`.
        """
        rows: List[dict] = []
        for row in range(self.table.rowCount()):
            try:
                name_item = self.table.item(row, COL_NAME)
                name = name_item.text() if name_item else f"J{row + 1}"

                type_combo = self.table.cellWidget(row, COL_TYPE)
                joint_type = (
                    JointType.REVOLUTE
                    if type_combo.currentText() == "Revolute"
                    else JointType.PRISMATIC
                )

                axis_combo = self.table.cellWidget(row, COL_AXIS)
                axis = JointAxis(axis_combo.currentText().lower())

                length_spin = self.table.cellWidget(row, COL_LINK_LENGTH)
                link_length = length_spin.value()

                min_spin = self.table.cellWidget(row, COL_MIN_LIMIT)
                max_spin = self.table.cellWidget(row, COL_MAX_LIMIT)
                min_display = min_spin.value()
                max_display = max_spin.value()

                rows.append(
                    {
                        "name": name,
                        "joint_type": joint_type,
                        "axis": axis,
                        "link_length": link_length,
                        "min_display": min_display,
                        "max_display": max_display,
                    }
                )
            except Exception:
                if ignore_errors:
                    continue
                raise
        return rows

    def _build_joint_configs(self) -> List[JointConfig]:
        """Convert the current table state into a list of JointConfig,
        applying the degrees->radians conversion for revolute joints
        at this GUI boundary (Section 27 of the design spec: never mix
        degrees and radians internally).
        """
        configs: List[JointConfig] = []
        for raw in self._read_raw_rows():
            if raw["joint_type"] is JointType.REVOLUTE:
                min_limit = np.radians(raw["min_display"])
                max_limit = np.radians(raw["max_display"])
            else:
                min_limit = raw["min_display"]
                max_limit = raw["max_display"]

            configs.append(
                JointConfig(
                    name=raw["name"],
                    joint_type=raw["joint_type"],
                    axis=raw["axis"],
                    link_length=raw["link_length"],
                    minimum_limit=min_limit,
                    maximum_limit=max_limit,
                )
            )
        return configs

    # ------------------------------------------------------------------
    # Build action
    # ------------------------------------------------------------------
    def _on_build_clicked(self) -> None:
        if self.table.rowCount() == 0:
            self._show_error(["No joints defined. Generate a joint table first."])
            return

        configs = self._build_joint_configs()
        errors = RobotModel.validate_configs(configs)
        if errors:
            self._show_error(errors)
            return

        robot_name = self.robot_name_edit.text().strip() or "Custom Robot"
        try:
            robot = RobotModel.from_config(robot_name, configs)
        except RobotModelError as exc:
            self._show_error([str(exc)])
            return

        self.status_label.setText(
            f"Built '{robot.name}' successfully ({robot.dof} DOF)."
        )
        self.status_label.setStyleSheet("color: #2E7D32;")  # green
        self.robot_built.emit(robot)

    def _show_error(self, errors: List[str]) -> None:
        message = "\n".join(f"\u2022 {e}" for e in errors)
        self.status_label.setText("Cannot build robot \u2014 see details.")
        self.status_label.setStyleSheet("color: #C0392B;")  # red
        QMessageBox.warning(self, "Invalid Robot Configuration", message)
