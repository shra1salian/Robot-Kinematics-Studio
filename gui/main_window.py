"""
main_window.py

Phase 1 main window: hosts the 3D viewport and displays the hard-coded
2-DOF demo robot. Joint sliders, kinematics panels, and analysis
panels are intentionally NOT implemented yet -- they arrive in later
phases (see Section 22 of the design spec).
"""

from __future__ import annotations

from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QFrame,
)
from PySide6.QtCore import Qt

from robot.robot_model import RobotModel, create_demo_2dof_robot
from gui.visualization_widget import VisualizationWidget


class MainWindow(QMainWindow):
    """Top-level application window for Robot Kinematics Studio."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Kinematics Studio — Phase 1")
        self.resize(1200, 800)

        self.robot: RobotModel = create_demo_2dof_robot()

        self._build_ui()
        self._refresh_view()

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)

        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(8)

        # Left info panel (placeholder for future Robot Builder panel)
        info_panel = self._build_info_panel()
        root_layout.addWidget(info_panel, stretch=0)

        # Center: 3D viewport
        self.viz_widget = VisualizationWidget(self)
        root_layout.addWidget(self.viz_widget, stretch=1)

        self.statusBar().showMessage(
            f"Loaded '{self.robot.name}' — {self.robot.dof} DOF"
        )

    def _build_info_panel(self) -> QWidget:
        panel = QFrame(self)
        panel.setFrameShape(QFrame.StyledPanel)
        panel.setFixedWidth(260)

        layout = QVBoxLayout(panel)
        layout.setAlignment(Qt.AlignTop)

        title = QLabel("Robot Info")
        title.setStyleSheet("font-weight: 600; font-size: 14px;")
        layout.addWidget(title)

        name_label = QLabel(f"Name: {self.robot.name}")
        dof_label = QLabel(f"DOF: {self.robot.dof}")
        layout.addWidget(name_label)
        layout.addWidget(dof_label)

        joints_title = QLabel("Joints")
        joints_title.setStyleSheet("font-weight: 600; margin-top: 12px;")
        layout.addWidget(joints_title)

        for joint in self.robot.joints:
            text = (
                f"{joint.name}: {joint.joint_type.value}, "
                f"axis={joint.axis.value.upper()}, "
                f"value={joint.joint_value:.3f} rad"
            )
            layout.addWidget(QLabel(text))

        note = QLabel(
            "\nJoint sliders, forward kinematics, and analysis "
            "panels will be added in later phases."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #777777; font-size: 11px;")
        layout.addWidget(note)

        return panel

    def _refresh_view(self) -> None:
        """Push the current robot state into the 3D viewport."""
        self.viz_widget.update_robot(self.robot)
        self.viz_widget.reset_camera()
