"""
main_window.py

Phase 4 main window: hosts the Robot Builder panel (left), the 3D
viewport (center), and a tabbed right-hand panel with "Robot Info",
"Joint Control" (interactive sliders), and "Forward Kinematics" (live
end-effector position/orientation/transform).

Moving a joint slider updates the robot model, then refreshes the 3D
viewport and the FK tab in real time -- see Section 8 of the design
spec. IK and analysis panels still arrive in later phases (Section 22).
"""

from __future__ import annotations

from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QFrame,
    QTabWidget,
)
from PySide6.QtCore import Qt

from robot.robot_model import RobotModel, create_demo_2dof_robot
from gui.visualization_widget import VisualizationWidget
from gui.robot_builder import RobotBuilderPanel
from gui.kinematics_panel import KinematicsPanel
from gui.joint_panel import JointControlPanel


class MainWindow(QMainWindow):
    """Top-level application window for Robot Kinematics Studio."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Kinematics Studio \u2014 Phase 4")
        self.resize(1650, 850)

        self.robot: RobotModel = create_demo_2dof_robot()

        self._build_ui()
        self._load_robot(self.robot)

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)

        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(8)

        # Left: Robot Builder panel
        builder_frame = QFrame(self)
        builder_frame.setFrameShape(QFrame.StyledPanel)
        builder_frame.setFixedWidth(620)
        builder_layout = QVBoxLayout(builder_frame)
        builder_layout.setContentsMargins(10, 10, 10, 10)

        self.builder_panel = RobotBuilderPanel(self)
        self.builder_panel.robot_built.connect(self._on_robot_built)
        builder_layout.addWidget(self.builder_panel)

        root_layout.addWidget(builder_frame, stretch=0)

        # Center: 3D viewport
        self.viz_widget = VisualizationWidget(self)
        root_layout.addWidget(self.viz_widget, stretch=1)

        # Right: tabbed panel -- Robot Info + Joint Control + Forward Kinematics
        right_frame = QFrame(self)
        right_frame.setFrameShape(QFrame.StyledPanel)
        right_frame.setFixedWidth(360)
        right_layout = QVBoxLayout(right_frame)
        right_layout.setContentsMargins(10, 10, 10, 10)

        self.right_tabs = QTabWidget(self)

        self.info_tab = QWidget()
        self.info_layout = QVBoxLayout(self.info_tab)
        self.info_layout.setAlignment(Qt.AlignTop)
        self.right_tabs.addTab(self.info_tab, "Robot Info")

        self.joint_panel = JointControlPanel(self)
        self.joint_panel.robot_changed.connect(self._on_joint_changed)
        self.right_tabs.addTab(self.joint_panel, "Joint Control")

        self.kinematics_panel = KinematicsPanel(self)
        self.right_tabs.addTab(self.kinematics_panel, "Forward Kinematics")

        right_layout.addWidget(self.right_tabs)
        root_layout.addWidget(right_frame, stretch=0)

    def _load_robot(self, robot: RobotModel) -> None:
        """Fully (re)initialize the UI around a new robot: 3D view,
        info panel, joint sliders, FK panel, and status bar.
        """
        self.robot = robot
        self.viz_widget.update_robot(self.robot)
        self.viz_widget.reset_camera()
        self._populate_info_panel()
        self.joint_panel.set_robot(self.robot)
        self.kinematics_panel.update_robot(self.robot)
        self.statusBar().showMessage(
            f"Loaded '{self.robot.name}' \u2014 {self.robot.dof} DOF"
        )

    def _on_robot_built(self, robot: RobotModel) -> None:
        """Slot connected to RobotBuilderPanel.robot_built: swap in the
        newly constructed robot and refresh everything that depends on
        it (3D view, info panel, joint sliders, FK panel, status bar).
        """
        self._load_robot(robot)

    def _on_joint_changed(self) -> None:
        """Slot connected to JointControlPanel.robot_changed: a slider
        moved (or Reset All was pressed). The robot's joint values are
        already updated in-place; just refresh the dependent views.
        """
        self.viz_widget.update_robot(self.robot)
        self.kinematics_panel.update_robot(self.robot)

    def _populate_info_panel(self) -> None:
        """(Re)draw the "Robot Info" tab to reflect self.robot."""
        while self.info_layout.count():
            item = self.info_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        title = QLabel("Robot Info")
        title.setStyleSheet("font-weight: 600; font-size: 14px;")
        self.info_layout.addWidget(title)

        name_label = QLabel(f"Name: {self.robot.name}")
        dof_label = QLabel(f"DOF: {self.robot.dof}")
        name_label.setWordWrap(True)
        self.info_layout.addWidget(name_label)
        self.info_layout.addWidget(dof_label)

        joints_title = QLabel("Joints")
        joints_title.setStyleSheet("font-weight: 600; margin-top: 12px;")
        self.info_layout.addWidget(joints_title)

        for joint in self.robot.joints:
            unit = "rad" if joint.is_revolute() else "m"
            text = (
                f"{joint.name}: {joint.joint_type.value}, "
                f"axis={joint.axis.value.upper()}, "
                f"value={joint.joint_value:.3f} {unit}\n"
                f"   limits=[{joint.minimum_limit:.3f}, "
                f"{joint.maximum_limit:.3f}] {unit}"
            )
            label = QLabel(text)
            label.setWordWrap(True)
            self.info_layout.addWidget(label)

        links_title = QLabel("Links")
        links_title.setStyleSheet("font-weight: 600; margin-top: 12px;")
        self.info_layout.addWidget(links_title)
        for link in self.robot.links:
            self.info_layout.addWidget(QLabel(f"{link.name}: {link.length:.3f} m"))

        note = QLabel(
            "\nUse the 'Joint Control' tab to move the robot "
            "interactively, and 'Forward Kinematics' to see live "
            "end-effector results."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #777777; font-size: 11px;")
        self.info_layout.addWidget(note)
