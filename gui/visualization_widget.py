"""
visualization_widget.py

Qt widget embedding a PyVista 3D viewport. This is the ONLY place in
Phase 1 where PyVista's Qt integration is set up. It exposes a simple
`update_robot(robot)` method so the rest of the GUI never has to touch
PyVista directly.
"""

from __future__ import annotations

from PySide6.QtWidgets import QVBoxLayout, QWidget
from pyvistaqt import QtInteractor

from robot.robot_model import RobotModel
from visualization.robot_renderer import RobotRenderer


class VisualizationWidget(QWidget):
    """A Qt widget hosting an interactive PyVista 3D viewport."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor)

        self.plotter.set_background("#F5F5F5")
        self.renderer = RobotRenderer(self.plotter)
        self.renderer.draw_world_axes(length=0.3)

        self.plotter.camera_position = "iso"
        self.plotter.add_axes()

    def update_robot(self, robot: RobotModel) -> None:
        """Re-render the given robot at its current joint configuration.

        Args:
            robot: The RobotModel to display.
        """
        self.renderer.clear()
        self.renderer.draw_world_axes(length=0.3)
        self.renderer.draw_robot(robot, show_frames=True)
        self.plotter.render()

    def reset_camera(self) -> None:
        """Reset the camera to a standard isometric view."""
        self.plotter.camera_position = "iso"
        self.plotter.reset_camera()
