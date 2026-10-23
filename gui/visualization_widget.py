"""
visualization_widget.py

Qt widget embedding a PyVista 3D viewport. This is the ONLY place
PyVista's Qt integration is set up. It exposes simple methods so the
rest of the GUI never has to touch PyVista directly:
`update_robot(robot)` for the robot itself, and (Phase 7)
`show_workspace(positions)` / `clear_workspace()` for an optional
workspace point-cloud overlay.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from PySide6.QtWidgets import QVBoxLayout, QWidget
from pyvistaqt import QtInteractor

from robot.robot_model import RobotModel
from visualization.robot_renderer import RobotRenderer
from visualization.workspace_renderer import WorkspaceRenderer


class VisualizationWidget(QWidget):
    """A Qt widget hosting an interactive PyVista 3D viewport."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor)

        self.plotter.set_background("#F5F5F5")
        self.renderer = RobotRenderer(self.plotter)
        self.workspace_renderer = WorkspaceRenderer(self.plotter)
        self.renderer.draw_world_axes(length=0.3)

        self.plotter.camera_position = "iso"
        self.plotter.add_axes()

    def update_robot(self, robot: RobotModel) -> None:
        """Re-render the given robot at its current joint configuration.
        The workspace overlay (if any) is preserved across this call.

        Args:
            robot: The RobotModel to display.
        """
        self.renderer.clear()
        self.renderer.draw_world_axes(length=0.3)
        self.renderer.draw_robot(robot, show_frames=True)
        self.plotter.render()

    def show_workspace(self, positions: np.ndarray) -> None:
        """Display a workspace point cloud, replacing any previous one.

        Args:
            positions: (N, 3) array of end-effector positions.
        """
        self.workspace_renderer.clear()
        self.workspace_renderer.draw_points(positions)
        self.plotter.render()

    def clear_workspace(self) -> None:
        """Remove the workspace point cloud overlay, if any."""
        self.workspace_renderer.clear()
        self.plotter.render()

    def reset_camera(self) -> None:
        """Reset the camera to a standard isometric view."""
        self.plotter.camera_position = "iso"
        self.plotter.reset_camera()
