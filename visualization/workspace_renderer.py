"""
workspace_renderer.py

Renders a workspace point cloud (analysis.workspace.WorkspaceResult)
into a PyVista plotter as small points. Contains NO sampling math --
it only turns already-computed positions into geometry, matching
robot_renderer.py's separation of concerns.
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np
import pyvista as pv

WORKSPACE_POINT_COLOR = "#8E44AD"  # purple, visually distinct from the robot


class WorkspaceRenderer:
    """Renders a workspace point cloud into a PyVista plotter."""

    def __init__(self, plotter: pv.Plotter) -> None:
        self.plotter = plotter
        self._actors: List = []

    def clear(self) -> None:
        """Remove any previously drawn workspace points."""
        for actor in self._actors:
            try:
                self.plotter.remove_actor(actor, render=False)
            except Exception:
                pass
        self._actors.clear()

    def draw_points(self, positions: np.ndarray, point_size: float = 4.0) -> None:
        """Draw a workspace point cloud.

        Args:
            positions: (N, 3) array of end-effector positions.
            point_size: Visual point size in pixels.
        """
        if positions.shape[0] == 0:
            return
        cloud = pv.PolyData(positions)
        actor = self.plotter.add_mesh(
            cloud,
            color=WORKSPACE_POINT_COLOR,
            render_points_as_spheres=True,
            point_size=point_size,
            opacity=0.35,
        )
        self._actors.append(actor)
