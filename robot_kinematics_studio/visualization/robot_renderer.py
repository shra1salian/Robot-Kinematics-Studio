"""
robot_renderer.py

Translates a RobotModel's current state into PyVista geometry: links
as cylinders, joints as spheres, and coordinate frame triads as small
colored arrows. Contains NO robotics math and NO GUI event handling --
it only reads Frame/Link data that has already been computed by the
robot model.
"""

from __future__ import annotations

from typing import List

import numpy as np
import pyvista as pv

from robot.robot_model import RobotModel
from robot.frame import Frame

# Consistent color scheme for a clean, professional look.
COLOR_LINK = "#5B7FA6"
COLOR_JOINT = "#D9822B"
COLOR_BASE = "#444444"
COLOR_END_EFFECTOR = "#C0392B"
COLOR_AXIS_X = "#E74C3C"
COLOR_AXIS_Y = "#2ECC71"
COLOR_AXIS_Z = "#3498DB"


class RobotRenderer:
    """Renders a RobotModel into a PyVista plotter."""

    def __init__(self, plotter: pv.Plotter) -> None:
        """
        Args:
            plotter: The PyVista plotter (or QtInteractor) to draw into.
        """
        self.plotter = plotter
        self._actors: List = []

    def clear(self) -> None:
        """Remove all previously drawn robot actors from the plotter."""
        for actor in self._actors:
            try:
                self.plotter.remove_actor(actor, render=False)
            except Exception:
                pass
        self._actors.clear()

    def draw_world_axes(self, length: float = 0.3) -> None:
        """Draw world-frame XYZ reference axes and a ground grid."""
        origin = np.zeros(3)
        self._draw_arrow(origin, np.array([1, 0, 0]), length, COLOR_AXIS_X)
        self._draw_arrow(origin, np.array([0, 1, 0]), length, COLOR_AXIS_Y)
        self._draw_arrow(origin, np.array([0, 0, 1]), length, COLOR_AXIS_Z)

        grid = pv.Plane(
            center=(0, 0, 0), direction=(0, 0, 1), i_size=1.5, j_size=1.5,
            i_resolution=15, j_resolution=15,
        )
        actor = self.plotter.add_mesh(
            grid, style="wireframe", color="#CCCCCC", line_width=1, opacity=0.5
        )
        self._actors.append(actor)

    def draw_robot(self, robot: RobotModel, show_frames: bool = True) -> None:
        """Draw the full robot (base, links, joints, end-effector, frames)
        based on its current joint configuration.

        Args:
            robot: The RobotModel to render.
            show_frames: Whether to draw small coordinate triads at
                each joint frame.
        """
        frames = robot.compute_frames()

        # Base marker
        base_actor = self.plotter.add_mesh(
            pv.Cube(center=frames[0].position(), x_length=0.06, y_length=0.06,
                    z_length=0.02),
            color=COLOR_BASE,
        )
        self._actors.append(base_actor)

        # Links: connect consecutive joint frames (skip base->J1 gap logic:
        # frames layout is [Base, J1, J2, ..., Jn, EndEffector])
        joint_frames = frames[1:-1]
        end_effector_frame = frames[-1]
        chain = [frames[0]] + joint_frames + [end_effector_frame]

        for i in range(len(chain) - 1):
            start = chain[i].position()
            end = chain[i + 1].position()
            self._draw_link_cylinder(start, end)

        # Joint spheres
        for jf in joint_frames:
            joint_actor = self.plotter.add_mesh(
                pv.Sphere(radius=0.035, center=jf.position()),
                color=COLOR_JOINT,
            )
            self._actors.append(joint_actor)

        # End-effector marker
        ee_actor = self.plotter.add_mesh(
            pv.Sphere(radius=0.03, center=end_effector_frame.position()),
            color=COLOR_END_EFFECTOR,
        )
        self._actors.append(ee_actor)

        if show_frames:
            for frame in chain:
                self._draw_frame_triad(frame, length=0.08)

    def _draw_link_cylinder(self, start: np.ndarray, end: np.ndarray) -> None:
        """Draw a cylinder representing a link between two points."""
        vec = end - start
        length = np.linalg.norm(vec)
        if length < 1e-6:
            return
        center = (start + end) / 2.0
        direction = vec / length
        cylinder = pv.Cylinder(
            center=center, direction=direction, radius=0.015, height=length,
        )
        actor = self.plotter.add_mesh(cylinder, color=COLOR_LINK, smooth_shading=True)
        self._actors.append(actor)

    def _draw_arrow(
        self, origin: np.ndarray, direction: np.ndarray, length: float, color: str
    ) -> None:
        arrow = pv.Arrow(
            start=origin, direction=direction, scale=length,
            tip_length=0.25, tip_radius=0.05, shaft_radius=0.02,
        )
        actor = self.plotter.add_mesh(arrow, color=color)
        self._actors.append(actor)

    def _draw_frame_triad(self, frame: Frame, length: float = 0.08) -> None:
        """Draw a small RGB coordinate triad (X=red, Y=green, Z=blue) at
        the given frame's pose."""
        origin = frame.position()
        self._draw_arrow(origin, frame.x_axis(), length, COLOR_AXIS_X)
        self._draw_arrow(origin, frame.y_axis(), length, COLOR_AXIS_Y)
        self._draw_arrow(origin, frame.z_axis(), length, COLOR_AXIS_Z)
