"""
test_joint_control.py

Tests for gui/joint_panel.py's slider <-> joint-value mapping logic
(Phase 4). These run headlessly (QT_QPA_PLATFORM=offscreen is set by
the test runner environment where needed) and exercise the panel
through its public Qt widgets rather than mocking Qt away, since the
mapping math is deliberately kept inside the widget class.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest

pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication

from gui.joint_panel import JointControlPanel, SLIDER_STEPS
from robot.robot_model import create_demo_2dof_robot, RobotModel, JointConfig
from robot.joint import JointType, JointAxis


@pytest.fixture(scope="module", autouse=True)
def qapp():
    app = QApplication.instance() or QApplication(sys.argv)
    yield app


def test_set_robot_creates_one_row_per_joint():
    panel = JointControlPanel()
    robot = create_demo_2dof_robot()
    panel.set_robot(robot)
    assert len(panel._rows) == robot.dof


def test_initial_slider_position_reflects_joint_value():
    panel = JointControlPanel()
    robot = create_demo_2dof_robot()
    panel.set_robot(robot)
    # J1 = 0.3 rad within [-pi, pi]; slider position should be roughly
    # proportional, not just defaulted to 0 or the midpoint.
    expected_frac = (0.3 - (-np.pi)) / (2 * np.pi)
    expected_raw = round(expected_frac * SLIDER_STEPS)
    assert abs(panel._rows[0].slider.value() - expected_raw) <= 1


def test_dragging_slider_updates_robot_joint_value():
    panel = JointControlPanel()
    robot = create_demo_2dof_robot()
    panel.set_robot(robot)

    panel._rows[0].slider.setValue(SLIDER_STEPS)  # full right = max limit
    assert robot.joints[0].joint_value == pytest.approx(np.pi, abs=1e-6)

    panel._rows[0].slider.setValue(0)  # full left = min limit
    assert robot.joints[0].joint_value == pytest.approx(-np.pi, abs=1e-6)


def test_slider_emits_robot_changed_signal():
    panel = JointControlPanel()
    robot = create_demo_2dof_robot()
    panel.set_robot(robot)

    received = []
    panel.robot_changed.connect(lambda: received.append(True))
    panel._rows[0].slider.setValue(200)
    assert len(received) == 1


def test_reset_all_zeros_every_joint():
    panel = JointControlPanel()
    robot = create_demo_2dof_robot()
    panel.set_robot(robot)

    panel._rows[0].slider.setValue(900)
    panel._rows[1].slider.setValue(100)
    assert robot.joints[0].joint_value != 0.0

    panel._on_reset_clicked()
    for joint in robot.joints:
        assert joint.joint_value == pytest.approx(0.0, abs=1e-9)


def test_prismatic_slider_respects_meters_range():
    configs = [JointConfig("P1", JointType.PRISMATIC, JointAxis.X, 0.0, 0.0, 0.5)]
    robot = RobotModel.from_config("Prismatic Test", configs)
    panel = JointControlPanel()
    panel.set_robot(robot)

    panel._rows[0].slider.setValue(SLIDER_STEPS)
    assert robot.joints[0].joint_value == pytest.approx(0.5, abs=1e-6)
    assert "mm" in panel._rows[0].value_label.text()


def test_set_robot_rebuilds_rows_for_different_dof():
    panel = JointControlPanel()
    robot2 = create_demo_2dof_robot()
    panel.set_robot(robot2)
    assert len(panel._rows) == 2

    configs = [
        JointConfig(f"J{i+1}", JointType.REVOLUTE, JointAxis.Z, 0.2, -np.pi, np.pi)
        for i in range(5)
    ]
    robot5 = RobotModel.from_config("5DOF", configs)
    panel.set_robot(robot5)
    assert len(panel._rows) == 5


def test_switching_robots_does_not_crosstalk_values():
    """Moving a slider on one robot must not affect a previously loaded
    robot's joint values (regression check for stale references)."""
    panel = JointControlPanel()
    robot_a = create_demo_2dof_robot()
    panel.set_robot(robot_a)
    panel._rows[0].slider.setValue(1000)
    a_value = robot_a.joints[0].joint_value

    robot_b = create_demo_2dof_robot()
    panel.set_robot(robot_b)
    panel._rows[0].slider.setValue(0)

    assert robot_a.joints[0].joint_value == a_value
    assert robot_b.joints[0].joint_value != a_value
