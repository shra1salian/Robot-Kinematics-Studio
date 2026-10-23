"""
application.py

Thin wrapper around QApplication + MainWindow so `main.py` stays a
one-liner and application-level setup (styling, config) has a single
home.
"""

from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from gui.main_window import MainWindow


class RobotKinematicsStudioApp:
    """Owns the QApplication instance and the main window."""

    def __init__(self, argv: list[str]) -> None:
        self.qt_app = QApplication(argv)
        self.qt_app.setApplicationName("Robot Kinematics Studio")
        self.main_window = MainWindow()

    def run(self) -> int:
        """Show the main window and start the Qt event loop."""
        self.main_window.show()
        return self.qt_app.exec()


def main() -> int:
    app = RobotKinematicsStudioApp(sys.argv)
    return app.run()
