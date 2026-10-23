"""
workspace_panel.py

Phase 7: the Workspace Analysis panel (Section 15). Lets the user set
a sample count and generate a workspace point cloud for the current
robot, displayed as an overlay in the 3D viewport. Reports sample
count, estimated maximum reach, and (when computable) estimated
workspace volume -- all explicitly labeled as estimates, never
presented as exact (Section 15's requirement).

Sampling runs on a background QThread (WorkspaceSamplingWorker) so
large sample counts don't freeze the GUI (Section 25) -- this is the
project's first genuinely long-running computation and its first use
of Qt threading.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QLabel,
    QSpinBox,
    QPushButton,
    QGroupBox,
    QProgressBar,
    QCheckBox,
)

from robot.robot_model import RobotModel
from analysis.workspace import sample_workspace, WorkspaceResult

MIN_SAMPLES = 100
MAX_SAMPLES = 100000
DEFAULT_SAMPLES = 5000


class WorkspaceSamplingWorker(QThread):
    """Runs analysis.workspace.sample_workspace() on a background
    thread so the GUI stays responsive during large sampling runs.

    Signals:
        finished_ok: emitted with the WorkspaceResult on success.
        failed: emitted with an error message string on failure.
    """

    finished_ok = Signal(object)
    failed = Signal(str)

    def __init__(
        self,
        joints,
        links,
        num_samples: int,
        base_transform: Optional[np.ndarray],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._joints = joints
        self._links = links
        self._num_samples = num_samples
        self._base_transform = base_transform

    def run(self) -> None:
        try:
            result = sample_workspace(
                self._joints,
                self._links,
                self._num_samples,
                base_transform=self._base_transform,
            )
            self.finished_ok.emit(result)
        except Exception as exc:  # noqa: BLE001 - surface any failure to the GUI
            self.failed.emit(str(exc))


class WorkspacePanel(QWidget):
    """Panel for configuring, running, and displaying workspace analysis.

    Signals:
        workspace_ready: emitted with a (N, 3) numpy array of sampled
            positions when a run completes successfully, so
            MainWindow can push it into the 3D viewport.
        workspace_cleared: emitted (no payload) when the user hides/
            clears the workspace overlay.
    """

    workspace_ready = Signal(object)
    workspace_cleared = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.robot: Optional[RobotModel] = None
        self._worker: Optional[WorkspaceSamplingWorker] = None
        self._last_result: Optional[WorkspaceResult] = None
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title = QLabel("Workspace Analysis")
        title.setStyleSheet("font-weight: 600; font-size: 14px;")
        layout.addWidget(title)

        # --- Sampling controls -------------------------------------------
        controls_box = QGroupBox("Sampling")
        form = QFormLayout(controls_box)

        self.samples_spin = QSpinBox()
        self.samples_spin.setRange(MIN_SAMPLES, MAX_SAMPLES)
        self.samples_spin.setSingleStep(500)
        self.samples_spin.setValue(DEFAULT_SAMPLES)
        form.addRow("Samples:", self.samples_spin)

        layout.addWidget(controls_box)

        button_row = QHBoxLayout()
        self.generate_btn = QPushButton("Generate Workspace")
        self.generate_btn.setStyleSheet("font-weight: 600;")
        self.generate_btn.clicked.connect(self._on_generate_clicked)
        button_row.addWidget(self.generate_btn)

        self.show_check = QCheckBox("Show in 3D view")
        self.show_check.setChecked(True)
        self.show_check.toggled.connect(self._on_show_toggled)
        button_row.addWidget(self.show_check)
        layout.addLayout(button_row)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 0)  # indeterminate (sampling isn't
        # incrementally reported; the bar just indicates "working")
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)

        # --- Results --------------------------------------------------------
        results_box = QGroupBox("Results")
        results_layout = QVBoxLayout(results_box)
        self.samples_label = QLabel("Samples: --")
        self.reach_label = QLabel("Estimated Maximum Reach: --")
        self.volume_label = QLabel("Estimated Workspace Volume: --")
        for label in (self.samples_label, self.reach_label, self.volume_label):
            label.setWordWrap(True)
            results_layout.addWidget(label)
        layout.addWidget(results_box)

        note = QLabel(
            "All figures above are estimated from random joint-space "
            "sampling, not computed exactly -- accuracy improves with "
            "more samples but is never exact."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #777777; font-size: 10px;")
        layout.addWidget(note)

        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        layout.addStretch(1)

    def set_robot(self, robot: RobotModel) -> None:
        """Point this panel at a (possibly new) robot. Clears any
        previously displayed workspace, since it belonged to the old
        robot."""
        self.robot = robot
        self._last_result = None
        self.samples_label.setText("Samples: --")
        self.reach_label.setText("Estimated Maximum Reach: --")
        self.volume_label.setText("Estimated Workspace Volume: --")
        self.status_label.setText("")
        self.workspace_cleared.emit()

    def _on_generate_clicked(self) -> None:
        if self.robot is None:
            self.status_label.setText("No robot loaded.")
            return
        if self._worker is not None and self._worker.isRunning():
            return  # a run is already in progress

        num_samples = self.samples_spin.value()

        self.generate_btn.setEnabled(False)
        self.generate_btn.setText("Generating...")
        self.progress_bar.setVisible(True)
        self.status_label.setText(f"Sampling {num_samples:,} configurations...")
        self.status_label.setStyleSheet("color: #777777;")

        self._worker = WorkspaceSamplingWorker(
            self.robot.joints,
            self.robot.links,
            num_samples,
            self.robot.base_frame.transform,
            self,
        )
        self._worker.finished_ok.connect(self._on_sampling_finished)
        self._worker.failed.connect(self._on_sampling_failed)
        self._worker.start()

    def _on_sampling_finished(self, result: WorkspaceResult) -> None:
        self._last_result = result
        self.generate_btn.setEnabled(True)
        self.generate_btn.setText("Generate Workspace")
        self.progress_bar.setVisible(False)

        self.samples_label.setText(f"Samples: {result.num_samples:,} (estimated)")
        self.reach_label.setText(
            f"Estimated Maximum Reach: {result.estimated_max_reach:.4f} m"
        )
        if result.estimated_volume is not None:
            self.volume_label.setText(
                f"Estimated Workspace Volume: {result.estimated_volume:.6f} m\u00b3"
            )
        else:
            self.volume_label.setText(
                "Estimated Workspace Volume: not available "
                "(degenerate/planar point cloud)"
            )

        self.status_label.setText("Workspace generated.")
        self.status_label.setStyleSheet("color: #2E7D32;")

        if self.show_check.isChecked():
            self.workspace_ready.emit(result.positions)

    def _on_sampling_failed(self, message: str) -> None:
        self.generate_btn.setEnabled(True)
        self.generate_btn.setText("Generate Workspace")
        self.progress_bar.setVisible(False)
        self.status_label.setText(f"Workspace generation failed: {message}")
        self.status_label.setStyleSheet("color: #C0392B;")

    def _on_show_toggled(self, checked: bool) -> None:
        if checked and self._last_result is not None:
            self.workspace_ready.emit(self._last_result.positions)
        else:
            self.workspace_cleared.emit()
