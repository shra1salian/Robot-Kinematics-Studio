# Robot Kinematics Studio — Phase 1

Foundation phase: project structure, PySide6 main window, PyVista 3D
viewport, robot/joint/link data model, transformation utilities, and a
hard-coded 2‑DOF planar revolute demo robot rendered in 3D.

## 1. Project structure

```
robot_kinematics_studio/
├── app/
│   ├── main.py            entry point
│   ├── application.py     QApplication + MainWindow bootstrap
│   └── config.py          app-wide constants
├── gui/
│   ├── main_window.py           Phase 1 main window
│   ├── visualization_widget.py  Qt wrapper around the PyVista viewport
│   └── (robot_builder.py, joint_panel.py, kinematics_panel.py,
│         analysis_panel.py, trajectory_panel.py — placeholders for
│         later phases)
├── robot/
│   ├── joint.py            Joint data model (type, axis, limits, value)
│   ├── link.py              Link data model (length, radius)
│   ├── frame.py              Named coordinate frame helper
│   └── robot_model.py    RobotModel serial chain + demo 2-DOF robot
├── kinematics/
│   ├── transformations.py   rotation/translation/homogeneous transform utils
│   └── (forward_kinematics.py, inverse_kinematics.py, jacobian.py —
│         placeholders for Phase 3 / 5 / 6)
├── analysis/                placeholders for Phase 6–8
├── visualization/
│   ├── robot_renderer.py    draws links/joints/frames into PyVista
│   └── (frame_renderer.py, workspace_renderer.py — placeholders)
├── io/                       placeholder for Phase 9 (save/load)
├── tests/
│   ├── test_transformations.py   unit tests for rotation/translation math
│   ├── test_fk.py                validates demo robot vs analytical
│   │                              2-link planar equations (Section 24)
│   ├── test_jacobian.py          skipped placeholder (Phase 6)
│   └── test_ik.py                skipped placeholder (Phase 5)
├── requirements.txt
└── README.md
```

## 2. Installation

Requires Python 3.11+.

```bash
cd robot_kinematics_studio
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## 3. Running the application

From the project root:

```bash
python -m app.main
```

## 4. Expected behavior

- A window titled **"Robot Kinematics Studio — Phase 1"** opens (1200×800).
- The left panel shows static info about the loaded robot (name, DOF,
  each joint's type/axis/value). No sliders yet — that's Phase 4.
- The center 3D viewport shows:
  - World XYZ reference axes (red/green/blue arrows) and a faint ground grid.
  - The 2‑DOF demo robot: a dark base cube, blue link cylinders, orange
    joint spheres, a red end-effector marker, and a small RGB coordinate
    triad at the base, each joint, and the end-effector.
  - You can rotate/pan/zoom with the mouse (standard PyVista/VTK controls).
- The status bar shows `Loaded 'Demo 2DOF Planar Robot' — 2 DOF`.

The demo robot is intentionally simple: both joints rotate about Z,
links extend along local X, joint 1 = 0.3 rad, joint 2 = 0.5 rad,
link lengths 0.4 m and 0.3 m.

## 5. Module overview

- **`robot/joint.py`** — `Joint` dataclass + `JointType`/`JointAxis`
  enums. Values are clamped to `[minimum_limit, maximum_limit]` on
  every `set_value()` call, so invalid configurations can't silently
  propagate.
- **`robot/link.py`** — `Link` dataclass; validates non-negative length.
- **`robot/frame.py`** — `Frame`: a name + 4×4 pose, with convenience
  accessors for position and axis vectors (used by the renderer).
- **`robot/robot_model.py`** — `RobotModel`: the serial chain of
  joints/links. `compute_frames()` walks the chain and returns the
  world pose of the base, every joint, and the end-effector. This is a
  Phase‑1 convenience implementation; it uses the same primitives
  (`kinematics/transformations.py`) that the generic FK engine will
  use in Phase 3, so upgrading later won't change this class's public
  interface. Also includes `create_demo_2dof_robot()`.
- **`kinematics/transformations.py`** — `rotation_x/y/z`,
  `rotation_about_axis` (Rodrigues' formula, enables arbitrary axes
  later), `translation`, `homogeneous_transform`, plus
  `is_valid_transform` for defensive checks.
- **`visualization/robot_renderer.py`** — Pure rendering: reads a
  `RobotModel`'s computed frames and draws cylinders/spheres/arrows
  into a PyVista plotter. No robotics math lives here.
- **`gui/visualization_widget.py`** — Qt widget embedding
  `pyvistaqt.QtInteractor`; exposes `update_robot(robot)` so the rest
  of the GUI never touches PyVista directly.
- **`gui/main_window.py`** — Assembles the info panel + 3D viewport.
  No robotics math or rendering code lives here either — it only
  calls into `RobotModel` and `VisualizationWidget`.
- **`app/application.py` / `app/main.py`** — Application bootstrap and
  entry point.

## 6. Tests

```bash
python -m pytest tests/ -v
```

Currently implemented:

- **`test_transformations.py`** — rotation matrices are orthonormal
  with det = +1; `rotation_x/y/z` map known vectors correctly;
  `rotation_about_axis` matches the principal-axis functions when
  given a principal axis; rejects a zero-length axis; translation and
  homogeneous-transform composition are correct; `is_valid_transform`
  rejects malformed matrices.
- **`test_fk.py`** — the demo robot's end-effector position (via
  `RobotModel.compute_frames()`) matches the analytical 2‑link planar
  manipulator equations
  `x = L1*cos(q1) + L2*cos(q1+q2)`, `y = L1*sin(q1) + L2*sin(q1+q2)`
  across several joint configurations; joint-limit clamping is
  respected; DOF/joint-count consistency.
- **`test_jacobian.py`**, **`test_ik.py`** — skipped placeholders,
  filled in during Phase 5/6.

All 13 active tests currently pass (2 skipped, by design).

## 7. Known limitations (Phase 1, by design)

- No joint sliders / interactive control (Phase 4).
- No generic FK engine — `RobotModel.compute_frames()` is a
  Phase‑1‑only convenience for this simple robot; a chain-agnostic
  engine arrives in Phase 3.
- No Robot Builder UI — DOF/joint config is hard-coded (Phase 2).
- No IK, Jacobian, singularity, workspace, or trajectory features.
- No save/load.
- Only X/Y/Z joint axes are exposed at the API level, though the math
  (`rotation_about_axis`) already supports arbitrary axes.

## 8. Next step: Phase 2 — Generic Robot Builder

Planned work:
- DOF selection UI (spin box + "Build" action).
- Dynamically generated joint configuration table (type, axis, link
  length, min/max limits) with validation that rejects impossible
  configurations before they reach the model.
- Wire the table into a new `RobotModel` construction path so users
  can build arbitrary serial robots without editing Python.
- Extend `robot/robot_model.py` with a `from_config(...)` constructor
  and basic serialization-friendly structure (in preparation for
  Phase 9 export).
