# Robot Kinematics Studio — Phase 3

Phase 1 (foundation): project structure, PySide6 main window, PyVista
3D viewport, robot/joint/link data model, transformation utilities,
and a hard-coded 2‑DOF planar revolute demo robot rendered in 3D.

Phase 2 added the **Robot Builder**: the user can pick a number of
degrees of freedom, edit a generated joint-configuration table (type,
axis, link length, limits), and construct an arbitrary serial robot
without touching Python source code. (The builder table's columns were
also widened this phase for readability at normal screen widths.)

Phase 3 added the **generic forward-kinematics engine**
(`kinematics/forward_kinematics.py`) — a single, chain-agnostic
implementation used by any robot built via the Robot Builder, not just
the 2-DOF demo — plus a live **Forward Kinematics** results tab showing
end-effector position, orientation (rotation matrix, Roll/Pitch/Yaw,
and quaternion), and the full 4×4 homogeneous transform.

Phase 4 (this phase) adds **interactive joint control**: a
**"Joint Control"** tab with one slider per joint (degrees for
revolute, millimeters for prismatic), each respecting that joint's own
limits. Dragging a slider updates the robot in real time — the 3D
viewport and the Forward Kinematics tab both refresh immediately.

## 1. Project structure

```
robot_kinematics_studio/
├── app/
│   ├── main.py            entry point
│   ├── application.py     QApplication + MainWindow bootstrap
│   └── config.py          app-wide constants
├── gui/
│   ├── main_window.py           Main window: builder + viewport + tabbed
│   │                            info/joint-control/FK panel
│   ├── robot_builder.py         Robot Builder panel (DOF + joint table, Phase 2)
│   ├── joint_panel.py           Interactive joint sliders (Phase 4)
│   ├── kinematics_panel.py      Live FK results panel (Phase 3): position,
│   │                            orientation, full transform
│   ├── visualization_widget.py  Qt wrapper around the PyVista viewport
│   └── (analysis_panel.py, trajectory_panel.py — placeholders for
│         later phases)
├── robot/
│   ├── joint.py            Joint data model (type, axis, limits, value)
│   ├── link.py              Link data model (length, radius)
│   ├── frame.py              Named coordinate frame helper
│   └── robot_model.py    RobotModel serial chain, JointConfig,
│                          RobotModel.from_config(), demo 2-DOF robot
├── kinematics/
│   ├── transformations.py     rotation/translation/homogeneous transform
│   │                          utils + Euler/RPY and quaternion conversions
│   ├── forward_kinematics.py  generic FK engine (Phase 3): FKResult,
│   │                          compute_forward_kinematics(), retains
│   │                          every intermediate joint transform
│   └── (inverse_kinematics.py, jacobian.py — placeholders for Phase 5 / 6)
├── analysis/                placeholders for Phase 6–8
├── visualization/
│   ├── robot_renderer.py    draws links/joints/frames into PyVista
│   └── (frame_renderer.py, workspace_renderer.py — placeholders)
├── io/                       placeholder for Phase 9 (save/load)
├── tests/
│   ├── test_transformations.py   unit tests for rotation/translation math,
│   │                              Euler/RPY and quaternion conversions
│   ├── test_fk.py                validates demo robot vs analytical
│   │                              2-link planar equations (Section 24),
│   │                              generic FK engine tests (Phase 3)
│   ├── test_robot_builder.py     JointConfig validation + from_config (Phase 2)
│   ├── test_joint_control.py     slider <-> joint-value mapping,
│   │                              unit conversion, signals (Phase 4)
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

- A window titled **"Robot Kinematics Studio — Phase 4"** opens (1650×850).
- On launch, the 3D viewport shows the same hard-coded 2‑DOF demo robot
  from Phase 1. Rotate/pan/zoom with the mouse.
- The **left panel ("Robot Builder")** works as in Phase 2: pick DOF,
  generate/edit the joint table, click "Build Robot" to construct a
  new robot. Building a robot resets the Joint Control sliders and the
  Forward Kinematics tab to match the new robot.
- The **right panel** is tabbed:
  - **"Robot Info"** — static description of the loaded robot's name,
    DOF, joints, and links (as in Phase 2/3). A note points you to the
    Joint Control tab for live values.
  - **"Joint Control"** (new) — one slider per joint. Revolute joints
    show degrees, prismatic joints show millimeters, and every slider
    is scaled to that joint's own min/max limits, so dragging fully
    left or right always lands exactly on the joint's limit. A live
    numeric readout sits above each slider. Dragging any slider
    immediately updates the 3D viewport and the Forward Kinematics
    tab. A **"Reset All Joints"** button zeros every joint at once
    (clamped to limits where 0 is out of range).
  - **"Forward Kinematics"** — end-effector position, orientation, and
    full transform (as in Phase 3), now updating live as you drag
    joint sliders, not just when a new robot is built.

The demo robot loaded at startup is intentionally simple: both joints
rotate about Z, links extend along local X, joint 1 = 0.3 rad, joint 2
= 0.5 rad, link lengths 0.4 m and 0.3 m.

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
  later), `translation`, `homogeneous_transform`, `is_valid_transform`
  for defensive checks, plus (Phase 3) `rotation_matrix_to_euler_xyz`
  (Roll/Pitch/Yaw, with a defined gimbal-lock behavior) and
  `rotation_matrix_to_quaternion` (Shepperd's method, numerically
  stable across all rotation magnitudes).
- **`kinematics/forward_kinematics.py`** (Phase 3) — the generic,
  chain-agnostic FK engine. `compute_forward_kinematics(joints, links,
  base_transform)` returns an `FKResult` holding the base transform,
  every intermediate joint transform (`T_0_1 ... T_0_n`, retained for
  the Jacobian in Phase 6), and the end-effector transform. Works on
  plain `Joint`/`Link` lists rather than `RobotModel`, keeping the math
  layer independent of the data model that owns it.
- **`visualization/robot_renderer.py`** — Pure rendering: reads a
  `RobotModel`'s computed frames and draws cylinders/spheres/arrows
  into a PyVista plotter. No robotics math lives here.
- **`gui/visualization_widget.py`** — Qt widget embedding
  `pyvistaqt.QtInteractor`; exposes `update_robot(robot)` so the rest
  of the GUI never touches PyVista directly.
- **`robot/robot_model.py`** — `JointConfig` (Phase 2): the
  GUI-unit-agnostic row object the builder constructs, with its own
  `validate()`; `RobotModel.validate_configs(...)`: whole-list checks
  (empty list, duplicate names) on top of each row's own validation;
  `RobotModel.from_config(...)`: validates then constructs a
  `RobotModel`, raising `RobotModelError` with every problem listed if
  validation fails — never a partially-built or silently-wrong robot.
  (Phase 3) `compute_fk()` delegates to the generic FK engine and
  `compute_frames()` is now a thin Frame-naming wrapper around it, so
  there is exactly one FK implementation in the whole project.
- **`gui/robot_builder.py`** — `RobotBuilderPanel`: DOF spin box, joint
  table (`QTableWidget` with per-cell combo/spin widgets), and a
  `robot_built` Qt signal emitted with a valid `RobotModel`. Handles
  the degrees↔radians conversion at this GUI boundary only; everything
  it hands to `RobotModel` is already validated SI data.
- **`gui/kinematics_panel.py`** (Phase 3) — `KinematicsPanel`:
  read-only display of `robot.compute_fk()` results — X/Y/Z position,
  the 3×3 rotation matrix, Roll/Pitch/Yaw, quaternion, and the full
  4×4 transform, all in a monospace font for readability. No robotics
  math lives here; it only reads `FKResult` and the transformation
  converters.
- **`gui/joint_panel.py`** (Phase 4) — `JointControlPanel`: builds one
  `_JointSliderRow` per joint. Each `QSlider` (integer, `SLIDER_STEPS`
  resolution) maps onto that joint's own `[minimum_limit,
  maximum_limit]`, converting to/from degrees (revolute) or
  millimeters (prismatic) at this GUI boundary only. Moving a slider
  calls `joint.set_value(...)` directly and emits `robot_changed`
  (no payload — listeners just re-read the robot) so the 3D view and
  FK panel can refresh. Rows are rebuilt only when DOF count changes;
  otherwise existing rows are reconfigured in place to avoid
  unnecessary widget churn while dragging.
- **`gui/main_window.py`** — Assembles the Robot Builder panel + 3D
  viewport + a tabbed right panel ("Robot Info" / "Joint Control" /
  "Forward Kinematics"). `_on_robot_built` reloads everything for a
  newly constructed robot; `_on_joint_changed` (from the slider panel)
  refreshes only the viewport and FK tab, since the robot's joint
  values are already updated in place. No robotics math or rendering
  code lives here — it only calls into `RobotModel`,
  `RobotBuilderPanel`, `JointControlPanel`, `KinematicsPanel`, and
  `VisualizationWidget`.
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
- **`test_robot_builder.py`** (Phase 2) — valid configs produce no
  errors; `from_config` produces the correct DOF/name/link count;
  mixed revolute+prismatic chains work; empty config list, negative
  link length, min>max, empty name, duplicate names, and
  wildly-out-of-range revolute limits are all correctly flagged;
  `from_config` raises `RobotModelError` on invalid input; an
  out-of-range initial joint value is clamped rather than left
  invalid; an 8-DOF robot builds and produces the right frame count.
- **`test_fk.py`** (Phase 3 additions) — `compute_fk()` retains all
  intermediate transforms and every transform is a valid homogeneous
  matrix; the generic engine matches the analytical 2-link planar
  equations across several configurations; `compute_frames()` agrees
  exactly with `compute_fk()`; mismatched joint/link lengths raise
  `ValueError`; a single prismatic joint moves correctly along its
  axis; a 3-DOF spatial (non-planar) robot produces a valid transform;
  Euler-XYZ round-trips through known angles; a quaternion from the
  identity matrix is `[1,0,0,0]`; quaternions from several rotations
  are unit length; quaternions correctly reconstruct their source
  rotation matrix.
- **`test_joint_control.py`** (Phase 4) — one slider row per joint;
  initial slider position matches the robot's actual starting joint
  value (not just defaulting to 0 or midpoint); dragging fully
  left/right lands exactly on the joint's min/max limit; moving a
  slider emits `robot_changed` exactly once; "Reset All Joints" zeros
  every joint; a prismatic joint's slider correctly scales in
  millimeters; switching to a different-DOF robot rebuilds the right
  number of rows; switching robots doesn't cross-talk values between
  the old and new robot's joints.
- **`test_jacobian.py`**, **`test_ik.py`** — skipped placeholders,
  filled in during Phase 5/6.

All 43 active tests currently pass (2 skipped, by design).

## 7. Known limitations (Phase 4, by design)

- The "Robot Info" tab shows a static snapshot taken when the robot is
  loaded/built — it does not live-update as you drag joint sliders
  (the "Joint Control" and "Forward Kinematics" tabs do). This avoids
  rebuilding that tab's widgets on every slider tick.
- No Denavit-Hartenberg convenience layer yet (Section 11) — deferred,
  as in Phase 3.
- Only X/Y/Z joint axes are exposed in the builder UI, though the
  underlying math (`rotation_about_axis`) already supports arbitrary
  axes.
- No IK, Jacobian, singularity, workspace, or trajectory features.
- No save/load — `RobotModel.from_config` takes `JointConfig` objects
  built by the GUI; JSON import/export of that same structure is
  Phase 9.
- Slider resolution is fixed at 1000 discrete steps per joint
  (`SLIDER_STEPS` in `gui/joint_panel.py`); this is smooth enough for
  interactive use but is not infinite precision.
- Euler-XYZ conversion picks one valid solution (roll = 0) at the
  gimbal-lock singularity (pitch = ±90°) rather than reporting both
  degenerate solutions.

## 8. Next step: Phase 5 — Numerical Inverse Kinematics

Planned work:
- Implement `kinematics/inverse_kinematics.py`: Jacobian-based
  numerical IK, starting with the pseudoinverse method, then damped
  least squares.
- Support position-only and position+orientation IK, with configurable
  max iterations, convergence tolerance, step size, damping, and joint
  limits.
- Add a target-pose input and an IK results display (converged/failed,
  iteration count, position/orientation error) as a new panel.
- Validate against a known-solvable 2-DOF planar case (reachable
  target) and a known-unreachable case (must correctly report
  failure, not a silently wrong pose), per Section 12's requirements.
