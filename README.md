# Robot Kinematics Studio — Phase 3

Phase 1 (foundation): project structure, PySide6 main window, PyVista
3D viewport, robot/joint/link data model, transformation utilities,
and a hard-coded 2‑DOF planar revolute demo robot rendered in 3D.

Phase 2 added the **Robot Builder**: the user can pick a number of
degrees of freedom, edit a generated joint-configuration table (type,
axis, link length, limits), and construct an arbitrary serial robot
without touching Python source code. (The builder table's columns were
also widened this phase for readability at normal screen widths.)

Phase 3 (this phase) adds the **generic forward-kinematics engine**
(`kinematics/forward_kinematics.py`) — a single, chain-agnostic
implementation used by any robot built via the Robot Builder, not just
the 2-DOF demo — plus a live **Forward Kinematics** results tab showing
end-effector position, orientation (rotation matrix, Roll/Pitch/Yaw,
and quaternion), and the full 4×4 homogeneous transform.

## 1. Project structure

```
robot_kinematics_studio/
├── app/
│   ├── main.py            entry point
│   ├── application.py     QApplication + MainWindow bootstrap
│   └── config.py          app-wide constants
├── gui/
│   ├── main_window.py           Main window: builder + viewport + tabbed
│   │                            info/FK panel
│   ├── robot_builder.py         Robot Builder panel (DOF + joint table, Phase 2)
│   ├── kinematics_panel.py      Live FK results panel (Phase 3): position,
│   │                            orientation, full transform
│   ├── visualization_widget.py  Qt wrapper around the PyVista viewport
│   └── (joint_panel.py, analysis_panel.py, trajectory_panel.py —
│         placeholders for later phases)
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

- A window titled **"Robot Kinematics Studio — Phase 3"** opens (1650×850).
- On launch, the 3D viewport shows the same hard-coded 2‑DOF demo robot
  from Phase 1 (base cube, blue link cylinders, orange joint spheres,
  red end-effector marker, RGB coordinate triads at each frame, world
  axes, and a faint ground grid). Rotate/pan/zoom with the mouse.
- The **left panel ("Robot Builder")** lets you:
  1. Enter a robot name and choose a DOF count (1–8) via the spin box.
  2. Click **"Generate Joint Table"** to create one row per joint, with
     editable Type (Revolute/Prismatic), Axis (X/Y/Z), Link Length
     (meters), and Min/Max Limit. Limits are entered in **degrees**
     for revolute joints and **meters** for prismatic joints — this is
     converted to radians internally at the GUI boundary, so the rest
     of the app never sees anything but SI units. Columns are sized so
     "Revolute"/"Prismatic" and numeric values are fully readable.
  3. Changing a row's Type resets that row's limit fields to sensible
     defaults for the new type.
  4. Click **"Build Robot"** to validate and construct the robot.
     - On success: the 3D viewport, the right-hand tabbed panel, and
       the status bar all update immediately to the new robot.
     - On failure (e.g. empty name, min > max, negative link length,
       duplicate joint names, or revolute limits far outside
       ±360°): a message box lists every problem found — nothing is
       silently allowed through.
  5. Changing the DOF count and regenerating preserves values already
     entered for rows that still exist, so you don't lose your edits
     when only adding/removing a joint or two.
- The **right panel** is now tabbed:
  - **"Robot Info"** — name, DOF, each joint's type/axis/value/limits,
    and each link's length (same as Phase 2).
  - **"Forward Kinematics"** (new) — end-effector X/Y/Z position; the
    3×3 rotation matrix; Roll/Pitch/Yaw in degrees; the quaternion
    [w, x, y, z]; and the full 4×4 homogeneous transform. All of these
    update immediately whenever a new robot is built.

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
- **`gui/main_window.py`** — Assembles the Robot Builder panel + 3D
  viewport + a tabbed right panel ("Robot Info" / "Forward
  Kinematics"), and connects `robot_built` to swap in the new robot
  and refresh the viewport, both right-panel tabs, and the status bar.
  No robotics math or rendering code lives here — it only calls into
  `RobotModel`, `RobotBuilderPanel`, `KinematicsPanel`, and
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
- **`test_jacobian.py`**, **`test_ik.py`** — skipped placeholders,
  filled in during Phase 5/6.

All 35 active tests currently pass (2 skipped, by design).

## 7. Known limitations (Phase 3, by design)

- No joint sliders / interactive control yet — the Robot Builder sets
  a robot's *shape* (DOF, types, axes, lengths, limits), but moving
  individual joints interactively is Phase 4.
- No Denavit-Hartenberg convenience layer yet (Section 11) — the
  current representation is axis + link-length based; DH parameters
  (a, alpha, d, theta) as an optional modeling layer are deferred to a
  later phase since Phase 3's explicit deliverables were the generic
  engine, intermediate transforms, and pose display.
- Only X/Y/Z joint axes are exposed in the builder UI, though the
  underlying math (`rotation_about_axis`) already supports arbitrary
  axes.
- No IK, Jacobian, singularity, workspace, or trajectory features.
- No save/load — `RobotModel.from_config` takes `JointConfig` objects
  built by the GUI; JSON import/export of that same structure is
  Phase 9.
- Revolute limit validation only flags values outside ±360°, as a
  guard against unit-entry mistakes — it does not otherwise second-guess
  unusual-but-valid limits.
- Euler-XYZ conversion picks one valid solution (roll = 0) at the
  gimbal-lock singularity (pitch = ±90°) rather than reporting both
  degenerate solutions.

## 8. Next step: Phase 4 — Interactive Joint Control

Planned work:
- Add `gui/joint_panel.py`: a slider per joint (degrees for revolute,
  millimeters/meters for prismatic), each respecting that joint's
  limits, with a numeric readout and a "Reset" action.
- Wire slider movement straight into `RobotModel.set_joint_value()` →
  `compute_fk()` → 3D viewport + Forward Kinematics tab, all updating
  in real time as the user drags.
- Verify interactive responsiveness holds up as DOF count increases
  (up to the 8-DOF ceiling the Robot Builder currently allows).
- Add a "Reset All" action that zeros every joint (clamped to limits
  where 0 is out of range, matching `RobotModel.reset()`).
