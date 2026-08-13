# Robot Kinematics Studio — Phase 2

Phase 1 (foundation) is complete: project structure, PySide6 main
window, PyVista 3D viewport, robot/joint/link data model,
transformation utilities, and a hard-coded 2‑DOF planar revolute demo
robot rendered in 3D.

Phase 2 (this phase) adds the **Robot Builder**: the user can now pick
a number of degrees of freedom, edit a generated joint-configuration
table (type, axis, link length, limits), and construct an arbitrary
serial robot without touching Python source code.

## 1. Project structure

```
robot_kinematics_studio/
├── app/
│   ├── main.py            entry point
│   ├── application.py     QApplication + MainWindow bootstrap
│   └── config.py          app-wide constants
├── gui/
│   ├── main_window.py           Main window: builder + viewport + info panel
│   ├── robot_builder.py         Robot Builder panel (DOF + joint table, Phase 2)
│   ├── visualization_widget.py  Qt wrapper around the PyVista viewport
│   └── (joint_panel.py, kinematics_panel.py, analysis_panel.py,
│         trajectory_panel.py — placeholders for later phases)
├── robot/
│   ├── joint.py            Joint data model (type, axis, limits, value)
│   ├── link.py              Link data model (length, radius)
│   ├── frame.py              Named coordinate frame helper
│   └── robot_model.py    RobotModel serial chain, JointConfig,
│                          RobotModel.from_config(), demo 2-DOF robot
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

- A window titled **"Robot Kinematics Studio — Phase 2"** opens (1400×850).
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
     of the app never sees anything but SI units.
  3. Changing a row's Type resets that row's limit fields to sensible
     defaults for the new type.
  4. Click **"Build Robot"** to validate and construct the robot.
     - On success: the 3D viewport, the right-hand "Robot Info" panel,
       and the status bar all update immediately to the new robot.
     - On failure (e.g. empty name, min > max, negative link length,
       duplicate joint names, or revolute limits far outside
       ±360°): a message box lists every problem found — nothing is
       silently allowed through.
  5. Changing the DOF count and regenerating preserves values already
     entered for rows that still exist, so you don't lose your edits
     when only adding/removing a joint or two.
- The **right panel ("Robot Info")** always reflects the currently
  loaded robot: name, DOF, each joint's type/axis/value/limits, and
  each link's length.

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
  later), `translation`, `homogeneous_transform`, plus
  `is_valid_transform` for defensive checks.
- **`visualization/robot_renderer.py`** — Pure rendering: reads a
  `RobotModel`'s computed frames and draws cylinders/spheres/arrows
  into a PyVista plotter. No robotics math lives here.
- **`gui/visualization_widget.py`** — Qt widget embedding
  `pyvistaqt.QtInteractor`; exposes `update_robot(robot)` so the rest
  of the GUI never touches PyVista directly.
- **`robot/robot_model.py`** (Phase 2 additions) — `JointConfig`: the
  GUI-unit-agnostic row object the builder constructs, with its own
  `validate()`; `RobotModel.validate_configs(...)`: whole-list checks
  (empty list, duplicate names) on top of each row's own validation;
  `RobotModel.from_config(...)`: validates then constructs a
  `RobotModel`, raising `RobotModelError` with every problem listed if
  validation fails — never a partially-built or silently-wrong robot.
- **`gui/robot_builder.py`** — `RobotBuilderPanel`: DOF spin box, joint
  table (`QTableWidget` with per-cell combo/spin widgets), and a
  `robot_built` Qt signal emitted with a valid `RobotModel`. Handles
  the degrees↔radians conversion at this GUI boundary only; everything
  it hands to `RobotModel` is already validated SI data.
- **`gui/main_window.py`** — Assembles the Robot Builder panel + 3D
  viewport + live info panel, and connects `robot_built` to swap in
  the new robot and refresh the viewport/info panel/status bar. No
  robotics math or rendering code lives here — it only calls into
  `RobotModel`, `RobotBuilderPanel`, and `VisualizationWidget`.
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
- **`test_jacobian.py`**, **`test_ik.py`** — skipped placeholders,
  filled in during Phase 5/6.

All 25 active tests currently pass (2 skipped, by design).

## 7. Known limitations (Phase 2, by design)

- No joint sliders / interactive control yet — the Robot Builder sets
  a robot's *shape* (DOF, types, axes, lengths, limits), but moving
  individual joints interactively is Phase 4.
- No generic, chain-agnostic FK engine — `RobotModel.compute_frames()`
  still does the job for any serial chain built via `from_config`
  (it was never actually limited to the 2-DOF demo, just under-tested
  until now), but a dedicated `kinematics/forward_kinematics.py`
  engine with retained intermediate transforms, DH-parameter support,
  and richer pose display (Euler/RPY/quaternion) arrives in Phase 3.
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

## 8. Next step: Phase 3 — Forward Kinematics

Planned work:
- Promote the FK logic currently inline in `RobotModel.compute_frames()`
  into a proper `kinematics/forward_kinematics.py` engine that also
  retains every intermediate transform `T_0_1 ... T_0_n` for later
  Jacobian/visualization use.
- Add a Denavit-Hartenberg convenience layer (Section 11) alongside
  the existing axis-based representation, clearly documented and not
  mixed with the modified-DH convention.
- Add a `gui/kinematics_panel.py` FK results display: position,
  orientation (rotation matrix first, Euler/RPY/quaternion later),
  and the full homogeneous transform, all updating live.
- Expand `tests/test_fk.py` to validate the generic engine (not just
  the 2-DOF demo) against additional known analytical robots.
