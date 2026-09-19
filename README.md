# Robot Kinematics Studio — Phase 5

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

Phase 4 added **interactive joint control**: a **"Joint Control"** tab
with one slider per joint (degrees for revolute, millimeters for
prismatic), each respecting that joint's own limits. Dragging a slider
updates the robot in real time — the 3D viewport and the Forward
Kinematics tab both refresh immediately.

Phase 5 (this phase) adds **numerical inverse kinematics**: a new
**"Inverse Kinematics"** tab where you enter a target position (and,
optionally, orientation), pick a solver (Jacobian pseudoinverse or
damped least squares), and solve. Convergence status, iteration count,
and position/orientation error are reported explicitly — a
non-converged result is never silently applied to the robot. This
phase also introduces the core geometric Jacobian
(`kinematics/jacobian.py`), a hard dependency of the IK solver; its
full rank/singularity/manipulability *display* is still Phase 6.

## 1. Project structure

```
robot_kinematics_studio/
├── app/
│   ├── main.py            entry point
│   ├── application.py     QApplication + MainWindow bootstrap
│   └── config.py          app-wide constants
├── gui/
│   ├── main_window.py           Main window: builder + viewport + tabbed
│   │                            info/joint-control/FK/IK panel
│   ├── robot_builder.py         Robot Builder panel (DOF + joint table, Phase 2)
│   ├── joint_panel.py           Interactive joint sliders (Phase 4)
│   ├── kinematics_panel.py      Live FK results panel (Phase 3): position,
│   │                            orientation, full transform
│   ├── ik_panel.py              Inverse Kinematics panel (Phase 5): target
│   │                            pose input, solver options, results, apply
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
│   ├── jacobian.py            geometric Jacobian core (Phase 5, used by
│   │                          IK); rank/condition-number helpers; full
│   │                          display + singularity analysis is Phase 6
│   └── inverse_kinematics.py  numerical IK (Phase 5): pseudoinverse +
│                              damped least squares, position and
│                              position+orientation targets, explicit
│                              convergence/failure reporting
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
│   ├── test_jacobian.py          geometric Jacobian validated against
│   │                              analytical 2-link planar Jacobian,
│   │                              singularity detection (Phase 5)
│   └── test_ik.py                IK convergence, FK(IK(target))==target,
│                                  unreachable-target reporting, joint-limit
│                                  respect, apply-only-on-convergence (Phase 5)
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

- A window titled **"Robot Kinematics Studio — Phase 5"** opens (1700×850).
- On launch, the 3D viewport shows the same hard-coded 2‑DOF demo robot
  from Phase 1. Rotate/pan/zoom with the mouse.
- The **left panel ("Robot Builder")** works as in Phase 2: pick DOF,
  generate/edit the joint table, click "Build Robot" to construct a
  new robot. Building a robot resets the Joint Control sliders and the
  Forward Kinematics/Inverse Kinematics tabs to match the new robot.
- The **right panel** is tabbed:
  - **"Robot Info"** — static description of the loaded robot's name,
    DOF, joints, and links.
  - **"Joint Control"** — one slider per joint, as in Phase 4. Dragging
    any slider immediately updates the 3D viewport and the Forward
    Kinematics tab.
  - **"Forward Kinematics"** — end-effector position, orientation, and
    full transform, updating live as you drag joint sliders or apply
    an IK solution.
  - **"Inverse Kinematics"** (new) — enter a target X/Y/Z position
    (pre-filled with the robot's current end-effector position),
    optionally check "Include orientation in target" and enter
    Roll/Pitch/Yaw in degrees, choose a solver method (Damped Least
    Squares or Pseudoinverse) and max iterations, then click
    **"Solve IK"**. The status line reports convergence, iteration
    count, and position/orientation error — or, on failure, one of
    four explicit reasons ("Target unreachable", "Maximum iterations
    exceeded", "Singularity detected", "Joint limit prevented
    convergence"). The **"Apply Solution"** button is only enabled
    after a converged solve, and applying it updates the robot's
    joints, refreshing the 3D view, Joint Control sliders, and the
    Forward Kinematics tab. Note that for robots with multiple valid
    solutions (e.g. elbow-up/elbow-down on a 2-link arm), the solver
    may converge to a different-but-equally-valid configuration than
    the one you might expect — check the achieved position/orientation
    error rather than the specific joint values.

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
  the Jacobian), and the end-effector transform. Works on
  plain `Joint`/`Link` lists rather than `RobotModel`, keeping the math
  layer independent of the data model that owns it.
- **`kinematics/jacobian.py`** (Phase 5) — `compute_geometric_jacobian(
  joints, links, fk_result)` returns the 6×n geometric Jacobian
  (`J_v` stacked on `J_w`), generalized to arbitrary joint axes via
  `forward_kinematics.joint_motion_axis_world`. Also exposes
  `jacobian_rank` and `jacobian_condition_number`, used internally by
  IK's singularity classification and ready for Phase 6's full display.
- **`kinematics/inverse_kinematics.py`** (Phase 5) — `solve_ik(...)`:
  Jacobian-based numerical IK supporting `IKMethod.PSEUDOINVERSE` and
  `IKMethod.DAMPED_LEAST_SQUARES`, position-only or
  position+orientation targets, configurable iterations/tolerance/step
  size/damping, and joint-limit clamping applied every iteration (not
  just at the end). Returns an `IKResult` with `converged`,
  `iterations`, `position_error`, `orientation_error`, and — on
  failure — an explicit `IKFailureReason` (target unreachable, max
  iterations exceeded, singularity detected, or joint limit prevented
  convergence). Orientation error uses the standard skew-symmetric
  axis-angle extraction from `R_target @ R_current.T`.
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
  (Phase 5) `solve_ik(...)`: convenience wrapper around
  `kinematics.inverse_kinematics.solve_ik` using the robot's current
  joint values as the initial guess; `apply_result=True` only ever
  writes back a *converged* solution — a failed solve never mutates
  the robot, even if requested.
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
- **`gui/ik_panel.py`** (Phase 5) — `IKPanel`: target-pose input
  (X/Y/Z always, Roll/Pitch/Yaw behind a checkbox), solver method and
  max-iterations controls, "Solve IK" / "Apply Solution" buttons, and
  a status line built from `IKResult.status_message()`. Converts
  degrees→radians for the orientation target at this GUI boundary
  only. "Apply Solution" is disabled until a solve actually converges.
- **`gui/main_window.py`** — Assembles the Robot Builder panel + 3D
  viewport + a tabbed right panel ("Robot Info" / "Joint Control" /
  "Forward Kinematics" / "Inverse Kinematics"). `_on_robot_built`
  reloads everything for a newly constructed robot; `_on_joint_changed`
  and `_on_ik_solution_applied` both refresh the viewport and FK tab
  (the latter also resyncs the joint sliders) without a full reload,
  since the robot's joint values are already updated in place. No
  robotics math or rendering code lives here — it only calls into
  `RobotModel`, `RobotBuilderPanel`, `JointControlPanel`,
  `KinematicsPanel`, `IKPanel`, and `VisualizationWidget`.
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
- **`test_jacobian.py`** (Phase 5) — the geometric Jacobian matches
  the analytical 2-link planar Jacobian across several configurations
  (Section 24); correct 6×n shape; a prismatic joint's column is
  `[axis, 0]`; a generic configuration's position sub-Jacobian is full
  rank; the classic fully-extended 2-link singularity (q2=0) is
  correctly detected via rank deficiency and a very high/infinite
  condition number; a well-conditioned configuration's condition
  number is finite and positive.
- **`test_ik.py`** (Phase 5) — both `PSEUDOINVERSE` and
  `DAMPED_LEAST_SQUARES` converge on a target that's guaranteed
  reachable (built via FK from a known configuration); `FK(IK(target))
  == target` within tolerance; a 3-DOF spatial robot converges on a
  combined position+orientation target; a target ~10x beyond the
  robot's reach is correctly reported as `TARGET_UNREACHABLE` (checked
  early, without grinding through max_iterations); a heavily-restricted
  joint never leaves its limits during iteration; a non-converged
  result is *never* applied to the robot even when `apply_result=True`
  is requested; a converged result *is* applied when requested;
  mismatched initial-guess length raises `ValueError`.

All 58 active tests currently pass (0 skipped — every placeholder from
earlier phases now has real implementations and tests behind it).

## 7. Known limitations (Phase 5, by design)

- The "Robot Info" tab shows a static snapshot taken when the robot is
  loaded/built — it does not live-update as you drag joint sliders or
  apply an IK solution (the "Joint Control" and "Forward Kinematics"
  tabs do).
- No Denavit-Hartenberg convenience layer yet (Section 11) — deferred,
  as in Phase 3.
- Only X/Y/Z joint axes are exposed in the builder UI, though the
  underlying math (`rotation_about_axis`) already supports arbitrary
  axes.
- The Jacobian module exposes only what IK needs (`compute_geometric_
  jacobian`, `jacobian_rank`, `jacobian_condition_number`) — there is
  no GUI display of the Jacobian matrix itself, no manipulability
  measure, and no live singularity warning as you move joints; that's
  Phase 6.
- No workspace or trajectory features yet.
- No save/load — `RobotModel.from_config` takes `JointConfig` objects
  built by the GUI; JSON import/export of that same structure is
  Phase 9.
- IK's "Target unreachable" check is a conservative approximation
  (sum of link lengths from the base), not an exact workspace test —
  it will not catch every unreachable target (e.g. ones inside the
  max-reach sphere but blocked by joint limits), though those cases
  are still correctly reported as non-converged with a different
  reason after the iteration loop runs.
- IK's singularity/joint-limit failure classification uses fixed
  thresholds (condition number > 1e4, ≥50% of iterations clamped) —
  reasonable defaults, not user-configurable yet.
- Slider resolution is fixed at 1000 discrete steps per joint
  (`SLIDER_STEPS` in `gui/joint_panel.py`); this is smooth enough for
  interactive use but is not infinite precision.
- Euler-XYZ conversion picks one valid solution (roll = 0) at the
  gimbal-lock singularity (pitch = ±90°) rather than reporting both
  degenerate solutions.

## 8. Next step: Phase 6 — Jacobian + Singularity Analysis

Planned work:
- Promote `kinematics/jacobian.py` from "IK's internal dependency" to
  a fully displayed analysis tool: a GUI panel showing the numeric
  Jacobian matrix live as joints move.
- Add manipulability (`w = sqrt(det(J J^T))`, with an appropriate
  singular-value-based metric for non-square Jacobians) alongside the
  rank/condition-number helpers already in place.
- Add live singularity warnings (e.g. "⚠ Approaching singularity")
  that update as the user drags joint sliders, not just during IK.
- Expand `tests/test_jacobian.py` with additional known singular
  configurations across different robot topologies (not just the
  2-link planar case).
