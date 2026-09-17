# Kinematic models

Each YAML here describes how a robot's commanded velocity turns into motion,
and how much the *true* motion deviates from the command. The simulator keeps
two poses per robot:

- **Truth** — integrated from the command *plus sampled noise*
  (`IdealMotionModel::update` / `OmniMotionModel::update`).
- **Odometry** — integrated from the *unperturbed* command
  (`IdealMotionModel::integrate` / `OmniMotionModel::integrate`).

So the `a_*` coefficients set how far truth drifts away from what the robot
believes. The noise scheme is the velocity motion model from Thrun et al.,
*Probabilistic Robotics* (`sample_motion_model_velocity`), implemented in
`stdr_simulation/src/motion/noise_model.cpp`.

## File layout

```yaml
kinematic:
  kinematic_specifications:
    kinematic_model: ideal        # ideal (differential drive) or omni
    odometry_model: velocity      # perfect (alphas ignored) or velocity
    kinematic_parameters:
      a_ux_ux: 0.0004
      ...
```

- `kinematic_model`
  - `ideal` — differential drive. Only `linear_x` and `angular_z` are used;
    `linear_y` is ignored, so the `a_uy_*` row and `*_uy` columns do nothing.
  - `omni` — holonomic. `linear_x`, `linear_y` and `angular_z` all integrate.
- `odometry_model`
  - `perfect` — no noise is drawn at all, odometry equals truth. All `a_*`
    values are ignored. This is the default when the key is absent.
  - `velocity` — noise below is applied every physics tick.

## What the `a_<out>_<in>` coefficients mean

Each coefficient couples one **command input** to noise on one **output**.
Read `a_ux_w` as "how much commanded angular velocity (`w`) adds noise to
the executed linear-x velocity (`ux`)".

Inputs (columns): `ux` = commanded `linear_x`, `uy` = commanded `linear_y`,
`w` = commanded `angular_z`.

Outputs (rows):

| Row    | Noise is added to                  | Effect on truth pose                         |
|--------|------------------------------------|----------------------------------------------|
| `a_ux_*` | executed `linear_x`               | along-track position error                   |
| `a_uy_*` | executed `linear_y` (omni only)   | cross-track position error                   |
| `a_w_*`  | executed `angular_z`              | heading error                                |
| `a_g_*`  | extra heading drift term `g`      | heading error (added to `angular_z` too)     |

Each tick, with timestep `dt`, the executed velocity is

```text
ux' = ux + N(0, (a_ux_ux*ux² + a_ux_uy*uy² + a_ux_w*w²) / dt)
uy' = uy + N(0, (a_uy_ux*ux² + a_uy_uy*uy² + a_uy_w*w²) / dt)
w'  = w  + N(0, (a_w_ux*ux²  + a_w_uy*uy²  + a_w_w*w²)  / dt)
g   =      N(0, (a_g_ux*ux²  + a_g_uy*uy²  + a_g_w*w²)  / dt)
```

and truth integrates `(ux', uy', w' + g)`.

Notes:

- **Noise only exists while moving.** Every term is scaled by a squared
  command. A stationary robot (`ux = uy = w = 0`) never drifts.
- **`a_w_*` and `a_g_*` are interchangeable in this simulator.** Both models
  add `g` straight onto `angular_z`, so `a_g_ux` behaves the same as
  `a_w_ux`, etc. The variances add: effective heading variance coefficient
  is `a_w_x + a_g_x`. The split is kept for fidelity to Thrun's
  formulation, where `γ` is the "final rotation" term.
- **Diagonal vs off-diagonal.** `a_ux_ux`, `a_uy_uy`, `a_w_w` are
  "speed makes that same axis noisy". Off-diagonals cross-couple: `a_w_ux`
  is "driving straight makes heading wander" (wheel radius mismatch),
  `a_ux_w` is "turning makes forward speed jitter" (wheel slip in a turn).

## Units and magnitude

The `1/dt` scaling makes accumulated drift independent of tick rate: over
`T` seconds at constant command `u`, the accumulated error variance is
`a * u² * T`. So each `a` has units of `(error unit)² / (command unit)² / s`
and the handy rule is:

```text
error std after T seconds  =  sqrt(a * T) * |u|
```

Examples at `T = 1 s`:

| `a`      | at `ux = 1 m/s` (translation) | at `w = 1 rad/s` (heading) |
|----------|-------------------------------|----------------------------|
| 0.0001   | 1 cm                          | 0.6°                       |
| 0.0004   | 2 cm                          | 1.1°                       |
| 0.001    | 3.2 cm                        | 1.8°                       |
| 0.005    | 7 cm                          | 4.1°                       |
| 0.02     | 14 cm                         | 8.1°                       |

Sanity check: `ideal_kinematic_realistic.yaml` uses `a_w_w = 0.0003`, so a
90° turn at 1 rad/s (1.57 s) ends with heading std of
`sqrt(0.0003 * 1.57) ≈ 0.022 rad ≈ 1.2°`. That is why odometry heading
tracks truth so well in the realistic profile.

## Provided profiles

| File                              | Model | Odometry | Character                                              |
|-----------------------------------|-------|----------|--------------------------------------------------------|
| `ideal_kinematic.yaml`            | ideal | perfect  | odometry == truth                                      |
| `ideal_kinematic_realistic.yaml`  | ideal | velocity | ~2 cm/s and ~1°/s drift, decent diff-drive on flat floor |
| `ideal_kinematic_noisy.yaml`      | ideal | velocity | exaggerated (~8°/s heading), visibly jittery at 100 Hz |
| `omni_kinematic.yaml`             | omni  | perfect  | odometry == truth                                      |
| `omni_kinematic_noisy.yaml`       | omni  | velocity | large lateral (`uy`) noise only                        |

## Recipe: more heading drift, no extra position drift

To make **only rotations** drift more without adding any position error of
their own:

- **Raise `a_w_w`** (or `a_g_w`, same effect). This is the only term that
  turns commanded rotation into heading error. `0.0003 → 0.005` gives ~4°
  std per second of turning at 1 rad/s, ~5° after a 90° turn.
- **Keep `a_ux_w` and `a_uy_w` at 0.** Those are the terms that let rotation
  inject *translational* noise. They are already 0 in the realistic profile.
- **Leave `a_ux_ux` alone.** It is the only source of along-track error in
  the ideal model.
- Optionally leave `a_w_ux` / `a_g_ux` at their small values. They add
  heading drift while driving *straight*, not while turning, so raise them
  only if you want heading to wander on straight runs too.

Caveat that no parameter can remove: a heading error, once present, rotates
every subsequent straight-line segment. The true `x, y` will diverge from
odometry `x, y` after the next drive even though the noise itself was purely
rotational. That is the physics of dead reckoning, not a coupling coefficient.
Also, if the robot has a non-zero `center_of_rotation`, a noisy `w` moves the
body origin during a pure spin, so position drifts slightly there too.
