# Realistic Odometry Simulation for a 2D Differential-Drive Robot — Conversation Highlights

**Purpose of this document:** self-contained context for another agent. It summarizes the design decisions and reference material from a discussion about simulating realistic odometry (wheel slip, under-tracking of velocity commands, drift) in a 2D mobile-robot simulator that accepts linear/angular velocity commands `(v, ω)`. Language is Python/NumPy. Intended downstream use: producing odometry edges (with covariances) for a pose-graph SLAM system.

---

## 1. Core principle: keep three quantities separate

| Quantity | Role |
|---|---|
| **Commanded** velocity `u = (v, ω)` | What the controller asks for |
| **True** motion / pose `x_true` | Ground truth. Used only for evaluation and for rendering sensors (e.g. lidar). Never exposed to the robot's estimator |
| **Odometry-reported** pose `x_odom` | What the robot believes about its own motion |

Realism comes from the *structure* of the gaps between these, not from adding white noise to a pose.

---

## 2. Approach A — physical wheel-level model

Produces temporally **correlated, biased** odometry errors. This is the harder, more realistic test for a SLAM back end.

### Error layers (in order of the signal path)

1. **Actuator tracking (command ≠ actual).** First-order lag `v += (v_cmd − v)·dt/τ`, acceleration limit, saturation, optional low-speed deadband. Encoders *see* this layer correctly, so it makes the robot slower than commanded but does not by itself create odometry error.
2. **Wheel–ground interaction (actual ≠ encoder).** Per-wheel longitudinal slip ratio `s`: wheel turns at `ω_wheel` but ground speed is `r·ω_wheel·(1 − s)`. Encoders count rotation, so odometry over-reports distance when `s > 0`. Model `s` as an Ornstein–Uhlenbeck process (temporally correlated, not white), add spikes proportional to wheel acceleration, optionally make its variance depend on a friction-map lookup at the true position. Add lateral skid in turns (∝ `v·ω`), which encoders cannot see at all.
3. **Calibration error (systematic, not noise).** True wheel radii and track width differ slightly from the nominal values the odometry code uses. A 1–2 % left/right radius mismatch turns a straight command into a gentle arc; a track-width error scales every heading change. This dominates drift on real robots and does not average out.
4. **Encoder quantization** (ticks per revolution).

Odometry integrates encoder ticks through **nominal** kinematics; truth integrates true wheel ground speeds through **true** kinematics.

### Reference implementation

```python
import numpy as np

class DiffDriveSim:
    def __init__(self, dt, rng, r_nom=0.10, b_nom=0.50):
        self.dt, self.rng = dt, rng
        self.r_nom, self.b_nom = r_nom, b_nom                          # what the odometry code believes
        self.r_true, self.b_true = np.array([0.100, 0.1015]), 0.505    # what is actually true
        self.tau, self.ticks = 0.15, 2048                              # velocity lag [s], encoder ticks/rev
        self.w, self.slip, self.frac = np.zeros(2), np.zeros(2), np.zeros(2)
        self.x_true, self.x_odom = np.zeros(3), np.zeros(3)

    def step(self, v_cmd, om_cmd):
        dt = self.dt
        w_ref = np.array([v_cmd - om_cmd*self.b_nom/2, v_cmd + om_cmd*self.b_nom/2]) / self.r_nom
        dw = np.clip((w_ref - self.w)*dt/self.tau, -15*dt, 15*dt)     # first-order lag + accel limit
        self.w += dw
        self.slip += -self.slip*dt/0.5 + 0.02*np.sqrt(dt)*self.rng.standard_normal(2) + 0.2*np.abs(dw)
        self.slip = np.clip(self.slip, 0.0, 0.5)                       # OU slip ratio, spikes on accel
        vg = self.w*self.r_true*(1 - self.slip)                        # true ground speed per wheel
        v, om = vg.mean(), (vg[1] - vg[0])/self.b_true
        self.x_true = self._integrate(self.x_true, v, om, v_lat=-0.02*v*om)   # skid outward in turns
        t = self.w*dt/(2*np.pi)*self.ticks + self.frac                 # encoders see rotation, not ground
        n = np.trunc(t); self.frac = t - n
        d = n/self.ticks*2*np.pi*self.r_nom                            # odometry uses nominal params
        self.x_odom = self._integrate(self.x_odom, d.mean()/dt, (d[1] - d[0])/self.b_nom/dt)
        return self.x_true, self.x_odom

    def _integrate(self, x, v, om, v_lat=0.0):
        th = x[2] + om*self.dt/2
        return x + self.dt*np.array([v*np.cos(th) - v_lat*np.sin(th), v*np.sin(th) + v_lat*np.cos(th), om])
```

**Sanity check:** drive a loop and plot `x_odom` against `x_true`. Expect slow, smooth, curving drift — not jitter.

---

## 3. Approach B — *Probabilistic Robotics* velocity motion model (Thrun, Burgard & Fox, §5.3)

A definition of the transition density `p(x_t | u_t, x_{t−1})` for control `u = (v, ω)` applied over a fixed interval `Δt`. Two uses: **sampling** a successor pose (particle-filter prediction, or a simulator) and **evaluating** the density of a given successor pose.

### Deterministic core: constant-velocity arc

With `r = v/ω`:

```text
x' = x − r·sin θ + r·sin(θ + ωΔt)
y' = y + r·cos θ − r·cos(θ + ωΔt)
θ' = θ + ωΔt
```

Straight-line limit when `ω → 0`: `x' = x + vΔt·cos θ`, `y' = y + vΔt·sin θ`.

### Noise: perturb the controls, not the pose

Noise is added to `(v, ω)` and pushed through the arc geometry, which makes the pose distribution non-Gaussian (the classic banana-shaped cloud). Three perturbations, six parameters:

| Perturbation | Variance | Meaning |
|---|---|---|
| `v̂ = v + N(0, α₁v² + α₂ω²)` | translational | from driving (α₁) and turning (α₂) |
| `ω̂ = ω + N(0, α₃v² + α₄ω²)` | rotational | from driving (α₃) and turning (α₄) |
| `γ̂ = N(0, α₅v² + α₆ω²)` | final extra rotation | `θ' = θ + ω̂Δt + γ̂Δt` |

α₁, α₄ are the obvious terms; α₂, α₃ are cross terms (e.g. one-sided slip during a straight run perturbs heading).

**Why γ exists:** without it every reachable pose lies on a 2-D surface in 3-D pose space (final heading is a deterministic function of final position), so the density would be zero almost everywhere and could not be evaluated. γ makes the distribution full-rank. It is a modeling fix, not a physical effect.

### Sampler

```python
def sample_velocity_model(x, u, alpha, dt, rng):
    v, om = u
    a1, a2, a3, a4, a5, a6 = alpha
    v_hat  = v  + rng.normal(0, np.sqrt(a1*v**2 + a2*om**2))
    om_hat = om + rng.normal(0, np.sqrt(a3*v**2 + a4*om**2))
    g_hat  =      rng.normal(0, np.sqrt(a5*v**2 + a6*om**2))
    th = x[2]
    if abs(om_hat) > 1e-6:
        r = v_hat / om_hat
        dx = -r*np.sin(th) + r*np.sin(th + om_hat*dt)
        dy =  r*np.cos(th) - r*np.cos(th + om_hat*dt)
    else:
        dx, dy = v_hat*dt*np.cos(th), v_hat*dt*np.sin(th)
    return np.array([x[0]+dx, x[1]+dy, th + om_hat*dt + g_hat*dt])
```

### Density evaluation (for reference)

Given start and end poses, invert the geometry: find the circle whose arc connects the two positions (center on the perpendicular bisector of the positions, intersected with the perpendicular to the start heading), read off the implied `v̂`, `ω̂` from radius and swept angle, take `γ̂` as the leftover heading change after `ω̂Δt`. Density = product of three 1-D Gaussians on `(v − v̂)`, `(ω − ω̂)`, `γ̂` with the α-defined variances. The book also offers a triangular distribution as a drop-in.

### Practical quirks

- **Parameterization ambiguity.** In the book the noise argument is a *variance* (std = `√(α₁v² + …)`). Many implementations use std = `α₁|v| + α₂|ω|`. Both fine, but α values are not interchangeable between conventions.
- **Not Δt-invariant** in its book form. Per-step position error ≈ v-noise × Δt, so applying with fresh samples at 100 Hz vs 10 Hz gives different total drift for the same α's. See §5 for the fix used in the simulator.
- **Tuning is empirical.** Drive a known path on the real robot, look at the spread of odometry endpoints, set α's until sampled endpoints show similar spread. Start at a few percent.

### Properties vs. Approach A

Errors are **white, zero-mean, uncorrelated** between steps. Its covariance maps directly onto a pose-graph odometry edge, so a Gaussian-edge optimizer is well-calibrated against it *by construction*. This makes it an easier (more optimistic) test than Approach A, whose errors are correlated and biased.

---

## 4. Sibling: the odometry motion model (§5.4)

Recommended by the book over the velocity model for anything but simulation, because measured odometry predicts actual motion better than the command. Treats the odometry-reported displacement as the "control," decomposes it into

```text
rotate by δ_rot1  →  translate by δ_trans  →  rotate by δ_rot2
```

and perturbs each step with four α's (rotation noise from rotation and from translation; translation noise from translation and from rotation). This is the model AMCL implements under `odom_alpha1`–`odom_alpha4`, and it is the natural way to assign a covariance to an odometry edge in a pose graph: linearize the three perturbed steps and propagate the α-scaled variances.

---

## 5. Decisions for the simulator

### Sample once — truth gets the noise, odometry is clean

```python
def step(x_true, x_odom, u, dt):
    x_true = sample_velocity_model(x_true, u, alpha, dt, rng)   # one draw: what actually happened
    x_odom = integrate_arc(x_odom, u, dt)                       # noise-free: what the robot believes
    return x_true, x_odom
```

- Truth is the sampled quantity: that is what the model is defined to describe, lidar is rendered from `x_true`, and any controller/planner closes the loop on a robot that does not do exactly what it is told.
- Odometry is the clean integration of the command ("I did what I commanded"). The odometry error is then exactly the one perturbation drawn, and its statistics are the chosen α's.
- Do **not** sample both independently: still valid, but roughly doubles the variance, both trajectories wander off the commanded path, and tuning becomes confusing.
- Refinement: run the command through the actuator lag *before* both integrators, so odometry reflects sluggish tracking (which real encoders see) but not slip (which they don't).

### Loop rates: physics 100 Hz, controller 50 Hz, odometry 10 Hz

| Loop | Rate | Job |
|---|---|---|
| Physics | 100 Hz (`dt = 0.01`) | Integrates true pose. **The only place noise is drawn.** Also runs the clean belief integrator. |
| Controller | 50 Hz | Produces `(v, ω)`. Zero-order hold between updates (each command covers 2 physics ticks). |
| Odometry | 10 Hz | Pure read-out of the belief integrator every 10th tick. Computes nothing of its own. |

```python
for k in range(steps):                                          # 100 Hz, dt = 0.01
    if k % 2 == 0:
        u = controller(state_estimate)                          # 50 Hz, held in between
    u_act = actuator(u, dt)                                     # lag / accel limit, seen by both
    x_true = sample_velocity_model(x_true, u_act, alpha, dt, rng)   # noisy truth
    x_odom = integrate_arc(x_odom, u_act, dt)                       # clean belief
    if k % 10 == 0:
        publish_odom(t=k*dt, pose=x_odom)                       # 10 Hz read-out
```

With exact arc integration and piecewise-constant commands, integrating the belief at 100 Hz or 50 Hz gives identical results.

### Rate-invariant noise scaling (fix for the Δt problem)

Drawing 10 independent perturbations per odometry interval makes the per-report error depend on the tick rate. Fix: scale per-tick **variance** by `1/dt`:

```text
v̂ = v + N(0, (α₁v² + α₂ω²) / dt)      (and likewise for ω̂)
```

Then per-tick position-increment variance is `(α₁v² + α₂ω²)·dt`, and over any interval `T` it sums to `(α₁v² + α₂ω²)·T` regardless of tick rate. The α's now describe an **error growth rate**, and the 100 ms odometry report has error variance ≈ `0.1·(α₁v² + α₂ω²)` by construction.

Side effects of per-tick sampling (both fine for a simulator):

- The γ term can be dropped — heading receives independent `ω̂` noise every tick, so the pose distribution is already full-rank.
- Per-report error is close to Gaussian rather than banana-shaped (sum of many small perturbations).

### SLAM-side edge covariance

Consecutive 10 Hz reports → relative transform → odometry edge. Its covariance should match the accumulated 100 ms error. Two routes:

- **Analytic:** translational variance ≈ `0.1·(α₁v² + α₂ω²)` along travel direction, rotational ≈ `0.1·(α₃v² + α₄ω²)`, plus lateral spread from heading error coupling over the interval.
- **Empirical (preferred):** log `odom_rel − true_rel` over a few thousand intervals and fit. Captures the coupling terms without algebra and is the same procedure used on a real robot.

### Timestamps and latency

Stamp each report with the physics time at which it was read. To simulate latency, publish the belief state from a few ticks earlier. Late-but-correctly-stamped odometry is what a real SLAM front end has to cope with.

---

## 6. Quick comparison

| | Approach A (physical) | Approach B (velocity model) |
|---|---|---|
| Error character | Correlated, biased, systematic drift | White, zero-mean |
| Realism | High | Moderate |
| SLAM test difficulty | Hard (Gaussian edges are mis-specified) | Easy (well-calibrated by construction) |
| Covariance for edges | Must be fit empirically | Follows directly from α's |
| Use when | Stress-testing a back end | Quick testing / sanity checks |

Both share the same architecture from §5 (truth noisy, odometry clean, noise at physics rate, odometry as a read-out).
