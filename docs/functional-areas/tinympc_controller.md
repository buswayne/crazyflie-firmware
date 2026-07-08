# TinyMPC controller

Position-tracking MPC controller for the Crazyflie 2.1 Brushless nanodrone
build, built on the vendored [TinyMPC](https://tinympc.org) ADMM solver.
Controller type `6` (`CONFIG_CONTROLLER_TINYMPC=y`).

Status: builds and flies tethered/bench-tested. Not yet cleared for free
flight — validate hover hold and takeoff behavior on a new build/tune before
removing the tether.

## Files

| File | Role |
| --- | --- |
| `vendor/tinympc/` | Vendored TinyMPC ADMM solver (fixed-size, no heap — see `types.hpp`) |
| `src/modules/src/controller/controller_tinympc.cpp` | Controller glue: feeds solver, hands result to `controllerPid` |
| `src/modules/interface/controller/crazyflie_tinympc_constants.h` | Model (A/B), cost (Q/R), solver tuning (iterations, tolerances), physical constants (mass, thrust limits) |

## Architecture: MPC plans, PID flies

The controller does **not** convert its output directly into thrust/attitude
commands. That conversion (geometric attitude control, integral action,
thrust mapping) was originally hand-rolled in this file and caused two real
bugs before it was removed: a mass mismatch that made the drone leap on
takeoff, and no integral term, which caused steady-state xy drift the
model couldn't explain away — see [Model](#model-and-tuning) and [History](#history)
below. `controllerPid` doesn't have either problem; it's already tuned and
already flying correctly, so we reuse it instead of re-solving problems it
already solved.

```
setpoint (position/velocity/accel target)
        │
        ▼
tiny_solve()  ── 6-state double integrator [pos, vel], 3-input [accel]
        │         horizon 10 steps @ 10ms (100Hz), warm-started
        ▼
solver->solution->x(:, 1)   ── the solved state one step (10ms) ahead
        │
        ▼
controllerPid()  ── existing, already-tuned inner loop: position/velocity
                     PID → attitude PID → thrust/torque → motors
```

MPC's job is exactly what it's good at: producing a constraint-aware,
horizon-optimal near-term trajectory. Everything below "what should the
motors do to reach that trajectory" is delegated to `controllerPid`, the
same inner loop the stock PID controller uses — it already holds position
without drift and already has correct thrust/attitude conversion for this
airframe.

This mirrors the pattern used in the reference implementation
([`RoboticExplorationLab/tinympc-crazyflie-firmware`](https://github.com/RoboticExplorationLab/tinympc-crazyflie-firmware),
linked from tinympc.org): their `controller_tinympc_safety` example also
solves TinyMPC purely as a trajectory generator and feeds the result as a
setpoint into an existing tuned controller (`controllerBrescianini` there,
`controllerPid` here).

### Manual (stick) control bypasses MPC entirely

`controllerTinyMPCFirmware` checks `setpoint->mode.x`/`mode.y` first:

- **`modeDisable`** (raw/manual RPYT stick flight): MPC is skipped
  completely, the original setpoint is passed straight to `controllerPid`.
  Flying manually is identical to the stock PID controller — TinyMPC is
  never involved.
- **Anything else** (assisted/hover mode: velocity or position setpoints,
  including stick-driven ones from cfclient's PosHold/Hover mode, or
  autonomous trajectories): MPC solves and hands `controllerPid` the
  smoothed, constraint-aware one-step-ahead target instead of the raw
  setpoint.

### Solve loop

`controllerTinyMPCFirmware` is called every stabilizer tick (1000Hz,
unconditionally, same as `controllerPid`'s own internal rate gating).
The MPC solve itself only runs when `RATE_DO_EXECUTE(POSITION_RATE, ...)`
fires (100Hz, i.e. every 10th tick):

1. Read current `[position, velocity]` into `current_state`.
2. Build the reference horizon from the incoming setpoint
   (`fillReferenceHorizon`): constant-acceleration extrapolation of
   position/velocity/acceleration across the horizon.
3. `tiny_set_x0` / `tiny_set_x_ref` / `tiny_set_u_ref`, then `tiny_solve`
   (warm-started from the previous solve's dual/slack variables — this is
   why solve time drops sharply after the first call).
4. Copy `solver->solution->x(:, 1)` (position+velocity one `CF_TINYMPC_DT`
   step ahead) into `mpc_setpoint`, preserving the rest of the original
   setpoint fields (yaw, modes, thrust) unchanged.
5. `controllerPid(control, &mpc_setpoint, ...)` runs every tick regardless,
   using whatever `mpc_setpoint` was last computed (zero-order hold between
   position ticks).

`tiny_setup()` (one-time LQR backward-pass precompute: `Kinf`/`Pinf`/
`Quu_inv`) happens lazily on the first call to `initSolver()`, which is
called both from `controllerTinyMPCFirmwareInit()` at boot and defensively
at the top of every `controllerTinyMPCFirmware()` call (no-op once
`solver != nullptr`).

## Model and tuning

State `x = [x, y, z, vx, vy, vz]` (double integrator), input
`u = [ax, ay, az]` (acceleration). `A`/`B` encode the double-integrator
dynamics at `CF_TINYMPC_DT` and don't depend on mass — mass only mattered
in the old direct thrust conversion, which no longer exists in this file.

All tunables live in `crazyflie_tinympc_constants.h`:

| Constant | Current value | Notes |
| --- | --- | --- |
| `CF_TINYMPC_HORIZON` | 10 | Must match `NHORIZON` in `vendor/tinympc/include/tinympc/types.hpp` |
| `CF_TINYMPC_DT` | 0.01 | Matches `POSITION_RATE` (100Hz) |
| `CF_TINYMPC_MAX_ITER` | 10 | ADMM iteration cap per solve |
| `CF_TINYMPC_ABS_PRI_TOL` / `ABS_DUA_TOL` | 5e-3 | Early-exit tolerance; loosened from 1e-3 for real-time budget |
| `CF_TINYMPC_Q` (pos/vel weights) | 5.6/5.6/9.8 pos, 2.5/2.5/4 vel | Softened from 8/8/14 for a conservative first flight; retune once bench-validated |
| `CF_TINYMPC_R` | 0.525/0.525/0.75 | Softened from 0.35/0.35/0.5, same reason |
| `CF_TINYMPC_MASS` | 0.0393 kg | Matches `CF_MASS` in `platform_defaults_cf21bl.h` — **weigh the actual craft (deck + battery) and correct if it differs**; now only used for reference, not thrust conversion |

`tinytype` (`vendor/tinympc/include/tinympc/types.hpp`) is **`float`, not
`double`**. This matters: the STM32F405's FPU (`fpv4-sp-d16`) is
single-precision only, so `double` arithmetic there is software-emulated —
this alone was responsible for a 356ms one-time setup and a 13ms per-solve
time that blocked the shared 1kHz stabilizer task and starved the Kalman
estimator. Don't change this back without re-checking solve time.

### Iterating on tuning conservatively

Don't guess — measure. `first tiny_solve took %lu us` only prints once; to
tune `MAX_ITER`/tolerances meaningfully you need `solver->solution->iter`
and per-solve timing logged continuously (not currently wired up — add if
doing a tuning pass). Change one axis at a time (max_iter, tolerance, or
Q/R) and validate on bench before the next change. Pass/fail gate before
considering flight: zero `ESTKALMAN: WARNING: Kalman prediction rate off`
messages over a sustained (~60s) bench run — any recurrence means the solve
is still eating into the shared loop's budget.

## History

Earlier revisions of this controller included a hand-rolled Mellinger-style
geometric attitude controller (quaternion-based thrust/attitude conversion,
gains copied from `controller_mellinger.c`) instead of delegating to
`controllerPid`. It caused two bugs worth remembering if this code is ever
reintroduced:

- **Mass mismatch**: `CF_TINYMPC_MASS` was 0.0482 (23% heavier than the
  platform's actual `CF_MASS`), which made the gravity feedforward term
  systematically over-command thrust — the drone leapt violently on
  takeoff.
- **No integral action**: a pure LQR/MPC regulator has no state to reject a
  constant, unmodeled disturbance (motor asymmetry, sensor bias) — it
  produces steady-state error a PID's `ki` term would cancel. This showed
  up as constant xy drift during hover that the stock PID controller
  didn't have. (The reference firmware's `controller_lqi.c` addresses this
  the same way — an augmented integral state, though only on `z` there.)

Both are structural gaps in *any* hand-rolled replacement for the inner
loop, which is why the controller now reuses `controllerPid` instead of
re-deriving it.
