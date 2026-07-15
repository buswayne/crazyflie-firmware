# Predictive Safety Filter

The safety filter is implemented as a Crazyflie controller named
`SafetyFilter`. It uses the vendored solver internally, but the firmware-facing
module is the safety filter.

## Files

| File | Role |
| --- | --- |
| `vendor/tinympc/` | Vendored ADMM solver |
| `src/modules/src/controller/controller_safety_filter.cpp` | Safety filter glue and runtime parameters |
| `src/modules/interface/controller/controller_safety_filter.h` | Controller interface |
| `src/modules/interface/controller/safety_filter_constants.h` | Model, costs, bounds, and solver settings |
| `configs/safety_filter_defconfig` | CF21BL build config |

## Behavior

Hover/PosHold joystick commands are treated as nominal commands. If the
predicted position remains inside the configured safety box, the original
setpoint is passed straight to `controllerPid`.

If the command would leave the box, the filter solves a constrained
double-integrator problem:

```text
x = [px, py, pz, vx, vy, vz]
u = [ax, ay, az]
```

The filtered position/velocity setpoint is then handed to `controllerPid`,
which still handles attitude, thrust, and motor mixing.

## Current Tuning

```text
CF_SAFETY_FILTER_HORIZON = 10
CF_SAFETY_FILTER_DT = 0.05
CF_SAFETY_FILTER_MAX_ITER = 8
```

The filter solves at `RATE_10_HZ`, giving a 0.5 s prediction horizon while
keeping the stock PID inner loop active at its normal rates.

## Limits

This is an online predictive safety filter prototype, not a formal safety proof.
It does not yet include a terminal invariant safe set, recursive feasibility
proof, or a fallback policy based on solver convergence.
