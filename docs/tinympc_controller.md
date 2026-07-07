# TinyMPC Controller

This branch adds a TinyMPC controller that matches the Python TinyMPC backend in
`nanodrone-controller-autotuning`.

## Update Constants

From the parent project root:

```bash
python scripts/mpc/export_mpc_constants.py \
  --backend tinympc \
  --dt 0.01 \
  --horizon 20 \
  --tinympc-header firmware/crazyflie-firmware/src/modules/interface/controller/crazyflie_tinympc_constants.h
```

## Build

TinyMPC is C++/Eigen based. Build with the C++ linker enabled:

```bash
make tinympc_defconfig
make OOT_USES_CXX=1
```

The controller can also be selected through Kconfig as `CONFIG_CONTROLLER_TINYMPC`.

## Runtime Interface

The controller is registered as `ControllerTypeTinyMPC` and named `TinyMPC`.
It consumes the standard Crazyflie setpoint/state fields:

- `state.position`
- `state.velocity`
- `state.attitudeQuaternion`
- `setpoint.position`
- `setpoint.velocity`
- `setpoint.acceleration`
- yaw from `setpoint.attitude.yaw` or `setpoint.attitudeRate.yaw`

TinyMPC returns the first acceleration command. The controller converts it to
total thrust and attitude error, then uses the legacy Mellinger-style
attitude/rate output path.

## Notes

The first integration keeps the same 6-state translational MPC used in
simulation:

```text
x = [px, py, pz, vx, vy, vz]
u = [ax, ay, az]
```

Input box constraints are enabled. Tube/racing constraints are not yet embedded
in this firmware controller.
