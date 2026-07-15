# Predictive Safety Filter

This build adds a predictive safety filter for Crazyflie 2.1 Brushless.

## Build

```bash
make safety_filter_defconfig
make
```

The controller can also be selected through Kconfig as
`CONFIG_CONTROLLER_SAFETY_FILTER`.

## Runtime

The firmware registers the controller as `SafetyFilter`. The public parameter
and log group is `safeFilt`.

Useful parameters:

```text
safeFilt.sfEnable
safeFilt.sfBoxXY
safeFilt.sfMinZ
safeFilt.sfMaxZ
safeFilt.sfLookahead
```

Useful logs:

```text
safeFilt.sfMode
safeFilt.sfActive
safeFilt.sfInterv
safeFilt.solveUs
safeFilt.iter
safeFilt.solved
safeFilt.priRes
safeFilt.duaRes
```

The filter lets safe Hover/PosHold joystick commands pass through unchanged.
When the predicted motion violates the configured position box, it solves a
constrained predictive optimization problem and passes the filtered
position/velocity setpoint to the stock PID controller.
