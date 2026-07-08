#define DEBUG_MODULE "CTRLTMPC"
#include "debug.h"

#include <cstddef>

#include "controller_tinympc.h"
extern "C" {
#include "controller_pid.h"
}
#include "crazyflie_tinympc_constants.h"
#include "stabilizer_types.h"
#include "tinympc/tiny_api.hpp"
#include "usec_time.h"

namespace {

TinySolver *solver = nullptr;
tinyMatrixNxNx solver_A;
tinyMatrixNxNu solver_B;
tinyMatrixNxNx solver_Q;
tinyMatrixNuNu solver_R;
tinyMatrixNxNh solver_x_min;
tinyMatrixNxNh solver_x_max;
tinyMatrixNuNhm1 solver_u_min;
tinyMatrixNuNhm1 solver_u_max;
tinyVectorNx current_state;
tinyMatrixNxNh reference_states;
tinyMatrixNuNhm1 reference_inputs;
setpoint_t mpc_setpoint;

template <typename MatrixT>
void fillRowMajorMatrix(MatrixT &out, const double *data)
{
  for (int row = 0; row < MatrixT::RowsAtCompileTime; ++row) {
    for (int col = 0; col < MatrixT::ColsAtCompileTime; ++col) {
      out(row, col) = static_cast<tinytype>(data[row * MatrixT::ColsAtCompileTime + col]);
    }
  }
}

template <typename MatrixT>
void fillRepeatedColumns(MatrixT &out, const double *data)
{
  for (int col = 0; col < MatrixT::ColsAtCompileTime; ++col) {
    for (int row = 0; row < MatrixT::RowsAtCompileTime; ++row) {
      out(row, col) = static_cast<tinytype>(data[row]);
    }
  }
}

int initSolver()
{
  if (solver != nullptr) {
    return 0;
  }

  fillRowMajorMatrix(solver_A, CF_TINYMPC_A);
  fillRowMajorMatrix(solver_B, CF_TINYMPC_B);
  fillRowMajorMatrix(solver_Q, CF_TINYMPC_Q);
  fillRowMajorMatrix(solver_R, CF_TINYMPC_R);
  fillRepeatedColumns(solver_x_min, CF_TINYMPC_X_MIN);
  fillRepeatedColumns(solver_x_max, CF_TINYMPC_X_MAX);
  fillRepeatedColumns(solver_u_min, CF_TINYMPC_U_MIN);
  fillRepeatedColumns(solver_u_max, CF_TINYMPC_U_MAX);

  const uint64_t setupStartUs = usecTimestamp();
  int status = tiny_setup(
    &solver,
    solver_A,
    solver_B,
    solver_Q,
    solver_R,
    static_cast<tinytype>(CF_TINYMPC_RHO),
    CF_TINYMPC_STATE_DIM,
    CF_TINYMPC_INPUT_DIM,
    CF_TINYMPC_HORIZON,
    solver_x_min,
    solver_x_max,
    solver_u_min,
    solver_u_max,
    0);
  DEBUG_PRINT("tiny_setup took %lu us\n", (unsigned long)(usecTimestamp() - setupStartUs));
  if (status != 0) {
    return status;
  }

  return tiny_update_settings(
    solver->settings,
    static_cast<tinytype>(CF_TINYMPC_ABS_PRI_TOL),
    static_cast<tinytype>(CF_TINYMPC_ABS_DUA_TOL),
    CF_TINYMPC_MAX_ITER,
    CF_TINYMPC_CHECK_TERMINATION,
    1,
    1);
}

void fillReferenceHorizon(const setpoint_t *setpoint, tinyMatrixNxNh &x_ref, tinyMatrixNuNhm1 &u_ref)
{
  const float pos_ref[3] = {setpoint->position.x, setpoint->position.y, setpoint->position.z};
  const float vel_ref[3] = {setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z};
  const float acc_ref[3] = {setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z};

  for (int k = 0; k < CF_TINYMPC_HORIZON; ++k) {
    const tinytype tau = static_cast<tinytype>(k) * static_cast<tinytype>(CF_TINYMPC_DT);
    for (int axis = 0; axis < 3; ++axis) {
      x_ref(axis, k) =
        static_cast<tinytype>(pos_ref[axis]) +
        tau * static_cast<tinytype>(vel_ref[axis]) +
        static_cast<tinytype>(0.5) * tau * tau * static_cast<tinytype>(acc_ref[axis]);
      x_ref(axis + 3, k) =
        static_cast<tinytype>(vel_ref[axis]) +
        tau * static_cast<tinytype>(acc_ref[axis]);
    }
  }

  for (int k = 0; k < CF_TINYMPC_HORIZON - 1; ++k) {
    u_ref(0, k) = static_cast<tinytype>(acc_ref[0]);
    u_ref(1, k) = static_cast<tinytype>(acc_ref[1]);
    u_ref(2, k) = static_cast<tinytype>(acc_ref[2]);
  }
}

} // namespace

extern "C" void controllerTinyMPCFirmwareInit(void)
{
  controllerPidInit();
  DEBUG_PRINT("Before initSolver\n");
  const int status = initSolver();
  DEBUG_PRINT("After initSolver: %d\n", status);
  if (status != 0) {
    DEBUG_PRINT("TinyMPC init failed: %d\n", status);
  }
}

extern "C" bool controllerTinyMPCFirmwareTest(void)
{
  return initSolver() == 0;
}

// ponytail: MPC only plans the position/velocity trajectory (its actual
// strength: constrained optimization over a horizon). Thrust/attitude
// conversion is delegated to controllerPid, the already-tuned, already-flown
// inner loop, instead of a hand-rolled geometric controller reimplementing
// what that loop already does correctly (see tinympc-crazyflie-firmware's
// controller_tinympc_safety, which follows the same MPC-feeds-setpoint
// pattern via controllerBrescianini).
extern "C" void controllerTinyMPCFirmware(control_t *control, const setpoint_t *setpoint,
                                          const sensorData_t *sensors,
                                          const state_t *state,
                                          const stabilizerStep_t stabilizerStep)
{
  if (initSolver() != 0 || setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
    // Solver not ready, or manual stick passthrough: no position setpoint to plan against.
    controllerPid(control, setpoint, sensors, state, stabilizerStep);
    return;
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, stabilizerStep)) {
    current_state(0) = static_cast<tinytype>(state->position.x);
    current_state(1) = static_cast<tinytype>(state->position.y);
    current_state(2) = static_cast<tinytype>(state->position.z);
    current_state(3) = static_cast<tinytype>(state->velocity.x);
    current_state(4) = static_cast<tinytype>(state->velocity.y);
    current_state(5) = static_cast<tinytype>(state->velocity.z);

    fillReferenceHorizon(setpoint, reference_states, reference_inputs);

    tiny_set_x0(solver, current_state);
    tiny_set_x_ref(solver, reference_states);
    tiny_set_u_ref(solver, reference_inputs);
    const uint64_t solveStartUs = usecTimestamp();
    tiny_solve(solver);
    static bool solveTimeReported = false;
    if (!solveTimeReported) {
      DEBUG_PRINT("first tiny_solve took %lu us\n", (unsigned long)(usecTimestamp() - solveStartUs));
      solveTimeReported = true;
    }

    // End of the solved horizon (~CF_TINYMPC_HORIZON * CF_TINYMPC_DT ahead), not one
    // solver step: at one step, the drone physically can't have moved (10ms at max
    // accel ~= 0.4mm), so the position setpoint handed to the PID was always ~= the
    // current position regardless of the commanded target, and the PID's position
    // error, and therefore its response, collapsed to ~zero. The far end of the
    // horizon carries the actual commanded intent instead.
    constexpr int kCmdHorizonIdx = CF_TINYMPC_HORIZON - 1;
    mpc_setpoint = *setpoint;
    mpc_setpoint.position.x = static_cast<float>(solver->solution->x(0, kCmdHorizonIdx));
    mpc_setpoint.position.y = static_cast<float>(solver->solution->x(1, kCmdHorizonIdx));
    mpc_setpoint.position.z = static_cast<float>(solver->solution->x(2, kCmdHorizonIdx));
    mpc_setpoint.velocity.x = static_cast<float>(solver->solution->x(3, kCmdHorizonIdx));
    mpc_setpoint.velocity.y = static_cast<float>(solver->solution->x(4, kCmdHorizonIdx));
    mpc_setpoint.velocity.z = static_cast<float>(solver->solution->x(5, kCmdHorizonIdx));
  }

  controllerPid(control, &mpc_setpoint, sensors, state, stabilizerStep);
}
