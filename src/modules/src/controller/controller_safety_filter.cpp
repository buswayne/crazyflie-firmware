#define DEBUG_MODULE "SAFEFILT"
#include "debug.h"

#include <cstddef>

#include "controller_safety_filter.h"
extern "C" {
#include "controller_pid.h"
#include "log.h"
#include "param.h"
}
#include "safety_filter_constants.h"
#include "stabilizer_types.h"
#include "tinympc/tiny_api.hpp"
#include "usec_time.h"

#include <math.h>

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
setpoint_t filtered_setpoint;
bool has_filtered_setpoint = false;
uint8_t sfEnable = 0;
uint8_t sfActive = 0;
uint8_t sfMode = 0;
uint32_t sfInterventions = 0;
uint32_t sfPassThrough = 0;
uint32_t sfUnsupported = 0;
uint32_t solveUs = 0;
uint16_t solveIter = 0;
uint8_t solveStatus = 0;
float primalResidual = 0.0f;
float dualResidual = 0.0f;
float primalStateResidual = 0.0f;
float primalInputResidual = 0.0f;
float dualStateResidual = 0.0f;
float dualInputResidual = 0.0f;
float nominalAx = 0.0f;
float nominalAy = 0.0f;
float nominalAz = 0.0f;
float solvedAx = 0.0f;
float solvedAy = 0.0f;
float solvedAz = 0.0f;
float sfBoxXY = 0.5f;
float sfMinZ = 0.2f;
float sfMaxZ = 1.5f;
float sfLookahead = 0.5f;
float sfOriginX = 0.0f;
float sfOriginY = 0.0f;
bool sfOriginValid = false;
uint8_t sfLastPrintedMode = 255;

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

  fillRowMajorMatrix(solver_A, CF_SAFETY_FILTER_A);
  fillRowMajorMatrix(solver_B, CF_SAFETY_FILTER_B);
  fillRowMajorMatrix(solver_Q, CF_SAFETY_FILTER_Q);
  fillRowMajorMatrix(solver_R, CF_SAFETY_FILTER_R);
  fillRepeatedColumns(solver_x_min, CF_SAFETY_FILTER_X_MIN);
  fillRepeatedColumns(solver_x_max, CF_SAFETY_FILTER_X_MAX);
  fillRepeatedColumns(solver_u_min, CF_SAFETY_FILTER_U_MIN);
  fillRepeatedColumns(solver_u_max, CF_SAFETY_FILTER_U_MAX);

  const uint64_t setupStartUs = usecTimestamp();
  int status = tiny_setup(
    &solver,
    solver_A,
    solver_B,
    solver_Q,
    solver_R,
    static_cast<tinytype>(CF_SAFETY_FILTER_RHO),
    CF_SAFETY_FILTER_STATE_DIM,
    CF_SAFETY_FILTER_INPUT_DIM,
    CF_SAFETY_FILTER_HORIZON,
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
    static_cast<tinytype>(CF_SAFETY_FILTER_ABS_PRI_TOL),
    static_cast<tinytype>(CF_SAFETY_FILTER_ABS_DUA_TOL),
    CF_SAFETY_FILTER_MAX_ITER,
    CF_SAFETY_FILTER_CHECK_TERMINATION,
    1,
    1);
}

void fillReferenceHorizon(const setpoint_t *setpoint, tinyMatrixNxNh &x_ref, tinyMatrixNuNhm1 &u_ref)
{
  const float pos_ref[3] = {setpoint->position.x, setpoint->position.y, setpoint->position.z};
  const float vel_ref[3] = {setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z};
  const float acc_ref[3] = {setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z};

  for (int k = 0; k < CF_SAFETY_FILTER_HORIZON; ++k) {
    const tinytype tau = static_cast<tinytype>(k) * static_cast<tinytype>(CF_SAFETY_FILTER_DT);
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

  for (int k = 0; k < CF_SAFETY_FILTER_HORIZON - 1; ++k) {
    u_ref(0, k) = static_cast<tinytype>(acc_ref[0]);
    u_ref(1, k) = static_cast<tinytype>(acc_ref[1]);
    u_ref(2, k) = static_cast<tinytype>(acc_ref[2]);
  }
}

float clampFloat(float value, float minValue, float maxValue)
{
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

void bodyVelocityToWorld(const setpoint_t *setpoint, const state_t *state, float *vx, float *vy)
{
  if (!setpoint->velocity_body) {
    *vx = setpoint->velocity.x;
    *vy = setpoint->velocity.y;
    return;
  }

  const float yaw = state->attitude.yaw * static_cast<float>(M_PI) / 180.0f;
  const float cosyaw = cosf(yaw);
  const float sinyaw = sinf(yaw);
  *vx = setpoint->velocity.x * cosyaw - setpoint->velocity.y * sinyaw;
  *vy = setpoint->velocity.x * sinyaw + setpoint->velocity.y * cosyaw;
}

bool isSafetyFilterSetpoint(const setpoint_t *setpoint)
{
  return setpoint->mode.x == modeVelocity &&
         setpoint->mode.y == modeVelocity &&
         (setpoint->mode.z == modeAbs || setpoint->mode.z == modeVelocity);
}

bool isDisabledSetpoint(const setpoint_t *setpoint)
{
  return setpoint->mode.x == modeDisable ||
         setpoint->mode.y == modeDisable ||
         setpoint->mode.z == modeDisable;
}

void printSafetyFilterMode(const uint8_t mode)
{
  if (mode == sfLastPrintedMode) {
    return;
  }
  sfLastPrintedMode = mode;
  DEBUG_PRINT("SF mode %u (0 off, 1 pass, 2 filter, 3 unsupported)\n", mode);
}

bool isInsideSafetyBox(const state_t *state, const float vx, const float vy, const float z)
{
  const float x = state->position.x + vx * sfLookahead;
  const float y = state->position.y + vy * sfLookahead;
  return x >= sfOriginX - sfBoxXY && x <= sfOriginX + sfBoxXY &&
         y >= sfOriginY - sfBoxXY && y <= sfOriginY + sfBoxXY &&
         state->position.z >= sfMinZ && state->position.z <= sfMaxZ &&
         z >= sfMinZ && z <= sfMaxZ;
}

void updateSolverSafetyBox()
{
  for (int k = 0; k < CF_SAFETY_FILTER_HORIZON; ++k) {
    solver->work->x_min(0, k) = static_cast<tinytype>(sfOriginX - sfBoxXY);
    solver->work->x_max(0, k) = static_cast<tinytype>(sfOriginX + sfBoxXY);
    solver->work->x_min(1, k) = static_cast<tinytype>(sfOriginY - sfBoxXY);
    solver->work->x_max(1, k) = static_cast<tinytype>(sfOriginY + sfBoxXY);
    solver->work->x_min(2, k) = static_cast<tinytype>(sfMinZ);
    solver->work->x_max(2, k) = static_cast<tinytype>(sfMaxZ);
  }
}

void resetSolverBounds()
{
  fillRepeatedColumns(solver->work->x_min, CF_SAFETY_FILTER_X_MIN);
  fillRepeatedColumns(solver->work->x_max, CF_SAFETY_FILTER_X_MAX);
}

void fillSafetyFilterReferenceHorizon(const state_t *state,
                                      const float ax, const float ay, const float az,
                                      tinyMatrixNxNh &x_ref, tinyMatrixNuNhm1 &u_ref)
{
  tinytype px = static_cast<tinytype>(state->position.x);
  tinytype py = static_cast<tinytype>(state->position.y);
  tinytype pz = static_cast<tinytype>(state->position.z);
  tinytype vx = static_cast<tinytype>(state->velocity.x);
  tinytype vy = static_cast<tinytype>(state->velocity.y);
  tinytype vz = static_cast<tinytype>(state->velocity.z);
  const tinytype ux = static_cast<tinytype>(ax);
  const tinytype uy = static_cast<tinytype>(ay);
  const tinytype uz = static_cast<tinytype>(az);
  const tinytype dt = static_cast<tinytype>(CF_SAFETY_FILTER_DT);
  const tinytype halfDt2 = static_cast<tinytype>(0.5f) * dt * dt;

  for (int k = 0; k < CF_SAFETY_FILTER_HORIZON; ++k) {
    x_ref(0, k) = px;
    x_ref(1, k) = py;
    x_ref(2, k) = pz;
    x_ref(3, k) = static_cast<tinytype>(vx);
    x_ref(4, k) = static_cast<tinytype>(vy);
    x_ref(5, k) = static_cast<tinytype>(vz);

    px += dt * vx + halfDt2 * ux;
    py += dt * vy + halfDt2 * uy;
    pz += dt * vz + halfDt2 * uz;
    vx += dt * ux;
    vy += dt * uy;
    vz += dt * uz;
  }

  for (int k = 0; k < CF_SAFETY_FILTER_HORIZON - 1; ++k) {
    u_ref(0, k) = ux;
    u_ref(1, k) = uy;
    u_ref(2, k) = uz;
  }
}

void runSafetySolver(const setpoint_t *setpoint, const state_t *state)
{
  current_state(0) = static_cast<tinytype>(state->position.x);
  current_state(1) = static_cast<tinytype>(state->position.y);
  current_state(2) = static_cast<tinytype>(state->position.z);
  current_state(3) = static_cast<tinytype>(state->velocity.x);
  current_state(4) = static_cast<tinytype>(state->velocity.y);
  current_state(5) = static_cast<tinytype>(state->velocity.z);

  tiny_set_x0(solver, current_state);
  tiny_set_x_ref(solver, reference_states);
  tiny_set_u_ref(solver, reference_inputs);
  const uint64_t solveStartUs = usecTimestamp();
  tiny_solve(solver);
  solveUs = static_cast<uint32_t>(usecTimestamp() - solveStartUs);
  solveIter = static_cast<uint16_t>(solver->solution->iter);
  solveStatus = static_cast<uint8_t>(solver->solution->solved);
  primalStateResidual = static_cast<float>(solver->work->primal_residual_state);
  primalInputResidual = static_cast<float>(solver->work->primal_residual_input);
  dualStateResidual = static_cast<float>(solver->work->dual_residual_state);
  dualInputResidual = static_cast<float>(solver->work->dual_residual_input);
  primalResidual = fmaxf(primalStateResidual, primalInputResidual);
  dualResidual = fmaxf(dualStateResidual, dualInputResidual);
  solvedAx = static_cast<float>(solver->solution->u(0, 0));
  solvedAy = static_cast<float>(solver->solution->u(1, 0));
  solvedAz = static_cast<float>(solver->solution->u(2, 0));
  static bool solveTimeReported = false;
  if (!solveTimeReported) {
    DEBUG_PRINT("first safety solve took %lu us, iter=%u, solved=%u\n",
                (unsigned long)solveUs, solveIter, solveStatus);
    solveTimeReported = true;
  }
}

} // namespace

extern "C" void controllerSafetyFilterInit(void)
{
  controllerPidInit();
  DEBUG_PRINT("Before safety solver init\n");
  const int status = initSolver();
  DEBUG_PRINT("After safety solver init: %d\n", status);
  if (status != 0) {
    DEBUG_PRINT("Safety filter init failed: %d\n", status);
  }
}

extern "C" bool controllerSafetyFilterTest(void)
{
  return initSolver() == 0;
}

// The safety filter only edits the position/velocity setpoint. Thrust/attitude
// conversion stays in controllerPid, the already-tuned inner loop.
extern "C" void controllerSafetyFilter(control_t *control, const setpoint_t *setpoint,
                                          const sensorData_t *sensors,
                                          const state_t *state,
                                          const stabilizerStep_t stabilizerStep)
{
  if (initSolver() != 0 ||
      (!sfEnable && (setpoint->mode.x != modeAbs || setpoint->mode.y != modeAbs || setpoint->mode.z != modeAbs))) {
    // Solver not ready, or manual modes without safety filter: use stock PID.
    has_filtered_setpoint = false;
    sfOriginValid = false;
    sfActive = 0;
    sfMode = 0;
    printSafetyFilterMode(sfMode);
    controllerPid(control, setpoint, sensors, state, stabilizerStep);
    return;
  }

  if (sfEnable && isSafetyFilterSetpoint(setpoint)) {
    if (!sfOriginValid) {
      sfOriginX = state->position.x;
      sfOriginY = state->position.y;
      sfOriginValid = true;
    }

    float vx = 0.0f;
    float vy = 0.0f;
    bodyVelocityToWorld(setpoint, state, &vx, &vy);
    const float predictedZ = setpoint->mode.z == modeVelocity
      ? state->position.z + setpoint->velocity.z * sfLookahead
      : setpoint->position.z;
    sfActive = !isInsideSafetyBox(state, vx, vy, predictedZ);
    sfMode = sfActive ? 2 : 1;
    printSafetyFilterMode(sfMode);

    if (!sfActive) {
      has_filtered_setpoint = false;
      sfPassThrough++;
      controllerPid(control, setpoint, sensors, state, stabilizerStep);
      return;
    }

    if (!has_filtered_setpoint || RATE_DO_EXECUTE(RATE_10_HZ, stabilizerStep)) {
      const float desiredVz = setpoint->mode.z == modeVelocity
        ? setpoint->velocity.z
        : (predictedZ - state->position.z) / sfLookahead;
      nominalAx = clampFloat((vx - state->velocity.x) / sfLookahead,
                             static_cast<float>(CF_SAFETY_FILTER_U_MIN[0]),
                             static_cast<float>(CF_SAFETY_FILTER_U_MAX[0]));
      nominalAy = clampFloat((vy - state->velocity.y) / sfLookahead,
                             static_cast<float>(CF_SAFETY_FILTER_U_MIN[1]),
                             static_cast<float>(CF_SAFETY_FILTER_U_MAX[1]));
      nominalAz = clampFloat((desiredVz - state->velocity.z) / sfLookahead,
                             static_cast<float>(CF_SAFETY_FILTER_U_MIN[2]),
                             static_cast<float>(CF_SAFETY_FILTER_U_MAX[2]));

      updateSolverSafetyBox();
      fillSafetyFilterReferenceHorizon(state, nominalAx, nominalAy, nominalAz, reference_states, reference_inputs);
      runSafetySolver(setpoint, state);

      constexpr int kCmdHorizonIdx = CF_SAFETY_FILTER_HORIZON - 1;
      filtered_setpoint = *setpoint;
      filtered_setpoint.mode.x = modeAbs;
      filtered_setpoint.mode.y = modeAbs;
      filtered_setpoint.mode.z = modeAbs;
      filtered_setpoint.velocity_body = false;
      filtered_setpoint.position.x = static_cast<float>(solver->solution->x(0, kCmdHorizonIdx));
      filtered_setpoint.position.y = static_cast<float>(solver->solution->x(1, kCmdHorizonIdx));
      filtered_setpoint.velocity.x = static_cast<float>(solver->solution->x(3, kCmdHorizonIdx));
      filtered_setpoint.velocity.y = static_cast<float>(solver->solution->x(4, kCmdHorizonIdx));
      filtered_setpoint.position.z = static_cast<float>(solver->solution->x(2, kCmdHorizonIdx));
      has_filtered_setpoint = true;
      sfInterventions++;
      DEBUG_PRINT("SF intervention: pos=(%.2f %.2f %.2f), uNom=(%.2f %.2f %.2f), uSol=(%.2f %.2f %.2f), solved=%u/%u\n",
                  (double)state->position.x, (double)state->position.y, (double)state->position.z,
                  (double)nominalAx, (double)nominalAy, (double)nominalAz,
                  (double)solvedAx, (double)solvedAy, (double)solvedAz,
                  solveStatus, solveIter);
    }

    controllerPid(control, &filtered_setpoint, sensors, state, stabilizerStep);
    return;
  }

  sfOriginValid = false;
  sfActive = 0;
  sfMode = 0;

  if (isDisabledSetpoint(setpoint)) {
    has_filtered_setpoint = false;
    printSafetyFilterMode(sfMode);
    controllerPid(control, setpoint, sensors, state, stabilizerStep);
    return;
  }

  if (setpoint->mode.x != modeAbs || setpoint->mode.y != modeAbs || setpoint->mode.z != modeAbs) {
    has_filtered_setpoint = false;
    if (sfEnable) {
      sfMode = 3;
      printSafetyFilterMode(sfMode);
      sfUnsupported++;
    }
    controllerPid(control, setpoint, sensors, state, stabilizerStep);
    return;
  }

  if (RATE_DO_EXECUTE(RATE_10_HZ, stabilizerStep)) {
    resetSolverBounds();
    fillReferenceHorizon(setpoint, reference_states, reference_inputs);
    runSafetySolver(setpoint, state);

    // End of the solved horizon (~CF_SAFETY_FILTER_HORIZON * CF_SAFETY_FILTER_DT ahead), not one
    // solver step: at one step, the drone physically can't have moved (10ms at max
    // accel ~= 0.4mm), so the position setpoint handed to the PID was always ~= the
    // current position regardless of the commanded target, and the PID's position
    // error, and therefore its response, collapsed to ~zero. The far end of the
    // horizon carries the actual commanded intent instead.
    constexpr int kCmdHorizonIdx = CF_SAFETY_FILTER_HORIZON - 1;
    filtered_setpoint = *setpoint;
    filtered_setpoint.position.x = static_cast<float>(solver->solution->x(0, kCmdHorizonIdx));
    filtered_setpoint.position.y = static_cast<float>(solver->solution->x(1, kCmdHorizonIdx));
    filtered_setpoint.position.z = static_cast<float>(solver->solution->x(2, kCmdHorizonIdx));
    filtered_setpoint.velocity.x = static_cast<float>(solver->solution->x(3, kCmdHorizonIdx));
    filtered_setpoint.velocity.y = static_cast<float>(solver->solution->x(4, kCmdHorizonIdx));
    filtered_setpoint.velocity.z = static_cast<float>(solver->solution->x(5, kCmdHorizonIdx));
    has_filtered_setpoint = true;
  }

  controllerPid(control, has_filtered_setpoint ? &filtered_setpoint : setpoint, sensors, state, stabilizerStep);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

PARAM_GROUP_START(safeFilt)
PARAM_ADD(PARAM_UINT8, sfEnable, &sfEnable)
PARAM_ADD(PARAM_FLOAT, sfBoxXY, &sfBoxXY)
PARAM_ADD(PARAM_FLOAT, sfMinZ, &sfMinZ)
PARAM_ADD(PARAM_FLOAT, sfMaxZ, &sfMaxZ)
PARAM_ADD(PARAM_FLOAT, sfLookahead, &sfLookahead)
PARAM_GROUP_STOP(safeFilt)

LOG_GROUP_START(safeFilt)
LOG_ADD(LOG_UINT8, sfActive, &sfActive)
LOG_ADD(LOG_UINT8, sfMode, &sfMode)
LOG_ADD(LOG_UINT32, sfInterv, &sfInterventions)
LOG_ADD(LOG_UINT32, sfPass, &sfPassThrough)
LOG_ADD(LOG_UINT32, sfUnsup, &sfUnsupported)
LOG_ADD(LOG_UINT32, solveUs, &solveUs)
LOG_ADD(LOG_UINT16, iter, &solveIter)
LOG_ADD(LOG_UINT8, solved, &solveStatus)
LOG_ADD(LOG_FLOAT, priRes, &primalResidual)
LOG_ADD(LOG_FLOAT, duaRes, &dualResidual)
LOG_ADD(LOG_FLOAT, priState, &primalStateResidual)
LOG_ADD(LOG_FLOAT, priInput, &primalInputResidual)
LOG_ADD(LOG_FLOAT, duaState, &dualStateResidual)
LOG_ADD(LOG_FLOAT, duaInput, &dualInputResidual)
LOG_ADD(LOG_FLOAT, uNomX, &nominalAx)
LOG_ADD(LOG_FLOAT, uNomY, &nominalAy)
LOG_ADD(LOG_FLOAT, uNomZ, &nominalAz)
LOG_ADD(LOG_FLOAT, uSolX, &solvedAx)
LOG_ADD(LOG_FLOAT, uSolY, &solvedAy)
LOG_ADD(LOG_FLOAT, uSolZ, &solvedAz)
LOG_ADD(LOG_FLOAT, sfOriginX, &sfOriginX)
LOG_ADD(LOG_FLOAT, sfOriginY, &sfOriginY)
LOG_GROUP_STOP(safeFilt)

#pragma GCC diagnostic pop
