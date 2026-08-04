#define DEBUG_MODULE "SAFEFILT"
#include "debug.h"

#include <cstddef>

#include "controller_safety_filter.h"
extern "C" {
#include "controller_mellinger.h"
#include "log.h"
#include "param.h"
}
#include "safety_filter_constants.h"
#include "stabilizer_types.h"
#ifndef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
#include "tinympc/tiny_api.hpp"
#endif
#include "usec_time.h"
#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
extern "C" {
#include "terminal_osqp_controller.h"
#include "residual_rls.h"
}
#endif

#include <math.h>

namespace {

#ifndef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
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
#endif
setpoint_t filtered_setpoint;
bool has_filtered_setpoint = false;
uint8_t sfEnable = 1;
uint8_t sfType = 2; // 1: CBF, 2: predictive safety filter
float cbfK1 = 4.0f;
float cbfK2 = 4.0f;
uint8_t sfActive = 0;
uint8_t sfMode = 0;
uint32_t sfInterventions = 0;
uint32_t sfUnsupported = 0;
uint32_t solveUs = 0;
uint16_t solveIter = 0;
uint8_t solveStatus = 0;
int8_t osqpStatus = 0;
uint32_t osqpSolveUs = 0;
uint32_t osqpSequence = 0;
#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
residualRls_t residualRls;
uint8_t residualEnable = 0;
uint8_t residualWasEnabled = 0;
float residualBlend = 0.25f;
float residualMeasurementBlend = 0.25f;
float residualGainX = 1.0f;
float residualGainY = 1.0f;
float residualGainZ = 1.0f;
float residualBiasX = 0.0f;
float residualBiasY = 0.0f;
float residualBiasZ = 0.0f;
float residualAlphaX = 0.75f;
float residualAlphaY = 0.75f;
float residualAlphaZ = 0.75f;
float residualBetaX = 0.20f;
float residualBetaY = 0.20f;
float residualBetaZ = 0.20f;
#endif
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
float nominalPositionRefX = 0.0f;
float nominalPositionRefY = 0.0f;
float nominalYawRef = 0.0f;
bool nominalPositionRefValid = false;
float sfLookahead = 0.5f;
float sfMaxVelocity = 0.5f;
uint8_t sfLastPrintedMode = 255;

#ifndef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
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

template <typename MatrixT>
void shiftWarmStart(MatrixT &matrix)
{
  constexpr int last = MatrixT::ColsAtCompileTime - 1;
  matrix.leftCols(last) = matrix.rightCols(last).eval();
  matrix.col(last) = matrix.col(last - 1);
}

void shiftSolverWarmStart()
{
  shiftWarmStart(solver->work->v);
  shiftWarmStart(solver->work->vnew);
  shiftWarmStart(solver->work->g);
  shiftWarmStart(solver->work->z);
  shiftWarmStart(solver->work->znew);
  shiftWarmStart(solver->work->y);
  shiftWarmStart(solver->work->d);
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
#endif

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

bool isVelocitySafetyFilterSetpoint(const setpoint_t *setpoint)
{
  return setpoint->mode.x == modeVelocity &&
         setpoint->mode.y == modeVelocity &&
         (setpoint->mode.z == modeAbs || setpoint->mode.z == modeVelocity);
}

bool isFullStateSafetyFilterSetpoint(const setpoint_t *setpoint)
{
  return setpoint->mode.x == modeAbs &&
         setpoint->mode.y == modeAbs &&
         setpoint->mode.z == modeAbs &&
         setpoint->mode.quat == modeAbs;
}

void printSafetyFilterMode(const uint8_t mode)
{
  if (mode == sfLastPrintedMode) {
    return;
  }
  sfLastPrintedMode = mode;
  DEBUG_PRINT("SF mode %u (0 off, 1 pass, 2 filter, 3 unsupported)\n", mode);
}

#ifndef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
void resetSolverBounds(const state_t *state)
{
  fillRepeatedColumns(solver->work->x_min, CF_SAFETY_FILTER_X_MIN);
  fillRepeatedColumns(solver->work->x_max, CF_SAFETY_FILTER_X_MAX);

  // If the craft is already outside the box (an earlier saturated push got
  // it there), a hard box across the whole horizon makes the QP permanently
  // infeasible: physically it can't re-enter the box within one 50ms step at
  // the allowed accel, so tiny_solve() never converges right when the
  // strongest recovery push is needed (observed: solved=0/20 once z escaped
  // X_MAX, falling back to the much weaker CF_SAFETY_FILTER_RECOVERY_SPEED
  // target). Taper the bound back to the real box linearly over the horizon
  // instead, so there is always a feasible, maximum-effort path back in.
  const float pos[3] = {state->position.x, state->position.y, state->position.z};
  for (int axis = 0; axis < 3; ++axis) {
    const float overshootHigh = pos[axis] - static_cast<float>(CF_SAFETY_FILTER_X_MAX[axis]);
    const float overshootLow = static_cast<float>(CF_SAFETY_FILTER_X_MIN[axis]) - pos[axis];
    if (overshootHigh > 0.0f) {
      const float axisMax = static_cast<float>(CF_SAFETY_FILTER_X_MAX[axis]);
      for (int k = 0; k < CF_SAFETY_FILTER_HORIZON; ++k) {
        const float taper = overshootHigh * (1.0f - static_cast<float>(k) / static_cast<float>(CF_SAFETY_FILTER_HORIZON - 1));
        solver->work->x_max(axis, k) = static_cast<tinytype>(axisMax + taper);
      }
    } else if (overshootLow > 0.0f) {
      const float axisMin = static_cast<float>(CF_SAFETY_FILTER_X_MIN[axis]);
      for (int k = 0; k < CF_SAFETY_FILTER_HORIZON; ++k) {
        const float taper = overshootLow * (1.0f - static_cast<float>(k) / static_cast<float>(CF_SAFETY_FILTER_HORIZON - 1));
        solver->work->x_min(axis, k) = static_cast<tinytype>(axisMin - taper);
      }
    }
  }

  // x0 is a measurement, not a decision variable. Constraining it makes the
  // QP permanently infeasible as soon as estimation noise crosses the box.
  for (int axis = 0; axis < 3; ++axis) {
    solver->work->x_min(axis, 0) = static_cast<tinytype>(-1e17);
    solver->work->x_max(axis, 0) = static_cast<tinytype>(1e17);
  }
}
#endif

float recoveryVelocity(float position, float minPosition, float maxPosition)
{
  if (position < minPosition) {
    return static_cast<float>(CF_SAFETY_FILTER_RECOVERY_SPEED);
  }
  if (position > maxPosition) {
    return -static_cast<float>(CF_SAFETY_FILTER_RECOVERY_SPEED);
  }
  return 0.0f;
}

#ifndef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
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

void runTinySafetySolver(const setpoint_t *setpoint, const state_t *state)
{
  current_state(0) = static_cast<tinytype>(state->position.x);
  current_state(1) = static_cast<tinytype>(state->position.y);
  current_state(2) = static_cast<tinytype>(state->position.z);
  current_state(3) = static_cast<tinytype>(state->velocity.x);
  current_state(4) = static_cast<tinytype>(state->velocity.y);
  current_state(5) = static_cast<tinytype>(state->velocity.z);

  shiftSolverWarmStart();
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
  if (solveStatus) {
    solvedAx = static_cast<float>(solver->solution->u(0, 0));
    solvedAy = static_cast<float>(solver->solution->u(1, 0));
    solvedAz = static_cast<float>(solver->solution->u(2, 0));
  } else {
    // A non-converged iterate is not a safe command. Hold hover instead.
    solvedAx = 0.0f;
    solvedAy = 0.0f;
    solvedAz = 0.0f;
  }
  static bool solveTimeReported = false;
  if (!solveTimeReported) {
    DEBUG_PRINT("first safety solve took %lu us, iter=%u, solved=%u\n",
                (unsigned long)solveUs, solveIter, solveStatus);
    solveTimeReported = true;
  }
}
#endif

void runSafetySolver(const setpoint_t *setpoint, const state_t *state)
{
#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
  (void)setpoint;
  const float osqpState[6] = {
    state->position.x, state->position.y, state->position.z,
    state->velocity.x, state->velocity.y, state->velocity.z,
  };
  const float nominal[3] = {nominalAx, nominalAy, nominalAz};
  float predicted[3];
  const float blend = residualEnable ? clampFloat(residualBlend, 0.0f, 1.0f) : 0.0f;
  residualRlsPredict(&residualRls, nominal, blend, predicted);
  terminalOsqpRequest(osqpState, predicted);
#else
  runTinySafetySolver(setpoint, state);
#endif
}

void runCbfSafetyFilter(const state_t *state)
{
  const float position[3] = {state->position.x, state->position.y, state->position.z};
  const float velocity[3] = {state->velocity.x, state->velocity.y, state->velocity.z};
  const float nominal[3] = {nominalAx, nominalAy, nominalAz};
  float safe[3];
  for (int axis = 0; axis < 3; ++axis) {
    const float positionMin = static_cast<float>(CF_SAFETY_FILTER_X_MIN[axis]);
    const float positionMax = static_cast<float>(CF_SAFETY_FILTER_X_MAX[axis]);
    const float accelerationMin = static_cast<float>(CF_SAFETY_FILTER_U_MIN[axis]);
    const float accelerationMax = static_cast<float>(CF_SAFETY_FILTER_U_MAX[axis]);
    const float cbfLower = -cbfK1 * cbfK2 * (position[axis] - positionMin)
                           - (cbfK1 + cbfK2) * velocity[axis];
    const float cbfUpper = cbfK1 * cbfK2 * (positionMax - position[axis])
                           - (cbfK1 + cbfK2) * velocity[axis];
    float lower = fmaxf(accelerationMin, cbfLower);
    float upper = fminf(accelerationMax, cbfUpper);
    if (lower > upper) lower = upper = 0.5f * (lower + upper);
    safe[axis] = clampFloat(nominal[axis], lower, upper);
  }
  solvedAx = safe[0];
  solvedAy = safe[1];
  solvedAz = safe[2];
  solveUs = 0;
  solveIter = 0;
  solveStatus = 1;
}

#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
void consumeOsqpResult(const state_t *state)
{
  terminalOsqpResult_t result;
  if (!terminalOsqpLatest(&result) || result.sequence == osqpSequence) {
    return;
  }
  osqpSequence = result.sequence;
  osqpSolveUs = result.solveUs;
  solveUs = result.solveUs;
  solveIter = result.iterations;
  osqpStatus = static_cast<int8_t>(result.status);
  primalResidual = result.primalResidual;
  dualResidual = result.dualResidual;
  solveStatus = result.solved;
  if (result.solved && has_filtered_setpoint) {
    float command[3];
    const float blend = residualEnable ? clampFloat(residualBlend, 0.0f, 1.0f) : 0.0f;
    residualRlsInverse(&residualRls, result.acceleration,
                       blend, command);
    solvedAx = clampFloat(command[0], CF_SAFETY_FILTER_U_MIN[0], CF_SAFETY_FILTER_U_MAX[0]);
    solvedAy = clampFloat(command[1], CF_SAFETY_FILTER_U_MIN[1], CF_SAFETY_FILTER_U_MAX[1]);
    solvedAz = clampFloat(command[2], CF_SAFETY_FILTER_U_MIN[2], CF_SAFETY_FILTER_U_MAX[2]);
    const float applied[3] = {solvedAx, solvedAy, solvedAz};
    residualRlsRecordCommand(&residualRls, applied);
    filtered_setpoint.acceleration.x = solvedAx;
    filtered_setpoint.acceleration.y = solvedAy;
    filtered_setpoint.acceleration.z = solvedAz;
  } else if (has_filtered_setpoint) {
    filtered_setpoint.acceleration.x = clampFloat(
      -state->velocity.x / sfLookahead,
      CF_SAFETY_FILTER_U_MIN[0], CF_SAFETY_FILTER_U_MAX[0]);
    filtered_setpoint.acceleration.y = clampFloat(
      -state->velocity.y / sfLookahead,
      CF_SAFETY_FILTER_U_MIN[1], CF_SAFETY_FILTER_U_MAX[1]);
    filtered_setpoint.acceleration.z = clampFloat(
      -state->velocity.z / sfLookahead,
      CF_SAFETY_FILTER_U_MIN[2], CF_SAFETY_FILTER_U_MAX[2]);
  }
}
#endif

} // namespace

extern "C" void controllerSafetyFilterInit(void)
{
  controllerMellingerFirmwareInit();
#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
  terminalOsqpInit();
  terminalOsqpStart();
  residualRlsInit(&residualRls, 20.0f);
#else
  initSolver();
#endif
}

extern "C" bool controllerSafetyFilterTest(void)
{
#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
  return true;
#else
  return initSolver() == 0;
#endif
}

// The selected backend returns u0; Mellinger converts it to thrust/attitude.
extern "C" void controllerSafetyFilter(control_t *control, const setpoint_t *setpoint,
                                          const sensorData_t *sensors,
                                          const state_t *state,
                                          const stabilizerStep_t stabilizerStep)
{
#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
  const bool solverUnavailable = false;
#else
  const bool solverUnavailable = initSolver() != 0;
#endif
  if (solverUnavailable || !sfEnable) {
    has_filtered_setpoint = false;
    nominalPositionRefValid = false;
    sfActive = 0;
    sfMode = 0;
    printSafetyFilterMode(sfMode);
    controllerMellingerFirmware(control, setpoint, sensors, state, stabilizerStep);
    return;
  }

  const bool velocitySetpoint = isVelocitySafetyFilterSetpoint(setpoint);
  const bool fullStateSetpoint = isFullStateSafetyFilterSetpoint(setpoint);
  if (sfEnable && (velocitySetpoint || fullStateSetpoint)) {
    float vx = 0.0f;
    float vy = 0.0f;
    if (velocitySetpoint) {
      bodyVelocityToWorld(setpoint, state, &vx, &vy);
      const float horizontalSpeed = sqrtf(vx * vx + vy * vy);
      if (horizontalSpeed > sfMaxVelocity) {
        const float scale = sfMaxVelocity / horizontalSpeed;
        vx *= scale;
        vy *= scale;
      }
    }
    sfActive = 1;
    sfMode = 2;
    printSafetyFilterMode(sfMode);

    if (!has_filtered_setpoint || RATE_DO_EXECUTE(20, stabilizerStep)) {
#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
      const float velocity[3] = {state->velocity.x, state->velocity.y, state->velocity.z};
      if (residualEnable && !residualWasEnabled) residualRlsInit(&residualRls, 20.0f);
      if (residualEnable) residualRlsObserve(
        &residualRls, velocity, 0.05f, clampFloat(residualMeasurementBlend, 0.05f, 1.0f));
      residualWasEnabled = residualEnable;
      residualGainX = residualRls.theta[0][1] / (1.0f - residualRls.theta[0][0]);
      residualGainY = residualRls.theta[1][1] / (1.0f - residualRls.theta[1][0]);
      residualGainZ = residualRls.theta[2][1] / (1.0f - residualRls.theta[2][0]);
      residualBiasX = residualRls.theta[0][2];
      residualBiasY = residualRls.theta[1][2];
      residualBiasZ = residualRls.theta[2][2];
      residualAlphaX = residualRls.theta[0][0];
      residualAlphaY = residualRls.theta[1][0];
      residualAlphaZ = residualRls.theta[2][0];
      residualBetaX = residualRls.theta[0][1];
      residualBetaY = residualRls.theta[1][1];
      residualBetaZ = residualRls.theta[2][1];
#endif
      setpoint_t nominalSetpoint = *setpoint;
      if (velocitySetpoint) {
        if (!nominalPositionRefValid) {
          nominalPositionRefX = state->position.x;
          nominalPositionRefY = state->position.y;
          nominalYawRef = state->attitude.yaw;
          nominalPositionRefValid = true;
        }
        nominalPositionRefX = clampFloat(nominalPositionRefX + 0.05f * vx,
                                         CF_SAFETY_FILTER_X_MIN[0], CF_SAFETY_FILTER_X_MAX[0]);
        nominalPositionRefY = clampFloat(nominalPositionRefY + 0.05f * vy,
                                         CF_SAFETY_FILTER_X_MIN[1], CF_SAFETY_FILTER_X_MAX[1]);
        nominalYawRef += 0.05f * setpoint->attitudeRate.yaw;
        if (nominalYawRef > 180.0f) {
          nominalYawRef -= 360.0f;
        } else if (nominalYawRef < -180.0f) {
          nominalYawRef += 360.0f;
        }
        nominalSetpoint.mode.x = modeAbs;
        nominalSetpoint.mode.y = modeAbs;
        nominalSetpoint.position.x = nominalPositionRefX;
        nominalSetpoint.position.y = nominalPositionRefY;
        nominalSetpoint.velocity.x = vx;
        nominalSetpoint.velocity.y = vy;
        nominalSetpoint.velocity.z = clampFloat(
          nominalSetpoint.velocity.z, -sfMaxVelocity, sfMaxVelocity);
        nominalSetpoint.velocity_body = false;
      } else {
        nominalPositionRefValid = false;
        nominalPositionRefX = nominalSetpoint.position.x;
        nominalPositionRefY = nominalSetpoint.position.y;
      }
      Axis3f nominalAcceleration;
      controllerMellingerFirmwareNominalAcceleration(&nominalAcceleration, &nominalSetpoint, state, 0.05f);
      nominalAx = clampFloat(nominalAcceleration.x,
                             static_cast<float>(CF_SAFETY_FILTER_U_MIN[0]),
                             static_cast<float>(CF_SAFETY_FILTER_U_MAX[0]));
      nominalAy = clampFloat(nominalAcceleration.y,
                             static_cast<float>(CF_SAFETY_FILTER_U_MIN[1]),
                             static_cast<float>(CF_SAFETY_FILTER_U_MAX[1]));
      nominalAz = clampFloat(nominalAcceleration.z,
                             static_cast<float>(CF_SAFETY_FILTER_U_MIN[2]),
                             static_cast<float>(CF_SAFETY_FILTER_U_MAX[2]));

#ifndef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
      resetSolverBounds(state);
      fillSafetyFilterReferenceHorizon(state, nominalAx, nominalAy, nominalAz, reference_states, reference_inputs);
#endif
      if (sfType == 1) {
        runCbfSafetyFilter(state);
      } else {
        runSafetySolver(setpoint, state);
      }

      filtered_setpoint = *setpoint;
      filtered_setpoint.mode.x = modeAbs;
      filtered_setpoint.mode.y = modeAbs;
      filtered_setpoint.mode.z = modeAbs;
      if (velocitySetpoint) {
        filtered_setpoint.mode.yaw = modeAbs;
        filtered_setpoint.attitude.yaw = nominalYawRef;
      }
      filtered_setpoint.velocity_body = false;
      filtered_setpoint.position = state->position;
      filtered_setpoint.velocity = state->velocity;
      if (solveStatus) {
        filtered_setpoint.acceleration.x = solvedAx;
        filtered_setpoint.acceleration.y = solvedAy;
        filtered_setpoint.acceleration.z = solvedAz;
      } else {
        const float lookahead = sfLookahead;
        filtered_setpoint.acceleration.x = clampFloat(
          (recoveryVelocity(state->position.x, CF_SAFETY_FILTER_X_MIN[0], CF_SAFETY_FILTER_X_MAX[0]) - state->velocity.x) / lookahead,
          CF_SAFETY_FILTER_U_MIN[0], CF_SAFETY_FILTER_U_MAX[0]);
        filtered_setpoint.acceleration.y = clampFloat(
          (recoveryVelocity(state->position.y, CF_SAFETY_FILTER_X_MIN[1], CF_SAFETY_FILTER_X_MAX[1]) - state->velocity.y) / lookahead,
          CF_SAFETY_FILTER_U_MIN[1], CF_SAFETY_FILTER_U_MAX[1]);
        filtered_setpoint.acceleration.z = clampFloat(
          (recoveryVelocity(state->position.z, CF_SAFETY_FILTER_X_MIN[2], CF_SAFETY_FILTER_X_MAX[2]) - state->velocity.z) / lookahead,
          CF_SAFETY_FILTER_U_MIN[2], CF_SAFETY_FILTER_U_MAX[2]);
      }
      has_filtered_setpoint = true;
      const float deltaAx = filtered_setpoint.acceleration.x - nominalAx;
      const float deltaAy = filtered_setpoint.acceleration.y - nominalAy;
      const float deltaAz = filtered_setpoint.acceleration.z - nominalAz;
      if (!solveStatus || sqrtf(deltaAx * deltaAx + deltaAy * deltaAy + deltaAz * deltaAz) > 0.05f) {
        sfInterventions++;
      }
    }

#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
    if (sfType == 2) consumeOsqpResult(state);
#endif
    filtered_setpoint.position = state->position;
    filtered_setpoint.velocity = state->velocity;
    controllerMellingerFirmwareFromAcceleration(control, &filtered_setpoint, sensors, state, stabilizerStep);
    return;
  }

  sfActive = 0;
  sfMode = 0;
  has_filtered_setpoint = false;
  nominalPositionRefValid = false;
  sfMode = 3;
  sfUnsupported++;
  printSafetyFilterMode(sfMode);
  controllerMellingerFirmware(control, setpoint, sensors, state, stabilizerStep);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

PARAM_GROUP_START(safeFilt)
PARAM_ADD(PARAM_UINT8, sfEnable, &sfEnable)
PARAM_ADD(PARAM_UINT8, sfType, &sfType)
PARAM_ADD(PARAM_FLOAT, cbfK1, &cbfK1)
PARAM_ADD(PARAM_FLOAT, cbfK2, &cbfK2)
PARAM_ADD(PARAM_FLOAT, sfLookahead, &sfLookahead)
PARAM_ADD(PARAM_FLOAT, maxVelocity, &sfMaxVelocity)
#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
PARAM_ADD(PARAM_UINT8, residual, &residualEnable)
PARAM_ADD(PARAM_FLOAT, resBlend, &residualBlend)
#endif
PARAM_GROUP_STOP(safeFilt)

LOG_GROUP_START(safeFilt)
LOG_ADD(LOG_UINT8, sfActive, &sfActive)
LOG_ADD(LOG_UINT8, sfMode, &sfMode)
LOG_ADD(LOG_UINT32, sfInterv, &sfInterventions)
LOG_ADD(LOG_UINT32, sfUnsup, &sfUnsupported)
LOG_ADD(LOG_UINT32, solveUs, &solveUs)
LOG_ADD(LOG_UINT16, iter, &solveIter)
LOG_ADD(LOG_UINT8, solved, &solveStatus)
LOG_ADD(LOG_INT8, osqpStatus, &osqpStatus)
LOG_ADD(LOG_UINT32, osqpUs, &osqpSolveUs)
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
#ifdef CONFIG_CONTROLLER_SAFETY_FILTER_OSQP
LOG_ADD(LOG_UINT8, residualOn, &residualEnable)
LOG_ADD(LOG_FLOAT, resGainX, &residualGainX)
LOG_ADD(LOG_FLOAT, resGainY, &residualGainY)
LOG_ADD(LOG_FLOAT, resGainZ, &residualGainZ)
LOG_ADD(LOG_FLOAT, resBiasX, &residualBiasX)
LOG_ADD(LOG_FLOAT, resBiasY, &residualBiasY)
LOG_ADD(LOG_FLOAT, resBiasZ, &residualBiasZ)
LOG_ADD(LOG_FLOAT, resAlphaX, &residualAlphaX)
LOG_ADD(LOG_FLOAT, resAlphaY, &residualAlphaY)
LOG_ADD(LOG_FLOAT, resAlphaZ, &residualAlphaZ)
LOG_ADD(LOG_FLOAT, resBetaX, &residualBetaX)
LOG_ADD(LOG_FLOAT, resBetaY, &residualBetaY)
LOG_ADD(LOG_FLOAT, resBetaZ, &residualBetaZ)
#endif
LOG_ADD(LOG_FLOAT, refX, &nominalPositionRefX)
LOG_ADD(LOG_FLOAT, refY, &nominalPositionRefY)
LOG_ADD(LOG_FLOAT, refYaw, &nominalYawRef)
LOG_GROUP_STOP(safeFilt)

#pragma GCC diagnostic pop
