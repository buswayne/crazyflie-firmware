#define DEBUG_MODULE "CTRLTMPC"
#include "debug.h"

#include <cmath>
#include <cstddef>

#include "controller_tinympc.h"
#include "crazyflie_tinympc_constants.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "stabilizer_types.h"
#include "tinympc/tiny_api.hpp"

namespace {

static constexpr float MASS_THRUST = 132000.0f;
static constexpr float KR_XY = 70000.0f;
static constexpr float KR_Z = 60000.0f;
static constexpr float KW_XY = 20000.0f;
static constexpr float KW_Z = 12000.0f;
static constexpr float KI_M_XY = 0.0f;
static constexpr float KI_M_Z = 500.0f;
static constexpr float KD_OMEGA_RP = 200.0f;
static constexpr float I_RANGE_M_XY = 1.0f;
static constexpr float I_RANGE_M_Z = 1500.0f;

TinySolver *solver = nullptr;
float i_error_m_x = 0.0f;
float i_error_m_y = 0.0f;
float i_error_m_z = 0.0f;
float prev_omega_roll = NAN;
float prev_omega_pitch = NAN;
float prev_setpoint_omega_roll = 0.0f;
float prev_setpoint_omega_pitch = 0.0f;

tinyMatrix rowMajorMatrix(const double *data, int rows, int cols)
{
  tinyMatrix out(rows, cols);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      out(row, col) = static_cast<tinytype>(data[row * cols + col]);
    }
  }
  return out;
}

tinyMatrix repeatedColumn(const double *data, int rows, int cols)
{
  tinyMatrix out(rows, cols);
  for (int col = 0; col < cols; ++col) {
    for (int row = 0; row < rows; ++row) {
      out(row, col) = static_cast<tinytype>(data[row]);
    }
  }
  return out;
}

void resetState()
{
  i_error_m_x = 0.0f;
  i_error_m_y = 0.0f;
  i_error_m_z = 0.0f;
  prev_omega_roll = NAN;
  prev_omega_pitch = NAN;
  prev_setpoint_omega_roll = 0.0f;
  prev_setpoint_omega_pitch = 0.0f;
}

int initSolver()
{
  if (solver != nullptr) {
    return 0;
  }

  tinyMatrix A = rowMajorMatrix(CF_TINYMPC_A, CF_TINYMPC_STATE_DIM, CF_TINYMPC_STATE_DIM);
  tinyMatrix B = rowMajorMatrix(CF_TINYMPC_B, CF_TINYMPC_STATE_DIM, CF_TINYMPC_INPUT_DIM);
  tinyMatrix Q = rowMajorMatrix(CF_TINYMPC_Q, CF_TINYMPC_STATE_DIM, CF_TINYMPC_STATE_DIM);
  tinyMatrix R = rowMajorMatrix(CF_TINYMPC_R, CF_TINYMPC_INPUT_DIM, CF_TINYMPC_INPUT_DIM);
  tinyMatrix x_min = repeatedColumn(CF_TINYMPC_X_MIN, CF_TINYMPC_STATE_DIM, CF_TINYMPC_HORIZON);
  tinyMatrix x_max = repeatedColumn(CF_TINYMPC_X_MAX, CF_TINYMPC_STATE_DIM, CF_TINYMPC_HORIZON);
  tinyMatrix u_min = repeatedColumn(CF_TINYMPC_U_MIN, CF_TINYMPC_INPUT_DIM, CF_TINYMPC_HORIZON - 1);
  tinyMatrix u_max = repeatedColumn(CF_TINYMPC_U_MAX, CF_TINYMPC_INPUT_DIM, CF_TINYMPC_HORIZON - 1);

  int status = tiny_setup(
    &solver,
    A,
    B,
    Q,
    R,
    static_cast<tinytype>(CF_TINYMPC_RHO),
    CF_TINYMPC_STATE_DIM,
    CF_TINYMPC_INPUT_DIM,
    CF_TINYMPC_HORIZON,
    x_min,
    x_max,
    u_min,
    u_max,
    0);
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

void fillReferenceHorizon(const setpoint_t *setpoint, tinyMatrix &x_ref, tinyMatrix &u_ref)
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

void quatToRotmatWxyz(const float q[4], float R[9])
{
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];

  R[0] = 1.0f - 2.0f * (y * y + z * z);
  R[1] = 2.0f * (x * y - w * z);
  R[2] = 2.0f * (x * z + w * y);
  R[3] = 2.0f * (x * y + w * z);
  R[4] = 1.0f - 2.0f * (x * x + z * z);
  R[5] = 2.0f * (y * z - w * x);
  R[6] = 2.0f * (x * z - w * y);
  R[7] = 2.0f * (y * z + w * x);
  R[8] = 1.0f - 2.0f * (x * x + y * y);
}

void cross3(const float a[3], const float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

void normalize3(float v[3])
{
  const float norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  const float inv_norm = 1.0f / std::fmax(norm, 1.0e-6f);
  v[0] *= inv_norm;
  v[1] *= inv_norm;
  v[2] *= inv_norm;
}

float desiredYawRad(const setpoint_t *setpoint, const state_t *state, float dt)
{
  if (setpoint->mode.yaw == modeVelocity) {
    return radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  }
  if (setpoint->mode.yaw == modeAbs) {
    return radians(setpoint->attitude.yaw);
  }
  return radians(state->attitude.yaw);
}

void accelerationToThrustAndER(
  const float accel_cmd[3],
  const state_t *state,
  float yaw_ref,
  float *thrust_newton,
  float eR[3])
{
  float R_act[9];
  float f_d[3];
  float z_b[3];
  float z_b_d[3];
  float x_c[3];
  float y_b_d[3];
  float x_b_d[3];
  float R_d[9];
  const float quat_wxyz[4] = {
    state->attitudeQuaternion.w,
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
  };

  f_d[0] = CF_TINYMPC_MASS * accel_cmd[0];
  f_d[1] = CF_TINYMPC_MASS * accel_cmd[1];
  f_d[2] = CF_TINYMPC_MASS * accel_cmd[2] + CF_TINYMPC_MASS * CF_TINYMPC_GRAVITY;

  quatToRotmatWxyz(quat_wxyz, R_act);
  z_b[0] = R_act[2];
  z_b[1] = R_act[5];
  z_b[2] = R_act[8];

  *thrust_newton = f_d[0] * z_b[0] + f_d[1] * z_b[1] + f_d[2] * z_b[2];
  *thrust_newton = clamp(*thrust_newton, CF_TINYMPC_MIN_THRUST_N, CF_TINYMPC_MAX_THRUST_N);

  z_b_d[0] = f_d[0];
  z_b_d[1] = f_d[1];
  z_b_d[2] = f_d[2];
  normalize3(z_b_d);

  x_c[0] = std::cos(yaw_ref);
  x_c[1] = std::sin(yaw_ref);
  x_c[2] = 0.0f;
  cross3(z_b_d, x_c, y_b_d);
  normalize3(y_b_d);
  cross3(y_b_d, z_b_d, x_b_d);

  R_d[0] = x_b_d[0];
  R_d[1] = y_b_d[0];
  R_d[2] = z_b_d[0];
  R_d[3] = x_b_d[1];
  R_d[4] = y_b_d[1];
  R_d[5] = z_b_d[1];
  R_d[6] = x_b_d[2];
  R_d[7] = y_b_d[2];
  R_d[8] = z_b_d[2];

  eR[0] = (R_d[6] * R_act[1] + R_d[7] * R_act[4] + R_d[8] * R_act[7]) -
          (R_act[6] * R_d[1] + R_act[7] * R_d[4] + R_act[8] * R_d[7]);
  eR[1] = (R_d[0] * R_act[2] + R_d[1] * R_act[5] + R_d[2] * R_act[8]) -
          (R_act[0] * R_d[2] + R_act[1] * R_d[5] + R_act[2] * R_d[8]);
  eR[2] = (R_d[3] * R_act[0] + R_d[4] * R_act[3] + R_d[5] * R_act[6]) -
          (R_act[3] * R_d[0] + R_act[4] * R_d[3] + R_act[5] * R_d[6]);

  // Match the legacy Crazyflie pitch convention used by Mellinger.
  eR[1] = -eR[1];
}

void applyLegacyAttitudeControl(
  control_t *control,
  const setpoint_t *setpoint,
  const sensorData_t *sensors,
  const float eR[3],
  float thrust_newton,
  float dt)
{
  const float state_omega_roll = radians(sensors->gyro.x);
  const float state_omega_pitch = -radians(sensors->gyro.y);
  const float state_omega_yaw = radians(sensors->gyro.z);
  const float setpoint_omega_roll = radians(setpoint->attitudeRate.roll);
  const float setpoint_omega_pitch = -radians(setpoint->attitudeRate.pitch);
  const float setpoint_omega_yaw = radians(setpoint->attitudeRate.yaw);
  float err_d_roll = 0.0f;
  float err_d_pitch = 0.0f;

  if (prev_omega_roll == prev_omega_roll) {
    err_d_roll = ((setpoint_omega_roll - prev_setpoint_omega_roll) - (state_omega_roll - prev_omega_roll)) / dt;
    err_d_pitch = ((setpoint_omega_pitch - prev_setpoint_omega_pitch) - (state_omega_pitch - prev_omega_pitch)) / dt;
  }

  prev_omega_roll = state_omega_roll;
  prev_omega_pitch = state_omega_pitch;
  prev_setpoint_omega_roll = setpoint_omega_roll;
  prev_setpoint_omega_pitch = setpoint_omega_pitch;

  i_error_m_x = clamp(i_error_m_x - eR[0] * dt, -I_RANGE_M_XY, I_RANGE_M_XY);
  i_error_m_y = clamp(i_error_m_y - eR[1] * dt, -I_RANGE_M_XY, I_RANGE_M_XY);
  i_error_m_z = clamp(i_error_m_z - eR[2] * dt, -I_RANGE_M_Z, I_RANGE_M_Z);

  const float ew_x = setpoint_omega_roll - state_omega_roll;
  const float ew_y = setpoint_omega_pitch - state_omega_pitch;
  const float ew_z = setpoint_omega_yaw - state_omega_yaw;
  const float M_x = -KR_XY * eR[0] + KW_XY * ew_x + KI_M_XY * i_error_m_x + KD_OMEGA_RP * err_d_roll;
  const float M_y = -KR_XY * eR[1] + KW_XY * ew_y + KI_M_XY * i_error_m_y + KD_OMEGA_RP * err_d_pitch;
  const float M_z = -KR_Z * eR[2] + KW_Z * ew_z + KI_M_Z * i_error_m_z;

  control->controlMode = controlModeLegacy;
  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = MASS_THRUST * thrust_newton;
  }

  if (control->thrust > 0.0f) {
    control->roll = static_cast<int16_t>(clamp(M_x, -32000.0f, 32000.0f));
    control->pitch = static_cast<int16_t>(clamp(M_y, -32000.0f, 32000.0f));
    control->yaw = static_cast<int16_t>(clamp(-M_z, -32000.0f, 32000.0f));
  } else {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    resetState();
  }
}

} // namespace

extern "C" void controllerTinyMPCFirmwareInit(void)
{
  resetState();
  const int status = initSolver();
  if (status != 0) {
    DEBUG_PRINT("TinyMPC init failed: %d\n", status);
  }
}

extern "C" bool controllerTinyMPCFirmwareTest(void)
{
  return initSolver() == 0;
}

extern "C" void controllerTinyMPCFirmware(control_t *control, const setpoint_t *setpoint,
                                          const sensorData_t *sensors,
                                          const state_t *state,
                                          const stabilizerStep_t stabilizerStep)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    return;
  }

  if (initSolver() != 0) {
    control->controlMode = controlModeLegacy;
    control->thrust = 0.0f;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    return;
  }

  tinyVector x0(CF_TINYMPC_STATE_DIM);
  x0(0) = static_cast<tinytype>(state->position.x);
  x0(1) = static_cast<tinytype>(state->position.y);
  x0(2) = static_cast<tinytype>(state->position.z);
  x0(3) = static_cast<tinytype>(state->velocity.x);
  x0(4) = static_cast<tinytype>(state->velocity.y);
  x0(5) = static_cast<tinytype>(state->velocity.z);

  tinyMatrix x_ref(CF_TINYMPC_STATE_DIM, CF_TINYMPC_HORIZON);
  tinyMatrix u_ref(CF_TINYMPC_INPUT_DIM, CF_TINYMPC_HORIZON - 1);
  fillReferenceHorizon(setpoint, x_ref, u_ref);

  tiny_set_x0(solver, x0);
  tiny_set_x_ref(solver, x_ref);
  tiny_set_u_ref(solver, u_ref);
  tiny_solve(solver);

  float accel_cmd[3] = {
    static_cast<float>(solver->solution->u(0, 0)),
    static_cast<float>(solver->solution->u(1, 0)),
    static_cast<float>(solver->solution->u(2, 0)),
  };
  float thrust_newton = 0.0f;
  float eR[3] = {0.0f, 0.0f, 0.0f};
  const float dt = 1.0f / static_cast<float>(ATTITUDE_RATE);
  accelerationToThrustAndER(accel_cmd, state, desiredYawRad(setpoint, state, dt), &thrust_newton, eR);
  applyLegacyAttitudeControl(control, setpoint, sensors, eR, thrust_newton, dt);
}
