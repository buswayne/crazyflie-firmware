#pragma once

/* Safety filter problem constants. Arrays are row-major. */
#define CF_SAFETY_FILTER_STATE_DIM 6
#define CF_SAFETY_FILTER_INPUT_DIM 3
// MUST equal NHORIZON in vendor/tinympc/include/tinympc/types.hpp
// (static constexpr int NHORIZON = 10) -- that's a compile-time fixed size
// for the solver's Eigen matrices, completely separate from this macro.
// Raising this past 10 without also changing NHORIZON there makes
// tiny_setup() write past fixed-size buffers -- crashed hardware, motors
// spin then hard-fault silently.
#define CF_SAFETY_FILTER_HORIZON 10
#define CF_SAFETY_FILTER_DT 0.05
#define CF_SAFETY_FILTER_GRAVITY 9.8100000000000005
#define CF_SAFETY_FILTER_RHO 1.0
#define CF_SAFETY_FILTER_MAX_ITER 20
#define CF_SAFETY_FILTER_CHECK_TERMINATION 1
#define CF_SAFETY_FILTER_ABS_PRI_TOL 5.0e-3
#define CF_SAFETY_FILTER_ABS_DUA_TOL 5.0e-3
#define CF_SAFETY_FILTER_MIN_THRUST_N 0.0
#define CF_SAFETY_FILTER_MAX_THRUST_N 1.1772
#define CF_SAFETY_FILTER_RECOVERY_SPEED 0.25

static const double CF_SAFETY_FILTER_A[36] = {1.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
static const double CF_SAFETY_FILTER_B[18] = {0.00125, 0.0, 0.0, 0.0, 0.00125, 0.0, 0.0, 0.0, 0.00125, 0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05};
// Minimize acceleration correction only; state tracking is not part of the filter.
static const double CF_SAFETY_FILTER_Q[36] = {0.0};
static const double CF_SAFETY_FILTER_R[9] = {0.35, 0.0, 0.0, 0.0, 0.35, 0.0, 0.0, 0.0, 0.5};
static const double CF_SAFETY_FILTER_QF[36] = {0.0};
static const double CF_SAFETY_FILTER_X_MIN[6] = {-1.5, -1.5, 0.3, -1e+17, -1e+17, -1e+17};
static const double CF_SAFETY_FILTER_X_MAX[6] = {1.5, 1.5, 2.0, 1e+17, 1e+17, 1e+17};
static const double CF_SAFETY_FILTER_U_MIN[3] = {-9.81255722, -9.81255722, -5.0};
static const double CF_SAFETY_FILTER_U_MAX[3] = {9.81255722, 9.81255722, 5.0};
