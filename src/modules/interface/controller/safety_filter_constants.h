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
// Matches CF_MASS in platform_defaults_cf21bl.h (was 0.0482, 23% too
// heavy -> excess gravity feedforward thrust -> violent takeoff). Reweigh the
// actual craft (deck + battery) and adjust if it differs from this default.
#define CF_SAFETY_FILTER_MASS 0.045
#define CF_SAFETY_FILTER_GRAVITY 9.8100000000000005
#define CF_SAFETY_FILTER_RHO 1.0
#define CF_SAFETY_FILTER_MAX_ITER 8
#define CF_SAFETY_FILTER_CHECK_TERMINATION 1
#define CF_SAFETY_FILTER_ABS_PRI_TOL 5.0e-3
#define CF_SAFETY_FILTER_ABS_DUA_TOL 5.0e-3
#define CF_SAFETY_FILTER_MIN_THRUST_N 0.0
#define CF_SAFETY_FILTER_MAX_THRUST_N 0.75

static const double CF_SAFETY_FILTER_A[36] = {1.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
static const double CF_SAFETY_FILTER_B[18] = {0.00125, 0.0, 0.0, 0.0, 0.00125, 0.0, 0.0, 0.0, 0.00125, 0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05};
// Safety filter cost: keep the solved acceleration close to the nominal pilot
// acceleration. Q is intentionally tiny, only to regularize the rollout.
static const double CF_SAFETY_FILTER_Q[36] = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01};
static const double CF_SAFETY_FILTER_R[9] = {10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0};
static const double CF_SAFETY_FILTER_QF[36] = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01};
// No debug position box in flight. z_min=0.5 made arming command an
// instant climb target from the ground; re-enable bounds only after bench logs.
static const double CF_SAFETY_FILTER_X_MIN[6] = {-1e+17, -1e+17, -1e+17, -1e+17, -1e+17, -1e+17};
static const double CF_SAFETY_FILTER_X_MAX[6] = {1e+17, 1e+17, 1e+17, 1e+17, 1e+17, 1e+17};
static const double CF_SAFETY_FILTER_U_MIN[3] = {-8.0, -8.0, -10.0};
static const double CF_SAFETY_FILTER_U_MAX[3] = {8.0, 8.0, 10.0};
