#pragma once

/* Generated TinyMPC problem constants. Arrays are row-major. */
#define CF_TINYMPC_STATE_DIM 6
#define CF_TINYMPC_INPUT_DIM 3
#define CF_TINYMPC_HORIZON 20
#define CF_TINYMPC_DT 0.01
#define CF_TINYMPC_MASS 0.0482
#define CF_TINYMPC_GRAVITY 9.8100000000000005
#define CF_TINYMPC_RHO 1.0
#define CF_TINYMPC_MAX_ITER 50
#define CF_TINYMPC_CHECK_TERMINATION 1
#define CF_TINYMPC_ABS_PRI_TOL 1.0e-3
#define CF_TINYMPC_ABS_DUA_TOL 1.0e-3
#define CF_TINYMPC_MIN_THRUST_N 0.0
#define CF_TINYMPC_MAX_THRUST_N 0.75

static const double CF_TINYMPC_A[36] = {1.0, 0.0, 0.0, 0.0099999997764825821, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0099999997764825821, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0099999997764825821, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
static const double CF_TINYMPC_B[18] = {4.9999998736893758e-05, 0.0, 0.0, 0.0, 4.9999998736893758e-05, 0.0, 0.0, 0.0, 4.9999998736893758e-05, 0.0099999997764825821, 0.0, 0.0, 0.0, 0.0099999997764825821, 0.0, 0.0, 0.0, 0.0099999997764825821};
static const double CF_TINYMPC_Q[36] = {8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 14.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0};
static const double CF_TINYMPC_R[9] = {0.34999999403953552, 0.0, 0.0, 0.0, 0.34999999403953552, 0.0, 0.0, 0.0, 0.5};
static const double CF_TINYMPC_QF[36] = {64.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 64.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 112.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 32.0};
static const double CF_TINYMPC_X_MIN[6] = {-1e+17, -1e+17, -1e+17, -1e+17, -1e+17, -1e+17};
static const double CF_TINYMPC_X_MAX[6] = {1e+17, 1e+17, 1e+17, 1e+17, 1e+17, 1e+17};
static const double CF_TINYMPC_U_MIN[3] = {-8.0, -8.0, -10.0};
static const double CF_TINYMPC_U_MAX[3] = {8.0, 8.0, 10.0};
