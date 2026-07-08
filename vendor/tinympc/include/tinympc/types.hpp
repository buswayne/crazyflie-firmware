#pragma once


#include <Eigen.h>
// #include <Eigen/Core>
// #include <Eigen/LU>

using namespace Eigen;


#ifdef __cplusplus
extern "C" {
#endif

    // float, not double: target FPU (fpv4-sp-d16) is single-precision only,
    // double arithmetic gets soft-emulated and is what was blowing the solve
    // time budget. Codegen tooling wants double; on-target runtime wants float.
    typedef float tinytype;

    // ponytail: fixed at compile time (no heap) instead of Eigen::Dynamic, since
    // this vendored copy is single-purpose for the Crazyflie MPC controller, not
    // a generic multi-robot library. Must match CF_TINYMPC_STATE_DIM/INPUT_DIM/
    // HORIZON in crazyflie_tinympc_constants.h if that ever changes.
    static constexpr int NSTATES = 6;
    static constexpr int NINPUTS = 3;
    static constexpr int NHORIZON = 10;

    typedef Matrix<tinytype, NSTATES, 1> tinyVectorNx;
    typedef Matrix<tinytype, NINPUTS, 1> tinyVectorNu;
    typedef Matrix<tinytype, NSTATES, NSTATES> tinyMatrixNxNx;
    typedef Matrix<tinytype, NSTATES, NINPUTS> tinyMatrixNxNu;
    typedef Matrix<tinytype, NINPUTS, NSTATES> tinyMatrixNuNx;
    typedef Matrix<tinytype, NINPUTS, NINPUTS> tinyMatrixNuNu;
    typedef Matrix<tinytype, NSTATES, NHORIZON> tinyMatrixNxNh;
    typedef Matrix<tinytype, NINPUTS, NHORIZON - 1> tinyMatrixNuNhm1;

    /**
     * Solution
     */
    typedef struct {
        int iter;
        int solved;
        tinyMatrixNxNh x;    // nx x N
        tinyMatrixNuNhm1 u;  // nu x N-1
    } TinySolution;

    /**
     * Matrices that must be recomputed with changes in time step, rho
     */
    typedef struct {
        tinytype rho;
        tinyMatrixNuNx Kinf;    // nu x nx
        tinyMatrixNxNx Pinf;    // nx x nx
        tinyMatrixNuNu Quu_inv; // nu x nu
        tinyMatrixNxNx AmBKt;   // nx x nx
    } TinyCache;

    /**
     * User settings
     */
    typedef struct {
        tinytype abs_pri_tol;
        tinytype abs_dua_tol;
        int max_iter;
        int check_termination;
        int en_state_bound;
        int en_input_bound;
    } TinySettings;

    /**
     * Problem variables
     */
    typedef struct {
        int nx; // Number of states
        int nu; // Number of control inputs
        int N;  // Number of knotpoints in the horizon

        // State and input
        tinyMatrixNxNh x;      // nx x N
        tinyMatrixNuNhm1 u;    // nu x N-1

        // Linear control cost terms
        tinyMatrixNxNh q;      // nx x N
        tinyMatrixNuNhm1 r;    // nu x N-1

        // Linear Riccati backward pass terms
        tinyMatrixNxNh p;      // nx x N
        tinyMatrixNuNhm1 d;    // nu x N-1

        // Auxiliary variables
        tinyMatrixNxNh v;      // nx x N
        tinyMatrixNxNh vnew;   // nx x N
        tinyMatrixNuNhm1 z;    // nu x N-1
        tinyMatrixNuNhm1 znew; // nu x N-1

        // Dual variables
        tinyMatrixNxNh g;      // nx x N
        tinyMatrixNuNhm1 y;    // nu x N-1

        // Q, R, A, B given by user
        tinyVectorNx Q;        // nx x 1
        tinyVectorNu R;        // nu x 1
        tinyMatrixNxNx Adyn;   // nx x nx
        tinyMatrixNxNu Bdyn;   // nx x nu

        // State and input bounds
        tinyMatrixNxNh x_min;  // nx x N
        tinyMatrixNxNh x_max;  // nx x N
        tinyMatrixNuNhm1 u_min; // nu x N-1
        tinyMatrixNuNhm1 u_max; // nu x N-1

        // Reference trajectory to track for one horizon
        tinyMatrixNxNh Xref;   // nx x N
        tinyMatrixNuNhm1 Uref; // nu x N-1

        // Temporaries
        tinyVectorNu Qu;       // nu x 1

        // Variables for keeping track of solve status
        tinytype primal_residual_state;
        tinytype primal_residual_input;
        tinytype dual_residual_state;
        tinytype dual_residual_input;
        int status;
        int iter;
    } TinyWorkspace;

    /**
     * Main TinyMPC solver structure that holds all information.
     */
    typedef struct {
        TinySolution *solution; // Solution
        TinySettings *settings; // Problem settings
        TinyCache *cache;       // Problem cache
        TinyWorkspace *work;    // Solver workspace
    } TinySolver;

#ifdef __cplusplus
}
#endif
