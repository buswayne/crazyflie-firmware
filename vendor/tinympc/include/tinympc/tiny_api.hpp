#pragma once

#include "admm.hpp"

#ifdef __cplusplus
extern "C" {
#endif

int tiny_setup(TinySolver** solverp,
                const tinyMatrixNxNx &Adyn, const tinyMatrixNxNu &Bdyn,
                const tinyMatrixNxNx &Q, const tinyMatrixNuNu &R,
                tinytype rho, int nx, int nu, int N,
                const tinyMatrixNxNh &x_min, const tinyMatrixNxNh &x_max,
                const tinyMatrixNuNhm1 &u_min, const tinyMatrixNuNhm1 &u_max,
                int verbose);
int tiny_precompute_and_set_cache(TinyCache *cache,
                                    const tinyMatrixNxNx &Adyn, const tinyMatrixNxNu &Bdyn,
                                    const tinyMatrixNxNx &Q, const tinyMatrixNuNu &R,
                                    int nx, int nu, tinytype rho, int verbose);

int tiny_solve(TinySolver *solver);

int tiny_update_settings(TinySettings* settings,
                            tinytype abs_pri_tol, tinytype abs_dua_tol,
                            int max_iter, int check_termination,
                            int en_state_bound, int en_input_bound);
int tiny_set_default_settings(TinySettings* settings);

int tiny_set_x0(TinySolver* solver, const tinyVectorNx &x0);
int tiny_set_x_ref(TinySolver* solver, const tinyMatrixNxNh &x_ref);
int tiny_set_u_ref(TinySolver* solver, const tinyMatrixNuNhm1 &u_ref);

#ifdef __cplusplus
}
#endif
