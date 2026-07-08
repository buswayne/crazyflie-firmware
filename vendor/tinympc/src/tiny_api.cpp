#include "tiny_api.hpp"
#include "tiny_api_constants.hpp"

#include <cstdint>
#include <cstdlib>
#include <new>


// ponytail: -nostdlib means libstdc++ isn't linked, so plain `new`/`delete`
// (used below to allocate TinySolver's members) have no backing implementation.
// Route them through the libc allocator that's already linked in.
void* operator new(std::size_t size) noexcept { return std::malloc(size); }
void operator delete(void* ptr) noexcept { std::free(ptr); }
void operator delete(void* ptr, std::size_t) noexcept { std::free(ptr); }

// ponytail: TinyCache/TinyWorkspace now hold fixed-size Eigen matrices, over-aligned
// past malloc's default guarantee, so `new` picks the aligned overload instead of the
// plain one above. malloc() doesn't promise that alignment, so pad and round up by
// hand (classic aligned-alloc-via-malloc trick) and stash the real pointer just
// before the returned block so delete can find it.
void* operator new(std::size_t size, std::align_val_t align) noexcept {
  const std::size_t a = static_cast<std::size_t>(align);
  void* raw = std::malloc(size + a - 1 + sizeof(void*));
  if (!raw) {
    return nullptr;
  }
  const std::uintptr_t base = reinterpret_cast<std::uintptr_t>(raw) + sizeof(void*);
  const std::uintptr_t aligned = (base + a - 1) & ~(a - 1);
  reinterpret_cast<void**>(aligned)[-1] = raw;
  return reinterpret_cast<void*>(aligned);
}
void operator delete(void* ptr, std::align_val_t) noexcept {
  if (ptr) {
    std::free(reinterpret_cast<void**>(ptr)[-1]);
  }
}
void operator delete(void* ptr, std::size_t, std::align_val_t align) noexcept {
  operator delete(ptr, align);
}

// ponytail: libsupc++ isn't linked either, so the guard variable runtime for a
// function-local static (Eigen's manage_caching_sizes) has no implementation.
// Single solver, single task calls into the solve path here, so a plain flag
// is enough; add real locking if this ever runs from more than one task.
extern "C" int __cxa_guard_acquire(int* guard) { return *guard == 0; }
extern "C" void __cxa_guard_release(int* guard) { *guard = 1; }
extern "C" void __cxa_guard_abort(int* guard) { (void)guard; }

#ifdef __cplusplus
extern "C" {
#endif

using namespace Eigen;

static TinySolution solution_storage;
static TinyCache cache_storage;
static TinySettings settings_storage;
static TinyWorkspace work_storage;
static TinySolver solver_storage;

int tiny_setup(TinySolver** solverp,
                const tinyMatrixNxNx &Adyn, const tinyMatrixNxNu &Bdyn,
                const tinyMatrixNxNx &Q, const tinyMatrixNuNu &R,
                tinytype rho, int nx, int nu, int N,
                const tinyMatrixNxNh &x_min, const tinyMatrixNxNh &x_max,
                const tinyMatrixNuNhm1 &u_min, const tinyMatrixNuNhm1 &u_max,
                int verbose) {
    (void)verbose;

    TinySolution *solution = &solution_storage;
    TinyCache *cache = &cache_storage;
    TinySettings *settings = &settings_storage;
    TinyWorkspace *work = &work_storage;
    TinySolver *solver = &solver_storage;

    solver->solution = solution;
    solver->cache = cache;
    solver->settings = settings;
    solver->work = work;

    *solverp = solver;

    // Initialize solution
    solution->iter = 0;
    solution->solved = 0;
    solution->x.setZero();
    solution->u.setZero();

    // Initialize settings
    tiny_set_default_settings(settings);

    // Initialize workspace
    // ponytail: nx/nu/N are still passed in and stored at runtime, but the
    // actual matrices below are fixed-size (NSTATES/NINPUTS/NHORIZON in
    // types.hpp) — a mismatch here would fail to compile, not silently misbehave.
    work->nx = nx;
    work->nu = nu;
    work->N = N;

    work->x.setZero();
    work->u.setZero();

    work->q.setZero();
    work->r.setZero();

    work->p.setZero();
    work->d.setZero();

    work->v.setZero();
    work->vnew.setZero();
    work->z.setZero();
    work->znew.setZero();

    work->g.setZero();
    work->y.setZero();

    work->Q = (Q + rho * tinyMatrixNxNx::Identity()).diagonal();
    work->R = (R + rho * tinyMatrixNuNu::Identity()).diagonal();
    work->Adyn = Adyn;
    work->Bdyn = Bdyn;

    work->x_min = x_min;
    work->x_max = x_max;
    work->u_min = u_min;
    work->u_max = u_max;

    work->Xref.setZero();
    work->Uref.setZero();

    work->Qu.setZero();

    work->primal_residual_state = 0;
    work->primal_residual_input = 0;
    work->dual_residual_state = 0;
    work->dual_residual_input = 0;
    work->status = 0;
    work->iter = 0;

    // Initialize cache
    int status = tiny_precompute_and_set_cache(cache, Adyn, Bdyn, work->Q.asDiagonal(), work->R.asDiagonal(), nx, nu, rho, verbose);
    if (status) {
        return status;
    }

    return 0;
}

int tiny_precompute_and_set_cache(TinyCache *cache,
                                  const tinyMatrixNxNx &Adyn, const tinyMatrixNxNu &Bdyn,
                                  const tinyMatrixNxNx &Q, const tinyMatrixNuNu &R,
                                  int nx, int nu, tinytype rho, int verbose) {
    (void)nx;
    (void)nu;
    (void)verbose; // ponytail: verbose printing dropped, no iostream on this target

    if (!cache) {
        return 1;
    }

    // Update by adding rho * identity matrix to Q, R
    tinyMatrixNxNx Q1 = Q + rho * tinyMatrixNxNx::Identity();
    tinyMatrixNuNu R1 = R + rho * tinyMatrixNuNu::Identity();

    // Riccati recursion to get Kinf, Pinf
    tinyMatrixNuNx Ktp1 = tinyMatrixNuNx::Zero();
    tinyMatrixNxNx Ptp1 = rho * tinyMatrixNxNx::Identity();
    tinyMatrixNuNx Kinf = tinyMatrixNuNx::Zero();
    tinyMatrixNxNx Pinf = tinyMatrixNxNx::Zero();

    for (int i = 0; i < 1000; i++)
    {
        Kinf = (R1 + Bdyn.transpose() * Ptp1 * Bdyn).inverse() * Bdyn.transpose() * Ptp1 * Adyn;
        Pinf = Q1 + Adyn.transpose() * Ptp1 * (Adyn - Bdyn * Kinf);
        // if Kinf converges, break
        if ((Kinf - Ktp1).cwiseAbs().maxCoeff() < 1e-5f)
        {
            break;
        }
        Ktp1 = Kinf;
        Ptp1 = Pinf;
    }

    // Compute cached matrices
    cache->rho = rho;
    cache->Kinf = Kinf;
    cache->Pinf = Pinf;
    cache->Quu_inv = (R1 + Bdyn.transpose() * Pinf * Bdyn).inverse();
    cache->AmBKt = (Adyn - Bdyn * Kinf).transpose();

    return 0; // return success
}


int tiny_solve(TinySolver* solver) {
    return solve(solver);
}

int tiny_update_settings(TinySettings* settings, tinytype abs_pri_tol, tinytype abs_dua_tol,
                    int max_iter, int check_termination,
                    int en_state_bound, int en_input_bound) {
    if (!settings) {
        return 1;
    }
    settings->abs_pri_tol = abs_pri_tol;
    settings->abs_dua_tol = abs_dua_tol;
    settings->max_iter = max_iter;
    settings->check_termination = check_termination;
    settings->en_state_bound = en_state_bound;
    settings->en_input_bound = en_input_bound;
    return 0;
}

int tiny_set_default_settings(TinySettings* settings) {
    if (!settings) {
        return 1;
    }
    settings->abs_pri_tol = TINY_DEFAULT_ABS_PRI_TOL;
    settings->abs_dua_tol = TINY_DEFAULT_ABS_DUA_TOL;
    settings->max_iter = TINY_DEFAULT_MAX_ITER;
    settings->check_termination = TINY_DEFAULT_CHECK_TERMINATION;
    settings->en_state_bound = TINY_DEFAULT_EN_STATE_BOUND;
    settings->en_input_bound = TINY_DEFAULT_EN_INPUT_BOUND;
    return 0;
}

int tiny_set_x0(TinySolver* solver, const tinyVectorNx &x0) {
    if (!solver) {
        return 1;
    }
    solver->work->x.col(0) = x0;
    return 0;
}

int tiny_set_x_ref(TinySolver* solver, const tinyMatrixNxNh &x_ref) {
    if (!solver) {
        return 1;
    }
    solver->work->Xref = x_ref;
    return 0;
}

int tiny_set_u_ref(TinySolver* solver, const tinyMatrixNuNhm1 &u_ref) {
    if (!solver) {
        return 1;
    }
    solver->work->Uref = u_ref;
    return 0;
}

#ifdef __cplusplus
}
#endif
