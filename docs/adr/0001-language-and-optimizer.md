# ADR 0001: Rust core with a wrapped GTSAM optimizer

- **Status:** accepted
- **Date:** 2026-06-09
- **Deciders:** Séverin Lemaignan

## Context

We need an implementation language for a heavily multi-threaded, soft-real-time 3D
SLAM engine that will be embedded in a ROS 2 stack with a hard requirement for
**efficient, ideally zero-copy** data bindings to Python and C++.

Findings from the SotA survey:

- Rust's numerics (nalgebra, faer for sparse Cholesky), spatial (kiddo), concurrency
  (rayon, crossbeam), GPU (cudarc), and inference (ONNX Runtime via `ort`) stories are
  production-viable in 2026.
- Rust is an excellent **zero-copy FFI hub**: a contiguous buffer can be lent to C++
  via `cxx::Slice` and to Python via the buffer protocol (`pyo3`/`pyo3-arrow`) with no
  copy — this directly serves the binding requirement, and more cleanly than a C++ core.
- The **one genuinely immature area** is mature sparse factor-graph optimisation with
  IMU preintegration, marginalisation, and incremental smoothing (iSAM2). Pure-Rust
  crates (`factrs` v0.3, `apex-solver` v1.3) are promising but young: `factrs` is
  single-threaded and lacks preintegration/marginalisation; `apex-solver` is unproven.
  GTSAM / Ceres represent ~15 years of hardening and are the backbone of LIO-SAM, GLIM,
  RTAB-Map, etc.

Loop closure / global consistency is our **top** requirement, and it lives in exactly
the part of the stack where Rust is weakest. We must not gamble it on an immature crate.

## Decision

1. **Rust is the core language** for the engine: pipeline orchestration, data structures,
   concurrency, sensor fusion bus, map structures, and the ROS 2 / Python / C++ bindings.
2. **The factor-graph backend wraps GTSAM** (iSAM2, IMU preintegration, robust kernels)
   behind a **thin C-ABI shim bridged with `cxx`**. The Rust side owns the graph
   topology and data; GTSAM is called as a solver.
3. Rust is the **zero-copy hub**: `cxx::Slice`/opaque types to C++, buffer protocol to
   Python. No serialisation on the hot intra-host path.
4. We **isolate the C++ surface** behind one crate (`slam-gtsam-sys` + safe `slam-backend`
   wrapper) so it can be swapped for a pure-Rust optimiser later with no API churn upstream.

## Consequences

- **Easier:** memory/thread safety on a massively concurrent codebase (data races become
  compile errors); clean zero-copy bindings; modern tooling (cargo, one build for the core).
- **Harder:** the build now includes C++ (GTSAM + Eigen) for the backend. We accept a
  CMake/`cxx` dependency and the cross-compilation cost on the robot target.
- **Risk accepted:** GTSAM API churn and build complexity. Mitigation: the C-ABI shim is
  narrow and fully owned by us; the optimiser is the *only* mandatory C++ dependency.
- **No PCL-for-Rust:** point-cloud primitives (voxel filters, normals, registration
  scaffolding) are implemented in-house or wrapped as needed; budgeted in the roadmap.
- **Real-time:** Rust gives no hard-RT guarantees for free (allocator, mutexes, scheduler).
  Determinism is a *deployment* discipline — thread pinning, arena/pre-allocation, lock-free
  hot paths, `SCHED_FIFO` — identical to what C++ would require.
- **Revisit when:** a pure-Rust optimiser demonstrably supports multi-threaded sparse
  incremental smoothing + IMU preintegration + marginalisation at our scale. At that point
  the wrapped GTSAM can be retired behind the unchanged `slam-backend` API.

## Alternatives considered

- **Pure Rust now (`factrs`/`apex-solver`):** cleaner single-language showcase, no C++
  build — but we would have to build and validate preintegration and incremental smoothing
  ourselves, against our most critical requirement, on immature single-threaded crates.
  Rejected as too risky for v1; revisitable later.
- **C++ core:** maximises library access (GTSAM, PCL, OpenCV natively) but loses the
  safety guarantees on a 24-thread codebase and makes the zero-copy *Rust* binding moot.
  The reverse hub (C++ lending to Rust) is messier. Rejected.
- **Pure Rust, validate `factrs` first:** considered; folded into the "revisit when" trigger
  above rather than blocking v1 on a de-risking spike.
