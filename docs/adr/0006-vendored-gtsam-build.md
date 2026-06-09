# ADR 0006: Vendor GTSAM as a pinned submodule, built static and Boost-free by cargo

- **Status:** accepted
- **Date:** 2026-06-09
- **Deciders:** Séverin Lemaignan

## Context

ADR 0001 wraps GTSAM behind `slam-gtsam-sys` (cxx shim) + `slam-backend` (safe API). That
left open *how the C++ dependency is obtained and built*. Constraints:

- Dev/test/CI must work on any GPU-less, ROS-less machine (ADR 0003) — including GitLab
  shared runners — with no manual "install GTSAM first" step.
- GTSAM has no Debian/Ubuntu package; system installs would pin us to whatever a given
  machine has, and GTSAM's API moves between minor versions.
- The robot target cross-compiles; a self-contained build is much easier to ship.

## Decision

1. **GTSAM is a git submodule** at `third_party/gtsam`, **pinned to the `4.3a1` tag**
   (`shallow = true`). No system GTSAM is ever searched for.
2. **`slam-gtsam-sys/build.rs` builds it via CMake** (the `cmake` crate) as part of
   `cargo build`:
   - **static** (`GTSAM_FORCE_STATIC_LIB=ON`, `BUILD_SHARED_LIBS=OFF`) — Rust binaries
     carry no runtime C++ dependency;
   - **Boost-free** (`GTSAM_ENABLE_BOOST_SERIALIZATION=OFF`, `GTSAM_USE_BOOST_FEATURES=OFF`,
     available since GTSAM 4.3) — the only build prerequisites are a C++17 toolchain and
     CMake;
   - **CPU-only, library-only**: no TBB, no MKL, no tests/examples/python, bundled Eigen;
   - **always `Release`**, even under `cargo build` (a Debug GTSAM is unusably slow, and
     the shim — not GTSAM internals — is our debugging surface).
3. Escape hatch: `SLAM_GTSAM_PREFIX=<prefix>` skips the vendored build and links an
   existing install (e.g. a prebuilt CI image later).
4. CI checks out submodules (`GIT_SUBMODULE_STRATEGY: recursive`, depth 1) and installs
   CMake; the GitLab `target/` cache keeps the one-time GTSAM compile warm.

## Consequences

- **Easier:** `git clone --recursive && cargo test` is the entire onboarding; identical
  GTSAM everywhere (version skew can't produce unreproducible numbers); cross-compilation
  inherits cargo's story.
- **Harder:** first build pays a one-time multi-minute GTSAM compile per profile
  (mitigated by ccache locally and the `target/` cache in CI). Repo checkout grows by the
  (shallow) submodule.
- **Risk accepted:** `4.3a1` is a pre-release tag — but it is the line that supports
  Boost-free builds, and we pin a *tag*, not a branch, so it cannot move under us.
  Upgrades are deliberate, reviewed bumps of the submodule pointer.
- The shim links *whatever static archives GTSAM installs* (gtsam first, then bundled
  metis/cephes), discovered at build time — robust to 3rd-party-lib changes across GTSAM
  versions.

## Alternatives considered

- **Require a system/manually-built GTSAM:** breaks "clone and build", invites version
  skew between machines and CI. Rejected.
- **Download a source tarball in `build.rs`:** network access inside builds is fragile
  and unauditable; a submodule pin is content-addressed and visible in review. Rejected.
- **Commit the GTSAM source tree:** ~100 MB of foreign code in our history, painful
  upgrades. Rejected.
- **Prebuilt binaries / container image:** reproducible only as long as someone maintains
  the artifact; still needed a from-source path for the robot target. May *complement*
  this later via `SLAM_GTSAM_PREFIX`.
