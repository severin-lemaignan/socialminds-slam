# ADR 0004: Map is a trait with multiple backends — GPU TSDF/ESDF for navigation, OpenVDB for geometric reasoning

- **Status:** accepted
- **Date:** 2026-06-09
- **Deciders:** Séverin Lemaignan

## Context

We need a 3D map that supports robot navigation, and separately we intend to plug in
**reMap**, an OpenVDB-based voxel world model used for geometric reasoning. These have
different ideal representations:

- **Navigation** wants a dense metric distance field (ESDF) usable as a Nav2 costmap,
  updated in real time, with dynamic objects evicted. GPU TSDF/ESDF (nvblox-style) is the
  SotA fit: 1–2 cm resolution in real time in well under the 8 GB budget, with occupancy
  decay to remove transient people/chairs/doors.
- **Geometric reasoning (reMap)** is OpenVDB/VDB-grid native.

The 8 GB shared VRAM rules out Gaussian-Splatting / NeRF as the primary map (they want
13–24 GB and run < 2 fps). And per [ADR 0003](0003-gpu-optional-cpu-fallback.md), anything
GPU must have a CPU fallback for dev/CI.

We do not need to *pick one* representation. We need an abstraction that lets both coexist.

## Decision

1. Define a first-class **`Map` trait** (integrate poses + depth/points, query
   occupancy/SDF, export). The rest of the engine depends on the trait, not a backend.
2. Ship (over the roadmap) **multiple backends behind it**:
   - **GPU TSDF/ESDF** (nvblox-style) → the navigation/Nav2 costmap layer; with a **CPU
     voxel-TSDF fallback** for dev/CI ([ADR 0003](0003-gpu-optional-cpu-fallback.md)).
   - **OpenVDB layer** → interop with **reMap** for geometric reasoning. OpenVDB is wrapped
     from C++ (Rust-native VDB is read-only today).
3. Both layers consume the **same optimised poses** from the backend; both apply
   **occupancy decay** so dynamic objects do not persist.
4. The decision of which layer(s) are active is configuration, not architecture — the
   trait makes them composable.

## Consequences

- **Easier:** navigation and geometric-reasoning needs are served without forcing a single
  representation; reMap integration is a backend, not a rewrite.
- **Harder:** a second C++ dependency (OpenVDB) alongside GTSAM; two map backends to
  maintain plus the CPU TSDF fallback.
- **Deferred specifics:** exact TSDF/ESDF parameters, the reMap data interface, and whether
  the two layers share voxel storage are left to a follow-up ADR once the front-end exists.
- **Revisit when:** the front-end is producing poses and we can measure map quality — then
  finalise resolutions, the reMap interface, and storage sharing.

## Alternatives considered

- **Single representation (only TSDF, or only OpenVDB):** simpler but fails one of the two
  consumers — TSDF is awkward for reMap's reasoning; OpenVDB alone is not the real-time
  GPU navigation field. Rejected.
- **Gaussian-Splatting / NeRF map:** does not fit 8 GB shared at real time and is a
  rendering, not a collision/planning, representation. Deferred; revisit only with a bigger
  GPU and a digital-twin use case.
