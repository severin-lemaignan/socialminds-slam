# ADR 0011: Visualization/debug stack — rerun for live & progressive 3D, matplotlib for quick 2D

- **Status:** accepted
- **Date:** 2026-06-10
- **Deciders:** Séverin Lemaignan

## Context

Debugging the 3D pipeline (ADR 0010) needs to *see* the TSDF map, and the requirement is
explicitly **live** — or at minimum a replay of the map being built progressively, not a
final-state screenshot. RGB-D streams arrive next, which adds images, depth and
per-camera transforms to the same need. The existing `python -m harness.viz` (matplotlib)
is a top-down 2D scan debugger — good at what it does, structurally wrong for 3D/streams.

## Decision

**[rerun](https://rerun.io) is the 3D/live visualization stack.** The engine logs
directly from Rust (`slam-replay --rerun MODE`, feature-gated behind `viz` so the heavy
SDK never burdens default builds or CI):

- `--rerun spawn` — stream to a live viewer **while the engine runs**;
- `--rerun save:run.rrd` — record the identical stream; opening the file gives
  timeline-scrubbable **progressive replay** (map chunks accumulate over sensor time);
- `--rerun connect` — attach to an already-running viewer.

Logged per scan on the `sensor_time` timeline: the current sweep, the growing estimate
trajectory, ground truth (static), accumulating map point chunks; at the end, the TSDF
surface coloured by height. RGB-D entities (images, depth, camera frusta) slot into the
same scheme later — rerun is built for exactly that. A `--rerun` run is a debugging run,
not a benchmark (logging adds overhead outside the per-event latency clock but inside
wall time).

Complementary, not replaced:

- `python -m harness.viz` (matplotlib) stays — instant, dependency-light 2D scan/
  trajectory inspection;
- `slam-replay --map-out FILE` dumps the raw TSDF voxels (tiny versioned binary:
  `STSD`, voxel size, `(ix iy iz tsdf weight)` records) for headless analysis and the
  future backend conformance suite — viewer-independent ground truth.

The viewer binary comes from `pip install rerun-sdk` (already the eval venv's ecosystem)
or `cargo install rerun-cli`; the engine-side SDK is pinned in `Cargo.toml`.

## Consequences

- **Easier:** map smear, ghosts, registration failures and submap hand-overs become
  visible as they happen; RGB-D debugging lands on an already-working stream; recorded
  `.rrd` files make bug reports replayable.
- **Harder:** a large optional dependency (~2 min extra clean build with `--features
  viz`); the viz code path must not bit-rot — it compiles in CI via a feature-enabled
  check build, and the stub keeps the CLI stable without the feature.
- **Revisit when:** rerun's API churns painfully across upgrades (it is pre-1.0), or the
  robot needs an on-board, network-served dashboard (rerun's web viewer is the likely
  answer there too).

## Alternatives considered

- **Open3D / PyVista viewer over `--map-out` dumps.** Interactive 3D, but post-hoc only —
  fails the "live / progressive" requirement without inventing a streaming layer that
  rerun already is.
- **RViz2.** The eventual ROS-side answer for operators, but it drags ROS into the
  middleware-independent core's debug loop, against the project's structure.
- **matplotlib 3D.** Unusable interactivity at point-cloud scale. Rejected.
- **Custom WebGL/wgpu viewer.** A project in itself; rejected as scope.
