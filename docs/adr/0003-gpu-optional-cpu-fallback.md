# ADR 0003: GPU is optional and feature-gated; CPU fallback is the default everywhere

- **Status:** accepted
- **Date:** 2026-06-09
- **Deciders:** Séverin Lemaignan

## Context

The robot has an RTX 5060 (8 GB, shared with other on-board processes). But:

- **Development and CI happen on machines without a GPU** (laptops, CI/CD build farms).
- The on-board GPU is **small (8 GB) and contended**, so even on the robot the GPU budget
  is tight and must be spent deliberately (segmentation + dense mapping), not assumed.

A pipeline that *requires* a GPU could not be developed, tested, or benchmarked on the
machines we actually use day to day. That is unacceptable for a project whose explicit
goal is reproducible, broadly-runnable tests.

## Decision

1. **Every algorithm has a CPU implementation, and CPU is the default build.** The full
   dev/test/benchmark pipeline runs on any platform with no GPU and no ROS.
2. **GPU acceleration is an opt-in fast-path**, behind a Cargo feature (e.g. `cuda`) and a
   runtime capability check. Absence of a GPU degrades performance, never correctness.
3. **A GPU code path must have a CPU counterpart that produces equivalent results**
   (within numerical tolerance) and is covered by the same tests.
4. **CI is CPU-only** and is the source of truth for correctness and for accuracy
   benchmarks. GPU performance numbers are reported separately, on the robot-class target.
5. GPU inference (segmentation) goes through a swappable interface with a CPU execution
   provider (e.g. ONNX Runtime CPU EP) so it runs — if slowly — without CUDA.

## Consequences

- **Easier:** anyone can `cargo test --workspace` and run the benchmark harness on a
  laptop; CI needs no special hardware; correctness is hardware-independent.
- **Harder:** we maintain two paths for GPU-accelerated kernels and must test their
  equivalence. We accept this cost for portability and testability.
- **Performance honesty:** wall-clock/real-time-factor numbers are always reported with the
  hardware they were measured on; CPU-only numbers on CI are a floor, not the robot figure.
- **Revisit when:** a kernel has *no* tractable CPU implementation (none identified yet). If
  that arises, it must be optional and excluded from the CPU correctness suite explicitly,
  with the gap logged — never silently GPU-only.

## Alternatives considered

- **GPU-required pipeline:** simpler (one path) but undevelopable/untestable on our actual
  dev and CI machines, and fragile on the contended 8 GB on-board GPU. Rejected outright.
