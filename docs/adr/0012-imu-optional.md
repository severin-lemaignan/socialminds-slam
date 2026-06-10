# ADR 0012: The IMU is an optional accuracy enhancer, never a prerequisite

- **Status:** accepted
- **Date:** 2026-06-10
- **Deciders:** Séverin Lemaignan

## Context

The hardware team confirmed the robot will ship **without an IMU initially** (one will
be added later). ADR 0010 had promoted IMU attitude to a "front-end prerequisite" for
tilt compensation; that stance must soften. An analysis of what each subsystem actually
takes from the IMU:

| Consumer | Without an IMU | Severity |
|---|---|---|
| Dynamic tilt compensation (suspension pitch/roll under accel) | Lost — scans treated as mounted; floor-hit gating disabled (it keys off attitude) | Bounded and now **measured**: cafe1-1 with the true rig reads 0.203 laser-only vs 0.164 with IMU (≈ 4 cm at cafe-gentle dynamics); the CSV identity-rig runs never used the IMU and hold 0.039/0.055. The synthetic 4° scenario shows metres of error only under *hard* accel with low-mounted lidars |
| **Static mounting tilt** | **Unaffected** — comes from the rig extrinsic (URDF/`tf_static`), applied in the SE(3) lift | none |
| Gravity reference (map z-axis, roll/pitch absolute) | Map plane = calibrated base plane; ramps/floor changes unobservable by the 2D lidar alone | Deferred until multi-floor; depth/floor-plane tracking can substitute later |
| Re-localization search (gravity pins roll/pitch) | Falls back to the planar assumption — same 3-DoF search indoors | none today |
| Constant-velocity prediction | Unchanged (it never used the IMU) | none |
| IMU dead-reckoning baseline | Unavailable | Wheel odometry baseline replaces it (`/odom` is on every robot/bag) |
| Time sync | Unchanged (ROS stamps, ADR 0009) | none |

The engine is **already IMU-optional by construction**: `process_imu` is a no-op
default, the attitude filter is inert (identity tilt) until samples arrive, and the
benchmark's scan systems have always run IMU-less — the parity-gate numbers *are*
no-IMU numbers.

## Decision

1. **No code path may require an IMU.** Systems that consume only IMU (dead-reckoning)
   bail with a clear message; everything else runs and degrades as analysed above.
   This is the current behaviour — now contractual, guarded by the fact that the
   default benchmark configuration is IMU-less.
2. **Wheel odometry is promoted** from "nice baseline" to the planned motion prior:
   without an IMU it is the only proprioception, it directly strengthens the
   constant-velocity prediction and the corridor-degeneracy fill-in, and the paper
   shows it is strong on exactly the scenes that hurt (market: ATE 4.26 at 99.9 % CR).
3. **When the IMU arrives**, nothing changes architecturally: its topics appear in the
   run configuration (ADR 0013) and the attitude filter wakes up. Tilt compensation,
   floor gating and the gravity reference switch on; measured deltas land in the
   benchmark report like any change.
4. ADR 0010's "IMU attitude is a front-end prerequisite" is hereby narrowed to:
   *prerequisite for dynamic-tilt compensation only*.

## Consequences

- **Easier:** one less integration blocker for the first robot bring-up; the no-IMU
  configuration is exactly what CI and the benchmark already exercise.
- **Harder / accepted risks:** transient pose error under aggressive accel/decel
  (uncompensated suspension tilt — flag to the HW/ops team: gentle motion profiles
  until the IMU lands); no absolute gravity reference (single-floor assumption);
  ramps invisible to the front-end.
- **Revisit when:** the IMU is fitted (re-run the tilt scenario + benchmark and record
  the delta), or multi-floor operation starts before it does.
