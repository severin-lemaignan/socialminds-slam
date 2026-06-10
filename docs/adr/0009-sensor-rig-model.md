# ADR 0009: Sensor rig from URDF + CameraInfo — frame-tagged measurements, no bespoke calibration format

- **Status:** accepted
- **Date:** 2026-06-10
- **Deciders:** Séverin Lemaignan

## Context

The robot carries **two** 2D lidars (opposite corners, H:270° each), **two** RGB-D
cameras (front + rear) and an IMU ([ADR 0002](0002-sensor-roles-and-pipeline.md)). The
engine today has no notion of *which* sensor a measurement came from or *where* that
sensor sits: `LaserScan2D` is implicitly "the lidar", and the scan front-end treats the
sensor frame as the body frame. That is fine for OpenLORIS (one Hokuyo, roughly centred)
and is exactly what must change before any multi-lidar or multi-camera work: fusing two
lidars mounted at opposite corners is *only* a geometry problem if the geometry is
modelled, and a mismodelled extrinsic poisons every factor that sensor produces. ADR 0002
already calls inter-sensor extrinsics and time-sync "make-or-break".

The robot side already publishes everything we need, in standard formats:

- the **URDF** (required by Nav2/ROS anyway) carries every fixed sensor extrinsic;
- **`sensor_msgs/CameraInfo`**, published alongside every RGB-D stream (and recorded in
  bags), carries intrinsics + distortion, per the standard ROS calibration pipeline;
- every sensor message **self-identifies its frame** via `header.frame_id`, whose values
  are the URDF link names.

A first draft of this ADR proposed an engine-native TOML rig file with URDF as an import
source. Review (2026-06-10) rejected that: it duplicates the URDF, creates a second
source of truth to keep in sync with the physical robot, and the one gap it claimed to
fill — camera intrinsics — is already filled by CameraInfo, which travels *with the data*.

## Decision

**1. The URDF is the primary geometric source, read directly by the engine.** Parsing it
is a tiny, ROS-runtime-free dependency (`urdf-rs`, pure XML). At startup the engine walks
the fixed-joint chains from the base frame (default `base_link`, overridable) to every
link and flattens them into an in-memory **`SensorRig`**: `FrameId → T_base_frame`
(SE(3)). A new small **`slam-rig`** crate owns this (parse, chain resolution,
validation); `slam-types` gains only a lightweight `FrameId` (interned index, `Copy`).
Validation is strict where it matters: an unknown `frame_id` at ingest is an error, not a
silent identity, and a `LaserScan2D` whose frame has roll/pitch beyond a planarity
tolerance triggers a warning (a tilted "planar" lidar violates the SE(2) front-end's
model).

**2. Camera intrinsics + distortion come from `sensor_msgs/CameraInfo`** — the topic on
the live robot and in bags, or the standard `camera_info_manager` YAML file offline. No
engine-specific calibration format exists at all.

**3. Measurements are frame-tagged at ingest using their own `header.frame_id`.**
`LaserScan2D` (and the future `RgbdFrame`, `ImuSample` when fused) gains a
`frame: FrameId`. The bag reader already decodes message headers; it starts surfacing
`frame_id`. Sensor *kind* needs no configuration either — the message type
(`LaserScan` / `Image` / `Imu`) says what it is, `frame_id` says where it is. The
topic→sensor binding table of the TOML draft simply disappears.

**4. `slam-replay` gains `--urdf <file>`** (and the harness passes it through). Without
it, a single-sensor identity rig is assumed — every existing dataset, test and benchmark
keeps working unchanged. For CI and synthetic data we commit **mock URDFs + mock
calibration YAMLs** as fixtures; the synthetic generator grows a **two-lidar mode** (two
offset virtual lidars over the same world, extrinsics deliberately perturbable) so
multi-lidar fusion and its failure modes — wrong extrinsic, clock skew — are CI-testable
long before robot data exists, in line with
[ADR 0005](0005-evaluation-methodology.md).

**5. Multi-lidar fusion: per-scan registration into a shared local map — never
pseudo-360° scan merging.** The two lidars are unsynchronized (40 Hz each, up to ~12 ms
apart; at 2 m/s the base moves ~2.5 cm between their scans), so concatenating them into
one virtual scan bakes motion error into the data. Instead the scan front-end generalises
from scan-to-keyframe to **scan-to-local-map**: each incoming scan, from either lidar, is
transformed by its (SE(2)-projected) extrinsic and registered against a local map built
from both sensors' recent registered points, producing one pose factor at its own stamp.
The existing point-to-line ICP machinery is unchanged; only the reference model changes.
The local map keeps per-sensor provenance so cross-lidar consistency is observable
(overlapping FOV residuals = a free online extrinsic health check), and the backend can
later promote `T_base_lidar2` to a graph variable with a strong prior if calibration
drift proves real (GTSAM handles constant-extrinsic variables natively).

**6. Multi-camera follows the same shape.** One visual front-end instance per RGB-D
camera (masking included), each emitting factors expressed in `base_link` through its
extrinsic; intrinsics from its CameraInfo. Nothing about camera count is baked into the
fusion layer.

**7. ROS timestamps are the single time base.** All streams arrive stamped on a common
clock (hardware/software-synchronized upstream — e.g. OpenLORIS at 1.7–7.4 ms residual
std); the engine consumes `header.stamp` as-is and models no per-device offsets.

## Consequences

- **Easier:** adding the second lidar, then the cameras, becomes "point the engine at the
  robot's existing URDF" plus a front-end instance — zero new configuration artifacts,
  nothing to keep in sync by hand. Extrinsic errors become *visible* (cross-sensor
  residuals) instead of silently degrading ATE. Datasets, CI mocks and the robot share
  one mechanism.
- **Harder:** the scan front-end must evolve scan-to-keyframe → scan-to-local-map (worth
  doing anyway for robustness); the bag reader must surface `header.frame_id`; the engine
  takes a (small, pure-XML) URDF parsing dependency.
- **Risk:** planar fusion of two lidars mounted at different heights sees different
  horizontal slices of furniture-height objects; cross-lidar matching could be
  inconsistent in cluttered areas even with perfect extrinsics. Mitigated by per-sensor
  provenance in the local map and robust kernels; walls (the loop-closure signal that
  matters) are height-invariant.
- **Risk:** the URDF's nominal extrinsics may differ from calibrated reality (URDFs are
  often CAD-derived). The robot's calibration procedure must write its results *back into
  the URDF* (or an xacro overlay) rather than into a side-channel — this is the price of
  a single source of truth, and the cross-lidar residual check above is the watchdog.
- **Revisit when:** a sensor becomes non-rigid w.r.t. the base (pan-tilt head, nodding
  lidar) — that re-opens TF-style time-varying transforms.

## Alternatives considered

- **An engine-native rig file (TOML) with URDF as import source.** The first draft of
  this ADR. Rejected on review: duplicates the URDF, second source of truth, inevitable
  drift; its only genuine additions (intrinsics, distortion) are already covered by
  CameraInfo, and its time-offset field is unnecessary — ROS timestamps are the
  synchronization mechanism.
- **A TF-style dynamic transform tree.** General, but the base is rigid — a static rig
  resolved at startup has zero runtime lookup cost. Dynamic transforms are the
  "revisit-when" trigger, not the default.
- **Pseudo-360° merged scans.** Tempting (the matcher sees full coverage per update) but
  wrong under motion: the two scans are not simultaneous and the merge invents a rigid
  snapshot that never existed. Rejected.
- **Independent per-lidar odometries fused at the pose level.** Simple (two
  `ScanOdometry` instances + backend fusion) and acceptable as a stepping stone, but it
  discards cross-lidar geometry (each matcher sees only 270°), double-maintains
  keyframes, and couples the lidars only through the backend. The shared local map
  subsumes it.
