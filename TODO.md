# TODO

Working list of known next steps, roughly prioritised within each category.
Architectural decisions stay in [`docs/adr/`](docs/adr/); measured findings that
motivate an item are noted inline.

## Dynamics & robustness (highest leverage)

- [ ] **Dynamics masking** (YOLO-seg + optical-flow/depth mask propagation + occupancy
      decay — ADR 0002). Three independent measurements say un-masked people are the
      single biggest accuracy blocker: depth-only odometry 2.8 m ATE on cafe1-1;
      depth→pose fusion 0.16→3.0; laser-band depth contribution 0.164→0.357. Unlocks
      the two gated bridges (`depth_updates_pose`, `reg_band_tolerance`).
- [ ] **Clean-3D-map project**: filter depth to remove outliers; post-hoc model
      filtering; … → target: a *compact* map suitable for downstream tasks like
      semantic segmentation. Plan drafted: [`docs/clean-map-plan.md`](docs/clean-map-plan.md).
- [x] **No-IMU option** — analysed and contractual (ADR 0012); measured cost ≈ 4 cm
      on cafe1-1 (0.203 laser-only vs 0.164 with IMU); `configs/no-imu.yaml`.

## Front-end / registration

- [x] **Range-adaptive depth sampling** — kept pixels spaced ≈ `target_spacing` at
      every depth (power-of-two local lattices, per-cloud point cap), plus a separate
      coarser 3D-field voxel (5 cm / 15 cm truncation vs the 2 D laser field's 2.5 cm).
      Market1-1 depth+odom+IMU now tracks: **ATE 4.38 m** over the 150–220 m loop
      (was frozen; paper's wheel-odom baseline: 4.26), 2034/2047 matched, 26× RT;
      depth loop closure verifies (5 closures on cafe). Swept trade-off recorded in
      code: finer 3D fields score better open-loop near-range (2.5 cm → 0.46 vs 0.81
      on cafe depth-only) but their narrow truncation basin kills loop verification.
- [ ] **Depth loop-closure basin**: loop seeds must land inside the 3D field's
      truncation (15 cm) — a coarse-to-fine seed pyramid (or scan-context-style
      pre-alignment) would decouple verification from the field's voxel size and let
      a finer field recover the near-range accuracy.
- [ ] **Hybrid per-point fan registration** (ADR 0010 refinement note): laser fans
      register against the 3D field where trilinear stencils are complete
      (camera-covered regions), 2D-field fallback elsewhere; the 2D field fades as
      RGB-D coverage grows (two cameras on the real robot).
- [x] **Wheel odometry** — `/odom` reader + motion prior in the front-end
      (`--odom-topic`, config `sensors.odometry`). Measured: depth-only cafe1-1
      2.8 m → **0.456 m** with the odom prior (558/569 clouds matched). Still open:
      odometry as a graph factor and as a standalone baseline in the harness.

- [ ] Photometric/colored registration — *research note only*: illumination variance
      is the hard part; lifelong-SLAM evidence favours learned features over raw RGB.

## Loop closure & re-localization

- [x] **Stage 3b — GTSAM pose graph**: wired end to end. Submaps store
      anchor-relative coordinates (re-posing = updating the anchor, voxels never
      rewritten); odometry edges + anchor-relative loop measurements recorded; the
      `AnchorGraph` seam keeps the front-end C++-free; GTSAM adapter in slam-replay
      optimises on every verified loop and re-poses all anchors. cafe1-2 with graph:
      0.0561 (≈ snap at noise-level drift; the win shows under real drift).
      Remaining niche: loops during the hand-over overlap window still snap-only.
- [ ] **Per-submap appearance signatures** (MapClosures-style density images):
      replace the proximity-only loop gating; prerequisite for corridor aliasing
      robustness and for re-localization.
- [ ] **Stage 4 — re-localization service** (< 1 s, verified; ADR 0010): cold-start /
      tracking-loss localization over frozen-submap signatures; scored with the
      OpenLORIS lifelong protocol (CR, CS-R) archived in `eval/reference/sota/`.

## Map

- [ ] **Stage 5 — OpenVDB backend** (system `libopenvdb-dev` 10.x, feature-gated,
      `cxx` shim; ADR 0010) + conformance suite vs `SparseTsdf`; in-process grid
      hand-over to reMap.
- [ ] **Voxel RGB**: decode `/d400/color` paired with aligned depth (same pixels) into
      colored clouds; optional per-voxel RGB accumulation (surface voxel only,
      config-gated so depth-only memory stays at 8 B/voxel); rerun `world/tsdf`
      colored by true RGB. Implement together with dynamics masking — same
      color-image decode path. Useful for reMap.
- [ ] Half-float TSDF voxels (halves map memory; ADR 0010 budget headroom).
- [ ] Occupancy decay (evict transient objects; ADR 0002/0004) — overlaps with
      dynamics masking.

## Evaluation & test data

- [ ] **Synthetic depth-camera scenario** (raycast 2.5D world → depth images/clouds):
      CI coverage for the depth path, clean + noisy variants (deferred from the M4
      round).
- [ ] Python synthetic generator two-lidar mode (ADR 0009 noise-suite item; the Rust
      raycast harness already covers CI — optional completeness).
- [ ] M1 leftover (operator step): run RTAB-Map/GLIM on the robot and archive the
      reference baseline (`eval/reference/`).

## Infrastructure

- [ ] First green GitLab CI with the vendored GTSAM build (M2 carry-over).
- [ ] CameraInfo distortion model is currently ignored (OpenLORIS aligned depth is
      rectified, so it is correct there) — handle D when raw/unrectified streams or
      the robot's cameras need it.
