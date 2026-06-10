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
      semantic segmentation.
- [ ] **No-IMU option** (graceful degradation when no usable IMU stream is present).

## Front-end / registration

- [ ] **Range-adaptive depth sampling**: market1-1 depth+IMU reads 0 matched / 2047
      coasted — aisle ranges 4–6 m put stride-4 sample spacing (5.7 cm) above the
      2.5 cm voxel, so the integrated surface is unsupportable clumps (cafe works
      because it is close-range). Finer pixel stride at distance, or per-range voxel
      sizing; then first real market odometry numbers.
- [ ] **Hybrid per-point fan registration** (ADR 0010 refinement note): laser fans
      register against the 3D field where trilinear stencils are complete
      (camera-covered regions), 2D-field fallback elsewhere; the 2D field fades as
      RGB-D coverage grows (two cameras on the real robot).
- [ ] **Wheel odometry** (`nav_msgs/Odometry` reader + baseline + later a factor):
      present in all OpenLORIS bags; the paper's strongest market baseline
      (ATE 4.26 at 99.9 % CR).
- [ ] Photometric/colored registration — *research note only*: illumination variance
      is the hard part; lifelong-SLAM evidence favours learned features over raw RGB.

## Loop closure & re-localization

- [ ] **Stage 3b — GTSAM pose graph**: feed the recorded `LoopClosure` edges +
      submap-anchor odometry into `slam-backend`; smooth graph corrections replace
      pose snapping (snap-servo works but costs jitter on drift-free revisits,
      measured 0.0543→0.0559 on cafe1-2); re-pose frozen submaps on optimisation.
      GTSAM builds locally; awaiting first green CI with the vendored build.
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
