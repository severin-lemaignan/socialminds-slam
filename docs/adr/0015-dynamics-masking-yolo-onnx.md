# ADR 0015: Dynamics masking — YOLO11-seg via ONNX Runtime, rejection at depth ingest

- **Status:** accepted
- **Date:** 2026-06-11
- **Deciders:** Séverin Lemaignan

## Context

Un-masked people dominate the depth path's error — three independent measurements
(ROADMAP M5): depth-only odometry 2.8 m ATE on cafe1-1, depth→pose fusion 0.16→3.0 m,
laser-band depth contribution 0.164→0.357 m. The two depth→pose bridges
(`depth_updates_pose`, `reg_band_tolerance`) are gated off behind exactly this
capability. The map side is already covered maskless (free-space carving, ADR 0014);
masking is the *sensor-side* complement — keeping dynamic points out of the field in
the first place instead of evicting them afterwards — and the remaining direction
after all three geometric map-side mechanisms for the newborn-submap bias traded one
regime for another (ADR 0014 alternatives).

A dedicated model survey (`docs/REPORT_HUMAN_DETECTION.md`, 2026-06-11) benchmarked
instance/semantic/open-vocabulary segmenters on standard scenes **and a hard set
matching the robot's near-floor camera placement** (heavily occluded, legs-only
views). Key findings:

- Every real-time candidate detects the person in 100 % of hard frames; they differ
  in mask quality and *chair* recall. **yolo11s-seg** wins on occluded/oblique
  furniture (41 vs 23 detections at conf 0.15 vs yolo11n) for +0.3 ms.
- **Confidence threshold matters more than model size**: conf 0.2 with the dynamic
  class set is the operating point — the error cost is asymmetric (a missed person
  corrupts the map; a false positive discards a few points).
- VOC-pretrained MobileNet semantic models cannot segment office chairs (training
  data, not threshold); SAM 3.1 has the best masks but is 325 ms/frame.
- A **square-letterboxed static export silently loses recall** (21.6 % vs 28.6 %
  mask coverage on the hard set — confirmed to be the letterboxing, not fp16);
  exporting at the camera's true shape (`imgsz=(480, 640)`) restores exact parity.
- TensorRT fp16 at the native shape is the deployment lever: 2.46 ms/frame, 407 FPS,
  tight p90 — but engines are GPU- and version-specific.

## Decision

1. **Model: `yolo11s-seg`, ONNX export, dynamic COCO class set, conf 0.2, mask
   dilated ~8 input px** — the survey's recommendation verbatim. Class filtering is
   post-hoc (the network always scores 80 classes), so the wider set costs nothing.
2. **Runtime: ONNX Runtime through the `ort` crate, CPU execution provider by
   default.** Dev, CI and benchmarks run GPU-less (ADR 0003); CUDA/TensorRT EPs are
   a later, feature-shaped addition on the robot. The committed fp16 export runs
   unmodified on the CPU EP (~0.25 s/frame debug, well under the offline-replay
   budget; the robot path is TensorRT at ~2.5 ms).
3. **A new `slam-dynamics` crate owns inference**; all pre/post-processing
   (letterbox, box decode, NMS, prototype-mask composition, dilation,
   un-letterboxed sampling) is pure Rust and unit-tested without the model. The
   input shape and fp16-ness are **read from the model**, so the square 640×640
   export and a rect `imgsz=(H, W)` export both work — per the survey, the rect
   export matched to the camera is the better artifact and should replace the
   square one when convenient.
4. **Masking applies at depth ingest** (`slam-datasets::parse_depth_image`), the
   formal mask hook of the clean-map plan: a `PixelMask` (in `slam-types`) computed
   on the colour frame suppresses pixels *before* back-projection, so masked points
   never exist anywhere downstream — registration, TSDF, colour channel, loop
   closure all inherit the rejection for free. The mask is stamp-gated exactly like
   the colour pairing (±50 ms) and rescales across resolutions.
5. **Feature-gated end to end** (`slam-replay --features dynamics`,
   `masking:` YAML section / `--mask-model`): ONNX Runtime is a prebuilt C++
   library, and ADR 0001 keeps GTSAM the only mandatory C++ dependency of the core.
   The default build's CLI surface is identical; using it without the feature is a
   clear error (same pattern as `viz`).
6. **Enhancer, never a foundation (ADR 0014 reaffirmed).** Inference failure, a
   missing colour stream, or a stale mask degrade that frame to unmasked ingest —
   never an error. Every accuracy/map gate must still pass maskless; masking exists
   to *unlock the gated depth bridges*, not to hold up anything that already works.

## Consequences

- `cargo test --workspace` now builds `slam-dynamics`, which downloads a prebuilt
  ONNX Runtime (~tens of MB, cached) at first build — CI pays it once per cache.
  The model itself (`onnx/yolo11s-seg.onnx`, 20 MB) is committed: the smoke test
  runs real CPU inference in CI; source-only checkouts skip it cleanly.
- **License caveat:** ultralytics publishes YOLO11 weights under **AGPL-3.0** while
  this repo is Apache-2.0. The model is a data artifact consumed at runtime, not
  linked code, and this is a research project — but any future redistribution or
  commercialisation must revisit this (swap to an Apache-licensed detector, an
  Ultralytics Enterprise license, or ship the model out-of-band).
- The masker runs synchronously in the bag-ingest pass at the decimated depth rate
  (memoised per colour frame). For the live robot this moves to its own pipeline
  stage (drop-frames contract, ADR 0001) with the TensorRT EP — the `YoloSeg` API
  already takes raw RGB frames and is stage-shaped.
- Per-class thresholds (person 0.25 / chair 0.1, suggested by the survey for finer
  recall control) are a straightforward extension of `slam_dynamics::ClassSet` if
  the global 0.2 proves too blunt.
- Mask *propagation* (optical-flow/depth bridging between keyframes, PLE-SLAM
  style) remains open M5 work: it matters once inference is decimated harder than
  the depth stream or people move fast between masked frames.

## Alternatives considered

- **lraspp/deeplabv3-MobileNet (semantic, VOC):** ~2× faster, but provably cannot
  segment office chairs and lacks carryable classes entirely — chairs and bags are
  exactly the movable clutter the map must reject.
- **SAM 3.1:** best masks, open vocabulary, 325 ms/frame — not per-frame viable.
  Kept in mind as a keyframe-rate semantic labeller alongside YOLO rejection.
- **`tract` (pure-Rust inference) instead of ONNX Runtime:** would avoid the C++
  dependency entirely, but has no GPU/TensorRT path — the robot deployment would
  need a second runtime anyway. `ort` gives one API across CPU EP (CI) and
  CUDA/TensorRT (robot), matching the ADR 0003 pattern of optional acceleration
  behind a tested CPU fallback.
- **Masking inside the front-end (per-point classification at registration):**
  rejected — it would spread dynamics knowledge across every consumer; the ingest
  hook removes the points once, for all of them.
