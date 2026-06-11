# Human / Dynamic-Object Segmentation Benchmark for SLAM Point-Cloud Filtering

**Date:** 2026-06-11
**Hardware:** NVIDIA GeForce RTX 5070 Laptop GPU (8 GB, Blackwell sm_120), 24-core CPU
**Stack:** Python 3.13 (uv), PyTorch 2.11.0+cu128, ultralytics 8.4.65, facebookresearch/sam3
**Input:** all frames resized to 640×480 (typical SLAM camera resolution); latency measured per-frame with CUDA synchronization, median over 30 runs after warmup.

## Goal

Find a segmentation model fast enough to run per-frame inside a SLAM pipeline,
to reject point-cloud points belonging to dynamic objects. Humans are the
primary target; movable furniture (office chairs) and other non-stationary
objects are a secondary target. A missed dynamic object corrupts the map; a
false positive merely discards a few extra points — so recall is favored over
precision throughout.

## Candidates

| model | type | classes | source |
|---|---|---|---|
| yolo11n-seg / yolo11s-seg | instance seg | 80 COCO | ultralytics |
| lraspp_mobilenetv3 | semantic seg | 21 VOC | torchvision |
| deeplabv3_mobilenetv3 | semantic seg | 21 VOC | torchvision |
| SAM 3.1 (`sam3.1_multiplex.pt`) | open-vocabulary concept seg | text prompt | facebook/sam3.1 (gated) |

## Results

### Person-only — standard images (COCO/ultralytics, multi-person street scenes)

| model | setup s | median ms | p90 ms | FPS | mask % |
|---|---|---|---|---|---|
| lraspp_mobilenetv3 | 1.6 | 2.17 | 2.66 | 461 | 22.7 |
| deeplabv3_mobilenetv3 | 1.5 | 3.19 | 3.34 | 314 | 22.3 |
| yolo11n-seg | 4.0 | 3.88 | 4.34 | 258 | 22.1 |
| yolo11s-seg | 1.2 | 4.18 | 4.90 | 239 | 22.5 |
| sam3.1 ("person") | 18.4 | 324.62 | 332.17 | 3.1 | 21.2 |

### Person-only — hard set (`images/legs`, 14 frames, heavily occluded, exactly one person each)

| model | median ms | FPS | mask % | detection rate |
|---|---|---|---|---|
| lraspp_mobilenetv3 | 2.23 | 449 | 11.6 | 100% |
| deeplabv3_mobilenetv3 | 3.22 | 311 | 12.4 | 100% |
| yolo11n-seg | 4.10 | 244 | 12.4 | 100% |
| yolo11s-seg | 4.36 | 229 | 12.6 | 100% |
| sam3.1 | 317.68 | 3.1 | 11.5 | 100% |

All models detect the person in 100% of frames (mask coverage > 0.1%), but
mask **quality** differs substantially (visual inspection, `out/` overlays):

- **SAM 3.1** — cleanest masks by far: full legs + sandals, crisp boundaries.
- **yolo11n/s-seg** — solid legs coverage; occasionally clips feet/sandals.
- **deeplabv3 / lraspp** — ragged: misses shins/feet in places, bleeds onto
  furniture, small false positives.

### Dynamic objects (person + chairs + vehicles/animals/carryables)

YOLO: 22 COCO classes (person, chair, couch, potted plant, vehicles, animals,
backpack/handbag/suitcase, laptop, phone, sports ball). Class filtering is
post-hoc — **no latency cost** (the network always evaluates all 80 classes).

Confidence-threshold study on the hard set (chair detections summed over 14 frames):

| model | conf 0.4 | conf 0.25 | conf 0.15 |
|---|---|---|---|
| yolo11n-seg | 7 | 13 | 23 |
| yolo11s-seg | 18 | 33 | 41 |

Final configuration (conf = 0.2), hard set:

| model | median ms | FPS | mask % |
|---|---|---|---|
| yolo11n-seg-dyn | 4.30 | 232 | 20.9 |
| yolo11s-seg-dyn | 4.41 | 227 | 28.6 |
| lraspp_mobilenetv3-dyn | 2.40 | 417 | 11.8 |
| deeplabv3_mobilenetv3-dyn | 3.39 | 295 | 12.6 |

### ONNX / TensorRT export (yolo11s-seg, hard set, same run)

onnxruntime 1.26 CUDA EP; TensorRT 11.0 (cu12), fp16 engine, ~3 min build.

| backend | median ms | p90 ms | FPS | mask % |
|---|---|---|---|---|
| PyTorch fp16 | 4.24 | 4.74 | 236 | 28.6 |
| ONNX fp16, static 640×640 | 5.52 | 7.72 | 181 | **21.6** |
| ONNX fp16, static 480×640 (`-rect`) | 4.71 | 6.16 | 212 | 28.6 |
| **TensorRT fp16, 480×640** | **2.46** | **2.66** | **407** | 28.6 |

Three lessons: (1) a square-letterboxed static export **silently loses
recall** (21.6 vs 28.6% coverage — an fp32 export confirmed it is the
letterboxing, not fp16); exporting at the true frame shape `imgsz=(480, 640)`
restores exact parity. (2) Plain onnxruntime gives no speed win over the
already-fp16 torch path — its value is deployability (C++/Jetson, no
Python/torch). (3) **TensorRT is the real lever: 1.7× faster than torch**
(2.46 ms, 407 FPS) with identical masks and a much tighter p90 — latency
jitter matters in a real-time pipeline. Engine files are GPU- and
TRT-version-specific; rebuild per deployment target.

## Main findings

1. **All real-time candidates are far faster than needed.** Even the heaviest
   YOLO config costs ~4.4 ms/frame (227 FPS) at 640×480 on this GPU.
2. **Confidence threshold matters more than model size for chair recall.**
   Dropping conf from 0.4 to 0.15 roughly triples chair detections at zero
   latency cost. For point rejection the asymmetric error cost makes low
   thresholds (0.15–0.25) the right operating point.
3. **yolo11s beats yolo11n specifically on occluded/oblique chairs**
   (41 vs 23 detections at conf 0.15) for only +0.3 ms.
4. **VOC-pretrained MobileNet models cannot segment office chairs.** Although
   VOC contains a `chair` class, adding it changed coverage only 11.6 → 11.8%.
   Probing raw probabilities confirms the failure is the training data (2012
   Pascal VOC, dining-style chairs), not the argmax/threshold: peak chair
   confidence in a three-chair frame was 0.54, in fragments. VOC also lacks
   backpacks, suitcases, laptops, phones entirely.
5. **SAM 3.1 is not per-frame viable (325 ms) but has the best masks** and is
   open-vocabulary (any text prompt), which is attractive for semantic
   mapping. Caveats: run here as a single-image detector at 1008 px via the
   `sam3` image model; the multiplex checkpoint lacks one FPN level of the
   image model (4 keys random-init — no visible mask degradation, but the
   `facebook/sam3` image checkpoint is the clean reference). Its intended
   streaming mode (`build_sam3_multiplex_video_predictor`) detects once and
   tracks at a claimed ~32 FPS fp16, untested here.

## Recommendation

**`yolo11s-seg`, dynamic class set, conf ≈ 0.2, as a TensorRT fp16 engine at
the camera's native shape → ~2.5 ms/frame (407 FPS).** During development the
plain PyTorch path (~4.4 ms) is equivalent in output and simpler to iterate on.

For the SLAM integration:

- Dilate the mask a few pixels before point rejection — cheap insurance at
  mask boundaries, where depth edges are noisiest anyway.
- If finer recall control is needed, apply per-class thresholds (e.g. person
  0.25, chair 0.1) by filtering raw boxes instead of a global `conf`.
- Artifacts: `yolo11s-seg-rect.engine` (TensorRT, this GPU only — rebuild on
  the target device) and `yolo11s-seg-rect.onnx` (portable, for C++/Jetson
  deployment without Python/torch). Both exported at the true frame shape
  and verified to match torch output exactly.
- For semantic mapping, a hybrid is plausible: YOLO per-frame for rejection,
  SAM 3.1 on keyframes for open-vocabulary labels (or its streaming tracker).

## Reproduction

```bash
uv sync                                   # env (CUDA 12.8 wheels for Blackwell)
uv run bench.py                           # all models, default image set
uv run bench.py --images images/legs      # hard occlusion set
uv run bench.py --models yolo11s-seg-dyn  # recommended config only
```

SAM 3.1 additionally requires the gated checkpoint:

```bash
HF_TOKEN=$HUGGINGFACE_TOKEN uv run hf download facebook/sam3.1 \
    sam3.1_multiplex.pt --local-dir checkpoints/sam3.1
```

Mask overlays for every model × frame are written to `out/`.
