# ONNX models

Segmentation models for dynamics masking (ADR 0015), consumed by `slam-dynamics`
through ONNX Runtime (CPU EP by default; TensorRT on the robot).

| file | provenance | license |
|---|---|---|
| `yolo11s-seg-rect.onnx` | ultralytics 8.4.65 `yolo export model=yolo11s-seg.pt format=onnx half=True` (COCO-pretrained, fp16, static 640×480, opset 18) | **AGPL-3.0** (ultralytics) |

⚠ **License:** the model weights are AGPL-3.0 while this repo is Apache-2.0 — fine
for research use; revisit before any redistribution/commercial deployment (see
ADR 0015 consequences).

