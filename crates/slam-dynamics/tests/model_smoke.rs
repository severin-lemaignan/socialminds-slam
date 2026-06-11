//! End-to-end smoke test against the committed yolo11s-seg export (`onnx/`, CPU EP).
//! Skips (with a notice) when the model file is absent, so source-only checkouts
//! still pass; CI carries the model and exercises the real inference path.

use slam_dynamics::{SegConfig, YoloSeg};
use slam_types::Stamp;

fn model_path() -> Option<std::path::PathBuf> {
    let path = std::env::var_os("SLAM_YOLO_MODEL")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| {
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../onnx/yolo11s-seg-rect.onnx")
        });
    path.exists().then_some(path)
}

#[test]
fn masks_a_synthetic_frame_at_source_resolution() {
    let Some(model) = model_path() else {
        eprintln!("skipping: onnx/yolo11s-seg.onnx not present (set SLAM_YOLO_MODEL)");
        return;
    };
    let mut seg = YoloSeg::load(&model, SegConfig::default()).expect("model loads on the CPU EP");
    assert_eq!(
        seg.input_size(),
        (640, 480),
        "the committed export is rect 640×480 (camera-shaped, ADR 0015)"
    );

    // A flat indoor-ish scene: grey wall, darker floor, a doorway-like band. No
    // photographic content, so the only safe semantic assertion is "not everything
    // is dynamic"; geometry (resolution, determinism, repeat runs) is exact.
    let (w, h) = (640usize, 480usize);
    let mut rgb = vec![0u8; 3 * w * h];
    for v in 0..h {
        for u in 0..w {
            let i = (v * w + u) * 3;
            let (r, g, b) = if v > 350 {
                (90u8, 80u8, 70u8) // floor
            } else if (200..280).contains(&u) {
                (60, 50, 45) // doorway band
            } else {
                (170, 168, 160) // wall
            };
            rgb[i] = r;
            rgb[i + 1] = g;
            rgb[i + 2] = b;
        }
    }

    let mask = seg
        .mask_rgb8(&rgb, w, h, Stamp::from_nanos(42))
        .expect("inference runs");
    assert_eq!(
        (mask.width, mask.height),
        (w, h),
        "mask at source resolution"
    );
    assert_eq!(mask.data.len(), w * h);
    assert_eq!(mask.stamp, Stamp::from_nanos(42));
    assert!(
        mask.coverage() < 0.5,
        "a flat synthetic scene must not be mostly dynamic (coverage {:.3})",
        mask.coverage()
    );

    // Same input → same mask (the pipeline is deterministic), and the scratch
    // state survives reuse.
    let again = seg.mask_rgb8(&rgb, w, h, Stamp::from_nanos(43)).unwrap();
    assert_eq!(again.data, mask.data);
}

#[test]
fn rejects_a_wrong_size_buffer() {
    let Some(model) = model_path() else {
        eprintln!("skipping: onnx/yolo11s-seg.onnx not present (set SLAM_YOLO_MODEL)");
        return;
    };
    let mut seg = YoloSeg::load(&model, SegConfig::default()).unwrap();
    assert!(seg
        .mask_rgb8(&[0u8; 30], 640, 480, Stamp::from_nanos(0))
        .is_err());
}
