//! Pure pre/post-processing for YOLO-seg: letterbox geometry, box decode, NMS,
//! prototype-mask composition, dilation, and the un-letterboxed sampling back to the
//! source image. Everything here is deterministic CPU code with no ONNX dependency,
//! so the whole decode path is unit-tested without a model.
//!
//! Ultralytics YOLO11-seg ONNX output layout (static export, batch 1):
//! - `output0`: `[1, 4 + 80 + 32, A]`, attribute-major over `A` anchors —
//!   box `(cx, cy, w, h)` in input pixels, 80 class scores (already sigmoided),
//!   32 mask coefficients.
//! - `output1`: `[1, 32, H/4, W/4]` mask prototypes; an instance mask is
//!   `sigmoid(coeffs · protos)` cropped to its box, so "masked" ⇔ the logit > 0.

/// How a `src_w × src_h` image maps into the model's `in_w × in_h` input: scale to
/// fit, centre, pad with neutral grey (the ultralytics letterbox).
#[derive(Debug, Clone, Copy)]
pub struct Letterbox {
    pub scale: f64,
    pub pad_x: f64,
    pub pad_y: f64,
    pub in_w: usize,
    pub in_h: usize,
}

impl Letterbox {
    pub fn fit(src_w: usize, src_h: usize, in_w: usize, in_h: usize) -> Letterbox {
        let scale = (in_w as f64 / src_w as f64).min(in_h as f64 / src_h as f64);
        Letterbox {
            scale,
            pad_x: (in_w as f64 - src_w as f64 * scale) / 2.0,
            pad_y: (in_h as f64 - src_h as f64 * scale) / 2.0,
            in_w,
            in_h,
        }
    }
}

/// Letterbox-resize an RGB8 image into a normalized CHW float buffer (`3·in_w·in_h`,
/// values in `[0, 1]`, pad = 114/255), bilinear.
pub fn letterbox_chw(rgb: &[u8], src_w: usize, src_h: usize, lb: &Letterbox) -> Vec<f32> {
    let plane = lb.in_w * lb.in_h;
    let mut out = vec![114.0 / 255.0; 3 * plane];
    // Destination span actually covered by the image.
    let x0 = lb.pad_x.floor().max(0.0) as usize;
    let y0 = lb.pad_y.floor().max(0.0) as usize;
    let x1 = ((lb.pad_x + src_w as f64 * lb.scale).ceil() as usize).min(lb.in_w);
    let y1 = ((lb.pad_y + src_h as f64 * lb.scale).ceil() as usize).min(lb.in_h);
    for y in y0..y1 {
        let sy = ((y as f64 - lb.pad_y + 0.5) / lb.scale - 0.5).clamp(0.0, src_h as f64 - 1.0);
        let fy = sy.floor();
        let wy = sy - fy;
        let r0 = fy as usize;
        let r1 = (r0 + 1).min(src_h - 1);
        for x in x0..x1 {
            let sx = ((x as f64 - lb.pad_x + 0.5) / lb.scale - 0.5).clamp(0.0, src_w as f64 - 1.0);
            let fx = sx.floor();
            let wx = sx - fx;
            let c0 = fx as usize;
            let c1 = (c0 + 1).min(src_w - 1);
            for ch in 0..3 {
                let s = |r: usize, c: usize| rgb[(r * src_w + c) * 3 + ch] as f64;
                let v = s(r0, c0) * (1.0 - wy) * (1.0 - wx)
                    + s(r0, c1) * (1.0 - wy) * wx
                    + s(r1, c0) * wy * (1.0 - wx)
                    + s(r1, c1) * wy * wx;
                out[ch * plane + y * lb.in_w + x] = (v / 255.0) as f32;
            }
        }
    }
    out
}

/// One thresholded detection: box in input pixels + its anchor column (for the mask
/// coefficients, which stay in the raw prediction tensor).
#[derive(Debug, Clone, Copy)]
pub struct Det {
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
    pub score: f32,
    pub anchor: usize,
}

/// Decode the raw `[4+80+32, anchors]` prediction: keep anchors whose best score over
/// `class_ids` clears `conf`. Scores are already sigmoided in the ultralytics export.
pub fn decode(pred: &[f32], anchors: usize, class_ids: &[usize], conf: f32) -> Vec<Det> {
    let mut dets = Vec::new();
    for i in 0..anchors {
        let mut best = 0.0f32;
        for &c in class_ids {
            let s = pred[(4 + c) * anchors + i];
            if s > best {
                best = s;
            }
        }
        if best < conf {
            continue;
        }
        let cx = pred[i];
        let cy = pred[anchors + i];
        let w = pred[2 * anchors + i];
        let h = pred[3 * anchors + i];
        dets.push(Det {
            x1: cx - w / 2.0,
            y1: cy - h / 2.0,
            x2: cx + w / 2.0,
            y2: cy + h / 2.0,
            score: best,
            anchor: i,
        });
    }
    dets
}

fn iou(a: &Det, b: &Det) -> f32 {
    let ix = (a.x2.min(b.x2) - a.x1.max(b.x1)).max(0.0);
    let iy = (a.y2.min(b.y2) - a.y1.max(b.y1)).max(0.0);
    let inter = ix * iy;
    let area = |d: &Det| (d.x2 - d.x1).max(0.0) * (d.y2 - d.y1).max(0.0);
    let union = area(a) + area(b) - inter;
    if union <= 0.0 {
        0.0
    } else {
        inter / union
    }
}

/// Greedy class-agnostic NMS (we union all masks anyway; suppression only bounds the
/// per-box mask-composition cost).
pub fn nms(mut dets: Vec<Det>, iou_thresh: f32) -> Vec<Det> {
    dets.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut kept: Vec<Det> = Vec::new();
    for d in dets {
        if kept.iter().all(|k| iou(k, &d) <= iou_thresh) {
            kept.push(d);
        }
    }
    kept
}

/// Union the instance masks of `dets` into one boolean plane at prototype resolution
/// (`pw × ph` = input/4): per detection, the mask logit `coeffs · protos` is evaluated
/// only inside its (proto-scaled) box and counts as masked where it exceeds 0.
pub fn compose_union(
    pred: &[f32],
    anchors: usize,
    protos: &[f32],
    pw: usize,
    ph: usize,
    dets: &[Det],
    proto_scale: f64,
) -> Vec<bool> {
    let plane = pw * ph;
    let mut union = vec![false; plane];
    for d in dets {
        let px1 = ((d.x1 as f64 / proto_scale).floor().max(0.0) as usize).min(pw);
        let px2 = ((d.x2 as f64 / proto_scale).ceil().max(0.0) as usize).min(pw);
        let py1 = ((d.y1 as f64 / proto_scale).floor().max(0.0) as usize).min(ph);
        let py2 = ((d.y2 as f64 / proto_scale).ceil().max(0.0) as usize).min(ph);
        for y in py1..py2 {
            for x in px1..px2 {
                if union[y * pw + x] {
                    continue;
                }
                let mut logit = 0.0f32;
                for k in 0..32 {
                    logit += pred[(84 + k) * anchors + d.anchor] * protos[k * plane + y * pw + x];
                }
                if logit > 0.0 {
                    union[y * pw + x] = true;
                }
            }
        }
    }
    union
}

/// Dilate a boolean plane by a square radius `r` (separable two-pass): cheap insurance
/// at mask boundaries, where depth edges are noisiest anyway.
pub fn dilate(mask: &[bool], w: usize, h: usize, r: usize) -> Vec<bool> {
    if r == 0 {
        return mask.to_vec();
    }
    let mut rows = vec![false; w * h];
    for y in 0..h {
        for x in 0..w {
            if mask[y * w + x] {
                for dx in x.saturating_sub(r)..(x + r + 1).min(w) {
                    rows[y * w + dx] = true;
                }
            }
        }
    }
    let mut out = vec![false; w * h];
    for y in 0..h {
        for x in 0..w {
            if rows[y * w + x] {
                for dy in y.saturating_sub(r)..(y + r + 1).min(h) {
                    out[dy * w + x] = true;
                }
            }
        }
    }
    out
}

/// Sample the prototype-resolution union plane back onto the source image grid,
/// undoing the letterbox (nearest neighbour — the dilation margin already absorbs
/// boundary quantisation).
pub fn union_to_source(
    union: &[bool],
    pw: usize,
    ph: usize,
    lb: &Letterbox,
    proto_scale: f64,
    src_w: usize,
    src_h: usize,
) -> Vec<bool> {
    let mut out = vec![false; src_w * src_h];
    for v in 0..src_h {
        let py = (((v as f64 + 0.5) * lb.scale + lb.pad_y) / proto_scale) as usize;
        let py = py.min(ph - 1);
        for u in 0..src_w {
            let px = (((u as f64 + 0.5) * lb.scale + lb.pad_x) / proto_scale) as usize;
            out[v * src_w + u] = union[py * pw + px.min(pw - 1)];
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn letterbox_centres_and_scales() {
        // 640×480 into 640×640: scale 1, pad_y 80.
        let lb = Letterbox::fit(640, 480, 640, 640);
        assert_eq!(lb.scale, 1.0);
        assert_eq!(lb.pad_x, 0.0);
        assert_eq!(lb.pad_y, 80.0);
        // 848×480 into 640×640: scale by width.
        let lb = Letterbox::fit(848, 480, 640, 640);
        assert!((lb.scale - 640.0 / 848.0).abs() < 1e-12);
        assert_eq!(lb.pad_x, 0.0);
        assert!(lb.pad_y > 0.0);
        // Rect export at the native shape: identity.
        let lb = Letterbox::fit(640, 480, 640, 480);
        assert_eq!((lb.scale, lb.pad_x, lb.pad_y), (1.0, 0.0, 0.0));
    }

    #[test]
    fn letterbox_chw_pads_with_grey_and_keeps_pixels() {
        // A 2×2 white image into 4×4: the centre is white, the pad rows grey.
        let rgb = [255u8; 2 * 2 * 3];
        let lb = Letterbox::fit(2, 2, 4, 4);
        let chw = letterbox_chw(&rgb, 2, 2, &lb);
        assert_eq!(chw.len(), 3 * 16);
        let plane = 16;
        // Centre pixel (2,2) of every channel ≈ 1.0.
        for ch in 0..3 {
            assert!((chw[ch * plane + 2 * 4 + 2] - 1.0).abs() < 1e-3);
        }
        // 2×2 into 8×4 letterboxes horizontally: corners are pad grey.
        let lb = Letterbox::fit(2, 2, 8, 4);
        let chw = letterbox_chw(&rgb, 2, 2, &lb);
        assert!((chw[0] - 114.0 / 255.0).abs() < 1e-6, "corner must be pad");
    }

    /// Build a raw prediction tensor: `anchors` columns of 116 attributes, all zero.
    fn empty_pred(anchors: usize) -> Vec<f32> {
        vec![0.0; 116 * anchors]
    }

    fn set_det(
        pred: &mut [f32],
        anchors: usize,
        i: usize,
        (cx, cy, w, h): (f32, f32, f32, f32),
        class: usize,
        score: f32,
    ) {
        pred[i] = cx;
        pred[anchors + i] = cy;
        pred[2 * anchors + i] = w;
        pred[3 * anchors + i] = h;
        pred[(4 + class) * anchors + i] = score;
    }

    #[test]
    fn decode_thresholds_on_selected_classes_only() {
        let anchors = 4;
        let mut pred = empty_pred(anchors);
        set_det(&mut pred, anchors, 0, (100.0, 100.0, 40.0, 80.0), 0, 0.9); // person
        set_det(&mut pred, anchors, 1, (200.0, 100.0, 40.0, 80.0), 56, 0.5); // chair
        set_det(&mut pred, anchors, 2, (300.0, 100.0, 40.0, 80.0), 39, 0.9); // bottle: not selected
        set_det(&mut pred, anchors, 3, (400.0, 100.0, 40.0, 80.0), 0, 0.1); // below conf

        let dets = decode(&pred, anchors, &[0, 56], 0.2);
        assert_eq!(dets.len(), 2);
        assert_eq!(dets[0].anchor, 0);
        assert!((dets[0].x1 - 80.0).abs() < 1e-6 && (dets[0].y2 - 140.0).abs() < 1e-6);
        assert_eq!(dets[1].anchor, 1);
    }

    #[test]
    fn nms_suppresses_overlaps_keeps_distinct() {
        let d = |x1: f32, score: f32, anchor: usize| Det {
            x1,
            y1: 0.0,
            x2: x1 + 100.0,
            y2: 100.0,
            score,
            anchor,
        };
        // Two near-identical boxes + one far away.
        let kept = nms(vec![d(0.0, 0.8, 0), d(5.0, 0.9, 1), d(300.0, 0.5, 2)], 0.45);
        assert_eq!(kept.len(), 2);
        assert_eq!(kept[0].anchor, 1, "highest score survives");
        assert!(kept.iter().any(|k| k.anchor == 2));
    }

    #[test]
    fn compose_union_crops_to_box_and_thresholds_logit() {
        // 1 anchor, 8×8 proto plane, proto_scale 4 (input 32×32). Prototype 0 is all
        // ones; coefficient +1 ⇒ logit 1 > 0 everywhere — but only the box survives.
        let anchors = 1;
        let mut pred = empty_pred(anchors);
        pred[84 * anchors] = 1.0; // coeff 0
        let (pw, ph) = (8, 8);
        let mut protos = vec![0.0f32; 32 * pw * ph];
        protos[..pw * ph].fill(1.0);
        let det = Det {
            x1: 8.0,
            y1: 8.0,
            x2: 16.0,
            y2: 16.0,
            score: 0.9,
            anchor: 0,
        };
        let union = compose_union(&pred, anchors, &protos, pw, ph, &[det], 4.0);
        // Box maps to proto cells [2,4) × [2,4).
        assert!(union[2 * pw + 2] && union[3 * pw + 3]);
        assert!(
            !union[0] && !union[5 * pw + 5],
            "outside the box stays clear"
        );
        assert_eq!(union.iter().filter(|&&m| m).count(), 4);
    }

    #[test]
    fn compose_union_negative_logit_stays_clear() {
        let anchors = 1;
        let mut pred = empty_pred(anchors);
        pred[84 * anchors] = -1.0;
        let (pw, ph) = (4, 4);
        let mut protos = vec![0.0f32; 32 * pw * ph];
        protos[..pw * ph].fill(1.0);
        let det = Det {
            x1: 0.0,
            y1: 0.0,
            x2: 16.0,
            y2: 16.0,
            score: 0.9,
            anchor: 0,
        };
        let union = compose_union(&pred, anchors, &protos, pw, ph, &[det], 4.0);
        assert!(union.iter().all(|&m| !m));
    }

    #[test]
    fn dilate_grows_square() {
        let (w, h) = (7, 7);
        let mut m = vec![false; w * h];
        m[3 * w + 3] = true;
        let d = dilate(&m, w, h, 1);
        assert_eq!(d.iter().filter(|&&x| x).count(), 9);
        assert!(d[2 * w + 2] && d[4 * w + 4]);
        let d0 = dilate(&m, w, h, 0);
        assert_eq!(d0, m);
    }

    #[test]
    fn union_to_source_undoes_the_letterbox() {
        // Source 8×4 into input 8×8 (pad_y 2, scale 1), proto_scale 2 → proto 4×4.
        let lb = Letterbox::fit(8, 4, 8, 8);
        assert_eq!(lb.pad_y, 2.0);
        let (pw, ph) = (4, 4);
        let mut union = vec![false; pw * ph];
        // Mark proto row 1 (input rows 2–3 = source rows 0–1), columns 0–1
        // (input/source columns 0–3).
        union[pw] = true;
        union[pw + 1] = true;
        let src = union_to_source(&union, pw, ph, &lb, 2.0, 8, 4);
        assert!(
            src[0] && src[3] && src[8 + 2],
            "source rows 0–1, cols 0–3 masked"
        );
        assert!(!src[4], "column 4 maps to proto column 2: clear");
        assert!(!src[2 * 8], "source row 2 maps past the marked proto row");
    }
}
