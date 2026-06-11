//! Dynamics masking (ADR 0015): YOLO11-seg ONNX inference producing per-pixel
//! [`PixelMask`]s that reject dynamic objects (people, chairs, …) at depth-ingest
//! time, before their points ever reach registration or the map.
//!
//! The model survey (`docs/REPORT_HUMAN_DETECTION.md`) selected **yolo11s-seg** with
//! the dynamic COCO class set at confidence ≈ 0.2: recall is favoured over precision
//! because a missed person corrupts the map while a false positive merely discards a
//! few points. Inference runs through ONNX Runtime's CPU execution provider by
//! default (ADR 0003 — dev/CI are GPU-less); on the robot the same ONNX exports to a
//! TensorRT engine (~2.5 ms/frame measured, vs tens of ms CPU).
//!
//! Per ADR 0014 the mask is an **enhancer, never a foundation**: the floor-level
//! cameras guarantee occluded, partial views of people, so every consumer must also
//! work maskless. Accordingly this crate only ever *removes* points, and a failed or
//! missing mask degrades to the unmasked pipeline.
//!
//! The input shape is read from the model, so both the square 640×640 export and the
//! recall-exact rect export at the camera's native shape (the survey's
//! recommendation) work unmodified — the survey measured the square letterbox
//! silently costing recall (21.6 % vs 28.6 % mask coverage on the hard set), so
//! prefer `imgsz=(480, 640)`-style exports matched to the camera.

use std::path::Path;

use half::f16;
use ort::session::builder::GraphOptimizationLevel;
use ort::session::Session;
use ort::value::Tensor;
use slam_types::{PixelMask, Stamp};

mod post;
pub use post::Letterbox;

/// The COCO classes treated as dynamic / movable (the survey's rejection set):
/// person, vehicles, animals likely indoors, carryables, chairs/couch/plant,
/// laptop, phone, sports ball. Filtering is post-hoc — the network always scores
/// all 80 classes, so a wider set costs no latency.
pub const DYNAMIC_CLASS_IDS: &[usize] = &[
    0,  // person
    1,  // bicycle
    2,  // car
    3,  // motorcycle
    5,  // bus
    6,  // train
    7,  // truck
    8,  // boat
    14, // bird
    15, // cat
    16, // dog
    17, // horse
    24, // backpack
    25, // umbrella
    26, // handbag
    28, // suitcase
    32, // sports ball
    56, // chair
    57, // couch
    58, // potted plant
    63, // laptop
    67, // cell phone
];

/// Which classes a mask rejects.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum ClassSet {
    /// People only.
    Person,
    /// The survey's dynamic-object set ([`DYNAMIC_CLASS_IDS`]) — the default.
    #[default]
    Dynamic,
    /// Explicit COCO class ids.
    Ids(Vec<usize>),
}

impl ClassSet {
    fn ids(&self) -> &[usize] {
        match self {
            ClassSet::Person => &[0],
            ClassSet::Dynamic => DYNAMIC_CLASS_IDS,
            ClassSet::Ids(ids) => ids,
        }
    }
}

/// Segmentation tuning. Defaults follow the survey's recommendation.
#[derive(Debug, Clone)]
pub struct SegConfig {
    /// Confidence threshold. Low (0.15–0.25) is the right operating point for point
    /// rejection: the error cost is asymmetric (survey finding 2).
    pub conf: f32,
    /// NMS IoU threshold (class-agnostic; masks are unioned anyway).
    pub iou: f32,
    /// Mask dilation radius in model-input pixels (applied at prototype resolution,
    /// so it rounds up to multiples of ~4 input px).
    pub dilate_px: usize,
    /// Classes to reject.
    pub classes: ClassSet,
}

impl Default for SegConfig {
    fn default() -> Self {
        SegConfig {
            conf: 0.2,
            iou: 0.45,
            dilate_px: 8,
            classes: ClassSet::Dynamic,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum SegError {
    #[error("ONNX Runtime: {0}")]
    Ort(#[from] ort::Error),
    #[error("model: {0}")]
    Model(String),
    #[error("image: {0}")]
    Image(String),
}

/// A loaded YOLO-seg model turning RGB8 frames into [`PixelMask`]s.
pub struct YoloSeg {
    session: Session,
    cfg: SegConfig,
    input_name: String,
    output_names: (String, String),
    in_w: usize,
    in_h: usize,
    fp16: bool,
}

impl YoloSeg {
    /// Load an ultralytics YOLO-seg ONNX export (static shape, batch 1). The input
    /// resolution and fp16-ness are read from the model itself.
    pub fn load(model: impl AsRef<Path>, cfg: SegConfig) -> Result<YoloSeg, SegError> {
        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)
            .map_err(ort::Error::from)?
            .commit_from_file(model.as_ref())?;

        let input = session
            .inputs()
            .first()
            .ok_or_else(|| SegError::Model("model has no inputs".into()))?;
        let input_name = input.name().to_string();
        let (dims, fp16) = tensor_shape(input.dtype())
            .ok_or_else(|| SegError::Model("input is not a tensor".into()))?;
        let [n, c, h, w] = dims[..] else {
            return Err(SegError::Model(format!(
                "expected NCHW input, got shape {dims:?}"
            )));
        };
        if n != 1 || c != 3 || h <= 0 || w <= 0 {
            return Err(SegError::Model(format!(
                "expected a static [1, 3, H, W] input (re-export with batch=1, \
                 dynamic=False — ideally imgsz matched to the camera), got {dims:?}"
            )));
        }
        if session.outputs().len() < 2 {
            return Err(SegError::Model(
                "expected detection + prototype outputs (a *-seg export)".into(),
            ));
        }
        let output_names = (
            session.outputs()[0].name().to_string(),
            session.outputs()[1].name().to_string(),
        );
        Ok(YoloSeg {
            session,
            cfg,
            input_name,
            output_names,
            in_w: w as usize,
            in_h: h as usize,
            fp16,
        })
    }

    /// The model's input resolution `(width, height)`.
    pub fn input_size(&self) -> (usize, usize) {
        (self.in_w, self.in_h)
    }

    /// Segment one packed RGB8 frame (`3·width·height` bytes) and return the dynamic
    /// mask at the *source* resolution, stamped for ingest-side pairing.
    pub fn mask_rgb8(
        &mut self,
        rgb: &[u8],
        width: usize,
        height: usize,
        stamp: Stamp,
    ) -> Result<PixelMask, SegError> {
        if width == 0 || height == 0 || rgb.len() != 3 * width * height {
            return Err(SegError::Image(format!(
                "RGB8 buffer is {} bytes, expected 3·{width}·{height}",
                rgb.len()
            )));
        }
        let lb = Letterbox::fit(width, height, self.in_w, self.in_h);
        let chw = post::letterbox_chw(rgb, width, height, &lb);

        let shape = [1usize, 3, self.in_h, self.in_w];
        let outputs = if self.fp16 {
            let data: Vec<f16> = chw.iter().map(|&v| f16::from_f32(v)).collect();
            self.session
                .run(ort::inputs![self.input_name.as_str() => Tensor::from_array((shape, data))?])?
        } else {
            self.session
                .run(ort::inputs![self.input_name.as_str() => Tensor::from_array((shape, chw))?])?
        };

        let (pred_shape, pred) = extract_f32(&outputs[self.output_names.0.as_str()])?;
        let (proto_shape, protos) = extract_f32(&outputs[self.output_names.1.as_str()])?;
        let [_, attrs, anchors] = pred_shape[..] else {
            return Err(SegError::Model(format!(
                "unexpected detection output shape {pred_shape:?}"
            )));
        };
        if attrs != 116 {
            return Err(SegError::Model(format!(
                "expected 4+80+32 = 116 attributes (COCO *-seg), got {attrs}"
            )));
        }
        let anchors = anchors as usize;
        let [_, nproto, ph, pw] = proto_shape[..] else {
            return Err(SegError::Model(format!(
                "unexpected prototype output shape {proto_shape:?}"
            )));
        };
        if nproto != 32 {
            return Err(SegError::Model(format!(
                "expected 32 prototypes, got {nproto}"
            )));
        }
        let (pw, ph) = (pw as usize, ph as usize);
        let proto_scale = self.in_w as f64 / pw as f64;

        let dets = post::decode(&pred, anchors, self.cfg.classes.ids(), self.cfg.conf);
        let dets = post::nms(dets, self.cfg.iou);
        let union = post::compose_union(&pred, anchors, &protos, pw, ph, &dets, proto_scale);
        let r = (self.cfg.dilate_px as f64 / proto_scale).ceil() as usize;
        let union = post::dilate(&union, pw, ph, r);
        let data = post::union_to_source(&union, pw, ph, &lb, proto_scale, width, height);
        Ok(PixelMask {
            stamp,
            width,
            height,
            data,
        })
    }
}

/// Static dims + fp16-ness of a tensor value type.
fn tensor_shape(vt: &ort::value::ValueType) -> Option<(Vec<i64>, bool)> {
    match vt {
        ort::value::ValueType::Tensor { ty, shape, .. } => Some((
            shape.iter().copied().collect(),
            *ty == ort::value::TensorElementType::Float16,
        )),
        _ => None,
    }
}

/// Extract any float tensor as f32 (the fp16 export keeps fp16 outputs).
fn extract_f32(value: &ort::value::Value) -> Result<(Vec<i64>, Vec<f32>), SegError> {
    if let Ok((shape, data)) = value.try_extract_tensor::<f32>() {
        return Ok((shape.iter().copied().collect(), data.to_vec()));
    }
    let (shape, data) = value.try_extract_tensor::<f16>()?;
    Ok((
        shape.iter().copied().collect(),
        data.iter().map(|v| v.to_f32()).collect(),
    ))
}
