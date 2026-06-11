//! YAML run configuration (ADR 0013): *which sensors to use*, with operational ingest
//! tuning — never calibration. Extrinsics stay in URDF/`tf_static`, intrinsics in
//! `CameraInfo` (ADR 0009); this file only selects and tunes streams.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunConfig {
    #[serde(default)]
    pub rig: RigConfig,
    #[serde(default)]
    pub sensors: SensorsConfig,
    /// Dynamics masking (ADR 0015): a YOLO-seg ONNX model rejecting dynamic objects'
    /// pixels at depth ingest. Applies to every depth sensor that has a `color:`
    /// topic. Needs a build with `--features dynamics`.
    pub masking: Option<MaskingConfig>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MaskingConfig {
    /// Path to the ONNX export (e.g. `onnx/yolo11s-seg-rect.onnx`). Static shape, batch 1;
    /// ideally exported at the camera's native shape (see ADR 0015 on letterboxing).
    pub model: PathBuf,
    /// Confidence threshold. Low is right for point rejection: a missed person
    /// corrupts the map, a false positive discards a few points.
    #[serde(default = "default_mask_conf")]
    pub conf: f32,
    /// Mask dilation in model-input pixels (boundary insurance; depth edges are
    /// noisiest exactly there).
    #[serde(default = "default_mask_dilate")]
    pub dilate_px: usize,
    /// `person` or `dynamic` (the survey's person+chairs+carryables set; default).
    #[serde(default = "default_mask_classes")]
    pub classes: crate::masking::MaskClasses,
}

fn default_mask_conf() -> f32 {
    0.2
}
fn default_mask_dilate() -> usize {
    8
}
fn default_mask_classes() -> crate::masking::MaskClasses {
    crate::masking::MaskClasses::Dynamic
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RigConfig {
    /// Where the rig comes from: the bag's `/tf_static`, a URDF file, or none.
    #[serde(default)]
    pub source: RigSource,
    /// URDF path, required when `source: urdf`.
    pub urdf: Option<PathBuf>,
    #[serde(default = "default_base_frame")]
    pub base_frame: String,
}

impl Default for RigConfig {
    fn default() -> Self {
        RigConfig {
            source: RigSource::default(),
            urdf: None,
            base_frame: default_base_frame(),
        }
    }
}

fn default_base_frame() -> String {
    "base_link".to_string()
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RigSource {
    /// Single-frame identity rig (everything is the base frame).
    #[default]
    Identity,
    /// The bag's recorded `/tf_static` (ADR 0009).
    Bag,
    /// A URDF file (`rig.urdf`).
    Urdf,
}

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorsConfig {
    #[serde(default)]
    pub scans: Vec<ScanSensor>,
    #[serde(default)]
    pub imus: Vec<ImuSensor>,
    #[serde(default)]
    pub depth: Vec<DepthSensor>,
    #[serde(default)]
    pub odometry: Vec<OdomSensor>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OdomSensor {
    pub topic: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScanSensor {
    pub topic: String,
}

/// One IMU: either a single 6-axis `topic`, or a RealSense-style split
/// `gyro_topic` + `accel_topic` pair (merged at ingest, gyro as time base).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuSensor {
    pub topic: Option<String>,
    pub gyro_topic: Option<String>,
    pub accel_topic: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DepthSensor {
    pub topic: String,
    /// Defaults to the sibling `…/camera_info` of `topic`.
    pub camera_info: Option<String>,
    /// Optional colour image topic riding with an *aligned* depth stream (e.g.
    /// `/d400/color/image_raw`): points carry per-pixel RGB for the coloured map.
    pub color: Option<String>,
    /// Range-adaptive sampling target: kept pixels are spaced ≈ this on the surface
    /// at every depth (match the 3D field's voxel size).
    #[serde(default = "default_target_spacing")]
    pub target_spacing: f64,
    /// Finest pixel stride (near-range floor).
    #[serde(default = "default_min_stride")]
    pub min_stride: usize,
    /// Per-cloud point cap (uniform re-decimation above it).
    #[serde(default = "default_max_points")]
    pub max_points: usize,
    #[serde(default = "default_min_range")]
    pub min_range: f64,
    #[serde(default = "default_max_range")]
    pub max_range: f64,
    /// Keep every Nth frame (30 fps depth is redundant at ≤ 2 m/s).
    #[serde(default = "default_every_nth")]
    pub every_nth: usize,
}

fn default_target_spacing() -> f64 {
    0.05
}
fn default_min_stride() -> usize {
    2
}
fn default_max_points() -> usize {
    20_000
}
fn default_min_range() -> f64 {
    0.3
}
fn default_max_range() -> f64 {
    6.0
}
fn default_every_nth() -> usize {
    3
}

impl DepthSensor {
    /// The CameraInfo topic: explicit, or the RealSense-layout sibling.
    pub fn info_topic(&self) -> String {
        self.camera_info
            .clone()
            .unwrap_or_else(|| match self.topic.rfind('/') {
                Some(i) => format!("{}/camera_info", &self.topic[..i]),
                None => format!("{}/camera_info", self.topic),
            })
    }
}

impl RunConfig {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading run config {}", path.display()))?;
        let cfg: RunConfig = serde_yaml::from_str(&text)
            .with_context(|| format!("parsing run config {}", path.display()))?;
        for imu in &cfg.sensors.imus {
            let split = imu.gyro_topic.is_some() || imu.accel_topic.is_some();
            match (&imu.topic, split) {
                (Some(_), true) => anyhow::bail!(
                    "run config: an IMU is either `topic` or a `gyro_topic`+`accel_topic` pair"
                ),
                (None, true) if imu.gyro_topic.is_none() || imu.accel_topic.is_none() => {
                    anyhow::bail!("run config: split IMU needs both gyro_topic and accel_topic")
                }
                (None, false) => {
                    anyhow::bail!("run config: IMU entry needs `topic` or a gyro/accel pair")
                }
                _ => {}
            }
        }
        if cfg.rig.source == RigSource::Urdf && cfg.rig.urdf.is_none() {
            anyhow::bail!("run config: rig.source is `urdf` but rig.urdf is not set");
        }
        Ok(cfg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_reference_shape() {
        let yaml = r#"
rig:
  source: bag
sensors:
  scans:
    - topic: /scan
  imus:
    - gyro_topic: /d400/gyro/sample
      accel_topic: /d400/accel/sample
  depth:
    - topic: /d400/aligned_depth_to_color/image_raw
      every_nth: 3
"#;
        let cfg: RunConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(cfg.sensors.depth[0].target_spacing, 0.05); // default
        assert_eq!(cfg.rig.source, RigSource::Bag);
        assert_eq!(cfg.sensors.scans[0].topic, "/scan");
        assert_eq!(
            cfg.sensors.depth[0].info_topic(),
            "/d400/aligned_depth_to_color/camera_info"
        );
        assert_eq!(cfg.sensors.depth[0].max_range, 6.0); // default
    }

    #[test]
    fn parses_the_masking_section_with_defaults() {
        let yaml = r#"
sensors:
  depth:
    - topic: /d400/aligned_depth_to_color/image_raw
      color: /d400/color/image_raw
masking:
  model: onnx/yolo11s-seg-rect.onnx
"#;
        let cfg: RunConfig = serde_yaml::from_str(yaml).unwrap();
        let m = cfg.masking.expect("masking section parses");
        assert_eq!(m.model, PathBuf::from("onnx/yolo11s-seg-rect.onnx"));
        assert_eq!(m.conf, 0.2);
        assert_eq!(m.dilate_px, 8);
        assert_eq!(m.classes, crate::masking::MaskClasses::Dynamic);
        // And the explicit form round-trips.
        let yaml = "masking:\n  model: m.onnx\n  conf: 0.15\n  dilate_px: 4\n  classes: person\n";
        let cfg: RunConfig = serde_yaml::from_str(yaml).unwrap();
        let m = cfg.masking.unwrap();
        assert_eq!(
            (m.conf, m.dilate_px, m.classes),
            (0.15, 4, crate::masking::MaskClasses::Person)
        );
    }

    #[test]
    fn rejects_calibration_like_keys() {
        // The ADR 0013 firewall: unknown fields (e.g. extrinsics) are errors.
        let yaml = "sensors:\n  scans:\n    - topic: /scan\n      translation: [1, 2, 3]\n";
        assert!(serde_yaml::from_str::<RunConfig>(yaml).is_err());
    }

    #[test]
    fn imu_must_be_single_or_pair() {
        let yaml = "sensors:\n  imus:\n    - gyro_topic: /g\n";
        let cfg: Result<RunConfig> = (|| {
            let c: RunConfig = serde_yaml::from_str(yaml)?;
            for imu in &c.sensors.imus {
                if imu.topic.is_none() && (imu.gyro_topic.is_none() || imu.accel_topic.is_none()) {
                    anyhow::bail!("incomplete");
                }
            }
            Ok(c)
        })();
        assert!(cfg.is_err());
    }
}
