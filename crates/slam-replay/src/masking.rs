//! Dynamics masking at depth ingest (ADR 0015): a YOLO-seg ONNX model runs on the
//! colour frame paired with each kept depth frame and its mask rejects dynamic
//! objects' pixels before back-projection — people never become points, so they
//! never reach registration or the map.
//!
//! Compiled only with `--features dynamics` (ONNX Runtime is a C++ dependency,
//! kept opt-in like the rerun stack); the stub keeps the CLI surface identical and
//! turns `--mask-model` into a clear error otherwise. Per ADR 0014 the mask is an
//! enhancer: any inference failure degrades that frame to unmasked ingest.

use std::path::PathBuf;

use anyhow::Result;

/// Which classes the mask rejects (`--mask-classes` / config `classes:`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MaskClasses {
    /// People only.
    Person,
    /// The survey's dynamic-object set: person + chairs/couch/plant, carryables,
    /// animals, vehicles (docs/REPORT_HUMAN_DETECTION.md).
    Dynamic,
}

/// Resolved masking settings (from `--mask-*` flags or the YAML `masking:` section).
/// Without the `dynamics` feature the stub bails before reading any field.
#[derive(Debug, Clone)]
#[cfg_attr(not(feature = "dynamics"), allow(dead_code))]
pub struct MaskSettings {
    pub model: PathBuf,
    pub conf: f32,
    pub dilate_px: usize,
    pub classes: MaskClasses,
}

#[cfg(feature = "dynamics")]
pub use real::Masking;

#[cfg(feature = "dynamics")]
mod real {
    use super::*;
    use slam_datasets::ColorImage;
    use slam_types::PixelMask;

    pub struct Masking {
        seg: slam_dynamics::YoloSeg,
        frames: usize,
        errors: usize,
        coverage_sum: f64,
    }

    impl Masking {
        pub fn new(settings: &MaskSettings) -> Result<Masking> {
            let cfg = slam_dynamics::SegConfig {
                conf: settings.conf,
                dilate_px: settings.dilate_px,
                classes: match settings.classes {
                    MaskClasses::Person => slam_dynamics::ClassSet::Person,
                    MaskClasses::Dynamic => slam_dynamics::ClassSet::Dynamic,
                },
                ..Default::default()
            };
            let seg = slam_dynamics::YoloSeg::load(&settings.model, cfg).map_err(|e| {
                anyhow::anyhow!("loading mask model {}: {e}", settings.model.display())
            })?;
            let (w, h) = seg.input_size();
            eprintln!(
                "slam-replay: dynamics mask: {} ({w}×{h} input, conf {}, dilate {} px, \
                 {:?} classes)",
                settings.model.display(),
                settings.conf,
                settings.dilate_px,
                settings.classes,
            );
            Ok(Masking {
                seg,
                frames: 0,
                errors: 0,
                coverage_sum: 0.0,
            })
        }

        /// Mask one colour frame; `None` (after a warning) on inference failure —
        /// the frame ingests unmasked (ADR 0014).
        pub fn mask(&mut self, color: &ColorImage) -> Option<PixelMask> {
            let rgb = color.to_rgb8();
            match self
                .seg
                .mask_rgb8(&rgb, color.width(), color.height(), color.stamp)
            {
                Ok(mask) => {
                    self.frames += 1;
                    self.coverage_sum += mask.coverage();
                    Some(mask)
                }
                Err(e) => {
                    if self.errors == 0 {
                        eprintln!("slam-replay: warning: dynamics mask failed ({e}); affected frames ingest unmasked");
                    }
                    self.errors += 1;
                    None
                }
            }
        }

        pub fn summary(&self) {
            if self.frames == 0 {
                eprintln!(
                    "slam-replay: warning: dynamics mask never ran — masking needs a \
                     depth stream with a colour topic (`color:` / --color-topic)"
                );
                return;
            }
            eprintln!(
                "slam-replay: dynamics mask: {} frames, mean coverage {:.1} %{}",
                self.frames,
                100.0 * self.coverage_sum / self.frames as f64,
                if self.errors > 0 {
                    format!(", {} inference failures", self.errors)
                } else {
                    String::new()
                },
            );
        }
    }
}

#[cfg(not(feature = "dynamics"))]
pub use stub::Masking;

#[cfg(not(feature = "dynamics"))]
mod stub {
    use super::*;

    /// CLI-compatible stub: masking without the `dynamics` feature is a clear error.
    pub struct Masking;

    impl Masking {
        pub fn new(_settings: &MaskSettings) -> Result<Masking> {
            anyhow::bail!(
                "slam-replay was built without dynamics masking; rebuild with \
                 `cargo build --release -p slam-replay --features dynamics`"
            )
        }

        pub fn mask(
            &mut self,
            _color: &slam_datasets::ColorImage,
        ) -> Option<slam_types::PixelMask> {
            None
        }

        pub fn summary(&self) {}
    }
}
