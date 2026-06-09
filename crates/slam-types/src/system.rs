//! The [`SlamSystem`] trait — the contract between sensor replay and any estimator.
//!
//! Lives in `slam-types` (not a front-end crate) so baselines, front-ends, and the fused
//! engine are all driven identically by `slam-replay` and benchmarked by the harness.

use crate::sensor::{ImuSample, LaserScan2D};
use crate::trajectory::StampedPose;

/// A SLAM system: consumes sensor samples and reports a current pose estimate.
///
/// Sensor methods have no-op defaults: a system overrides what it consumes and silently
/// ignores the rest, so `slam-replay` can feed every available stream to any system.
/// Samples of each stream are delivered in non-decreasing timestamp order.
pub trait SlamSystem {
    /// Stable identifier used in benchmark reports and output filenames.
    fn name(&self) -> &str;

    /// Ingest one IMU sample.
    fn process_imu(&mut self, _sample: &ImuSample) {}

    /// Ingest one planar laser scan.
    fn process_scan(&mut self, _scan: &LaserScan2D) {}

    /// The best current pose estimate, stamped at the latest processed sample.
    ///
    /// Returns `None` before any input has been processed.
    fn current_estimate(&self) -> Option<StampedPose>;
}
