//! Trivial reference baselines and the [`SlamSystem`] trait they implement.
//!
//! These systems are intentionally *not* good at SLAM. Their job (ADR 0005) is to exercise
//! the whole pipeline — sensor ingest → pose estimate → TUM trajectory → `evo` metrics → CI
//! gate — before any real algorithm exists, and to provide a sanity floor every real system
//! must beat:
//!
//! - [`Stationary`] never moves. On a moving sequence its error grows without bound; it is
//!   the absolute floor.
//! - [`ImuDeadReckoning`] integrates the IMU (strapdown). It tracks motion well over short
//!   spans and drifts over long ones — exactly the behaviour the harness should reveal, and
//!   it must beat [`Stationary`] on any real trajectory.

#![forbid(unsafe_code)]

mod dead_reckoning;
mod stationary;

pub use dead_reckoning::{ImuDeadReckoning, STANDARD_GRAVITY};
pub use stationary::Stationary;

use slam_types::{ImuSample, StampedPose};

/// A SLAM system: consumes sensor samples and reports a current pose estimate.
///
/// For M0 only the IMU is wired up. As the roadmap adds front-ends, this trait grows
/// `process_scan` / `process_rgbd` methods (with default no-op bodies so existing systems
/// stay valid). Keeping the surface minimal now lets `slam-replay` drive any system
/// generically.
pub trait SlamSystem {
    /// Stable identifier used in benchmark reports and output filenames.
    fn name(&self) -> &str;

    /// Ingest one IMU sample. Samples are delivered in non-decreasing timestamp order.
    fn process_imu(&mut self, sample: &ImuSample);

    /// The best current pose estimate, stamped at the latest processed sample.
    ///
    /// Returns `None` before any input has been processed.
    fn current_estimate(&self) -> Option<StampedPose>;
}
