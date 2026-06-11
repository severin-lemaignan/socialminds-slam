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
//! - [`OdomDeadReckoning`] replays the platform's wheel odometry, re-anchored at the
//!   initial pose — the floor for anything consuming odometry as a prior (ADR 0012).

#![forbid(unsafe_code)]

mod dead_reckoning;
mod odom_dead_reckoning;
mod stationary;

pub use dead_reckoning::{ImuDeadReckoning, STANDARD_GRAVITY};
pub use odom_dead_reckoning::OdomDeadReckoning;
pub use stationary::Stationary;

/// The system contract now lives in `slam-types` (front-ends implement it too);
/// re-exported here so existing `slam_baseline::SlamSystem` imports keep working.
pub use slam_types::SlamSystem;
