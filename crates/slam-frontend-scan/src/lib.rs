//! 2D planar scan-matching front-end (ADR 0007).
//!
//! The planar backbone of the engine (ADR 0002): trimmed point-to-line ICP
//! ([`match_scans`]) wrapped in scan-to-keyframe odometry ([`ScanOdometry`], a
//! [`slam_types::SlamSystem`]). Estimates SE(2), embedded into SE(3) at the output —
//! the 2D lidars cannot observe out-of-plane motion and this crate never invents it.

#![forbid(unsafe_code)]

mod icp;
mod odometry;
mod se2;

pub use icp::{match_scans, MatchConfig, MatchResult, ScanMatcher};
pub use odometry::{ScanOdometry, ScanOdometryConfig, ScanOdometryStats};
pub use se2::Se2;
