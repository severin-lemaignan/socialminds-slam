//! 2D planar scan-matching front-end (ADR 0007, extended by ADR 0010).
//!
//! The planar backbone of the engine (ADR 0002): trimmed point-to-line ICP
//! ([`match_scans`]) wrapped in scan-to-keyframe odometry ([`ScanOdometry`], a
//! [`slam_types::SlamSystem`]). Estimates the planar motion of a **3D body**: beams are
//! lifted through their sensor's SE(3) rig extrinsic and the IMU's gravity tilt
//! ([`AttitudeFilter`]) before matching, so an accelerating (tilting) base does not
//! corrupt the planar solve — and the lidar still never *invents* out-of-plane motion.

#![forbid(unsafe_code)]

mod attitude;
mod icp;
mod odometry;
mod scan_to_map;
mod se2;

pub use attitude::{AttitudeConfig, AttitudeFilter};
pub use icp::{match_scans, MatchConfig, MatchResult, ScanMatcher};
pub use odometry::{ScanOdometry, ScanOdometryConfig, ScanOdometryStats};
pub use scan_to_map::{LoopClosure, ScanToMapConfig, ScanToMapOdometry};
pub use se2::Se2;
