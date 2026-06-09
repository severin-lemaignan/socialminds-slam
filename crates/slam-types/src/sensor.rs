//! Sensor sample types.
//!
//! These are the inputs the engine consumes. For now only the IMU is needed (M0
//! baselines); lidar scans, RGB-D frames, and wheel odometry land as the roadmap
//! reaches their front-ends.

use crate::geometry::Vec3;
use crate::time::Stamp;

/// A single inertial measurement: angular rate (gyroscope) and proper acceleration
/// (accelerometer), both in the IMU body frame, SI units.
///
/// Sign/frame convention: `accel` is the *specific force* measured by the accelerometer,
/// i.e. it includes the reaction to gravity (a stationary, level IMU reads `+g` upward,
/// `≈ (0, 0, 9.81)`). Strapdown integration subtracts gravity explicitly — see
/// `slam-baseline`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub stamp: Stamp,
    /// Angular velocity ω (rad/s), body frame.
    pub gyro: Vec3,
    /// Specific force / proper acceleration (m/s²), body frame.
    pub accel: Vec3,
}

impl ImuSample {
    #[inline]
    pub fn new(stamp: Stamp, gyro: Vec3, accel: Vec3) -> Self {
        ImuSample { stamp, gyro, accel }
    }
}
