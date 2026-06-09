//! The stationary baseline: report a fixed pose forever.

use slam_types::{ImuSample, Pose, Stamp, StampedPose};

use crate::SlamSystem;

/// Reports a constant pose (identity by default), only advancing its timestamp as samples
/// arrive. The sanity floor: any system that cannot beat this is broken.
#[derive(Debug, Clone)]
pub struct Stationary {
    pose: Pose,
    last_stamp: Option<Stamp>,
}

impl Stationary {
    /// Anchored at the identity pose.
    pub fn new() -> Self {
        Stationary {
            pose: Pose::identity(),
            last_stamp: None,
        }
    }

    /// Anchored at a given pose (e.g. a known ground-truth start).
    pub fn anchored_at(pose: Pose) -> Self {
        Stationary {
            pose,
            last_stamp: None,
        }
    }
}

impl Default for Stationary {
    fn default() -> Self {
        Stationary::new()
    }
}

impl SlamSystem for Stationary {
    fn name(&self) -> &str {
        "stationary"
    }

    fn process_imu(&mut self, sample: &ImuSample) {
        self.last_stamp = Some(sample.stamp);
    }

    fn current_estimate(&self) -> Option<StampedPose> {
        self.last_stamp
            .map(|stamp| StampedPose::new(stamp, self.pose))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use slam_types::Vec3;

    #[test]
    fn none_before_any_input() {
        assert!(Stationary::new().current_estimate().is_none());
    }

    #[test]
    fn pose_is_fixed_timestamp_advances() {
        let mut s = Stationary::new();
        s.process_imu(&ImuSample::new(
            Stamp::from_seconds(1.0),
            Vec3::zeros(),
            Vec3::new(0.0, 0.0, 9.81),
        ));
        s.process_imu(&ImuSample::new(
            Stamp::from_seconds(2.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(5.0, 5.0, 5.0),
        ));
        let est = s.current_estimate().unwrap();
        assert_eq!(est.stamp, Stamp::from_seconds(2.0));
        assert_eq!(est.pose.translation(), Vec3::zeros());
    }
}
