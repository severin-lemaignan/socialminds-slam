//! Wheel-odometry dead-reckoning baseline.
//!
//! Replays the platform's own odometry stream, re-anchored at the run's initial pose:
//! `est(t) = init ∘ (odom(t₀)⁻¹ ∘ odom(t))`. No fusion, no correction — the estimate
//! inherits the odometry's every flaw (wheel slip, scale error, yaw bias), which is the
//! point (ADR 0005): it is the floor any system *consuming* wheel odometry as a prior
//! (ADR 0012) must beat, and the OpenLORIS paper's own reference baseline.
//!
//! The absolute frame of the incoming stream never matters: only relative motion since
//! the first sample is used (the convention `OdomSample` documents).

use slam_types::{OdomSample, Pose, StampedPose};

use crate::SlamSystem;

#[derive(Debug, Clone)]
pub struct OdomDeadReckoning {
    initial: Pose,
    /// The first sample's pose in the odometry frame — the re-anchoring origin.
    first: Option<Pose>,
    last: Option<StampedPose>,
}

impl OdomDeadReckoning {
    /// Start at the identity pose.
    pub fn new() -> Self {
        OdomDeadReckoning::with_initial_pose(Pose::identity())
    }

    /// Start from a known pose, so benchmarks measure *drift*, not initialisation error.
    pub fn with_initial_pose(initial: Pose) -> Self {
        OdomDeadReckoning {
            initial,
            first: None,
            last: None,
        }
    }
}

impl Default for OdomDeadReckoning {
    fn default() -> Self {
        OdomDeadReckoning::new()
    }
}

impl SlamSystem for OdomDeadReckoning {
    fn name(&self) -> &str {
        "odom_dead_reckoning"
    }

    fn process_odometry(&mut self, odom: &OdomSample) {
        let first = *self.first.get_or_insert(odom.pose);
        let pose = self.initial * (first.inverse() * odom.pose);
        self.last = Some(StampedPose::new(odom.stamp, pose));
    }

    fn current_estimate(&self) -> Option<StampedPose> {
        self.last
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slam_types::{FrameId, Rotation, Stamp, Vec3};
    use std::f64::consts::FRAC_PI_2;

    fn sample(t: f64, pose: Pose) -> OdomSample {
        OdomSample {
            stamp: Stamp::from_seconds(t),
            frame: FrameId::BASE,
            pose,
        }
    }

    #[test]
    fn none_before_any_input() {
        assert!(OdomDeadReckoning::new().current_estimate().is_none());
    }

    #[test]
    fn replays_relative_motion_from_identity() {
        let mut dr = OdomDeadReckoning::new();
        dr.process_odometry(&sample(0.0, Pose::identity()));
        let step = Pose::new(Rotation::identity(), Vec3::new(1.0, 2.0, 0.0));
        dr.process_odometry(&sample(1.0, step));
        let est = dr.current_estimate().unwrap();
        assert_relative_eq!(
            est.pose.translation(),
            Vec3::new(1.0, 2.0, 0.0),
            epsilon = 1e-12
        );
    }

    /// The odometry frame's arbitrary origin must cancel: only motion *since the first
    /// sample* shows up in the estimate.
    #[test]
    fn absolute_odometry_frame_cancels() {
        let offset = Pose::new(
            Rotation::exp(Vec3::new(0.0, 0.0, FRAC_PI_2)),
            Vec3::new(-7.0, 3.0, 0.5),
        );
        let motion = Pose::new(
            Rotation::exp(Vec3::new(0.0, 0.0, 0.3)),
            Vec3::new(2.0, 0.0, 0.0),
        );
        let mut dr = OdomDeadReckoning::new();
        dr.process_odometry(&sample(0.0, offset));
        dr.process_odometry(&sample(1.0, offset * motion));
        let est = dr.current_estimate().unwrap();
        assert_relative_eq!(
            est.pose.translation(),
            motion.translation(),
            epsilon = 1e-12
        );
        assert_relative_eq!(
            est.pose.rotation().log(),
            motion.rotation().log(),
            epsilon = 1e-12
        );
    }

    #[test]
    fn initial_pose_re_anchors_the_stream() {
        let init = Pose::new(
            Rotation::exp(Vec3::new(0.0, 0.0, FRAC_PI_2)),
            Vec3::new(10.0, 0.0, 0.0),
        );
        let mut dr = OdomDeadReckoning::with_initial_pose(init);
        dr.process_odometry(&sample(0.0, Pose::identity()));
        // 1 m forward in the odometry frame → 1 m along the rotated +Y in the world.
        dr.process_odometry(&sample(
            1.0,
            Pose::new(Rotation::identity(), Vec3::new(1.0, 0.0, 0.0)),
        ));
        let est = dr.current_estimate().unwrap();
        assert_relative_eq!(
            est.pose.translation(),
            Vec3::new(10.0, 1.0, 0.0),
            epsilon = 1e-12
        );
    }
}
