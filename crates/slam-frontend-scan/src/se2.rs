//! SE(2): planar rigid transforms, the scan front-end's native group (ADR 0007).

use slam_types::{Pose, Rotation, Vec2, Vec3};

/// A planar rigid transform / pose: rotation `theta` then translation `(x, y)`.
///
/// As a pose it maps *body* coordinates into the *reference* frame:
/// `p_ref = R(theta) · p_body + (x, y)` — the planar restriction of [`Pose`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Se2 {
    pub x: f64,
    pub y: f64,
    /// Heading (rad), wrapped to `(-π, π]` by all constructors and operations.
    pub theta: f64,
}

/// Wrap an angle to `(-π, π]`.
#[inline]
fn wrap(theta: f64) -> f64 {
    let mut t = theta.rem_euclid(2.0 * std::f64::consts::PI);
    if t > std::f64::consts::PI {
        t -= 2.0 * std::f64::consts::PI;
    }
    t
}

impl Se2 {
    #[inline]
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Se2 {
            x,
            y,
            theta: wrap(theta),
        }
    }

    #[inline]
    pub fn identity() -> Self {
        Se2::new(0.0, 0.0, 0.0)
    }

    /// Apply to a point: `p_ref = R(theta) p + t`.
    #[inline]
    pub fn apply(&self, p: Vec2) -> Vec2 {
        let (s, c) = self.theta.sin_cos();
        Vec2::new(c * p.x - s * p.y + self.x, s * p.x + c * p.y + self.y)
    }

    /// Compose: `(self ∘ rhs)` applies `rhs` first.
    #[inline]
    pub fn compose(&self, rhs: &Se2) -> Se2 {
        let (s, c) = self.theta.sin_cos();
        Se2::new(
            self.x + c * rhs.x - s * rhs.y,
            self.y + s * rhs.x + c * rhs.y,
            self.theta + rhs.theta,
        )
    }

    #[inline]
    pub fn inverse(&self) -> Se2 {
        let (s, c) = self.theta.sin_cos();
        Se2::new(
            -(c * self.x + s * self.y),
            -(-s * self.x + c * self.y),
            -self.theta,
        )
    }

    /// Translation magnitude (m).
    #[inline]
    pub fn translation_norm(&self) -> f64 {
        self.x.hypot(self.y)
    }

    /// Embed into SE(3): z = 0, roll = pitch = 0 — the planar front-end never invents
    /// out-of-plane motion (ADR 0007).
    pub fn to_pose(&self) -> Pose {
        Pose::new(
            Rotation::exp(Vec3::new(0.0, 0.0, self.theta)),
            Vec3::new(self.x, self.y, 0.0),
        )
    }

    /// Project an SE(3) pose onto SE(2) (x, y, yaw), reporting how non-planar it was.
    ///
    /// Used on `T_base_sensor` extrinsics from the rig (ADR 0009): a 2D lidar is modelled
    /// as scanning the base's motion plane, so its mounting roll/pitch must be ≈ 0. The
    /// second value is the out-of-plane rotation magnitude (rad) — the caller decides
    /// what tolerance deserves a warning. The z offset is dropped (planar geometry of
    /// walls is height-invariant).
    pub fn planar_projection_of(pose: &Pose) -> (Se2, f64) {
        let (roll, pitch, yaw) = pose.rotation().to_rpy();
        let t = pose.translation();
        (Se2::new(t.x, t.y, yaw), roll.hypot(pitch))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn compose_then_inverse_is_identity() {
        let a = Se2::new(1.0, -2.0, 0.7);
        let id = a.compose(&a.inverse());
        assert_relative_eq!(id.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(id.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(id.theta, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn apply_matches_compose_on_origin_offset() {
        // Pose at (1, 0) facing +Y: body +X maps to world +Y.
        let pose = Se2::new(1.0, 0.0, FRAC_PI_2);
        let p = pose.apply(Vec2::new(2.0, 0.0));
        assert_relative_eq!(p.x, 1.0, epsilon = 1e-12);
        assert_relative_eq!(p.y, 2.0, epsilon = 1e-12);
    }

    #[test]
    fn angles_wrap() {
        let a = Se2::new(0.0, 0.0, PI + 0.1);
        assert_relative_eq!(a.theta, -PI + 0.1, epsilon = 1e-12);
        let b = Se2::new(0.0, 0.0, 3.0).compose(&Se2::new(0.0, 0.0, 3.0));
        assert!(b.theta <= PI && b.theta > -PI);
    }

    #[test]
    fn to_pose_embeds_planar_motion() {
        let pose = Se2::new(1.0, 2.0, FRAC_PI_2).to_pose();
        assert_relative_eq!(
            pose.translation(),
            Vec3::new(1.0, 2.0, 0.0),
            epsilon = 1e-12
        );
        let fwd = pose.rotation().rotate(Vec3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(fwd, Vec3::new(0.0, 1.0, 0.0), epsilon = 1e-12);
    }
}
