//! Rigid-body geometry: 3D rotations and poses.
//!
//! We wrap `nalgebra`'s `UnitQuaternion` and `Isometry3` in thin newtypes ([`Rotation`],
//! [`Pose`]) so the engine speaks in SLAM vocabulary (SE(3)/SO(3), `exp`/`log`) and so we
//! can swap the backing representation later without touching call sites. All quantities
//! are `f64`.

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

/// A 3D vector (point, velocity, angular rate, …).
pub type Vec3 = Vector3<f64>;

/// A 3D rotation, i.e. an element of SO(3), stored as a unit quaternion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rotation(UnitQuaternion<f64>);

impl Rotation {
    #[inline]
    pub fn identity() -> Self {
        Rotation(UnitQuaternion::identity())
    }

    #[inline]
    pub fn from_quaternion(q: UnitQuaternion<f64>) -> Self {
        Rotation(q)
    }

    /// From `(x, y, z, w)` as stored in TUM/ROS trajectory files. Normalised on ingest.
    #[inline]
    pub fn from_xyzw(x: f64, y: f64, z: f64, w: f64) -> Self {
        Rotation(UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            w, x, y, z,
        )))
    }

    /// `(x, y, z, w)` ordering, matching TUM/ROS.
    #[inline]
    pub fn to_xyzw(self) -> [f64; 4] {
        let q = self.0.quaternion();
        [q.i, q.j, q.k, q.w]
    }

    /// Exponential map: a rotation vector (axis × angle, radians) → SO(3).
    #[inline]
    pub fn exp(omega: Vec3) -> Self {
        Rotation(UnitQuaternion::new(omega))
    }

    /// Logarithm map: SO(3) → rotation vector (axis × angle, radians).
    #[inline]
    pub fn log(self) -> Vec3 {
        self.0.scaled_axis()
    }

    #[inline]
    pub fn inverse(self) -> Self {
        Rotation(self.0.inverse())
    }

    #[inline]
    pub fn rotate(self, v: Vec3) -> Vec3 {
        self.0 * v
    }

    #[inline]
    pub fn as_quaternion(self) -> UnitQuaternion<f64> {
        self.0
    }
}

impl std::ops::Mul for Rotation {
    type Output = Rotation;
    #[inline]
    fn mul(self, rhs: Rotation) -> Rotation {
        Rotation(self.0 * rhs.0)
    }
}

/// A rigid-body pose, i.e. an element of SE(3): rotation + translation.
///
/// Interpreted as the transform that maps points in the *body* frame into the *reference*
/// frame (`p_ref = pose * p_body`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose(Isometry3<f64>);

impl Pose {
    #[inline]
    pub fn identity() -> Self {
        Pose(Isometry3::identity())
    }

    #[inline]
    pub fn new(rotation: Rotation, translation: Vec3) -> Self {
        Pose(Isometry3::from_parts(
            Translation3::from(translation),
            rotation.as_quaternion(),
        ))
    }

    #[inline]
    pub fn rotation(&self) -> Rotation {
        Rotation(self.0.rotation)
    }

    #[inline]
    pub fn translation(&self) -> Vec3 {
        self.0.translation.vector
    }

    #[inline]
    pub fn inverse(&self) -> Pose {
        Pose(self.0.inverse())
    }

    /// Transform a point from the body frame into the reference frame.
    #[inline]
    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        (self.0 * nalgebra::Point3::from(p)).coords
    }

    #[inline]
    pub fn as_isometry(&self) -> &Isometry3<f64> {
        &self.0
    }
}

impl std::ops::Mul for Pose {
    type Output = Pose;
    /// Compose two poses (`self` then... no — standard convention: `(a*b)` applies `b` first).
    #[inline]
    fn mul(self, rhs: Pose) -> Pose {
        Pose(self.0 * rhs.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn rotation_exp_log_roundtrip() {
        let omega = Vec3::new(0.1, -0.4, 0.7);
        let r = Rotation::exp(omega);
        assert_relative_eq!(r.log(), omega, epsilon = 1e-12);
    }

    #[test]
    fn quaternion_xyzw_roundtrip() {
        let r = Rotation::from_xyzw(0.0, 0.0, (FRAC_PI_2 / 2.0).sin(), (FRAC_PI_2 / 2.0).cos());
        let [x, y, z, w] = r.to_xyzw();
        assert_relative_eq!(x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(z, (FRAC_PI_2 / 2.0).sin(), epsilon = 1e-12);
        assert_relative_eq!(w, (FRAC_PI_2 / 2.0).cos(), epsilon = 1e-12);
    }

    #[test]
    fn quarter_turn_about_z_maps_x_to_y() {
        let r = Rotation::exp(Vec3::new(0.0, 0.0, FRAC_PI_2));
        let v = r.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(v, Vec3::new(0.0, 1.0, 0.0), epsilon = 1e-12);
    }

    #[test]
    fn pose_inverse_composes_to_identity() {
        let p = Pose::new(
            Rotation::exp(Vec3::new(0.2, 0.3, -0.1)),
            Vec3::new(1.0, 2.0, 3.0),
        );
        let id = p * p.inverse();
        assert_relative_eq!(id.translation(), Vec3::zeros(), epsilon = 1e-12);
        assert_relative_eq!(id.rotation().log(), Vec3::zeros(), epsilon = 1e-12);
    }

    #[test]
    fn pose_transforms_point_with_rotation_then_translation() {
        let p = Pose::new(
            Rotation::exp(Vec3::new(0.0, 0.0, FRAC_PI_2)),
            Vec3::new(10.0, 0.0, 0.0),
        );
        // (1,0,0) rotates to (0,1,0), then translates to (10,1,0).
        assert_relative_eq!(
            p.transform_point(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(10.0, 1.0, 0.0),
            epsilon = 1e-12
        );
    }
}
