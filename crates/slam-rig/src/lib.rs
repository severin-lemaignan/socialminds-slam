//! Sensor rig model (ADR 0009): where every sensor sits on the robot.
//!
//! The **URDF is the primary geometric source** — the same file Nav2/ROS already
//! require — parsed directly (pure XML, no ROS runtime). At startup the fixed-joint
//! kinematic tree is flattened into a [`SensorRig`]: an interned table of frame names
//! (URDF link names, the values sensors put in `header.frame_id`) with their rigid
//! `T_base_frame` extrinsics. Measurements are then tagged with a cheap
//! [`FrameId`](slam_types::FrameId) at ingest and front-ends look extrinsics up by index.
//!
//! Frames behind a *non-fixed* joint (wheels, a future pan-tilt head) are deliberately
//! not resolvable: a sensor mounted there is not rigid w.r.t. the base, which this model
//! does not represent (ADR 0009's "revisit when").

#![forbid(unsafe_code)]

use std::collections::BTreeMap;
use std::path::Path;

use slam_types::{FrameId, Pose, Rotation, Vec3};

/// Errors from building a rig out of a URDF.
#[derive(Debug, thiserror::Error)]
pub enum RigError {
    #[error("reading URDF: {0}")]
    Urdf(String),
    #[error("base frame {0:?} is not a link of the URDF")]
    BaseFrameMissing(String),
}

/// The robot's rigid sensor geometry: frame names ↔ [`FrameId`]s ↔ `T_base_frame`.
///
/// [`FrameId::BASE`] is always the base frame itself, with an identity extrinsic.
#[derive(Debug, Clone)]
pub struct SensorRig {
    /// Frame names by id; `names[0]` is the base frame.
    names: Vec<String>,
    /// `T_base_frame` by id (maps points in the frame into the base frame).
    extrinsics: Vec<Pose>,
}

impl SensorRig {
    /// The trivial single-frame rig: everything is the base frame. This is the implicit
    /// default when no URDF is given — existing single-sensor pipelines are unchanged.
    pub fn identity() -> Self {
        SensorRig {
            names: vec!["base_link".to_string()],
            extrinsics: vec![Pose::identity()],
        }
    }

    /// Build a rig from a URDF file, with `base_frame` as the body frame.
    pub fn from_urdf_file<P: AsRef<Path>>(path: P, base_frame: &str) -> Result<Self, RigError> {
        let robot = urdf_rs::read_file(path).map_err(|e| RigError::Urdf(e.to_string()))?;
        Self::from_robot(&robot, base_frame)
    }

    /// Build a rig from URDF XML, with `base_frame` as the body frame.
    pub fn from_urdf_str(urdf: &str, base_frame: &str) -> Result<Self, RigError> {
        let robot = urdf_rs::read_from_string(urdf).map_err(|e| RigError::Urdf(e.to_string()))?;
        Self::from_robot(&robot, base_frame)
    }

    fn from_robot(robot: &urdf_rs::Robot, base_frame: &str) -> Result<Self, RigError> {
        if !robot.links.iter().any(|l| l.name == base_frame) {
            return Err(RigError::BaseFrameMissing(base_frame.to_string()));
        }
        let edges: Vec<(String, String, Pose)> = robot
            .joints
            .iter()
            .filter(|j| j.joint_type == urdf_rs::JointType::Fixed)
            .map(|j| {
                (
                    j.parent.link.clone(),
                    j.child.link.clone(),
                    urdf_pose(&j.origin),
                )
            })
            .collect();
        Self::from_edges(base_frame, &edges)
    }

    /// Build a rig from raw rigid parent→child transforms — e.g. a recorded
    /// `/tf_static` stream, the bag-side counterpart of the URDF's fixed joints
    /// (ADR 0009). `transforms` items are `(parent, child, T_parent_child)`.
    pub fn from_transforms(
        base_frame: &str,
        transforms: &[(String, String, Pose)],
    ) -> Result<Self, RigError> {
        if !transforms
            .iter()
            .any(|(p, c, _)| p == base_frame || c == base_frame)
        {
            return Err(RigError::BaseFrameMissing(base_frame.to_string()));
        }
        Self::from_edges(base_frame, transforms)
    }

    fn from_edges(base_frame: &str, edges: &[(String, String, Pose)]) -> Result<Self, RigError> {
        // Undirected adjacency: frame → (neighbour, T_frame_neighbour) — undirected so
        // any frame of the rigid assembly can serve as the base.
        let mut adj: BTreeMap<&str, Vec<(&str, Pose)>> = BTreeMap::new();
        for (parent, child, t) in edges {
            adj.entry(parent.as_str())
                .or_default()
                .push((child.as_str(), *t));
            adj.entry(child.as_str())
                .or_default()
                .push((parent.as_str(), t.inverse()));
        }

        // BFS from the base over the rigid assembly, composing T_base_frame as we go.
        let mut names = vec![base_frame.to_string()];
        let mut extrinsics = vec![Pose::identity()];
        let mut queue = std::collections::VecDeque::from([(base_frame, Pose::identity())]);
        let mut seen = std::collections::BTreeSet::from([base_frame]);
        while let Some((link, t_base_link)) = queue.pop_front() {
            for (neighbour, t_link_neighbour) in adj.get(link).into_iter().flatten() {
                if seen.insert(neighbour) {
                    let t = t_base_link * *t_link_neighbour;
                    names.push((*neighbour).to_string());
                    extrinsics.push(t);
                    queue.push_back((neighbour, t));
                }
            }
        }
        Ok(SensorRig { names, extrinsics })
    }

    /// Resolve a frame name (a `header.frame_id`) to its id, if it is part of the rigid
    /// assembly around the base.
    pub fn resolve(&self, frame_name: &str) -> Option<FrameId> {
        self.names
            .iter()
            .position(|n| n == frame_name)
            .map(|i| FrameId(i as u32))
    }

    /// `T_base_frame`: maps points expressed in `frame` into the base frame.
    pub fn extrinsic(&self, frame: FrameId) -> Pose {
        self.extrinsics[frame.0 as usize]
    }

    pub fn frame_name(&self, frame: FrameId) -> &str {
        &self.names[frame.0 as usize]
    }

    /// Number of frames (≥ 1; the base frame is always frame 0).
    pub fn len(&self) -> usize {
        self.names.len()
    }

    pub fn is_empty(&self) -> bool {
        false // a rig always has at least the base frame
    }

    /// All `T_base_frame` extrinsics, indexed by `FrameId.0` — the table front-ends
    /// consume directly.
    pub fn extrinsics(&self) -> &[Pose] {
        &self.extrinsics
    }
}

/// URDF `<origin xyz rpy>` → [`Pose`]. URDF rpy is fixed-axis roll-pitch-yaw:
/// `R = Rz(yaw)·Ry(pitch)·Rx(roll)`, which is exactly nalgebra's `from_euler_angles`.
fn urdf_pose(origin: &urdf_rs::Pose) -> Pose {
    let [r, p, y] = origin.rpy.0;
    let [x, yy, z] = origin.xyz.0;
    Pose::new(Rotation::from_rpy(r, p, y), Vec3::new(x, yy, z))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_4;

    const DUAL_LIDAR_URDF: &str = r#"
<robot name="testbot">
  <link name="base_link"/>
  <link name="mast"/>
  <link name="laser_front_left"/>
  <link name="laser_rear_right"/>
  <link name="camera_front"/>
  <link name="wheel_left"/>
  <joint name="mast_joint" type="fixed">
    <parent link="base_link"/> <child link="mast"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
  </joint>
  <joint name="laser_fl_joint" type="fixed">
    <parent link="mast"/> <child link="laser_front_left"/>
    <origin xyz="0.31 0.22 0.08" rpy="0 0 0.7853981633974483"/>
  </joint>
  <joint name="laser_rr_joint" type="fixed">
    <parent link="base_link"/> <child link="laser_rear_right"/>
    <origin xyz="-0.31 -0.22 0.18" rpy="0 0 3.9269908169872414"/>
  </joint>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/> <child link="camera_front"/>
    <origin xyz="0.35 0 0.95" rpy="0 0 0"/>
  </joint>
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/> <child link="wheel_left"/>
    <origin xyz="0 0.25 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
"#;

    #[test]
    fn base_frame_is_frame_zero_with_identity() {
        let rig = SensorRig::from_urdf_str(DUAL_LIDAR_URDF, "base_link").unwrap();
        assert_eq!(rig.resolve("base_link"), Some(FrameId::BASE));
        let t = rig.extrinsic(FrameId::BASE);
        assert!(t.translation().norm() < 1e-12);
    }

    #[test]
    fn corner_lidar_extrinsic_maps_sensor_points_into_base() {
        let rig = SensorRig::from_urdf_str(DUAL_LIDAR_URDF, "base_link").unwrap();
        let id = rig.resolve("laser_front_left").expect("lidar resolvable");
        let t = rig.extrinsic(id);
        // A return 1 m ahead of the sensor (+X), with the sensor yawed +45° at the
        // front-left corner, lands at corner + (cos45, sin45) in base coordinates.
        let p = t.transform_point(Vec3::new(1.0, 0.0, 0.0));
        assert!((p.x - (0.31 + FRAC_PI_4.cos())).abs() < 1e-12);
        assert!((p.y - (0.22 + FRAC_PI_4.sin())).abs() < 1e-12);
        // Chain through the mast: 0.10 (mast) + 0.08 (sensor on mast).
        assert!((p.z - 0.18).abs() < 1e-12);
    }

    #[test]
    fn fixed_chains_compose_through_intermediate_links() {
        let rig = SensorRig::from_urdf_str(DUAL_LIDAR_URDF, "base_link").unwrap();
        let mast = rig.resolve("mast").unwrap();
        assert!((rig.extrinsic(mast).translation().z - 0.10).abs() < 1e-12);
    }

    #[test]
    fn non_fixed_joints_are_not_part_of_the_rig() {
        let rig = SensorRig::from_urdf_str(DUAL_LIDAR_URDF, "base_link").unwrap();
        assert_eq!(rig.resolve("wheel_left"), None);
    }

    #[test]
    fn any_link_of_the_rigid_assembly_can_be_the_base() {
        // Using the mast as base: base_link sits 0.10 m *below* it.
        let rig = SensorRig::from_urdf_str(DUAL_LIDAR_URDF, "mast").unwrap();
        let base = rig.resolve("base_link").unwrap();
        assert!((rig.extrinsic(base).translation().z - (-0.10)).abs() < 1e-12);
    }

    #[test]
    fn missing_base_frame_is_an_error() {
        let err = SensorRig::from_urdf_str(DUAL_LIDAR_URDF, "nope").unwrap_err();
        assert!(matches!(err, RigError::BaseFrameMissing(_)));
    }

    #[test]
    fn unknown_frame_does_not_resolve() {
        let rig = SensorRig::from_urdf_str(DUAL_LIDAR_URDF, "base_link").unwrap();
        assert_eq!(rig.resolve("laser_imaginary"), None);
    }

    #[test]
    fn identity_rig_is_a_single_base_frame() {
        let rig = SensorRig::identity();
        assert_eq!(rig.len(), 1);
        assert_eq!(rig.resolve("base_link"), Some(FrameId::BASE));
    }
}
