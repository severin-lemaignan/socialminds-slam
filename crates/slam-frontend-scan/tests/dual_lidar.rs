//! Dual-lidar fusion against the raycast world, with extrinsics from a mock URDF
//! (ADR 0009): two 270°-FOV lidars at opposite corners of the base, unsynchronized
//! (alternating scans), both correcting the one shared base pose.
//!
//! Also exercises the failure mode the rig model exists to prevent: a wrong extrinsic
//! must visibly degrade tracking, not be silently absorbed.

use slam_frontend_scan::{ScanOdometry, ScanOdometryConfig, Se2};
use slam_types::{FrameId, LaserScan2D, SlamSystem};
use std::f64::consts::PI;
use std::path::PathBuf;

use slam_rig::SensorRig;

mod common;
use common::{pose_error, simulate_scan_at, world_segments};

const LIDAR_FOV: f64 = 1.5 * PI; // 270°
const LIDAR_BEAMS: usize = 540; // 0.5° spacing

fn mock_urdf() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/dual_lidar.urdf")
}

/// The planar pose of each rig frame, for *simulating* what its lidar sees.
fn planar_of(rig: &SensorRig, frame: FrameId) -> Se2 {
    Se2::planar_projection_of(&rig.extrinsic(frame)).0
}

/// Simulate the scan a lidar at `extrinsic` (planar `T_base_sensor`) takes from the
/// base pose `truth`, tagged with its frame.
fn lidar_scan(
    truth: &Se2,
    extrinsic: &Se2,
    frame: FrameId,
    stamp_s: f64,
    segments: &[(slam_types::Vec2, slam_types::Vec2)],
) -> LaserScan2D {
    let sensor_world = truth.compose(extrinsic);
    simulate_scan_at(
        &sensor_world,
        stamp_s,
        segments,
        LIDAR_FOV,
        LIDAR_BEAMS,
        frame,
    )
}

/// Drive the arc with both lidars alternating; return the worst (dt, dyaw) and stats.
fn drive(
    extrinsics_table: Vec<slam_types::Pose>,
    true_extrinsics: [(FrameId, Se2); 2],
) -> (f64, f64, slam_frontend_scan::ScanOdometryStats) {
    let segments = world_segments();

    // Forward 3 cm + 0.45° left per scan event; sensors alternate, so each lidar sees
    // the world at 6 cm / 0.9° intervals — the single-lidar test's cadence.
    let step = Se2::new(0.03, 0.0, 0.45_f64.to_radians());
    let mut truth = Se2::new(-3.0, -2.0, 0.0);
    let mut odo = ScanOdometry::with_extrinsics(
        truth.to_pose(),
        ScanOdometryConfig::default(),
        extrinsics_table,
    );

    let (mut worst_dt, mut worst_dyaw): (f64, f64) = (0.0, 0.0);
    for k in 0..240 {
        let (frame, ext) = &true_extrinsics[k % 2];
        odo.process_scan(&lidar_scan(
            &truth,
            ext,
            *frame,
            k as f64 * 0.025,
            &segments,
        ));
        let est = odo.current_estimate().expect("estimate after first scan");
        let (dt, dyaw) = pose_error(&est.pose, &truth);
        worst_dt = worst_dt.max(dt);
        worst_dyaw = worst_dyaw.max(dyaw);
        truth = truth.compose(&step);
    }
    (worst_dt, worst_dyaw, odo.stats())
}

#[test]
fn dual_lidar_tracks_with_urdf_extrinsics() {
    let rig = SensorRig::from_urdf_file(mock_urdf(), "base_link").unwrap();
    let fl = rig.resolve("laser_front_left").unwrap();
    let rr = rig.resolve("laser_rear_right").unwrap();
    let truth_ext = [(fl, planar_of(&rig, fl)), (rr, planar_of(&rig, rr))];

    let (worst_dt, worst_dyaw, stats) = drive(rig.extrinsics().to_vec(), truth_ext);

    assert_eq!(stats.scans, 240);
    assert_eq!(stats.skipped, 0);
    assert!(
        stats.matched >= 220,
        "matcher should almost always succeed: {stats:?}"
    );
    // Both sensors must contribute keyframes (≥ 1 adoption each).
    assert!(stats.keyframes >= 2, "{stats:?}");
    assert!(
        worst_dt < 0.06 && worst_dyaw < 0.02,
        "dual-lidar drift too large: {worst_dt:.4} m / {worst_dyaw:.4} rad ({stats:?})"
    );
}

#[test]
fn wrong_extrinsic_degrades_tracking() {
    let rig = SensorRig::from_urdf_file(mock_urdf(), "base_link").unwrap();
    let fl = rig.resolve("laser_front_left").unwrap();
    let rr = rig.resolve("laser_rear_right").unwrap();
    let truth_ext = [(fl, planar_of(&rig, fl)), (rr, planar_of(&rig, rr))];

    let (good_dt, _, _) = drive(rig.extrinsics().to_vec(), truth_ext);

    // Mis-calibrate the rear lidar by 8° of yaw: the world it reports is rotated
    // w.r.t. where the front lidar puts it, and the shared pose must suffer.
    let mut bad_table = rig.extrinsics().to_vec();
    bad_table[rr.0 as usize] = bad_table[rr.0 as usize]
        * slam_types::Pose::new(
            slam_types::Rotation::from_rpy(0.0, 0.0, 8.0_f64.to_radians()),
            slam_types::Vec3::zeros(),
        );

    let (bad_dt, _, _) = drive(bad_table, truth_ext);

    assert!(
        bad_dt > 0.15 && bad_dt > 3.0 * good_dt,
        "an 8° extrinsic error should visibly degrade tracking: \
         good {good_dt:.4} m vs bad {bad_dt:.4} m"
    );
}

#[test]
fn scans_from_an_unknown_frame_are_skipped_not_guessed() {
    let rig = SensorRig::from_urdf_file(mock_urdf(), "base_link").unwrap();
    let mut odo = ScanOdometry::with_extrinsics(
        Se2::identity().to_pose(),
        Default::default(),
        rig.extrinsics().to_vec(),
    );

    let segments = world_segments();
    let scan = simulate_scan_at(
        &Se2::identity(),
        0.0,
        &segments,
        LIDAR_FOV,
        LIDAR_BEAMS,
        FrameId(99), // not a rig frame
    );
    odo.process_scan(&scan);
    assert_eq!(odo.stats().skipped, 1);
    assert!(odo.current_estimate().is_none());
}
