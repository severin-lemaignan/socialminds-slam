//! Scan-to-submap odometry (ADR 0010 stage 2) over the raycast worlds: the same
//! scenarios the planar front-end passes — curved drive, dual lidar from the mock URDF,
//! tilt pulses — must hold with TSDF registration. In the tilt scenario the floor needs
//! no gating at all: it is real structure in a 3D map.

use slam_frontend_scan::{ScanToMapConfig, ScanToMapOdometry, Se2};
use slam_types::{FrameId, ImuSample, Pose, Rotation, SlamSystem, Stamp, Vec3};
use std::f64::consts::PI;
use std::path::PathBuf;

use slam_rig::SensorRig;

mod common;
use common::{pose_error, simulate_scan_25d, simulate_scan_at, world_25d, world_segments};

#[test]
fn tracks_the_curved_drive() {
    let segments = world_segments();
    let step = Se2::new(0.06, 0.0, 0.9_f64.to_radians());
    let mut truth = Se2::new(-3.0, -2.0, 0.0);
    let mut odo = ScanToMapOdometry::anchored_at(truth.to_pose(), ScanToMapConfig::default());

    let (mut worst_dt, mut worst_dyaw): (f64, f64) = (0.0, 0.0);
    for k in 0..120 {
        odo.process_scan(&simulate_scan_at(
            &truth,
            k as f64 * 0.1,
            &segments,
            2.0 * PI,
            1440,
            FrameId::BASE,
        ));
        let est = odo.current_estimate().expect("estimate after first scan");
        let (dt, dyaw) = pose_error(&est.pose, &truth);
        worst_dt = worst_dt.max(dt);
        worst_dyaw = worst_dyaw.max(dyaw);
        truth = truth.compose(&step);
    }
    let stats = odo.stats();
    assert_eq!(stats.skipped, 0);
    assert!(
        stats.matched >= 110,
        "registration should succeed: {stats:?}"
    );
    assert!(
        worst_dt < 0.06 && worst_dyaw < 0.02,
        "scan-to-map drift too large: {worst_dt:.4} m / {worst_dyaw:.4} rad ({stats:?})"
    );
}

#[test]
fn dual_lidar_fuses_into_one_submap() {
    let urdf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/dual_lidar.urdf");
    let rig = SensorRig::from_urdf_file(urdf, "base_link").unwrap();
    let fl = rig.resolve("laser_front_left").unwrap();
    let rr = rig.resolve("laser_rear_right").unwrap();
    let planar = |f: FrameId| Se2::planar_projection_of(&rig.extrinsic(f)).0;
    let sensors = [(fl, planar(fl)), (rr, planar(rr))];

    let segments = world_segments();
    let step = Se2::new(0.03, 0.0, 0.45_f64.to_radians());
    let mut truth = Se2::new(-3.0, -2.0, 0.0);
    let mut odo = ScanToMapOdometry::with_extrinsics(
        truth.to_pose(),
        ScanToMapConfig::default(),
        rig.extrinsics().to_vec(),
    );

    let mut worst_dt: f64 = 0.0;
    for k in 0..240 {
        let (frame, ext) = &sensors[k % 2];
        let sensor_world = truth.compose(ext);
        odo.process_scan(&simulate_scan_at(
            &sensor_world,
            k as f64 * 0.025,
            &segments,
            1.5 * PI,
            1080,
            *frame,
        ));
        let est = odo.current_estimate().expect("estimate after first scan");
        let (dt, _) = pose_error(&est.pose, &truth);
        worst_dt = worst_dt.max(dt);
        truth = truth.compose(&step);
    }
    let stats = odo.stats();
    assert_eq!(stats.skipped, 0);
    assert!(
        worst_dt < 0.06,
        "dual-lidar scan-to-map drift too large: {worst_dt:.4} m ({stats:?})"
    );
}

#[test]
fn tilt_pulses_without_floor_gating() {
    const SENSOR_Z: f64 = 0.18;
    const G: f64 = 9.80665;
    let amp = 4.0 * PI / 180.0;
    let pulse = |t0: f64, dur: f64, t: f64| {
        if (t0..t0 + dur).contains(&t) {
            (PI * (t - t0) / dur).sin().powi(2)
        } else {
            0.0
        }
    };
    let pitch_at = |t: f64| amp * (pulse(2.0, 1.5, t) - pulse(6.0, 1.5, t));
    let rate_at = |t: f64| (pitch_at(t + 1e-5) - pitch_at(t - 1e-5)) / 2e-5;

    let walls = world_25d();
    let t_bs = Pose::new(Rotation::identity(), Vec3::new(0.0, 0.0, SENSOR_Z));
    let lidar = FrameId(1);
    let step = Se2::new(0.06, 0.0, 0.0);
    let mut truth = Se2::new(-4.5, -2.5, 0.25);
    let mut odo = ScanToMapOdometry::with_extrinsics(
        truth.to_pose(),
        ScanToMapConfig::default(),
        vec![Pose::identity(), t_bs],
    );

    let mut worst_dt: f64 = 0.0;
    for k in 0..140 {
        let t = k as f64 * 0.05;
        for j in 0..10 {
            let ti = t - 0.05 + (j + 1) as f64 * 0.005;
            let att = Rotation::from_rpy(0.0, pitch_at(ti), 0.0);
            odo.process_imu(&ImuSample::new(
                Stamp::from_seconds(ti),
                Vec3::new(0.0, rate_at(ti), 0.0),
                att.inverse().rotate(Vec3::new(0.0, 0.0, G)),
            ));
        }
        let t_ws = truth.to_pose()
            * Pose::new(Rotation::from_rpy(0.0, pitch_at(t), 0.0), Vec3::zeros())
            * t_bs;
        odo.process_scan(&simulate_scan_25d(
            &t_ws,
            t,
            &walls,
            1.5 * PI,
            1080,
            lidar,
            None,
        ));
        let est = odo.current_estimate().expect("estimate after first scan");
        let (dt, _) = pose_error(&est.pose, &truth);
        worst_dt = worst_dt.max(dt);
        truth = truth.compose(&step);
    }
    let stats = odo.stats();
    assert!(
        worst_dt < 0.10,
        "tilted scan-to-map drift too large: {worst_dt:.4} m ({stats:?})"
    );
}
