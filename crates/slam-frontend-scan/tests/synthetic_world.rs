//! Scan odometry against a raycast synthetic world with an analytically known trajectory.
//!
//! A rectangular room with a pillar (the pillar breaks the rectangle's symmetries and
//! adds close-range structure), a simulated 360° scanner, and a curved drive through it:
//! the odometry estimate must track the true pose closely, scan after scan.

use slam_frontend_scan::{ScanOdometry, ScanOdometryConfig, Se2};
use slam_types::{LaserScan2D, SlamSystem, Stamp, Vec2};
use std::f64::consts::PI;

/// Wall segments of the world: a 12×8 m room centred at the origin, with a 1×1 m pillar
/// whose corner sits at (2, 0.5).
fn world_segments() -> Vec<(Vec2, Vec2)> {
    let mut segs = Vec::new();
    let (hw, hh) = (6.0, 4.0);
    let room = [
        Vec2::new(-hw, -hh),
        Vec2::new(hw, -hh),
        Vec2::new(hw, hh),
        Vec2::new(-hw, hh),
    ];
    for i in 0..4 {
        segs.push((room[i], room[(i + 1) % 4]));
    }
    let pillar = [
        Vec2::new(2.0, 0.5),
        Vec2::new(3.0, 0.5),
        Vec2::new(3.0, 1.5),
        Vec2::new(2.0, 1.5),
    ];
    for i in 0..4 {
        segs.push((pillar[i], pillar[(i + 1) % 4]));
    }
    segs
}

/// Distance along the ray from `origin` in direction `dir` to segment `(a, b)`, if hit.
fn ray_segment(origin: Vec2, dir: Vec2, a: Vec2, b: Vec2) -> Option<f64> {
    let v = b - a; // segment direction
    let denom = dir.x * (-v.y) - dir.y * (-v.x); // det[dir, -v]
    if denom.abs() < 1e-12 {
        return None; // parallel
    }
    let w = a - origin;
    let t = (w.x * (-v.y) - w.y * (-v.x)) / denom; // along the ray
    let u = (dir.x * w.y - dir.y * w.x) / -denom; // along the segment
    (t > 1e-9 && (0.0..=1.0).contains(&u)).then_some(t)
}

/// Simulate one 360-beam revolution from the sensor pose.
fn simulate_scan(pose: &Se2, stamp_s: f64, segments: &[(Vec2, Vec2)]) -> LaserScan2D {
    let beams = 360;
    let angle_min = -PI;
    let angle_increment = 2.0 * PI / beams as f64;
    let origin = Vec2::new(pose.x, pose.y);
    let ranges = (0..beams)
        .map(|i| {
            let angle = pose.theta + angle_min + i as f64 * angle_increment;
            let dir = Vec2::new(angle.cos(), angle.sin());
            segments
                .iter()
                .filter_map(|(a, b)| ray_segment(origin, dir, *a, *b))
                .fold(f32::INFINITY, |acc, t| acc.min(t as f32))
        })
        .collect();
    LaserScan2D {
        stamp: Stamp::from_seconds(stamp_s),
        angle_min,
        angle_increment,
        range_min: 0.05,
        range_max: 30.0,
        ranges,
    }
}

fn pose_error(estimate: &slam_types::Pose, truth: &Se2) -> (f64, f64) {
    let t = estimate.translation();
    let dt = (t.x - truth.x).hypot(t.y - truth.y);
    let yaw = estimate.rotation().log().z;
    let mut dyaw = (yaw - truth.theta).rem_euclid(2.0 * PI);
    if dyaw > PI {
        dyaw -= 2.0 * PI;
    }
    (dt, dyaw.abs())
}

#[test]
fn tracks_a_curved_drive_through_the_room() {
    let segments = world_segments();

    // Drive an arc: forward 6 cm + 0.9° left per scan, 120 scans (≈ 7 m, ≈ 108°).
    let step = Se2::new(0.06, 0.0, 0.9_f64.to_radians());
    let mut truth = Se2::new(-3.0, -2.0, 0.0);
    // Odometry reports motion relative to its start: anchor it at the true start so the
    // estimate is directly comparable to `truth`.
    let mut odo = ScanOdometry::anchored_at(truth.to_pose(), ScanOdometryConfig::default());

    let mut worst_dt: f64 = 0.0;
    let mut worst_dyaw: f64 = 0.0;
    for k in 0..120 {
        odo.process_scan(&simulate_scan(&truth, k as f64 * 0.1, &segments));
        let est = odo.current_estimate().expect("estimate after first scan");
        let (dt, dyaw) = pose_error(&est.pose, &truth);
        worst_dt = worst_dt.max(dt);
        worst_dyaw = worst_dyaw.max(dyaw);
        truth = truth.compose(&step);
    }

    let stats = odo.stats();
    assert_eq!(stats.scans, 120);
    assert_eq!(stats.skipped, 0);
    assert!(
        stats.matched >= 110,
        "matcher should almost always succeed: {stats:?}"
    );
    assert!(
        worst_dt < 0.05 && worst_dyaw < 0.02,
        "drift too large: {worst_dt:.4} m / {worst_dyaw:.4} rad ({stats:?})"
    );
}

#[test]
fn stationary_robot_stays_put_through_noise_free_scans() {
    let segments = world_segments();
    let mut odo = ScanOdometry::new(ScanOdometryConfig::default());
    let truth = Se2::new(1.0, -1.0, 0.7);

    for k in 0..20 {
        odo.process_scan(&simulate_scan(&truth, k as f64 * 0.1, &segments));
    }
    let est = odo.current_estimate().unwrap();
    // Odometry starts at identity: it reports *relative* motion, which must stay ~zero.
    assert!(est.pose.translation().norm() < 1e-3);
    assert!(est.pose.rotation().log().norm() < 1e-3);
}

#[test]
fn anchor_pose_offsets_the_whole_trajectory() {
    let segments = world_segments();
    let anchor = Se2::new(10.0, 5.0, PI / 2.0).to_pose();
    let mut odo = ScanOdometry::anchored_at(anchor, ScanOdometryConfig::default());

    let truth = Se2::new(0.0, 0.0, 0.0);
    odo.process_scan(&simulate_scan(&truth, 0.0, &segments));
    let est = odo.current_estimate().unwrap();
    assert!((est.pose.translation() - anchor.translation()).norm() < 1e-9);
}

#[test]
fn a_walking_person_does_not_derail_the_odometry() {
    let segments = world_segments();

    let step = Se2::new(0.05, 0.0, 0.0);
    let mut truth = Se2::new(-3.0, -2.0, 0.2);
    let mut odo = ScanOdometry::anchored_at(truth.to_pose(), ScanOdometryConfig::default());

    for k in 0..60 {
        // The "person": a 40 cm-wide obstacle crossing the room, simulated as an extra
        // square segment moving independently of the robot.
        let px = -1.0 + 0.08 * k as f64;
        let py = 0.5;
        let mut dynamic = segments.clone();
        dynamic.push((Vec2::new(px, py), Vec2::new(px + 0.4, py)));
        dynamic.push((Vec2::new(px, py - 0.2), Vec2::new(px, py)));
        dynamic.push((Vec2::new(px + 0.4, py - 0.2), Vec2::new(px + 0.4, py)));

        odo.process_scan(&simulate_scan(&truth, k as f64 * 0.1, &dynamic));
        truth = truth.compose(&step);
    }
    // `truth` is one step ahead of the last processed scan.
    let last = truth.compose(&step.inverse());
    let est = odo.current_estimate().unwrap();
    let (dt, dyaw) = pose_error(&est.pose, &last);
    assert!(
        dt < 0.08 && dyaw < 0.03,
        "person broke the odometry: {dt:.4} m / {dyaw:.4} rad ({:?})",
        odo.stats()
    );
}
