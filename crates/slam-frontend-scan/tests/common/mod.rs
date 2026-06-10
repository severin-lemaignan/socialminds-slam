//! Shared raycast world for front-end integration tests: a rectangular room with a
//! pillar, segment-intersection lidar simulation, and pose-error helpers.

// Each integration-test binary compiles this module separately and uses a subset.
#![allow(dead_code)]

use slam_frontend_scan::Se2;
use slam_types::{FrameId, LaserScan2D, Stamp, Vec2};
use std::f64::consts::PI;

/// Wall segments of the world: a 12×8 m room centred at the origin, with a 1×1 m pillar
/// whose corner sits at (2, 0.5) (the pillar breaks the rectangle's symmetries and adds
/// close-range structure).
pub fn world_segments() -> Vec<(Vec2, Vec2)> {
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

/// Simulate one revolution of a `fov`-wide, `beams`-beam scanner at the world pose
/// `sensor`, tagging the scan with `frame`. The FOV is centred on the sensor's +X.
pub fn simulate_scan_at(
    sensor: &Se2,
    stamp_s: f64,
    segments: &[(Vec2, Vec2)],
    fov: f64,
    beams: usize,
    frame: FrameId,
) -> LaserScan2D {
    let angle_min = -fov / 2.0;
    let angle_increment = fov / beams as f64;
    let origin = Vec2::new(sensor.x, sensor.y);
    let ranges = (0..beams)
        .map(|i| {
            let angle = sensor.theta + angle_min + i as f64 * angle_increment;
            let dir = Vec2::new(angle.cos(), angle.sin());
            segments
                .iter()
                .filter_map(|(a, b)| ray_segment(origin, dir, *a, *b))
                .fold(f32::INFINITY, |acc, t| acc.min(t as f32))
        })
        .collect();
    LaserScan2D {
        stamp: Stamp::from_seconds(stamp_s),
        frame,
        angle_min,
        angle_increment,
        range_min: 0.05,
        range_max: 30.0,
        ranges,
    }
}

/// Simulate one full-circle 360-beam revolution from a base-frame sensor pose.
pub fn simulate_scan(pose: &Se2, stamp_s: f64, segments: &[(Vec2, Vec2)]) -> LaserScan2D {
    simulate_scan_at(pose, stamp_s, segments, 2.0 * PI, 360, FrameId::BASE)
}

/// (translation error (m), |yaw error| (rad)) of an SE(3) estimate vs a planar truth.
pub fn pose_error(estimate: &slam_types::Pose, truth: &Se2) -> (f64, f64) {
    let t = estimate.translation();
    let dt = (t.x - truth.x).hypot(t.y - truth.y);
    let yaw = estimate.rotation().log().z;
    let mut dyaw = (yaw - truth.theta).rem_euclid(2.0 * PI);
    if dyaw > PI {
        dyaw -= 2.0 * PI;
    }
    (dt, dyaw.abs())
}
