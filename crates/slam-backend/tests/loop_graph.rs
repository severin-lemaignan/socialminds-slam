//! The stage-3b graph shape (ADR 0010): submap anchors as nodes, odometry
//! between-edges, one verified loop edge — optimisation must distribute the
//! accumulated drift across the chain instead of snapping the last pose.

use slam_backend::{FactorGraph, Key, PoseNoise};
use slam_types::{Pose, Rotation, Vec3};

fn planar(x: f64, y: f64, yaw: f64) -> Pose {
    Pose::new(Rotation::from_rpy(0.0, 0.0, yaw), Vec3::new(x, y, 0.0))
}

#[test]
fn loop_edge_distributes_drift_over_submap_anchors() {
    // Truth: five anchors every 10 m along +X. Odometry drifts +0.1 m in y per hop
    // (0.4 m total at the last anchor). The loop edge measures the TRUE relative
    // between anchor 4 and anchor 0.
    let mut graph = FactorGraph::new();
    let odo_noise = PoseNoise::isotropic(0.02, 0.1);
    let loop_noise = PoseNoise::isotropic(0.01, 0.05);

    for i in 0..5u64 {
        let drift = 0.1 * i as f64;
        graph.insert_pose(Key::pose(i), &planar(10.0 * i as f64, drift, 0.0));
    }
    graph.add_pose_prior(
        Key::pose(0),
        &planar(0.0, 0.0, 0.0),
        &PoseNoise::isotropic(1e-4, 1e-4),
    );
    for i in 0..4u64 {
        // Odometry as estimated (drifting): each hop says (+10, +0.1).
        graph.add_between(
            Key::pose(i),
            Key::pose(i + 1),
            &planar(10.0, 0.1, 0.0),
            &odo_noise,
        );
    }
    // Verified loop closure: anchor 4 is truly at (40, 0) relative to anchor 0.
    graph.add_between(
        Key::pose(0),
        Key::pose(4),
        &planar(40.0, 0.0, 0.0),
        &loop_noise,
    );

    let report = graph.optimize(50).expect("optimisation converges");
    assert!(report.final_error < report.initial_error);

    let last = graph.pose(Key::pose(4)).unwrap().translation();
    assert!(
        last.y.abs() < 0.05,
        "loop edge should pull anchor 4 back to y≈0, got {:.3}",
        last.y
    );
    // The correction is distributed: middle anchors move part-way, monotonically.
    let mid = graph.pose(Key::pose(2)).unwrap().translation();
    assert!(
        mid.y.abs() < 0.2 && mid.y.abs() < 0.21,
        "middle anchor should be partially corrected, got {:.3}",
        mid.y
    );
}
