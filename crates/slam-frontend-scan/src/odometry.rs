//! Scan-to-keyframe odometry: the planar backbone (ADR 0002/0007).
//!
//! Every scan is matched against a held *keyframe* scan, renewed only after enough
//! motion — drift accrues per keyframe, not per scan. A constant-velocity prediction
//! seeds each match, and a failed/unhealthy match coasts on that prediction instead of
//! corrupting the trajectory (the health stats are exposed for the fusion layer).

use slam_types::{LaserScan2D, Pose, SlamSystem, Stamp, StampedPose, Vec2};

use crate::icp::{MatchConfig, ScanMatcher};
use crate::se2::Se2;

/// Tuning for [`ScanOdometry`].
#[derive(Debug, Clone)]
pub struct ScanOdometryConfig {
    pub matcher: MatchConfig,
    /// Renew the keyframe after this much translation (m) …
    pub keyframe_translation: f64,
    /// … or this much rotation (rad) relative to it.
    pub keyframe_rotation: f64,
    /// Scans with fewer valid returns are skipped outright.
    pub min_valid_points: usize,
    /// Matches keeping a smaller inlier fraction are treated as failures (coast).
    pub min_inlier_fraction: f64,
}

impl Default for ScanOdometryConfig {
    fn default() -> Self {
        ScanOdometryConfig {
            matcher: MatchConfig::default(),
            keyframe_translation: 0.3,
            keyframe_rotation: 0.3,
            min_valid_points: 50,
            min_inlier_fraction: 0.4,
        }
    }
}

/// Health counters — exposed so the harness and (later) the fusion layer can see how
/// often the matcher actually worked rather than coasted.
#[derive(Debug, Clone, Copy, Default)]
pub struct ScanOdometryStats {
    pub scans: u64,
    pub matched: u64,
    pub coasted: u64,
    pub skipped: u64,
    pub keyframes: u64,
}

struct Keyframe {
    /// The keyframe's points, indexed once for repeated matching.
    matcher: ScanMatcher,
    /// Keyframe sensor pose in the odometry frame.
    pose: Se2,
}

/// 2D scan-matching odometry implementing [`SlamSystem`].
///
/// The sensor is assumed coincident with the body frame for now; lidar extrinsics land
/// with the fusion layer (M3 step 3).
pub struct ScanOdometry {
    cfg: ScanOdometryConfig,
    /// SE(3) anchor: planar odometry is composed on top of this (e.g. a ground-truth
    /// initial pose on benchmark sequences).
    base: Pose,
    keyframe: Option<Keyframe>,
    /// Current sensor pose in the odometry (anchor-relative) frame.
    current: Se2,
    /// Last per-scan motion, used as the constant-velocity prediction.
    last_motion: Se2,
    last_stamp: Option<Stamp>,
    stats: ScanOdometryStats,
    /// Reused scan-point buffer (surrendered to the matcher on keyframe adoption).
    points_buf: Vec<Vec2>,
}

impl ScanOdometry {
    pub fn new(cfg: ScanOdometryConfig) -> Self {
        Self::anchored_at(Pose::identity(), cfg)
    }

    /// Start the odometry frame at `base` (the planar motion is embedded on top of it).
    pub fn anchored_at(base: Pose, cfg: ScanOdometryConfig) -> Self {
        ScanOdometry {
            cfg,
            base,
            keyframe: None,
            current: Se2::identity(),
            last_motion: Se2::identity(),
            last_stamp: None,
            stats: ScanOdometryStats::default(),
            points_buf: Vec::new(),
        }
    }

    pub fn stats(&self) -> ScanOdometryStats {
        self.stats
    }

    fn adopt_keyframe(&mut self, points: Vec<Vec2>) {
        self.keyframe = Some(Keyframe {
            matcher: ScanMatcher::new(points, self.cfg.matcher.clone()),
            pose: self.current,
        });
        self.stats.keyframes += 1;
    }
}

impl SlamSystem for ScanOdometry {
    fn name(&self) -> &str {
        "scan_matching"
    }

    fn process_scan(&mut self, scan: &LaserScan2D) {
        self.stats.scans += 1;
        let mut points = std::mem::take(&mut self.points_buf);
        scan.points_into(&mut points);
        if points.len() < self.cfg.min_valid_points {
            self.stats.skipped += 1;
            self.points_buf = points;
            return; // estimate (and its stamp) unchanged: nothing was learned
        }

        let Some(keyframe) = &mut self.keyframe else {
            self.last_stamp = Some(scan.stamp);
            self.adopt_keyframe(points);
            return;
        };

        // Constant-velocity prediction, expressed relative to the keyframe.
        let predicted = self.current.compose(&self.last_motion);
        let initial = keyframe.pose.inverse().compose(&predicted);

        let matched = keyframe
            .matcher
            .match_to(&points, initial)
            .filter(|m| m.inlier_fraction >= self.cfg.min_inlier_fraction);

        let previous = self.current;
        match matched {
            Some(result) => {
                // The match transform *is* the sensor pose in the keyframe frame.
                self.current = keyframe.pose.compose(&result.transform);
                self.stats.matched += 1;

                if result.transform.translation_norm() > self.cfg.keyframe_translation
                    || result.transform.theta.abs() > self.cfg.keyframe_rotation
                {
                    self.adopt_keyframe(points);
                } else {
                    self.points_buf = points;
                }
            }
            None => {
                // Unmatchable scan (dynamics, occlusion, degenerate geometry): coast on
                // the prediction rather than freeze — and re-anchor so the next scan
                // matches against fresh geometry.
                self.current = predicted;
                self.stats.coasted += 1;
                self.adopt_keyframe(points);
            }
        }
        self.last_motion = previous.inverse().compose(&self.current);
        self.last_stamp = Some(scan.stamp);
    }

    fn current_estimate(&self) -> Option<StampedPose> {
        self.last_stamp.map(|stamp| StampedPose {
            stamp,
            pose: self.base * self.current.to_pose(),
        })
    }
}
