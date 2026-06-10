//! Scan-to-keyframe odometry: the planar backbone (ADR 0002/0007).
//!
//! Every scan is matched against a held *keyframe* scan, renewed only after enough
//! motion — drift accrues per keyframe, not per scan. A constant-velocity prediction
//! seeds each match, and a failed/unhealthy match coasts on that prediction instead of
//! corrupting the trajectory (the health stats are exposed for the fusion layer).
//!
//! **Multi-lidar** (ADR 0009): scans are tagged with their sensor frame; each frame's
//! beams are mapped into the base frame through its rig extrinsic before matching, so
//! every sensor corrects the *one shared base pose* — fusion happens through the common
//! state. Keyframes are kept per sensor (matching always compares like FOV with like),
//! a stepping stone towards the shared local map of ADR 0009.
//!
//! **3D body** (ADR 0010): the extrinsics are full SE(3) and the IMU's gravity tilt
//! ([`AttitudeFilter`]) is applied when lifting beams, so a pitching/rolling base feeds
//! the planar matcher gravity-aligned points instead of a distorted slice. Returns that
//! land near the floor under tilt (the beam dipped) are gated out rather than matched
//! as phantom walls.

use std::collections::HashMap;

use slam_types::{
    FrameId, LaserScan2D, Pose, Rotation, SlamSystem, Stamp, StampedPose, Vec2, Vec3,
};

use crate::attitude::{AttitudeConfig, AttitudeFilter};
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
    /// Attitude (gravity tilt) filter tuning; active once IMU samples arrive.
    pub attitude: AttitudeConfig,
    /// Lifted points lower than this above the floor (m, gravity frame, floor at the
    /// base's z = 0) are floor hits of a dipped beam — gated out. Only applies when the
    /// attitude is initialised *and* the sensor sits comfortably above the clearance.
    pub floor_clearance: f64,
}

impl Default for ScanOdometryConfig {
    fn default() -> Self {
        ScanOdometryConfig {
            matcher: MatchConfig::default(),
            keyframe_translation: 0.3,
            keyframe_rotation: 0.3,
            min_valid_points: 50,
            min_inlier_fraction: 0.4,
            attitude: AttitudeConfig::default(),
            floor_clearance: 0.05,
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
    /// Matches with an unobservable translation direction, filled from the prediction.
    pub degenerate: u64,
}

struct Keyframe {
    /// The keyframe's base-frame points, indexed once for repeated matching.
    matcher: ScanMatcher,
    /// Keyframe base pose in the odometry frame.
    pose: Se2,
}

/// 2D scan-matching odometry implementing [`SlamSystem`].
///
/// Estimates the **base** pose: each scan's beams are mapped through the sensor's rig
/// extrinsic into the base frame before matching. Without extrinsics (the default),
/// every scan is treated as [`FrameId::BASE`] — the single-centred-lidar behaviour.
pub struct ScanOdometry {
    cfg: ScanOdometryConfig,
    /// SE(3) anchor: planar odometry is composed on top of this (e.g. a ground-truth
    /// initial pose on benchmark sequences).
    base: Pose,
    /// SE(3) `T_base_sensor` per [`FrameId`] index (empty = base-frame scans only).
    extrinsics: Vec<Pose>,
    /// Gravity tilt from the IMU stream; identity until IMU samples arrive (ADR 0010).
    attitude: AttitudeFilter,
    /// One keyframe per sensor frame: every sensor corrects the shared `current` pose,
    /// but matches against its own FOV's geometry.
    keyframes: HashMap<FrameId, Keyframe>,
    /// Current base pose in the odometry (anchor-relative) frame.
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
        Self::with_extrinsics(base, cfg, Vec::new())
    }

    /// Multi-lidar odometry: `extrinsics[frame.0]` is the SE(3) `T_base_sensor` of each
    /// rig frame (`SensorRig::extrinsics`). Scans tagged with a frame outside the table
    /// are *skipped* (counted in [`ScanOdometryStats::skipped`]) — an untagged
    /// ([`FrameId::BASE`]) scan against an empty table is the identity fast path.
    pub fn with_extrinsics(base: Pose, cfg: ScanOdometryConfig, extrinsics: Vec<Pose>) -> Self {
        let attitude = AttitudeFilter::new(cfg.attitude.clone());
        ScanOdometry {
            cfg,
            base,
            extrinsics,
            attitude,
            keyframes: HashMap::new(),
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

    /// The SE(3) extrinsic for `frame`, or `None` if the frame is unknown to the rig.
    fn extrinsic(&self, frame: FrameId) -> Option<Pose> {
        match self.extrinsics.get(frame.0 as usize) {
            Some(t) => Some(*t),
            None if frame == FrameId::BASE => Some(Pose::identity()),
            None => None,
        }
    }

    /// Lift the scan's beams into the gravity-aligned base frame: sensor plane → SE(3)
    /// extrinsic → IMU tilt, then drop floor hits and project onto the motion plane.
    /// This is where the 3D body model meets the planar matcher (ADR 0010).
    fn lift_scan(&self, scan: &LaserScan2D, t_base_sensor: &Pose, out: &mut Vec<Vec2>) {
        out.clear();
        let tilt: Rotation = self.attitude.tilt();
        let tilted = self.attitude.is_initialized();
        // Gate floor hits only when tilt information exists and the sensor sits clearly
        // above the clearance (a base-frame z=0 stream carries no height to gate on).
        let sensor_z = tilt.rotate(t_base_sensor.translation()).z;
        let gate_floor = tilted && sensor_z > 2.0 * self.cfg.floor_clearance;

        for (i, &r) in scan.ranges.iter().enumerate() {
            let r = r as f64;
            if !r.is_finite() || r < scan.range_min || r > scan.range_max {
                continue;
            }
            let angle = scan.angle_min + i as f64 * scan.angle_increment;
            let p_sensor = Vec3::new(r * angle.cos(), r * angle.sin(), 0.0);
            let p_base = t_base_sensor.transform_point(p_sensor);
            let p = if tilted { tilt.rotate(p_base) } else { p_base };
            if gate_floor && p.z < self.cfg.floor_clearance {
                continue; // a dipped beam hit the floor: not wall geometry
            }
            out.push(Vec2::new(p.x, p.y));
        }
    }

    fn adopt_keyframe(&mut self, frame: FrameId, points: Vec<Vec2>) {
        self.keyframes.insert(
            frame,
            Keyframe {
                matcher: ScanMatcher::new(points, self.cfg.matcher.clone()),
                pose: self.current,
            },
        );
        self.stats.keyframes += 1;
    }
}

impl SlamSystem for ScanOdometry {
    fn name(&self) -> &str {
        "scan_matching"
    }

    fn process_imu(&mut self, sample: &slam_types::ImuSample) {
        self.attitude.process(sample);
    }

    fn process_scan(&mut self, scan: &LaserScan2D) {
        self.stats.scans += 1;
        let Some(extrinsic) = self.extrinsic(scan.frame) else {
            // Unknown sensor frame: a mis-wired rig. Never guess an identity extrinsic.
            self.stats.skipped += 1;
            return;
        };
        // Express the beams in the gravity-aligned base frame: the match below then
        // estimates base motion directly, and all sensors correct the same pose state.
        let mut points = std::mem::take(&mut self.points_buf);
        self.lift_scan(scan, &extrinsic, &mut points);
        if points.len() < self.cfg.min_valid_points {
            self.stats.skipped += 1;
            self.points_buf = points;
            return; // estimate (and its stamp) unchanged: nothing was learned
        }

        let Some(keyframe) = self.keyframes.get_mut(&scan.frame) else {
            self.last_stamp = Some(scan.stamp);
            self.adopt_keyframe(scan.frame, points);
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
                // The match transform *is* the base pose in the keyframe frame — except
                // along a degenerate direction (corridor axis), where the geometry
                // measured nothing and the constant-velocity prediction must fill in.
                let mut transform = result.transform;
                if let Some(dir) = result.degenerate_direction {
                    let slip =
                        dir.x * (initial.x - transform.x) + dir.y * (initial.y - transform.y);
                    transform = Se2::new(
                        transform.x + dir.x * slip,
                        transform.y + dir.y * slip,
                        transform.theta,
                    );
                    self.stats.degenerate += 1;
                }
                self.current = keyframe.pose.compose(&transform);
                self.stats.matched += 1;

                if transform.translation_norm() > self.cfg.keyframe_translation
                    || transform.theta.abs() > self.cfg.keyframe_rotation
                {
                    self.adopt_keyframe(scan.frame, points);
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
                self.adopt_keyframe(scan.frame, points);
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
