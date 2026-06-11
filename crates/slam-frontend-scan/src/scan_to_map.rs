//! Scan-to-submap odometry (ADR 0010): register each lifted 3D fan against the local
//! TSDF submap instead of a keyframe scan.
//!
//! This completes what the per-sensor keyframes of [`ScanOdometry`](crate::ScanOdometry)
//! approximated: **one shared local map** — every lidar (and later RGB-D) fuses into and
//! registers against the same structure, floors included (a tilted beam's floor hit is
//! real geometry here, not a phantom to gate away). The solve stays 3-DoF
//! (x, y, yaw in the gravity frame): the planar lidars cannot observe z/roll/pitch and
//! this front-end still never invents out-of-plane motion — roll/pitch come from the
//! IMU ([`AttitudeFilter`]), z from the submap anchor.
//!
//! Submaps are bounded: after `submap_extent` of travel a fresh map is started, with an
//! overlap window (register against the old, integrate into both) so registration never
//! faces an empty model. Submap re-posing on loop closure is the pose-graph's job
//! (stage 3); here submaps only bound memory and drift accumulation.

use slam_map::{SdfSample, SparseTsdf, TsdfConfig, TsdfMap};
use slam_types::{FrameId, LaserScan2D, Pose, Rotation, SlamSystem, Stamp, StampedPose, Vec3};

use crate::attitude::{AttitudeConfig, AttitudeFilter};
use crate::icp::weak_translation_direction;
use crate::odometry::ScanOdometryStats;
use crate::se2::Se2;

/// Tuning for [`ScanToMapOdometry`].
#[derive(Debug, Clone)]
pub struct ScanToMapConfig {
    /// The **2D laser registration field** (gravity-plane projection). Fine voxels:
    /// scan accuracy is parity-gated against PLICP (ADR 0010).
    pub tsdf: TsdfConfig,
    /// The **3D field** (map product + depth registration + depth loop closure).
    /// Coarser than the 2D field: RealSense depth error at range (≈ 2–6 % of z)
    /// dwarfs fine voxels, and a coarser grid is what makes range-adaptive depth
    /// sampling stencil-supportable at bounded point counts.
    pub tsdf_3d: TsdfConfig,
    /// Gauss-Newton iterations per scan.
    pub max_iterations: usize,
    /// Converged when one step moves less than this (m / rad).
    pub translation_epsilon: f64,
    pub rotation_epsilon: f64,
    /// Scans with fewer valid returns are skipped outright.
    pub min_valid_points: usize,
    /// Registrations keeping a smaller fraction of in-band samples are failures (coast).
    pub min_inlier_fraction: f64,
    /// See [`crate::MatchConfig::degeneracy_eigenvalue_ratio`].
    pub degeneracy_eigenvalue_ratio: f64,
    /// Lifted points lower than this above the floor (m) are **excluded from the
    /// registration residuals** (a horizontal floor cannot constrain x/y/yaw, and its
    /// single-viewpoint projective TSDF gradients alias into the plane) — but they are
    /// still *integrated*: the floor is real structure for the map. Active only once
    /// the attitude is initialised and the sensor sits above the clearance.
    pub floor_clearance: f64,
    /// Attitude (gravity tilt) filter tuning; active once IMU samples arrive.
    pub attitude: AttitudeConfig,
    /// Integrate into the map only after this much motion since the last fusion (m /
    /// rad) — PLICP's keyframe diet: fusing every scan at 20-40 Hz writes the
    /// estimate's own noise into the map hundreds of times per metre (error feedback)
    /// and gives passing people hundreds of chances to become ghosts.
    pub integrate_translation: f64,
    pub integrate_rotation: f64,
    /// Depth points within this distance (m) of an observed laser plane height are
    /// fused into the **2D registration field** too: they measure the same slice the
    /// laser scans (a person occluding the laser no longer erases the wall behind
    /// them from the scan matcher's world). Geometry at other heights stays out — the
    /// laser can never confirm it. **Default 0 (off)**: measured on cafe1-1, un-masked
    /// people at head height (≈ the laser plane) degrade the scan matcher 0.164→0.357;
    /// enable (≈ 0.15) together with dynamics masking, like `depth_updates_pose`.
    pub reg_band_tolerance: f64,
    /// When false (the current default), depth clouds fuse into the map and the
    /// 2D laser band but do **not** correct the pose — *as long as scans are
    /// present*: un-masked dynamics (people) dominate indoor depth views and a
    /// lidar-corrected pose is strictly better. On scan-less datasets (OpenLORIS
    /// `market`) depth is the only exteroceptive stream and always updates the
    /// pose. Flips on when dynamics masking lands (ADR 0002).
    pub depth_updates_pose: bool,
    /// Cap on cloud points used per registration solve (deterministic subsample) —
    /// accuracy saturates long before a full VGA back-projection's 6 k points, and the
    /// trilinear sampling cost is linear in points.
    pub max_registration_points: usize,
    /// Travel distance (m) before a fresh submap is started.
    pub submap_extent: f64,
    /// Scans integrated into both maps after a submap hand-over.
    pub submap_overlap_scans: usize,
    /// Loop closure (stage 3a, ADR 0010): attempt re-registration against *frozen*
    /// submaps whose anchor lies within this radius (m) of the current pose. Proximity
    /// gating only — appearance signatures (MapClosures) arrive with re-localization.
    pub loop_radius: f64,
    /// A verified loop must keep at least this in-band sample fraction (stricter than
    /// odometry: a wrong loop is worse than a missed one — ADR 0002).
    pub loop_min_inliers: f64,
    /// Half-extent (m) and yaw half-extent (rad) of the seed grid searched around the
    /// current estimate when attempting a loop (covers accumulated drift beyond the
    /// TSDF truncation basin).
    pub loop_search_radius: f64,
    pub loop_search_yaw: f64,
}

impl Default for ScanToMapConfig {
    fn default() -> Self {
        ScanToMapConfig {
            // Finer than the 5 cm map default: registration accuracy is bounded by the
            // voxel quantisation noise floor, and the planar-parity gate (ADR 0010)
            // demands PLICP-level accuracy. 2.5 cm costs ~8x voxels on a *local* submap
            // and the runtime headroom is ample (TSDF registration outruns PLICP).
            tsdf: slam_map::TsdfConfig {
                voxel_size: 0.025,
                truncation: 0.075,
                max_weight: 100000.0,
                // Carved like every active field (ADR 0014): on the 120 s busy
                // scenario an uncarved registration field accumulates person
                // crescents until tracking collapses (ATE 114 m scan-only, 1.7 m
                // with the odom prior; carved: 2.7 m / 0.90 m). Short runs hide
                // this — the ghosts are only harmful once current people overlap
                // earlier stamps.
                carve_factor: 0.5,
            },
            // 5 cm / 15 cm: the swept trade-off (see commit). Finer fields score
            // better open-loop near-range (2.5 cm: 0.46 vs 0.81 on cafe depth-only)
            // but their narrow truncation basin never verifies depth loop closures
            // and coasts at range; 5 cm tracks market, verifies loops, barely coasts.
            tsdf_3d: slam_map::TsdfConfig {
                voxel_size: 0.05,
                truncation: 0.15,
                max_weight: 100000.0,
                // The map product carves (ADR 0014): measured free on cafe1 with the
                // block-skip walk (p99 within noise of carve-off, ATE unchanged).
                carve_factor: 0.5,
            },
            max_iterations: 12,
            translation_epsilon: 1e-6,
            rotation_epsilon: 1e-7,
            min_valid_points: 50,
            min_inlier_fraction: 0.4,
            degeneracy_eigenvalue_ratio: 0.02,
            floor_clearance: 0.05,
            attitude: AttitudeConfig::default(),
            integrate_translation: 0.1,
            integrate_rotation: 0.1,
            reg_band_tolerance: 0.0,
            depth_updates_pose: false,
            max_registration_points: 1500,
            submap_extent: 20.0,
            submap_overlap_scans: 40,
            loop_radius: 12.0,
            loop_min_inliers: 0.55,
            loop_search_radius: 0.5,
            loop_search_yaw: 0.12,
        }
    }
}

/// A verified loop closure: the current base pose re-registered against an old submap.
#[derive(Debug, Clone, Copy)]
pub struct LoopClosure {
    /// Index of the frozen (target) submap the scan re-registered against.
    pub submap: usize,
    /// Index of the submap that was *active* at detection (graph node id).
    pub active_submap: usize,
    /// The verified base pose (odometry frame) according to that submap.
    pub pose: Se2,
    /// Base pose relative to the target submap's anchor — the verified measurement.
    pub rel_target: Se2,
    /// Base pose relative to the active submap's anchor at detection (odometry side).
    pub rel_active: Se2,
    /// In-band sample fraction of the verifying registration.
    pub inliers: f64,
}

/// Optimises the submap-anchor pose graph. Implemented over GTSAM in the composition
/// layer (`slam-replay`); the front-end stays C++-free (ADR 0001).
pub trait AnchorGraph {
    /// `anchors[i]`: current anchor estimates (last = the active submap's anchor).
    /// `odometry[i]`: measured relative anchor_i → anchor_{i+1}.
    /// `loops`: `(target_idx, active_idx, measured relative target → active anchor)`.
    /// Returns optimised anchors (same length), or `None` when optimisation failed.
    fn optimize(
        &mut self,
        anchors: &[Se2],
        odometry: &[Se2],
        loops: &[(usize, usize, Se2)],
    ) -> Option<Vec<Se2>>;
}

/// Scan-to-submap odometry implementing [`SlamSystem`] (ADR 0010 stage 2).
pub struct ScanToMapOdometry {
    cfg: ScanToMapConfig,
    /// SE(3) anchor: planar odometry is composed on top of this.
    base: Pose,
    /// SE(3) `T_base_sensor` per [`FrameId`] index (empty = base-frame scans only).
    extrinsics: Vec<Pose>,
    attitude: AttitudeFilter,
    /// The active 3D submap (odometry frame) — the map *product* (viz, reMap, RGB-D).
    map: SparseTsdf,
    /// The active **registration field**: the same submap projected onto the gravity
    /// plane (z = 0). A planar fan — even tilt-corrected — is a 1D curve through 3D
    /// voxel space, so no 3D interpolation stencil is supportable by its samples; the
    /// 3-DoF solve's natural substrate is the 2D projection, which the fan covers
    /// densely. Floor hits are gated out before projection. RGB-D registration (true
    /// 2D surfaces in 3D) will use the 3D field directly (ADR 0010).
    reg: SparseTsdf,
    submap_birth: Se2,
    /// Arc length travelled since the active submap was born (m). A submap bounds
    /// *travel*, not displacement — Euclidean distance saturates on a tight loop.
    submap_travel: f64,
    /// Previous submaps during the hand-over window: registration target while the new
    /// maps fill, integration target alongside them.
    prev_map: Option<SparseTsdf>,
    prev_reg: Option<SparseTsdf>,
    /// Anchor of the submap currently in `prev_*` (frozen once the overlap ends).
    prev_anchor: Se2,
    overlap_left: usize,
    /// Frozen submaps: `(anchor pose at birth, 2D registration field, 3D field)` —
    /// retained for loop closure (scans re-register against the 2D field, depth
    /// clouds against the 3D field) and re-localization signatures.
    frozen: Vec<(Se2, SparseTsdf, SparseTsdf)>,
    /// Verified loop closures, in detection order (the pose-graph's loop edges).
    loops: Vec<LoopClosure>,
    /// Measured odometry relative between consecutive submap anchors (graph edges) —
    /// recorded at hand-over time so later re-posing never rewrites measurements.
    odometry_edges: Vec<Se2>,
    /// The pose-graph optimiser (GTSAM via the composition layer); None = snap-only.
    graph: Option<Box<dyn AnchorGraph>>,
    /// Whether any laser scan has been processed (gates depth→pose suppression:
    /// on scan-less datasets depth must keep correcting the pose).
    scan_seen: bool,
    /// Whether `lifted` currently holds a depth cloud (loop closure then verifies
    /// against frozen 3D fields instead of the 2D laser fields).
    lifted_is_cloud: bool,
    /// Wheel odometry as the motion prior (ADR 0012): planar pose at the latest
    /// sample, and at the last exteroceptive update — their relative replaces the
    /// constant-velocity prediction when present.
    odom_now: Option<Se2>,
    odom_at_event: Option<Se2>,
    /// Gravity-frame plane height of each lidar frame seen (for the depth band).
    lidar_planes: Vec<(FrameId, f64)>,
    /// Pose of the last map fusion, per modality (a shared threshold lets the 40 Hz
    /// scans starve the ~10 Hz clouds of integration entirely).
    last_integrated: Option<Se2>,
    last_integrated_cloud: Option<Se2>,
    /// Current base pose in the odometry (anchor-relative, gravity-aligned) frame.
    current: Se2,
    last_motion: Se2,
    /// Time the last motion covered (s) — events arrive at heterogeneous rates
    /// (40 Hz scans interleaving ~10 Hz clouds), so prediction must scale by dt.
    last_motion_dt: f64,
    last_stamp: Option<Stamp>,
    stats: ScanOdometryStats,
    /// Reused buffers (hot path: no steady-state allocation).
    lifted: Vec<Vec3>,
    /// Per-point sRGB parallel to `lifted` when the current sweep is a coloured
    /// depth cloud (empty otherwise) — feeds the map's voxel colour channel at
    /// integration, never the solve.
    lifted_colors: Vec<[u8; 3]>,
    world: Vec<Vec3>,
    samples: Vec<Option<SdfSample>>,
}

impl ScanToMapOdometry {
    pub fn new(cfg: ScanToMapConfig) -> Self {
        Self::with_extrinsics(Pose::identity(), cfg, Vec::new())
    }

    pub fn anchored_at(base: Pose, cfg: ScanToMapConfig) -> Self {
        Self::with_extrinsics(base, cfg, Vec::new())
    }

    /// Multi-lidar: `extrinsics[frame.0]` = SE(3) `T_base_sensor` per rig frame.
    pub fn with_extrinsics(base: Pose, cfg: ScanToMapConfig, extrinsics: Vec<Pose>) -> Self {
        let attitude = AttitudeFilter::new(cfg.attitude.clone());
        let map = SparseTsdf::new(cfg.tsdf_3d.clone());
        let reg = SparseTsdf::new(cfg.tsdf.clone());
        ScanToMapOdometry {
            cfg,
            base,
            extrinsics,
            attitude,
            map,
            reg,
            submap_birth: Se2::identity(),
            submap_travel: 0.0,
            prev_map: None,
            prev_reg: None,
            prev_anchor: Se2::identity(),
            overlap_left: 0,
            frozen: Vec::new(),
            loops: Vec::new(),
            odometry_edges: Vec::new(),
            graph: None,
            scan_seen: false,
            lifted_is_cloud: false,
            odom_now: None,
            odom_at_event: None,
            lidar_planes: Vec::new(),
            last_integrated: None,
            last_integrated_cloud: None,
            current: Se2::identity(),
            last_motion: Se2::identity(),
            last_motion_dt: 0.0,
            last_stamp: None,
            stats: ScanOdometryStats::default(),
            lifted: Vec::new(),
            lifted_colors: Vec::new(),
            world: Vec::new(),
            samples: Vec::new(),
        }
    }

    pub fn stats(&self) -> ScanOdometryStats {
        self.stats
    }

    /// The active 3D submap (the map *product*) — export/visualization path.
    /// **Anchor-local**: see [`submaps_3d`](Self::submaps_3d) for world placement.
    pub fn map(&self) -> &SparseTsdf {
        &self.map
    }

    /// Every 3D submap field with its (possibly graph-optimised) anchor: frozen
    /// submaps in order, then the active one. Each field's voxels are in the
    /// *anchor-local* frame — the full map is the union after applying each anchor.
    /// All 3D fields in creation order: frozen, then the predecessor still in its
    /// overlap window (it exists and is being fused into — hiding it made whole
    /// submaps vanish from the viewer for the overlap's duration), then the active
    /// field. A submap's position in this list is its creation ordinal and never
    /// changes — viz entities keyed on it keep a stable identity across hand-overs.
    pub fn submaps_3d(&self) -> Vec<(Se2, &SparseTsdf)> {
        let mut out: Vec<(Se2, &SparseTsdf)> = self
            .frozen
            .iter()
            .map(|(anchor, _reg, map3d)| (*anchor, map3d))
            .collect();
        if let Some(prev) = &self.prev_map {
            out.push((self.prev_anchor, prev));
        }
        out.push((self.submap_birth, &self.map));
        out
    }

    /// How many leading entries of [`submaps_3d`](Self::submaps_3d) are frozen —
    /// immutable from here on (the tail still mutates: the active field, plus the
    /// overlap-window predecessor when present).
    pub fn frozen_submap_count(&self) -> usize {
        self.frozen.len()
    }

    fn extrinsic(&self, frame: FrameId) -> Option<Pose> {
        match self.extrinsics.get(frame.0 as usize) {
            Some(t) => Some(*t),
            None if frame == FrameId::BASE => Some(Pose::identity()),
            None => None,
        }
    }

    /// Beams → tilt-compensated 3D points in the base's gravity-aligned frame.
    /// Unlike the planar path, **z is kept and floor hits are kept**: in a 3D map the
    /// floor is structure, not noise.
    fn lift_scan(&mut self, scan: &LaserScan2D, t_base_sensor: &Pose) {
        self.lifted.clear();
        self.lifted_colors.clear();
        let tilt: Rotation = self.attitude.tilt();
        let tilted = self.attitude.is_initialized();
        for (i, &r) in scan.ranges.iter().enumerate() {
            let r = r as f64;
            if !r.is_finite() || r < scan.range_min || r > scan.range_max {
                continue;
            }
            let angle = scan.angle_min + i as f64 * scan.angle_increment;
            let p_sensor = Vec3::new(r * angle.cos(), r * angle.sin(), 0.0);
            let p_base = t_base_sensor.transform_point(p_sensor);
            self.lifted
                .push(if tilted { tilt.rotate(p_base) } else { p_base });
        }
    }

    /// Apply the planar pose to a gravity-aligned 3D point (z passes through).
    #[inline]
    fn apply_planar(t: &Se2, p: Vec3) -> Vec3 {
        let (s, c) = t.theta.sin_cos();
        Vec3::new(c * p.x - s * p.y + t.x, s * p.x + c * p.y + t.y, p.z)
    }

    /// 3-DoF Gauss-Newton: minimise the registration-field value at the transformed,
    /// gravity-plane-projected points.
    /// Returns (pose, inlier fraction, weak translation direction).
    fn register(&mut self, initial: Se2, sensor_z: f64) -> (Se2, f64, Option<slam_types::Vec2>) {
        // Fields store anchor-relative coordinates: solve in the target's local frame
        // and map the result back through its (possibly re-posed) anchor.
        let (map, anchor): (&dyn TsdfMap, Se2) = match &self.prev_reg {
            Some(prev) => (prev, self.prev_anchor),
            None => (&self.reg, self.submap_birth),
        };
        let initial = anchor.inverse().compose(&initial);
        let band = self.cfg.tsdf.truncation * 0.9;
        // Floor residuals carry no planar information (see ScanToMapConfig); gate them
        // out of the solve when the rig geometry lets us tell floor from wall.
        let gate_floor =
            self.attitude.is_initialized() && sensor_z > 2.0 * self.cfg.floor_clearance;
        let z_min = self.cfg.floor_clearance;
        let mut transform = initial;
        let mut inlier_fraction = 0.0;
        let mut h_translation = [0.0; 3];

        for _ in 0..self.cfg.max_iterations {
            self.world.clear();
            self.world.extend(self.lifted.iter().filter_map(|&p| {
                if gate_floor && p.z < z_min {
                    return None; // floor hit: not wall geometry
                }
                let q = Self::apply_planar(&transform, p);
                Some(Vec3::new(q.x, q.y, 0.0)) // gravity-plane projection
            }));
            map.sample_batch(&self.world, &mut self.samples);

            // NOTE: no PLICP-style residual trimming here — measured on cafe1, it
            // *hurts* (0.090→0.030 worse ATE): TSDF residuals near convergence are not
            // outlier-contaminated distances, and trimming the largest |sdf| discards
            // precisely the correcting signal. Dynamics robustness comes from the
            // keyframed integration diet + truncation band instead.
            let mut h = nalgebra::Matrix3::<f64>::zeros();
            let mut g = nalgebra::Vector3::<f64>::zeros();
            let mut used = 0usize;
            for (q, s) in self.world.iter().zip(self.samples.iter()) {
                let Some(s) = s else { continue };
                if s.sdf.abs() > band {
                    continue;
                }
                let (gx, gy) = (s.gradient.x, s.gradient.y);
                let jac = nalgebra::Vector3::new(gx, gy, gx * -q.y + gy * q.x);
                h += jac * jac.transpose();
                g += jac * s.sdf;
                used += 1;
            }
            inlier_fraction = used as f64 / self.lifted.len().max(1) as f64;
            if used < self.cfg.min_valid_points {
                return (transform, inlier_fraction, None);
            }
            h_translation = [h[(0, 0)], h[(0, 1)], h[(1, 1)]];

            let Some(delta) = h.cholesky().map(|ch| ch.solve(&(-g))) else {
                return (transform, 0.0, None);
            };
            transform = Se2::new(delta.x, delta.y, delta.z).compose(&transform);
            if delta.x.hypot(delta.y) < self.cfg.translation_epsilon
                && delta.z.abs() < self.cfg.rotation_epsilon
            {
                break;
            }
        }
        let weak = weak_translation_direction(h_translation, self.cfg.degeneracy_eigenvalue_ratio);
        // Back to the odometry frame (the weak direction rotates with the anchor).
        let world = anchor.compose(&transform);
        let weak_world = weak.map(|d| {
            let (s, c) = anchor.theta.sin_cos();
            slam_types::Vec2::new(c * d.x - s * d.y, s * d.x + c * d.y)
        });
        (world, inlier_fraction, weak_world)
    }

    /// Fuse the lifted points at the (odometry-frame) `pose`. Each submap's fields
    /// store **anchor-relative** coordinates (stage 3b prerequisite): re-posing a
    /// submap after graph optimisation is then just updating its anchor — voxels are
    /// never rewritten (ADR 0010).
    fn integrate(&mut self, pose: Se2, sensor_origin_base: Vec3, update_reg: bool) {
        for which in 0..2 {
            let anchor = match which {
                0 => self.submap_birth,
                _ if self.prev_map.is_some() => self.prev_anchor,
                _ => break,
            };
            let local = anchor.inverse().compose(&pose);
            let origin = Self::apply_planar(&local, sensor_origin_base);
            self.world.clear();
            self.world
                .extend(self.lifted.iter().map(|&p| Self::apply_planar(&local, p)));
            // Coloured clouds also feed the voxel colour channel (1:1 with the
            // transformed points) — the SDF and the solve never see colour.
            let field: &mut SparseTsdf = match which {
                0 => &mut self.map,
                _ => self.prev_map.as_mut().expect("checked above"),
            };
            if self.lifted_colors.len() == self.lifted.len() {
                field.integrate_points_colored(origin, &self.world, &self.lifted_colors);
            } else {
                field.integrate_points(origin, &self.world);
            }

            let flat_origin = Vec3::new(origin.x, origin.y, 0.0);
            self.world.clear();
            if update_reg {
                // Planar scan: everything except floor hits is slice content.
                let gate_floor = self.attitude.is_initialized()
                    && self.attitude.tilt().rotate(sensor_origin_base).z
                        > 2.0 * self.cfg.floor_clearance;
                let z_min = self.cfg.floor_clearance;
                self.world.extend(self.lifted.iter().filter_map(|&p| {
                    if gate_floor && p.z < z_min {
                        return None;
                    }
                    let q = Self::apply_planar(&local, p);
                    Some(Vec3::new(q.x, q.y, 0.0))
                }));
            } else if !self.lidar_planes.is_empty() {
                // Depth cloud: only points inside a laser plane's band measure the
                // slice the scan matcher registers against (step 1; step 2 — hybrid
                // per-point 3D/2D fan registration — is the planned successor).
                let tol = self.cfg.reg_band_tolerance;
                let planes = &self.lidar_planes;
                let world = &mut self.world;
                world.extend(self.lifted.iter().filter_map(|&p| {
                    if !planes.iter().any(|&(_, z)| (p.z - z).abs() <= tol) {
                        return None;
                    }
                    let q = Self::apply_planar(&local, p);
                    Some(Vec3::new(q.x, q.y, 0.0))
                }));
            }
            if !self.world.is_empty() {
                match which {
                    0 => self.reg.integrate_points(flat_origin, &self.world),
                    _ => {
                        if let Some(prev) = &mut self.prev_reg {
                            prev.integrate_points(flat_origin, &self.world);
                        }
                    }
                }
            }
        }

        if self.prev_map.is_some() {
            self.overlap_left = self.overlap_left.saturating_sub(1);
            if self.overlap_left == 0 {
                if let (Some(reg), Some(map3d)) = (self.prev_reg.take(), self.prev_map.take()) {
                    self.frozen.push((self.prev_anchor, reg, map3d));
                }
                self.prev_map = None;
            }
        }
    }

    /// Hand over to a fresh submap once enough has been *travelled* (arc length, not
    /// displacement — a tight loop never moves far from its centre).
    fn maybe_spawn_submap(&mut self) {
        if self.submap_travel > self.cfg.submap_extent && self.prev_map.is_none() {
            let fresh = SparseTsdf::new(self.cfg.tsdf_3d.clone());
            self.prev_map = Some(std::mem::replace(&mut self.map, fresh));
            let fresh = SparseTsdf::new(self.cfg.tsdf.clone());
            self.prev_reg = Some(std::mem::replace(&mut self.reg, fresh));
            self.prev_anchor = self.submap_birth;
            self.overlap_left = self.cfg.submap_overlap_scans;
            // The graph's odometry edge: measured relative between the two anchors,
            // frozen at hand-over time.
            self.odometry_edges
                .push(self.submap_birth.inverse().compose(&self.current));
            self.submap_birth = self.current;
            self.submap_travel = 0.0;
            self.stats.keyframes += 1; // a submap hand-over is the new "keyframe" event
        }
    }

    /// Verified loop closures detected so far (the pose-graph's loop edges).
    pub fn loop_closures(&self) -> &[LoopClosure] {
        &self.loops
    }

    /// Attach a pose-graph optimiser: verified loops then trigger optimisation and
    /// anchor re-posing instead of bare pose snapping (ADR 0010 stage 3b).
    pub fn set_graph(&mut self, graph: Box<dyn AnchorGraph>) {
        self.graph = Some(graph);
    }

    /// Anchor estimates: frozen submaps in order, then the active submap's anchor.
    pub fn anchors(&self) -> Vec<Se2> {
        let mut a: Vec<Se2> = self.frozen.iter().map(|(anchor, ..)| *anchor).collect();
        a.push(self.submap_birth);
        a
    }

    /// Shared post-registration tail: accept/coast, advance the motion model, run the
    /// keyframe-diet integration, loop closure and submap hand-over.
    fn apply_registration(
        &mut self,
        stamp: Stamp,
        sensor_origin: Vec3,
        predicted: Se2,
        (mut pose, inliers, weak): (Se2, f64, Option<slam_types::Vec2>),
        update_reg: bool,
    ) {
        let previous = self.current;
        if inliers >= self.cfg.min_inlier_fraction {
            if let Some(dir) = weak {
                // Unobservable direction (corridor): take the prediction's component.
                let slip = dir.x * (predicted.x - pose.x) + dir.y * (predicted.y - pose.y);
                pose = Se2::new(pose.x + dir.x * slip, pose.y + dir.y * slip, pose.theta);
                self.stats.degenerate += 1;
            }
            self.current = pose;
            self.stats.matched += 1;
        } else {
            // Unregistrable (dynamics, occlusion, empty model): coast on prediction.
            self.current = predicted;
            self.stats.coasted += 1;
        }
        self.last_motion = previous.inverse().compose(&self.current);
        self.last_motion_dt = self
            .last_stamp
            .map_or(0.0, |prev| (stamp - prev).as_seconds());
        self.submap_travel += self.last_motion.translation_norm();
        self.last_stamp = Some(stamp);
        self.odom_at_event = self.odom_now;

        let last = if update_reg {
            self.last_integrated
        } else {
            self.last_integrated_cloud
        };
        let due = match last {
            None => true,
            Some(li) => {
                let d = li.inverse().compose(&self.current);
                d.translation_norm() > self.cfg.integrate_translation
                    || d.theta.abs() > self.cfg.integrate_rotation
            }
        };
        if due {
            if !self.frozen.is_empty() {
                self.try_loop_closure();
            }
            self.integrate(self.current, sensor_origin, update_reg);
            if update_reg {
                self.last_integrated = Some(self.current);
            } else {
                self.last_integrated_cloud = Some(self.current);
            }
            self.maybe_spawn_submap();
        }
    }

    /// Motion prediction. Wheel odometry, when streaming, is the prior (its relative
    /// motion since the last exteroceptive update — the IMU-less robot's
    /// proprioception, ADR 0012); otherwise dt-scaled constant velocity.
    fn predict(&self, stamp: Stamp) -> Se2 {
        if let (Some(now), Some(at_event)) = (self.odom_now, self.odom_at_event) {
            return self.current.compose(&at_event.inverse().compose(&now));
        }
        self.predict_const_velocity(stamp)
    }

    /// Constant-velocity prediction, scaled to the actual time since the last event
    /// (mixed-rate streams make a fixed per-event step model wrong).
    fn predict_const_velocity(&self, stamp: Stamp) -> Se2 {
        let dt = self
            .last_stamp
            .map_or(0.0, |prev| (stamp - prev).as_seconds());
        if self.last_motion_dt <= 1e-6 || dt <= 0.0 {
            return self.current;
        }
        let k = (dt / self.last_motion_dt).clamp(0.0, 4.0);
        self.current.compose(&Se2::new(
            self.last_motion.x * k,
            self.last_motion.y * k,
            self.last_motion.theta * k,
        ))
    }

    /// Cloud points → tilt-compensated base frame (the cloud analogue of `lift_scan`).
    fn lift_cloud(&mut self, cloud: &slam_types::PointCloud, t_base_sensor: &Pose) {
        self.lifted.clear();
        self.lifted_colors.clear();
        let tilt: Rotation = self.attitude.tilt();
        let tilted = self.attitude.is_initialized();
        self.lifted.extend(cloud.points.iter().map(|&p| {
            let p_base = t_base_sensor.transform_point(p);
            if tilted {
                tilt.rotate(p_base)
            } else {
                p_base
            }
        }));
        if cloud.colors.len() == cloud.points.len() {
            self.lifted_colors.extend_from_slice(&cloud.colors);
        }
    }

    /// 3-DoF Gauss-Newton against the **3D** field (full trilinear): the depth path.
    /// Floor points contribute near-zero planar Jacobians (vertical gradients) and are
    /// kept — they are structure, and they cannot bias an (x, y, yaw) solve.
    fn register_3d(&mut self, initial: Se2) -> (Se2, f64, Option<slam_types::Vec2>) {
        let (map, anchor): (&dyn TsdfMap, Se2) = match &self.prev_map {
            Some(prev) => (prev, self.prev_anchor),
            None => (&self.map, self.submap_birth),
        };
        let initial = anchor.inverse().compose(&initial);
        let band = self.cfg.tsdf_3d.truncation * 0.9;
        let mut transform = initial;
        let mut inlier_fraction = 0.0;
        let mut h_translation = [0.0; 3];
        // Deterministic subsample: the solve saturates well below a full cloud.
        let stride = (self.lifted.len() / self.cfg.max_registration_points).max(1);

        for _ in 0..self.cfg.max_iterations {
            self.world.clear();
            self.world.extend(
                self.lifted
                    .iter()
                    .step_by(stride)
                    .map(|&p| Self::apply_planar(&transform, p)),
            );
            map.sample_batch(&self.world, &mut self.samples);

            let mut h = nalgebra::Matrix3::<f64>::zeros();
            let mut g = nalgebra::Vector3::<f64>::zeros();
            let mut used = 0usize;
            for (q, s) in self.world.iter().zip(self.samples.iter()) {
                let Some(s) = s else { continue };
                if s.sdf.abs() > band {
                    continue;
                }
                let (gx, gy) = (s.gradient.x, s.gradient.y);
                let jac = nalgebra::Vector3::new(gx, gy, gx * -q.y + gy * q.x);
                h += jac * jac.transpose();
                g += jac * s.sdf;
                used += 1;
            }
            inlier_fraction = used as f64 / self.world.len().max(1) as f64;
            if used < self.cfg.min_valid_points {
                return (transform, inlier_fraction, None);
            }
            h_translation = [h[(0, 0)], h[(0, 1)], h[(1, 1)]];
            let Some(delta) = h.cholesky().map(|ch| ch.solve(&(-g))) else {
                return (transform, 0.0, None);
            };
            transform = Se2::new(delta.x, delta.y, delta.z).compose(&transform);
            if delta.x.hypot(delta.y) < self.cfg.translation_epsilon
                && delta.z.abs() < self.cfg.rotation_epsilon
            {
                break;
            }
        }
        let weak = weak_translation_direction(h_translation, self.cfg.degeneracy_eigenvalue_ratio);
        // Back to the odometry frame (the weak direction rotates with the anchor).
        let world = anchor.compose(&transform);
        let weak_world = weak.map(|d| {
            let (s, c) = anchor.theta.sin_cos();
            slam_types::Vec2::new(c * d.x - s * d.y, s * d.x + c * d.y)
        });
        (world, inlier_fraction, weak_world)
    }

    /// Attempt loop closure against frozen submaps near the current pose: a seed-grid
    /// of registrations against the old submap's field, accepted only when the best
    /// solve verifies geometrically (ADR 0002: never trust proximity alone). On
    /// acceptance the pose snaps to the verified one — the full graph optimisation
    /// (GTSAM, stage 3b) will distribute the correction instead.
    fn try_loop_closure(&mut self) {
        let active_submap = self.frozen.len();
        let mut best: Option<LoopClosure> = None;
        let as_cloud = self.lifted_is_cloud;
        // Verification band follows the field being verified against.
        let band = if as_cloud {
            self.cfg.tsdf_3d.truncation * 0.9
        } else {
            self.cfg.tsdf.truncation * 0.9
        };
        let stride = if as_cloud {
            (self.lifted.len() / self.cfg.max_registration_points).max(1)
        } else {
            1
        };
        for (idx, (anchor, reg, map3d)) in self.frozen.iter().enumerate() {
            let field: &SparseTsdf = if as_cloud { map3d } else { reg };
            let dx = anchor.x - self.current.x;
            let dy = anchor.y - self.current.y;
            if dx.hypot(dy) > self.cfg.loop_radius {
                continue;
            }
            let (r, yw) = (self.cfg.loop_search_radius, self.cfg.loop_search_yaw);
            for sx in [-r, 0.0, r] {
                for sy in [-r, 0.0, r] {
                    for st in [-yw, 0.0, yw] {
                        // Frozen fields store anchor-relative coordinates: localise
                        // the seed, solve locally — the result *is* the verified
                        // anchor-relative measurement the graph needs.
                        let seed_world = Se2::new(
                            self.current.x + sx,
                            self.current.y + sy,
                            self.current.theta + st,
                        );
                        let seed = anchor.inverse().compose(&seed_world);
                        let (rel_target, inliers) = Self::register_against(
                            field,
                            &self.lifted,
                            &mut self.world,
                            &mut self.samples,
                            seed,
                            band,
                            self.cfg.max_iterations,
                            self.cfg.translation_epsilon,
                            self.cfg.rotation_epsilon,
                            !as_cloud,
                            stride,
                        );
                        if inliers >= self.cfg.loop_min_inliers
                            && best.is_none_or(|b| inliers > b.inliers)
                        {
                            best = Some(LoopClosure {
                                submap: idx,
                                active_submap,
                                pose: anchor.compose(&rel_target),
                                rel_target,
                                rel_active: self.submap_birth.inverse().compose(&self.current),
                                inliers,
                            });
                        }
                    }
                }
            }
        }
        if let Some(found) = best {
            self.loops.push(found);
            // With a pose graph attached (stage 3b) — and outside the transient
            // hand-over overlap, whose extra in-flight submap the simple anchor list
            // does not model — optimise and re-pose; otherwise snap (the servo: under
            // large drift each individual correction is small because registration
            // converges near its seed, so *repetition* re-localizes; its measured
            // price is millimetre jitter on drift-free revisits, cafe1-2
            // 0.0543→0.0559, and gating it was catastrophic on the ring circuit).
            if self.graph.is_some() && self.prev_map.is_none() {
                self.optimize_graph(found);
            } else {
                self.current = found.pose;
            }
        }
    }

    /// Run the pose graph over all anchors + recorded edges, re-pose every submap
    /// anchor, and carry the current pose through the active anchor's correction.
    fn optimize_graph(&mut self, latest: LoopClosure) {
        let anchors = self.anchors();
        let loops: Vec<(usize, usize, Se2)> = self
            .loops
            .iter()
            .map(|l| {
                // Measured relative anchor_target → anchor_active:
                // anchor_active = (anchor_target ∘ rel_target) ∘ rel_active⁻¹.
                (
                    l.submap,
                    l.active_submap,
                    l.rel_target.compose(&l.rel_active.inverse()),
                )
            })
            .collect();
        let graph = self.graph.as_mut().expect("checked by caller");
        let Some(optimized) = graph.optimize(&anchors, &self.odometry_edges, &loops) else {
            self.current = latest.pose; // graph failed: fall back to the snap
            return;
        };
        debug_assert_eq!(optimized.len(), anchors.len());
        for ((anchor, ..), new_anchor) in self.frozen.iter_mut().zip(&optimized) {
            *anchor = *new_anchor;
        }
        let new_active = *optimized.last().expect("non-empty");
        // Re-express the current pose under the re-posed active anchor, then apply
        // the loop's verified local measurement under the *updated* target anchor.
        let target = optimized[latest.submap];
        self.submap_birth = new_active;
        self.current = target.compose(&latest.rel_target);
    }

    /// One registration of `lifted` (gravity-plane projected) against an arbitrary
    /// field, without touching odometry state. Returns (pose, in-band fraction).
    #[allow(clippy::too_many_arguments)]
    fn register_against(
        field: &SparseTsdf,
        lifted: &[Vec3],
        world: &mut Vec<Vec3>,
        samples: &mut Vec<Option<SdfSample>>,
        seed: Se2,
        band: f64,
        max_iterations: usize,
        translation_epsilon: f64,
        rotation_epsilon: f64,
        flatten: bool,
        stride: usize,
    ) -> (Se2, f64) {
        let mut transform = seed;
        let mut inliers = 0.0;
        for _ in 0..max_iterations {
            world.clear();
            world.extend(lifted.iter().step_by(stride.max(1)).map(|&p| {
                let q = Self::apply_planar(&transform, p);
                if flatten {
                    Vec3::new(q.x, q.y, 0.0)
                } else {
                    q
                }
            }));
            field.sample_batch(world, samples);
            let mut h = nalgebra::Matrix3::<f64>::zeros();
            let mut g = nalgebra::Vector3::<f64>::zeros();
            let mut used = 0usize;
            for (q, s) in world.iter().zip(samples.iter()) {
                let Some(s) = s else { continue };
                if s.sdf.abs() > band {
                    continue;
                }
                let (gx, gy) = (s.gradient.x, s.gradient.y);
                let jac = nalgebra::Vector3::new(gx, gy, gx * -q.y + gy * q.x);
                h += jac * jac.transpose();
                g += jac * s.sdf;
                used += 1;
            }
            inliers = used as f64 / world.len().max(1) as f64;
            if used < 20 {
                return (transform, 0.0);
            }
            let Some(delta) = h.cholesky().map(|ch| ch.solve(&(-g))) else {
                return (transform, 0.0);
            };
            transform = Se2::new(delta.x, delta.y, delta.z).compose(&transform);
            if delta.x.hypot(delta.y) < translation_epsilon && delta.z.abs() < rotation_epsilon {
                break;
            }
        }
        (transform, inliers)
    }
}

impl SlamSystem for ScanToMapOdometry {
    fn name(&self) -> &str {
        "scan_matching_3d"
    }

    fn process_imu(&mut self, sample: &slam_types::ImuSample) {
        // Multi-IMU rigs: rotate rates/forces into the base frame; an unknown frame is
        // a mis-wired rig and is dropped, never guessed (ADR 0009).
        if sample.frame == slam_types::FrameId::BASE {
            self.attitude.process(sample);
        } else if let Some(t) = self.extrinsic(sample.frame) {
            self.attitude.process_in_frame(sample, &t);
        }
    }

    fn process_odometry(&mut self, odom: &slam_types::OdomSample) {
        // Planar projection of the platform's own pose estimate; consumed as
        // relative motion in `predict` so its absolute frame never matters.
        let planar = Se2::planar_projection_of(&odom.pose).0;
        if self.odom_at_event.is_none() {
            self.odom_at_event = Some(planar);
        }
        self.odom_now = Some(planar);
    }

    fn process_scan(&mut self, scan: &LaserScan2D) {
        self.stats.scans += 1;
        let Some(extrinsic) = self.extrinsic(scan.frame) else {
            self.stats.skipped += 1;
            return;
        };
        self.lift_scan(scan, &extrinsic);
        self.lifted_is_cloud = false;
        if self.lifted.len() < self.cfg.min_valid_points {
            self.stats.skipped += 1;
            return;
        }
        let sensor_origin = self.attitude.tilt().rotate(extrinsic.translation());
        self.scan_seen = true;
        match self.lidar_planes.iter_mut().find(|(f, _)| *f == scan.frame) {
            Some(entry) => entry.1 = sensor_origin.z,
            None => self.lidar_planes.push((scan.frame, sensor_origin.z)),
        }

        // First measurement of the run: the map is empty — seed it.
        if self.last_stamp.is_none() {
            self.last_stamp = Some(scan.stamp);
            self.integrate(self.current, sensor_origin, true);
            self.last_integrated = Some(self.current);
            return;
        }

        let predicted = self.predict(scan.stamp);
        let result = self.register(predicted, sensor_origin.z);
        self.apply_registration(scan.stamp, sensor_origin, predicted, result, true);
    }

    /// Ingest one back-projected RGB-D depth cloud (M4): lifted like a scan, but
    /// registered against the **3D** field with full trilinear sampling — the camera
    /// observes true 2D surfaces in 3D, so no gravity-plane projection is needed.
    /// Both modalities correct the one shared pose and fuse into the same submap.
    fn process_points(&mut self, cloud: &slam_types::PointCloud) {
        self.stats.scans += 1;
        let Some(extrinsic) = self.extrinsic(cloud.frame) else {
            self.stats.skipped += 1;
            return;
        };
        self.lift_cloud(cloud, &extrinsic);
        self.lifted_is_cloud = true;
        if self.lifted.len() < self.cfg.min_valid_points {
            self.stats.skipped += 1;
            return;
        }
        let sensor_origin = self.attitude.tilt().rotate(extrinsic.translation());

        if self.last_stamp.is_none() {
            self.last_stamp = Some(cloud.stamp);
            self.integrate(self.current, sensor_origin, false);
            self.last_integrated_cloud = Some(self.current);
            return;
        }

        if self.cfg.depth_updates_pose || !self.scan_seen {
            let predicted = self.predict(cloud.stamp);
            let result = self.register_3d(predicted);
            self.apply_registration(cloud.stamp, sensor_origin, predicted, result, false);
            return;
        }
        // Map-only mode: fuse at the (scan-corrected) pose on the cloud's own diet.
        let due = match self.last_integrated_cloud {
            None => true,
            Some(li) => {
                let d = li.inverse().compose(&self.current);
                d.translation_norm() > self.cfg.integrate_translation
                    || d.theta.abs() > self.cfg.integrate_rotation
            }
        };
        if due {
            self.integrate(self.current, sensor_origin, false);
            self.last_integrated_cloud = Some(self.current);
        }
    }

    fn current_estimate(&self) -> Option<StampedPose> {
        self.last_stamp.map(|stamp| StampedPose {
            stamp,
            pose: self.base * self.current.to_pose(),
        })
    }
}
