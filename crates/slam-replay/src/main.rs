//! `slam-replay`: drive a [`SlamSystem`] over recorded sensor input, emit a TUM trajectory.
//!
//! For M0/M1 the only input is an IMU CSV and the only systems are the trivial baselines.
//! As real front-ends and dataset adapters arrive, this binary gains input formats and
//! system choices while keeping the same "inputs → TUM trajectory" contract the harness
//! depends on. With `--metrics` it also writes a compute-metrics JSON sidecar.

mod metrics;

use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use slam_baseline::{ImuDeadReckoning, SlamSystem, Stationary};
use slam_types::{ImuSample, Pose, Trajectory, Vec3};

use metrics::ProcessingMetrics;

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Baseline {
    /// Fixed pose; the sanity floor.
    Stationary,
    /// IMU strapdown integration.
    DeadReckoning,
}

#[derive(Parser, Debug)]
#[command(
    name = "slam-replay",
    about = "Run a baseline SLAM system over recorded input and write a TUM trajectory."
)]
struct Args {
    /// Which system to run.
    #[arg(long, value_enum)]
    baseline: Baseline,

    /// IMU CSV input (`t gx gy gz ax ay az`).
    #[arg(long, value_name = "FILE")]
    imu: PathBuf,

    /// Output TUM trajectory file. Defaults to stdout.
    #[arg(long, value_name = "FILE")]
    out: Option<PathBuf>,

    /// Write a compute-metrics JSON sidecar (latency, throughput, real-time factor).
    #[arg(long, value_name = "FILE")]
    metrics: Option<PathBuf>,

    /// Initialise the system's pose (and, for dead-reckoning, velocity) from the first
    /// pose(s) of this TUM ground-truth file. Gives odometry a fair start on real data.
    #[arg(long, value_name = "FILE")]
    init_pose_from_tum: Option<PathBuf>,

    /// Gravity magnitude (m/s²) for dead-reckoning.
    #[arg(long, default_value_t = slam_baseline::STANDARD_GRAVITY)]
    gravity: f64,
}

/// Initial pose + velocity for a run.
struct InitialState {
    pose: Pose,
    velocity: Vec3,
}

impl Default for InitialState {
    fn default() -> Self {
        InitialState {
            pose: Pose::identity(),
            velocity: Vec3::zeros(),
        }
    }
}

/// Derive an initial pose and velocity from the first samples of a TUM trajectory.
/// Velocity is a finite difference of the first two positions (zero if fewer).
fn initial_state_from_tum(path: &Path) -> Result<InitialState> {
    let traj = Trajectory::read_tum_file(path)
        .with_context(|| format!("reading init pose from {}", path.display()))?;
    let poses = traj.poses();
    let first = poses
        .first()
        .with_context(|| format!("{} has no poses", path.display()))?;
    let velocity = match poses.get(1) {
        Some(second) => {
            let dt = (second.stamp - first.stamp).as_seconds();
            if dt > 0.0 {
                (second.pose.translation() - first.pose.translation()) / dt
            } else {
                Vec3::zeros()
            }
        }
        None => Vec3::zeros(),
    };
    Ok(InitialState {
        pose: first.pose,
        velocity,
    })
}

/// Run the system over the samples, collecting the trajectory and per-sample latencies.
fn run_timed(
    system: &mut dyn SlamSystem,
    samples: &[ImuSample],
) -> (Trajectory, Vec<u64>, std::time::Duration) {
    let mut traj = Trajectory::new();
    let mut latencies = Vec::with_capacity(samples.len());
    let start = Instant::now();
    for s in samples {
        let t0 = Instant::now();
        system.process_imu(s);
        let est = system.current_estimate();
        latencies.push(t0.elapsed().as_nanos() as u64);
        if let Some(est) = est {
            traj.push(est);
        }
    }
    (traj, latencies, start.elapsed())
}

fn input_span_seconds(samples: &[ImuSample]) -> f64 {
    match (samples.first(), samples.last()) {
        (Some(a), Some(b)) => (b.stamp - a.stamp).as_seconds(),
        _ => 0.0,
    }
}

fn main() -> Result<()> {
    let args = Args::parse();

    let file = std::fs::File::open(&args.imu)
        .with_context(|| format!("opening IMU file {}", args.imu.display()))?;
    let samples = slam_types::read_imu(io::BufReader::new(file))
        .with_context(|| format!("reading IMU file {}", args.imu.display()))?;

    let init = match &args.init_pose_from_tum {
        Some(path) => initial_state_from_tum(path)?,
        None => InitialState::default(),
    };

    let mut system: Box<dyn SlamSystem> = match args.baseline {
        Baseline::Stationary => Box::new(Stationary::anchored_at(init.pose)),
        Baseline::DeadReckoning => Box::new(ImuDeadReckoning::with_initial_state(
            init.pose,
            init.velocity,
            args.gravity,
        )),
    };

    let (traj, latencies, wall) = run_timed(system.as_mut(), &samples);
    let span = input_span_seconds(&samples);
    let m = ProcessingMetrics::new(system.name(), samples.len(), span, wall, latencies);
    eprintln!(
        "slam-replay: ran '{}' over {} IMU samples -> {} poses ({:.0}x real-time)",
        system.name(),
        samples.len(),
        traj.len(),
        m.real_time_factor,
    );

    match &args.out {
        Some(path) => traj
            .write_tum_file(path)
            .with_context(|| format!("writing {}", path.display()))?,
        None => {
            let stdout = io::stdout();
            traj.write_tum(stdout.lock())?;
            io::stdout().flush()?;
        }
    }

    if let Some(path) = &args.metrics {
        std::fs::write(path, m.to_json())
            .with_context(|| format!("writing metrics {}", path.display()))?;
    }

    Ok(())
}
