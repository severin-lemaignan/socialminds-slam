//! `slam-replay`: drive a [`SlamSystem`] over recorded sensor input, emit a TUM trajectory.
//!
//! For M0 the only input is an IMU CSV and the only systems are the trivial baselines.
//! As real front-ends and dataset adapters arrive, this binary gains input formats and
//! system choices while keeping the same "inputs → TUM trajectory" contract the harness
//! depends on.

use std::io::{self, Write};
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use slam_baseline::{ImuDeadReckoning, SlamSystem, Stationary};
use slam_types::Trajectory;

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

    /// Gravity magnitude (m/s²) for dead-reckoning.
    #[arg(long, default_value_t = slam_baseline::STANDARD_GRAVITY)]
    gravity: f64,
}

fn run(system: &mut dyn SlamSystem, samples: &[slam_types::ImuSample]) -> Trajectory {
    let mut traj = Trajectory::new();
    for s in samples {
        system.process_imu(s);
        if let Some(est) = system.current_estimate() {
            traj.push(est);
        }
    }
    traj
}

fn main() -> Result<()> {
    let args = Args::parse();

    let file = std::fs::File::open(&args.imu)
        .with_context(|| format!("opening IMU file {}", args.imu.display()))?;
    let samples = slam_types::read_imu(io::BufReader::new(file))
        .with_context(|| format!("reading IMU file {}", args.imu.display()))?;

    let mut system: Box<dyn SlamSystem> = match args.baseline {
        Baseline::Stationary => Box::new(Stationary::new()),
        Baseline::DeadReckoning => Box::new(ImuDeadReckoning::with_initial_state(
            slam_types::Pose::identity(),
            slam_types::Vec3::zeros(),
            args.gravity,
        )),
    };

    let traj = run(system.as_mut(), &samples);
    eprintln!(
        "slam-replay: ran '{}' over {} IMU samples -> {} poses",
        system.name(),
        samples.len(),
        traj.len()
    );

    match args.out {
        Some(path) => traj
            .write_tum_file(&path)
            .with_context(|| format!("writing {}", path.display()))?,
        None => {
            let stdout = io::stdout();
            traj.write_tum(stdout.lock())?;
            io::stdout().flush()?;
        }
    }
    Ok(())
}
