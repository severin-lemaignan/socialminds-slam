//! `slam-bag2imu`: extract the IMU stream from a ROS1 bag into our IMU CSV format.
//!
//! Used by the evaluation harness to turn an OpenLORIS-Scene bag into a `slam-replay`
//! input. `--list` inspects a bag's topics without converting.

use std::io::{self, Write};
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    name = "slam-bag2imu",
    about = "Extract sensor_msgs/Imu from a ROS1 bag to IMU CSV."
)]
struct Args {
    /// Input ROS1 `.bag` file.
    #[arg(long, value_name = "FILE")]
    bag: PathBuf,

    /// IMU topic to extract. If omitted, the unique sensor_msgs/Imu topic is used.
    #[arg(long, value_name = "TOPIC")]
    imu_topic: Option<String>,

    /// Output IMU CSV. Defaults to stdout.
    #[arg(long, value_name = "FILE")]
    out: Option<PathBuf>,

    /// List topics (and message types) in the bag, then exit.
    #[arg(long)]
    list: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    if args.list {
        let topics = slam_datasets::list_topics(&args.bag)
            .with_context(|| format!("listing topics in {}", args.bag.display()))?;
        for t in topics {
            println!("{}\t{}", t.topic, t.message_type);
        }
        return Ok(());
    }

    let samples = slam_datasets::read_imu_from_bag(&args.bag, args.imu_topic.as_deref())
        .with_context(|| format!("reading IMU from {}", args.bag.display()))?;
    eprintln!(
        "slam-bag2imu: extracted {} IMU samples from {}",
        samples.len(),
        args.bag.display()
    );

    match args.out {
        Some(path) => {
            let file = std::fs::File::create(&path)
                .with_context(|| format!("creating {}", path.display()))?;
            slam_types::write_imu(&samples, io::BufWriter::new(file))?;
        }
        None => {
            let stdout = io::stdout();
            slam_types::write_imu(&samples, stdout.lock())?;
            io::stdout().flush()?;
        }
    }
    Ok(())
}
