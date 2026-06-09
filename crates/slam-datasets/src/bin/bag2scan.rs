//! `slam-bag2scan`: extract a planar laser-scan stream from a ROS1 bag into scan CSV.
//!
//! Used by the evaluation harness to turn an OpenLORIS-Scene bag into a `slam-replay`
//! input for the 2D scan-matching front-end. `--list` inspects a bag's topics.

use std::io::{self, Write};
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    name = "slam-bag2scan",
    about = "Extract sensor_msgs/LaserScan from a ROS1 bag to scan CSV."
)]
struct Args {
    /// Input ROS1 `.bag` file.
    #[arg(long, value_name = "FILE")]
    bag: PathBuf,

    /// Scan topic to extract. If omitted, the unique sensor_msgs/LaserScan topic is used.
    #[arg(long, value_name = "TOPIC")]
    scan_topic: Option<String>,

    /// Output scan CSV. Defaults to stdout.
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

    let scans = slam_datasets::read_scans_from_bag(&args.bag, args.scan_topic.as_deref())
        .with_context(|| format!("reading scans from {}", args.bag.display()))?;
    eprintln!(
        "slam-bag2scan: extracted {} scans from {}",
        scans.len(),
        args.bag.display()
    );

    match args.out {
        Some(path) => {
            let file = std::fs::File::create(&path)
                .with_context(|| format!("creating {}", path.display()))?;
            slam_types::write_scans(&scans, io::BufWriter::new(file))?;
        }
        None => {
            let stdout = io::stdout();
            slam_types::write_scans(&scans, stdout.lock())?;
            io::stdout().flush()?;
        }
    }
    Ok(())
}
