//! Render dynamics masks (ADR 0015) over real colour frames from a ROS1 bag, for
//! visual inspection — exactly the inference path the replay uses (`slam-dynamics`),
//! not a Python re-implementation.
//!
//!     cargo run --release -p slam-replay --example mask_dump --features dynamics -- \
//!         --bag data/openloris/cafe1-1.bag --out eval/results/masking-ab/frames
//!
//! Output: PPM images, original pixels with masked regions tinted — red for the
//! person class, yellow for the rest of the dynamic set (chairs/couch/plant,
//! carryables, animals, vehicles). Pixels in both sets render red.

use std::collections::BTreeSet;
use std::io::Write as _;
use std::path::PathBuf;

use clap::Parser;
use slam_datasets::bag::BagFile;
use slam_datasets::parse_color_image;
use slam_dynamics::{ClassSet, SegConfig, YoloSeg};

#[derive(Parser)]
struct Args {
    /// ROS1 bag with a colour image topic.
    #[arg(long)]
    bag: PathBuf,
    /// Colour topic (OpenLORIS d400 default).
    #[arg(long, default_value = "/d400/color/image_raw")]
    color_topic: String,
    /// YOLO-seg ONNX export.
    #[arg(long, default_value = "onnx/yolo11s-seg-rect.onnx")]
    model: PathBuf,
    /// Confidence threshold (ADR 0015 operating point).
    #[arg(long, default_value_t = 0.2)]
    conf: f32,
    /// Mask dilation in model-input pixels.
    #[arg(long, default_value_t = 8)]
    dilate_px: usize,
    /// How many frames, evenly spaced through the bag.
    #[arg(long, default_value_t = 8)]
    frames: usize,
    /// Output directory for the PPM overlays.
    #[arg(long)]
    out: PathBuf,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    std::fs::create_dir_all(&args.out)?;

    let mut bag = BagFile::open(&args.bag)?;
    let wanted: BTreeSet<u32> = bag
        .connections()
        .iter()
        .filter(|c| c.topic == args.color_topic && c.message_type == "sensor_msgs/Image")
        .map(|c| c.id)
        .collect();
    if wanted.is_empty() {
        return Err(format!("no sensor_msgs/Image on {}", args.color_topic).into());
    }

    // Pass 1: count frames so the selection spans the whole sequence.
    let mut total = 0usize;
    bag.for_each_message(&wanted, |_, _| {
        total += 1;
        Ok(())
    })?;
    let n = args.frames.min(total);
    let picks: BTreeSet<usize> = (0..n).map(|i| i * total / n).collect();
    eprintln!("{total} colour frames; dumping {n}");

    let seg = |classes: ClassSet| {
        YoloSeg::load(
            &args.model,
            SegConfig {
                conf: args.conf,
                dilate_px: args.dilate_px,
                classes,
                ..SegConfig::default()
            },
        )
    };
    let mut person = seg(ClassSet::Person)?;
    let mut dynamic = seg(ClassSet::Dynamic)?;

    // Pass 2: decode + infer + write only the picked frames.
    let mut idx = 0usize;
    bag.for_each_message(&wanted, |_, data| {
        let i = idx;
        idx += 1;
        if !picks.contains(&i) {
            return Ok(());
        }
        let img = parse_color_image(data).expect("colour frame decodes");
        let (w, h) = (img.width(), img.height());
        let mut rgb = img.to_rgb8();
        let mp = person.mask_rgb8(&rgb, w, h, img.stamp).expect("inference");
        let md = dynamic.mask_rgb8(&rgb, w, h, img.stamp).expect("inference");
        for v in 0..h {
            for u in 0..w {
                let (p, d) = (mp.data[v * w + u], md.data[v * w + u]);
                if !(p || d) {
                    continue;
                }
                let px = &mut rgb[(v * w + u) * 3..(v * w + u) * 3 + 3];
                // Tint 60 % towards red (person) / yellow (other dynamic classes).
                let tint: [u8; 3] = if p { [255, 0, 0] } else { [255, 220, 0] };
                for (c, t) in px.iter_mut().zip(tint) {
                    *c = ((*c as u32 * 2 + t as u32 * 3) / 5) as u8;
                }
            }
        }
        let cov = |m: &slam_types::PixelMask| {
            100.0 * m.data.iter().filter(|&&b| b).count() as f64 / (w * h) as f64
        };
        let name = format!("frame-{i:05}-t{:.3}.ppm", img.stamp.as_seconds());
        let mut f = std::fs::File::create(args.out.join(&name)).expect("create ppm");
        write!(f, "P6\n{w} {h}\n255\n").expect("ppm header");
        f.write_all(&rgb).expect("ppm pixels");
        eprintln!(
            "{name}: person {:.1} %, dynamic-set {:.1} %",
            cov(&mp),
            cov(&md)
        );
        Ok(())
    })?;
    Ok(())
}
