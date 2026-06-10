//! Decoding `sensor_msgs/CameraInfo` and depth `sensor_msgs/Image` from ROS1 wire
//! bytes, back-projecting straight into engine point clouds (M4: RGB-D-inertial).
//!
//! Per ADR 0009, intrinsics come from the CameraInfo topic that rides alongside every
//! image stream — no engine-specific calibration format. Depth images are decoded and
//! back-projected immediately (`p = ((u−cx)/fx·z, (v−cy)/fy·z, z)`, camera frame,
//! z forward) with a pixel stride and a range clip, so the raw image bytes are never
//! retained: a 10-minute bag of VGA depth stays in the hundreds of MB, not tens of GB.
//!
//! Wire layouts (little-endian, length-prefixed strings/arrays):
//!
//! ```text
//! sensor_msgs/CameraInfo: Header; u32 height, width; string distortion_model;
//!                         f64[] D; f64[9] K; f64[9] R; f64[12] P; u32 binning_x/y;
//!                         RegionOfInterest (4 × u32 + u8)
//! sensor_msgs/Image:      Header; u32 height, width; string encoding;
//!                         u8 is_bigendian; u32 step; u8[] data
//! ```
//!
//! Depth encodings handled: `16UC1`/`mono16` (millimetres) and `32FC1` (metres).
//! Colour encodings handled: `rgb8`, `bgr8`, `rgba8`, `bgra8`.

use slam_types::{PointCloud, Stamp, Vec3};

use crate::BagError;

/// Pinhole intrinsics from a `CameraInfo` message.
#[derive(Debug, Clone, Copy)]
pub struct Intrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

/// Depth → cloud conversion tuning.
///
/// Sampling is **range-adaptive**: the projected spacing of kept pixels on the surface
/// is `stride·z/fx`, so a *fixed* stride oversamples near range and — fatally —
/// undersamples far range, where the integrated TSDF band degenerates into isolated
/// clumps no interpolation stencil can use (measured: market aisles at 4–6 m read
/// 0 matched / 2047 coasted at fixed stride 4 with 2.5 cm voxels). Instead, each
/// pixel's local stride is chosen so the spacing stays ≈ `target_spacing` at its own
/// depth: `s(z) = clamp(target_spacing·fx/z, min_stride, ∞)`, quantised to powers of
/// two of `min_stride` so kept pixels form locally regular grids.
#[derive(Debug, Clone)]
pub struct DepthConfig {
    /// Desired sample spacing on the surface (m). Match the 3D field's voxel size.
    pub target_spacing: f64,
    /// Finest pixel stride (near-range floor; bounds the point count).
    pub min_stride: usize,
    /// Hard cap per cloud: above it, the cloud is uniformly re-decimated (memory and
    /// integration cost stay bounded on pathological all-far frames).
    pub max_points: usize,
    /// Range clip (m): RealSense depth is noise beyond a few metres.
    pub min_range: f64,
    pub max_range: f64,
}

impl Default for DepthConfig {
    fn default() -> Self {
        DepthConfig {
            target_spacing: 0.05,
            min_stride: 2,
            max_points: 20_000,
            min_range: 0.3,
            max_range: 6.0,
        }
    }
}

/// A decoded colour `sensor_msgs/Image`, kept only long enough to colour the depth
/// frame it rides with (RealSense publishes aligned depth and colour on the same
/// pixel grid and stamp). Raw RGB is *stored* on the cloud — illumination
/// normalization is a consumer concern (viz, future voxel-colour channel), never
/// an ingest one: information is not destroyed at the sensor boundary.
pub struct ColorImage {
    pub stamp: Stamp,
    width: usize,
    height: usize,
    step: usize,
    /// Byte offsets of (r, g, b) within a pixel.
    rgb: (usize, usize, usize),
    bpp: usize,
    data: Vec<u8>,
}

impl ColorImage {
    fn rgb_at(&self, u: usize, v: usize) -> [u8; 3] {
        let off = v * self.step + u * self.bpp;
        [
            self.data[off + self.rgb.0],
            self.data[off + self.rgb.1],
            self.data[off + self.rgb.2],
        ]
    }
}

/// Decode one colour `sensor_msgs/Image` body.
pub fn parse_color_image(data: &[u8]) -> Result<ColorImage, BagError> {
    let mut c = LeCursor { data, pos: 0 };
    let (stamp, _frame_id) = c.header()?;
    let height = c.u32()? as usize;
    let width = c.u32()? as usize;
    let encoding = c.string()?;
    let _is_bigendian = c.take(1)?;
    let step = c.u32()? as usize;
    let len = c.u32()? as usize;
    let pixels = c.take(len)?;
    let (rgb, bpp) = match encoding.as_str() {
        "rgb8" => ((0, 1, 2), 3),
        "bgr8" => ((2, 1, 0), 3),
        "rgba8" => ((0, 1, 2), 4),
        "bgra8" => ((2, 1, 0), 4),
        _ => {
            eprintln!("slam-datasets: unsupported colour encoding {encoding:?}");
            return Err(BagError::Format("unsupported colour encoding"));
        }
    };
    if step < width * bpp || len < step * height {
        return Err(BagError::Format("colour image size mismatch"));
    }
    Ok(ColorImage {
        stamp,
        width,
        height,
        step,
        rgb,
        bpp,
        data: pixels.to_vec(),
    })
}

struct LeCursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> LeCursor<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], BagError> {
        let end = self
            .pos
            .checked_add(n)
            .ok_or(BagError::Format("image message length overflow"))?;
        let slice = self
            .data
            .get(self.pos..end)
            .ok_or(BagError::Format("image message truncated"))?;
        self.pos = end;
        Ok(slice)
    }

    fn u32(&mut self) -> Result<u32, BagError> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f64(&mut self) -> Result<f64, BagError> {
        let b = self.take(8)?;
        Ok(f64::from_le_bytes([
            b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
        ]))
    }

    fn string(&mut self) -> Result<String, BagError> {
        let len = self.u32()? as usize;
        Ok(String::from_utf8_lossy(self.take(len)?).into_owned())
    }

    /// Skip the `std_msgs/Header`, returning (stamp, frame_id).
    fn header(&mut self) -> Result<(Stamp, String), BagError> {
        let _seq = self.u32()?;
        let secs = self.u32()?;
        let nsecs = self.u32()?;
        let frame_id = self.string()?;
        Ok((
            Stamp::from_nanos(secs as i64 * 1_000_000_000 + nsecs as i64),
            frame_id,
        ))
    }
}

/// Decode one `sensor_msgs/CameraInfo` body into pinhole intrinsics + its frame_id.
pub fn parse_camera_info(data: &[u8]) -> Result<(Intrinsics, String), BagError> {
    let mut c = LeCursor { data, pos: 0 };
    let (_stamp, frame_id) = c.header()?;
    let _height = c.u32()?;
    let _width = c.u32()?;
    let _distortion_model = c.string()?;
    let d_len = c.u32()? as usize;
    if d_len > data.len() / 8 {
        return Err(BagError::Format("camera_info D length exceeds message"));
    }
    for _ in 0..d_len {
        c.f64()?;
    }
    // K, row-major 3×3: [fx 0 cx; 0 fy cy; 0 0 1].
    let mut k = [0.0f64; 9];
    for v in &mut k {
        *v = c.f64()?;
    }
    Ok((
        Intrinsics {
            fx: k[0],
            fy: k[4],
            cx: k[2],
            cy: k[5],
        },
        frame_id,
    ))
}

/// Decode one depth `sensor_msgs/Image` body and back-project it into a camera-frame
/// point cloud (no image bytes retained). Returns the cloud + its frame_id.
pub fn parse_depth_image(
    data: &[u8],
    intrinsics: &Intrinsics,
    cfg: &DepthConfig,
    color: Option<&ColorImage>,
) -> Result<(PointCloud, String), BagError> {
    let mut c = LeCursor { data, pos: 0 };
    let (stamp, frame_id) = c.header()?;
    let height = c.u32()? as usize;
    let width = c.u32()? as usize;
    let encoding = c.string()?;
    let _is_bigendian = c.take(1)?;
    let step = c.u32()? as usize;
    let len = c.u32()? as usize;
    let pixels = c.take(len)?;

    // `mono16` is how OpenLORIS labels its 16-bit millimetre depth (same layout
    // as the canonical `16UC1`).
    let depth_at: fn(&[u8], usize) -> f64 = match encoding.as_str() {
        "16UC1" | "mono16" => |b, off| u16::from_le_bytes([b[off], b[off + 1]]) as f64 * 1e-3,
        "32FC1" => |b, off| f32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]]) as f64,
        _ => {
            eprintln!("slam-datasets: unsupported depth encoding {encoding:?}");
            return Err(BagError::Format("unsupported depth encoding"));
        }
    };
    let bpp = if encoding == "32FC1" { 4 } else { 2 };
    if step < width * bpp || len < step * height {
        return Err(BagError::Format("depth image size mismatch"));
    }

    // Colour rides only when the paired frame is plausibly the *same* moment —
    // a stale frame would paint the wrong wall.
    let color = color.filter(|c| (c.stamp.as_seconds() - stamp.as_seconds()).abs() < 0.05);
    let base = cfg.min_stride.max(1);
    let mut points = Vec::with_capacity(4096);
    let mut colors: Vec<[u8; 3]> = Vec::with_capacity(if color.is_some() { 4096 } else { 0 });
    let mut v = 0;
    while v < height {
        let row = v * step;
        let mut u = 0;
        while u < width {
            let z = depth_at(pixels, row + u * bpp);
            if z.is_finite() && z >= cfg.min_range && z <= cfg.max_range {
                // Local stride for this depth, as a power-of-two multiple of the base
                // grid: keep the pixel only when it sits on its own stride's lattice,
                // so kept pixels form locally regular grids matched to their range.
                let needed = (cfg.target_spacing * intrinsics.fx / z).max(base as f64);
                let mut k = base;
                while ((k * 2) as f64) <= needed {
                    k *= 2;
                }
                if u % k == 0 && v % k == 0 {
                    points.push(Vec3::new(
                        (u as f64 - intrinsics.cx) / intrinsics.fx * z,
                        (v as f64 - intrinsics.cy) / intrinsics.fy * z,
                        z,
                    ));
                    if let Some(c) = color {
                        // Aligned depth shares the colour pixel grid; rescale if the
                        // resolutions differ anyway.
                        let cu = (u * c.width / width).min(c.width - 1);
                        let cv = (v * c.height / height).min(c.height - 1);
                        colors.push(c.rgb_at(cu, cv));
                    }
                }
            }
            u += base;
        }
        v += base;
    }
    if points.len() > cfg.max_points {
        let keep_every = points.len().div_ceil(cfg.max_points);
        let mut i = 0;
        points.retain(|_| {
            let keep = i % keep_every == 0;
            i += 1;
            keep
        });
        let mut i = 0;
        colors.retain(|_| {
            let keep = i % keep_every == 0;
            i += 1;
            keep
        });
    }

    Ok((
        PointCloud {
            stamp,
            frame: slam_types::FrameId::BASE,
            points,
            colors,
        },
        frame_id,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(frame: &str) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&7u32.to_le_bytes());
        v.extend_from_slice(&100u32.to_le_bytes());
        v.extend_from_slice(&500u32.to_le_bytes());
        v.extend_from_slice(&(frame.len() as u32).to_le_bytes());
        v.extend_from_slice(frame.as_bytes());
        v
    }

    fn encode_camera_info(frame: &str, fx: f64, fy: f64, cx: f64, cy: f64) -> Vec<u8> {
        let mut v = header(frame);
        v.extend_from_slice(&480u32.to_le_bytes()); // height
        v.extend_from_slice(&848u32.to_le_bytes()); // width
        v.extend_from_slice(&5u32.to_le_bytes());
        v.extend_from_slice(b"plumb"); // distortion model
        v.extend_from_slice(&5u32.to_le_bytes()); // D: 5 zeros
        for _ in 0..5 {
            v.extend_from_slice(&0f64.to_le_bytes());
        }
        for k in [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0] {
            v.extend_from_slice(&k.to_le_bytes());
        }
        v
    }

    fn encode_depth_16u(frame: &str, w: usize, h: usize, mm: &[u16]) -> Vec<u8> {
        let mut v = header(frame);
        v.extend_from_slice(&(h as u32).to_le_bytes());
        v.extend_from_slice(&(w as u32).to_le_bytes());
        v.extend_from_slice(&5u32.to_le_bytes());
        v.extend_from_slice(b"16UC1");
        v.push(0); // little-endian
        v.extend_from_slice(&((w * 2) as u32).to_le_bytes()); // step
        v.extend_from_slice(&((w * h * 2) as u32).to_le_bytes());
        for d in mm {
            v.extend_from_slice(&d.to_le_bytes());
        }
        v
    }

    #[test]
    fn camera_info_yields_pinhole_k() {
        let body = encode_camera_info("d400_color", 421.5, 421.0, 424.0, 240.5);
        let (k, frame) = parse_camera_info(&body).unwrap();
        assert_eq!(frame, "d400_color");
        assert_eq!((k.fx, k.fy, k.cx, k.cy), (421.5, 421.0, 424.0, 240.5));
    }

    #[test]
    fn depth_backprojects_in_range_pixels() {
        // 4×4 image, fx=fy=2, cx=cy=2. With target_spacing tiny, every base-grid
        // pixel in range is kept: (0,0)=invalid 0, (2,0)=4 m (clipped), (0,2)=1 m,
        // (2,2)=2 m.
        let k = Intrinsics {
            fx: 2.0,
            fy: 2.0,
            cx: 2.0,
            cy: 2.0,
        };
        let cfg = DepthConfig {
            target_spacing: 1e-6,
            min_stride: 2,
            max_points: 100,
            min_range: 0.5,
            max_range: 3.0,
        };
        let mut mm = [0u16; 16];
        mm[2] = 4000; // (u=2, v=0): clipped
        mm[2 * 4] = 1000; // (u=0, v=2): 1 m
        mm[2 * 4 + 2] = 2000; // (u=2, v=2): 2 m
        let body = encode_depth_16u("d400_color", 4, 4, &mm);
        let (cloud, frame) = parse_depth_image(&body, &k, &cfg, None).unwrap();
        assert_eq!(frame, "d400_color");
        assert_eq!(cloud.stamp.as_nanos(), 100_000_000_500);
        assert_eq!(cloud.points.len(), 2);
        // (0,2) at 1 m: ((0−2)/2·1, (2−2)/2·1, 1) = (−1, 0, 1).
        let p = cloud.points[0];
        assert!((p.x + 1.0).abs() < 1e-12 && p.y.abs() < 1e-12 && (p.z - 1.0).abs() < 1e-12);
        // (2,2) at 2 m: (0, 0, 2).
        let q = cloud.points[1];
        assert!(q.x.abs() < 1e-12 && q.y.abs() < 1e-12 && (q.z - 2.0).abs() < 1e-12);
    }

    #[test]
    fn sampling_is_range_adaptive() {
        // fx = 100, target spacing 0.05 m → required stride: 10 px at z=0.5 m
        // (lattice 8 with base 2), 2.5 px at z=2 m (lattice 2). A 16×16 image split:
        // left half near (0.5 m), right half far (2 m) — the far half must yield
        // ~(8/2)² = 16 points, the near half only ~(16/8)·(16/8)/2... measured by
        // counting kept points per half.
        let k = Intrinsics {
            fx: 100.0,
            fy: 100.0,
            cx: 8.0,
            cy: 8.0,
        };
        let cfg = DepthConfig {
            target_spacing: 0.05,
            min_stride: 2,
            max_points: 10_000,
            min_range: 0.1,
            max_range: 6.0,
        };
        let mut mm = [0u16; 256];
        for v in 0..16 {
            for u in 0..16 {
                mm[v * 16 + u] = if u < 8 { 500 } else { 2000 };
            }
        }
        let body = encode_depth_16u("f", 16, 16, &mm);
        let (cloud, _) = parse_depth_image(&body, &k, &cfg, None).unwrap();
        let near = cloud.points.iter().filter(|p| p.z < 1.0).count();
        let far = cloud.points.iter().filter(|p| p.z > 1.0).count();
        // Far half sampled at lattice 2 → 8×4 columns... at least 4× denser than near.
        assert!(far >= 4 * near.max(1), "near={near} far={far}");
        assert!(near >= 1 && far >= 12, "near={near} far={far}");
    }

    #[test]
    fn point_cap_bounds_pathological_frames() {
        let k = Intrinsics {
            fx: 100.0,
            fy: 100.0,
            cx: 8.0,
            cy: 8.0,
        };
        let cfg = DepthConfig {
            target_spacing: 1e-6,
            min_stride: 1,
            max_points: 20,
            min_range: 0.1,
            max_range: 6.0,
        };
        let mm = [2000u16; 256];
        let body = encode_depth_16u("f", 16, 16, &mm);
        let (cloud, _) = parse_depth_image(&body, &k, &cfg, None).unwrap();
        assert!(cloud.points.len() <= 20, "{}", cloud.points.len());
    }

    fn encode_rgb8(frame: &str, w: usize, h: usize, px: &[[u8; 3]]) -> Vec<u8> {
        let mut v = header(frame);
        v.extend_from_slice(&(h as u32).to_le_bytes());
        v.extend_from_slice(&(w as u32).to_le_bytes());
        v.extend_from_slice(&4u32.to_le_bytes());
        v.extend_from_slice(b"rgb8");
        v.push(0);
        v.extend_from_slice(&((w * 3) as u32).to_le_bytes()); // step
        v.extend_from_slice(&((w * h * 3) as u32).to_le_bytes());
        for c in px {
            v.extend_from_slice(c);
        }
        v
    }

    #[test]
    fn depth_points_pick_up_aligned_color() {
        let k = Intrinsics {
            fx: 2.0,
            fy: 2.0,
            cx: 2.0,
            cy: 2.0,
        };
        let cfg = DepthConfig {
            target_spacing: 1e-6,
            min_stride: 2,
            max_points: 100,
            min_range: 0.5,
            max_range: 3.0,
        };
        let mut mm = [0u16; 16];
        mm[2 * 4] = 1000; // (u=0, v=2)
        mm[2 * 4 + 2] = 2000; // (u=2, v=2)
        let depth = encode_depth_16u("d400_color", 4, 4, &mm);
        // Same 4×4 grid: paint (0,2) red, (2,2) blue. Same header stamp → paired.
        let mut px = [[0u8; 3]; 16];
        px[2 * 4] = [200, 10, 10];
        px[2 * 4 + 2] = [10, 10, 200];
        let color = parse_color_image(&encode_rgb8("d400_color", 4, 4, &px)).unwrap();
        let (cloud, _) = parse_depth_image(&depth, &k, &cfg, Some(&color)).unwrap();
        assert_eq!(cloud.points.len(), 2);
        assert_eq!(cloud.colors, vec![[200, 10, 10], [10, 10, 200]]);
    }

    #[test]
    fn stale_color_is_dropped() {
        let k = Intrinsics {
            fx: 2.0,
            fy: 2.0,
            cx: 2.0,
            cy: 2.0,
        };
        let mut mm = [0u16; 16];
        mm[2 * 4] = 1000;
        let depth = encode_depth_16u("f", 4, 4, &mm);
        let mut color = parse_color_image(&encode_rgb8("f", 4, 4, &[[9u8; 3]; 16])).unwrap();
        color.stamp = Stamp::from_nanos(color.stamp.as_nanos() + 200_000_000); // +0.2 s
        let (cloud, _) =
            parse_depth_image(&depth, &k, &DepthConfig::default(), Some(&color)).unwrap();
        assert_eq!(cloud.points.len(), 1);
        assert!(cloud.colors.is_empty(), "stale colour must not pair");
    }

    #[test]
    fn truncated_image_is_rejected() {
        let k = Intrinsics {
            fx: 2.0,
            fy: 2.0,
            cx: 2.0,
            cy: 2.0,
        };
        let body = encode_depth_16u("f", 4, 4, &[0u16; 16]);
        assert!(
            parse_depth_image(&body[..body.len() - 10], &k, &DepthConfig::default(), None).is_err()
        );
    }

    #[test]
    fn unknown_encoding_is_rejected() {
        let k = Intrinsics {
            fx: 1.0,
            fy: 1.0,
            cx: 0.0,
            cy: 0.0,
        };
        let mut body = header("f");
        body.extend_from_slice(&1u32.to_le_bytes());
        body.extend_from_slice(&1u32.to_le_bytes());
        body.extend_from_slice(&4u32.to_le_bytes());
        body.extend_from_slice(b"rgb8");
        body.push(0);
        body.extend_from_slice(&3u32.to_le_bytes());
        body.extend_from_slice(&3u32.to_le_bytes());
        body.extend_from_slice(&[0, 0, 0]);
        assert!(parse_depth_image(&body, &k, &DepthConfig::default(), None).is_err());
    }
}
