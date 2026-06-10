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
#[derive(Debug, Clone)]
pub struct DepthConfig {
    /// Keep every `stride`-th pixel in u and v (e.g. 8 → ~6 k points from VGA).
    pub stride: usize,
    /// Range clip (m): RealSense depth is noise beyond a few metres.
    pub min_range: f64,
    pub max_range: f64,
}

impl Default for DepthConfig {
    fn default() -> Self {
        DepthConfig {
            // The stride must keep the projected sample spacing *under* the map's
            // voxel size or the integrated surface degenerates into isolated clumps
            // no interpolation stencil can use: stride·z/fx ≤ voxel. At 848-wide
            // RealSense (fx ≈ 420) and 2.5 cm voxels, stride 4 holds to z ≈ 2.6 m
            // and degrades gracefully beyond.
            stride: 4,
            min_range: 0.3,
            max_range: 6.0,
        }
    }
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

    let mut points = Vec::with_capacity((width / cfg.stride + 1) * (height / cfg.stride + 1));
    let mut v = cfg.stride / 2;
    while v < height {
        let row = v * step;
        let mut u = cfg.stride / 2;
        while u < width {
            let z = depth_at(pixels, row + u * bpp);
            if z.is_finite() && z >= cfg.min_range && z <= cfg.max_range {
                points.push(Vec3::new(
                    (u as f64 - intrinsics.cx) / intrinsics.fx * z,
                    (v as f64 - intrinsics.cy) / intrinsics.fy * z,
                    z,
                ));
            }
            u += cfg.stride;
        }
        v += cfg.stride;
    }

    Ok((
        PointCloud {
            stamp,
            frame: slam_types::FrameId::BASE,
            points,
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
    fn depth_backprojects_strided_in_range_pixels() {
        // 4×4 image, fx=fy=2, cx=cy=2; stride 2 keeps pixels (1,1),(3,1),(1,3),(3,3).
        let k = Intrinsics {
            fx: 2.0,
            fy: 2.0,
            cx: 2.0,
            cy: 2.0,
        };
        let cfg = DepthConfig {
            stride: 2,
            min_range: 0.5,
            max_range: 3.0,
        };
        // Row-major depths (mm): pixel (1,1)=1000, (3,1)=4000 (clipped), (1,3)=2000,
        // (3,3)=0 (invalid).
        let mut mm = [0u16; 16];
        mm[4 + 1] = 1000;
        mm[4 + 3] = 4000;
        mm[3 * 4 + 1] = 2000;
        let body = encode_depth_16u("d400_color", 4, 4, &mm);
        let (cloud, frame) = parse_depth_image(&body, &k, &cfg).unwrap();
        assert_eq!(frame, "d400_color");
        assert_eq!(cloud.stamp.as_nanos(), 100_000_000_500);
        assert_eq!(cloud.points.len(), 2);
        // (1,1) at 1 m: ((1−2)/2·1, (1−2)/2·1, 1) = (−0.5, −0.5, 1).
        let p = cloud.points[0];
        assert!(
            (p.x + 0.5).abs() < 1e-12 && (p.y + 0.5).abs() < 1e-12 && (p.z - 1.0).abs() < 1e-12
        );
        // (1,3) at 2 m: ((1−2)/2·2, (3−2)/2·2, 2) = (−1, 1, 2).
        let q = cloud.points[1];
        assert!(
            (q.x + 1.0).abs() < 1e-12 && (q.y - 1.0).abs() < 1e-12 && (q.z - 2.0).abs() < 1e-12
        );
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
        assert!(parse_depth_image(&body[..body.len() - 10], &k, &DepthConfig::default()).is_err());
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
        assert!(parse_depth_image(&body, &k, &DepthConfig::default()).is_err());
    }
}
