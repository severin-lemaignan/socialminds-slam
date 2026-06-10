//! Decoding `sensor_msgs/LaserScan` from ROS1 wire bytes.
//!
//! ROS1 serialization is little-endian and tightly packed; arrays are u32-length-prefixed.
//! The `sensor_msgs/LaserScan` layout:
//!
//! ```text
//! std_msgs/Header header        # uint32 seq; uint32 stamp.sec; uint32 stamp.nsec;
//!                               # uint32 frame_id_len; frame_id bytes
//! float32 angle_min             # rad, first beam
//! float32 angle_max
//! float32 angle_increment       # rad between beams
//! float32 time_increment        # s between beams (unused)
//! float32 scan_time             # s per revolution (unused)
//! float32 range_min             # m
//! float32 range_max             # m
//! float32[] ranges              # u32 count, then count × f32
//! float32[] intensities         # trailing; not read
//! ```
//!
//! Ranges are kept exactly as recorded (including NaN/inf) — validity filtering belongs
//! to [`slam_types::sensor::LaserScan2D::points`], not ingest.

use slam_types::{LaserScan2D, Stamp};

use crate::BagError;

/// A little-endian byte cursor with bounds checking.
struct LeCursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> LeCursor<'a> {
    fn new(data: &'a [u8]) -> Self {
        LeCursor { data, pos: 0 }
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], BagError> {
        let end = self
            .pos
            .checked_add(n)
            .ok_or(BagError::ScanDecode("length overflow"))?;
        let slice = self
            .data
            .get(self.pos..end)
            .ok_or(BagError::ScanDecode("message truncated"))?;
        self.pos = end;
        Ok(slice)
    }

    fn u32(&mut self) -> Result<u32, BagError> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f32(&mut self) -> Result<f32, BagError> {
        let b = self.take(4)?;
        Ok(f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
}

/// Decode one `sensor_msgs/LaserScan` message body into a [`LaserScan2D`] plus the
/// header's `frame_id` (the URDF link name the beams are expressed in, ADR 0009).
///
/// The returned scan's `frame` is [`slam_types::FrameId::BASE`]; resolving the string
/// against a rig and re-tagging is the caller's job (the reader has no rig).
pub fn parse_scan(data: &[u8]) -> Result<(LaserScan2D, String), BagError> {
    let mut c = LeCursor::new(data);

    // std_msgs/Header
    let _seq = c.u32()?;
    let secs = c.u32()?;
    let nsecs = c.u32()?;
    let frame_id_len = c.u32()? as usize;
    let frame_id = String::from_utf8_lossy(c.take(frame_id_len)?).into_owned();

    let angle_min = c.f32()? as f64;
    let _angle_max = c.f32()?;
    let angle_increment = c.f32()? as f64;
    let _time_increment = c.f32()?;
    let _scan_time = c.f32()?;
    let range_min = c.f32()? as f64;
    let range_max = c.f32()? as f64;

    let count = c.u32()? as usize;
    // Sanity-bound before allocating: each range is 4 bytes and must fit the buffer.
    if count > data.len() / 4 {
        return Err(BagError::ScanDecode("range count exceeds message size"));
    }
    let mut ranges = Vec::with_capacity(count);
    for _ in 0..count {
        ranges.push(c.f32()?);
    }
    // trailing intensities are not needed.

    let stamp = Stamp::from_nanos(secs as i64 * 1_000_000_000 + nsecs as i64);
    Ok((
        LaserScan2D {
            stamp,
            frame: slam_types::FrameId::BASE,
            angle_min,
            angle_increment,
            range_min,
            range_max,
            ranges,
        },
        frame_id,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal valid LaserScan body.
    fn encode_scan(secs: u32, nsecs: u32, frame_id: &str, ranges: &[f32]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&7u32.to_le_bytes()); // seq
        v.extend_from_slice(&secs.to_le_bytes());
        v.extend_from_slice(&nsecs.to_le_bytes());
        v.extend_from_slice(&(frame_id.len() as u32).to_le_bytes());
        v.extend_from_slice(frame_id.as_bytes());
        for x in [-1.5f32, 1.5, 0.01, 0.0001, 0.025, 0.1, 25.0] {
            v.extend_from_slice(&x.to_le_bytes());
        }
        v.extend_from_slice(&(ranges.len() as u32).to_le_bytes());
        for r in ranges {
            v.extend_from_slice(&r.to_le_bytes());
        }
        v.extend_from_slice(&0u32.to_le_bytes()); // empty intensities
        v
    }

    #[test]
    fn decodes_a_well_formed_message() {
        let body = encode_scan(1560000084, 5, "laser", &[1.0, f32::INFINITY, 2.5]);
        let (s, frame_id) = parse_scan(&body).unwrap();
        assert_eq!(frame_id, "laser");
        assert_eq!(s.stamp.as_nanos(), 1_560_000_084_000_000_005);
        assert_eq!(s.ranges.len(), 3);
        assert!(s.ranges[1].is_infinite());
        assert!((s.angle_min - (-1.5)).abs() < 1e-6);
        assert!((s.angle_increment - 0.01).abs() < 1e-6);
        assert!((s.range_max - 25.0).abs() < 1e-6);
    }

    #[test]
    fn rejects_truncated_message() {
        let body = encode_scan(1, 2, "laser", &[1.0, 2.0, 3.0]);
        let err = parse_scan(&body[..body.len() - 20]).unwrap_err();
        assert!(matches!(err, BagError::ScanDecode(_)));
    }

    #[test]
    fn rejects_bogus_range_count() {
        let mut body = encode_scan(1, 2, "", &[]);
        // Overwrite the (now last-8..-4) range count with an absurd value.
        let n = body.len();
        body[n - 8..n - 4].copy_from_slice(&u32::MAX.to_le_bytes());
        let err = parse_scan(&body).unwrap_err();
        assert!(matches!(err, BagError::ScanDecode(_)));
    }
}
