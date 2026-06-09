//! Decoding `sensor_msgs/Imu` from ROS1 wire bytes.
//!
//! ROS1 serialization is little-endian and tightly packed (no alignment padding); strings
//! and arrays are length-prefixed. The `sensor_msgs/Imu` layout:
//!
//! ```text
//! std_msgs/Header header        # uint32 seq; uint32 stamp.sec; uint32 stamp.nsec;
//!                               # uint32 frame_id_len; frame_id bytes
//! geometry_msgs/Quaternion orientation                # 4 × f64  (x, y, z, w)
//! float64[9] orientation_covariance                   # 9 × f64
//! geometry_msgs/Vector3 angular_velocity              # 3 × f64  (x, y, z)
//! float64[9] angular_velocity_covariance              # 9 × f64
//! geometry_msgs/Vector3 linear_acceleration           # 3 × f64  (x, y, z)
//! float64[9] linear_acceleration_covariance           # 9 × f64  (unused; not read)
//! ```
//!
//! We read the header stamp (sensor time), gyro, and accelerometer; covariances and
//! orientation are skipped. The accelerometer reports specific force (gravity included),
//! matching [`slam_types::sensor::ImuSample`].

use slam_types::{ImuSample, Stamp, Vec3};

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
            .ok_or(BagError::ImuDecode("length overflow"))?;
        let slice = self
            .data
            .get(self.pos..end)
            .ok_or(BagError::ImuDecode("message truncated"))?;
        self.pos = end;
        Ok(slice)
    }

    fn skip(&mut self, n: usize) -> Result<(), BagError> {
        self.take(n).map(|_| ())
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
}

/// Decode one `sensor_msgs/Imu` message body into an [`ImuSample`].
pub fn parse_imu(data: &[u8]) -> Result<ImuSample, BagError> {
    let mut c = LeCursor::new(data);

    // std_msgs/Header
    let _seq = c.u32()?;
    let secs = c.u32()?;
    let nsecs = c.u32()?;
    let frame_id_len = c.u32()? as usize;
    c.skip(frame_id_len)?;

    // orientation (4×f64) + orientation_covariance (9×f64) — skipped.
    c.skip((4 + 9) * 8)?;

    let gyro = Vec3::new(c.f64()?, c.f64()?, c.f64()?);
    c.skip(9 * 8)?; // angular_velocity_covariance

    let accel = Vec3::new(c.f64()?, c.f64()?, c.f64()?);
    // trailing linear_acceleration_covariance is not needed.

    let stamp = Stamp::from_nanos(secs as i64 * 1_000_000_000 + nsecs as i64);
    Ok(ImuSample::new(stamp, gyro, accel))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal valid Imu body with the given fields and `frame_id`.
    fn encode_imu(
        secs: u32,
        nsecs: u32,
        frame_id: &str,
        gyro: [f64; 3],
        accel: [f64; 3],
    ) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&0u32.to_le_bytes()); // seq
        v.extend_from_slice(&secs.to_le_bytes());
        v.extend_from_slice(&nsecs.to_le_bytes());
        v.extend_from_slice(&(frame_id.len() as u32).to_le_bytes());
        v.extend_from_slice(frame_id.as_bytes());
        // orientation (x,y,z,w) then orientation_covariance[9]
        for x in [0.0f64, 0.0, 0.0, 1.0] {
            v.extend_from_slice(&x.to_le_bytes());
        }
        for _ in 0..9 {
            v.extend_from_slice(&0.0f64.to_le_bytes());
        }
        for x in gyro {
            v.extend_from_slice(&x.to_le_bytes());
        }
        for _ in 0..9 {
            v.extend_from_slice(&0.0f64.to_le_bytes());
        }
        for x in accel {
            v.extend_from_slice(&x.to_le_bytes());
        }
        for _ in 0..9 {
            v.extend_from_slice(&0.0f64.to_le_bytes());
        }
        v
    }

    #[test]
    fn decodes_a_well_formed_message() {
        let body = encode_imu(
            1560000083,
            920771360,
            "imu_link",
            [0.1, -0.2, 0.3],
            [8.1, -0.3, 4.5],
        );
        let s = parse_imu(&body).unwrap();
        assert_eq!(s.stamp.as_nanos(), 1_560_000_083_920_771_360);
        assert_eq!(s.gyro, Vec3::new(0.1, -0.2, 0.3));
        assert_eq!(s.accel, Vec3::new(8.1, -0.3, 4.5));
    }

    #[test]
    fn handles_empty_frame_id() {
        let body = encode_imu(1, 2, "", [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]);
        let s = parse_imu(&body).unwrap();
        assert_eq!(s.stamp.as_nanos(), 1_000_000_002);
        assert_eq!(s.accel, Vec3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn rejects_truncated_message() {
        // Cut into the read region (header/orientation/gyro), not just the unused trailing
        // covariance the parser skips.
        let body = encode_imu(1, 2, "x", [0.0; 3], [0.0; 3]);
        let err = parse_imu(&body[..body.len() / 2]).unwrap_err();
        assert!(matches!(err, BagError::ImuDecode(_)));
    }

    #[test]
    fn rejects_bogus_frame_id_length() {
        // seq, secs, nsecs, then a frame_id length far beyond the buffer.
        let mut body = Vec::new();
        body.extend_from_slice(&0u32.to_le_bytes());
        body.extend_from_slice(&1u32.to_le_bytes());
        body.extend_from_slice(&2u32.to_le_bytes());
        body.extend_from_slice(&9_999u32.to_le_bytes());
        let err = parse_imu(&body).unwrap_err();
        assert!(matches!(err, BagError::ImuDecode(_)));
    }
}
