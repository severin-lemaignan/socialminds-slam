//! Foundational types shared across the SLAM engine.
//!
//! This crate is deliberately small and dependency-light: time, rigid-body geometry,
//! sensor samples, and trajectories with TUM-format I/O. Everything downstream speaks in
//! these types, so they are the natural place for the zero-copy data structures that will
//! later cross the Python/C++ FFI boundary (see ADR 0001).
//!
//! Conventions:
//! - Time is integer nanoseconds ([`time::Stamp`]); seconds are for I/O only.
//! - Geometry is `f64`; poses are SE(3) mapping *body → reference* frame.
//! - Quaternions are stored/exchanged as `(x, y, z, w)`, matching TUM and ROS.

#![forbid(unsafe_code)]

pub mod geometry;
pub mod imu_csv;
pub mod scan_csv;
pub mod sensor;
pub mod system;
pub mod time;
pub mod trajectory;

pub use geometry::{Pose, Rotation, Vec2, Vec3};
pub use imu_csv::{read_imu, write_imu, ImuCsvError};
pub use scan_csv::{read_scans, write_scans, ScanCsvError};
pub use sensor::{FrameId, ImuSample, LaserScan2D};
pub use system::SlamSystem;
pub use time::{Duration, Stamp};
pub use trajectory::{StampedPose, Trajectory, TumParseError};
