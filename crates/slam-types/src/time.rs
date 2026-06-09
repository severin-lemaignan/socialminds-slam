//! Time as used throughout the engine.
//!
//! Sensors range from a ~1 kHz IMU to ~20 fps cameras, so we keep time in integer
//! **nanoseconds** to avoid the rounding drift that accumulates with floating-point
//! seconds. [`Stamp`] is a monotonic point in time; [`Duration`] is a signed span.

use std::fmt;

/// A timestamp in integer nanoseconds since an arbitrary but fixed epoch.
///
/// Stored as `i64`: ~292 years of range, and signed so differences are well-defined.
/// Integer nanoseconds are exact, which matters when associating a 1 kHz IMU stream
/// with 20 fps frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Stamp {
    nanos: i64,
}

impl Stamp {
    pub const ZERO: Stamp = Stamp { nanos: 0 };

    #[inline]
    pub const fn from_nanos(nanos: i64) -> Self {
        Stamp { nanos }
    }

    /// Build a stamp from floating-point seconds.
    ///
    /// Convenient, but **lossy for epoch-scale values**: a Unix timestamp like
    /// `1305031910.765238` needs ~19 significant digits in nanoseconds, far beyond
    /// `f64`'s exact-integer range (2⁵³ ≈ 9.0e15 ns ≈ 104 days). Use this only for small
    /// relative times (tests, synthetic data). To ingest dataset timestamps without
    /// rounding error, parse the decimal text with [`Stamp::from_seconds_str`].
    #[inline]
    pub fn from_seconds(seconds: f64) -> Self {
        Stamp {
            nanos: (seconds * 1e9).round() as i64,
        }
    }

    /// Parse a decimal seconds string (e.g. `"1305031910.765238"`) into an exact
    /// nanosecond stamp, without going through `f64`.
    ///
    /// Accepts an optional sign, an integer part, and up to 9 fractional digits (extra
    /// fractional digits are truncated toward zero). This is the path dataset/IMU readers
    /// use so high-rate streams stay exactly associable.
    pub fn from_seconds_str(s: &str) -> Result<Stamp, StampParseError> {
        let s = s.trim();
        if s.is_empty() {
            return Err(StampParseError::Empty);
        }
        let (negative, body) = match s.strip_prefix('-') {
            Some(rest) => (true, rest),
            None => (false, s.strip_prefix('+').unwrap_or(s)),
        };
        let (int_str, frac_str) = match body.split_once('.') {
            Some((i, f)) => (i, f),
            None => (body, ""),
        };
        // Both parts must be pure ASCII digits (reject `1e9`, `1_000`, `.`-only, etc.).
        let valid = |part: &str| part.bytes().all(|b| b.is_ascii_digit());
        if (int_str.is_empty() && frac_str.is_empty()) || !valid(int_str) || !valid(frac_str) {
            return Err(StampParseError::Invalid(s.to_string()));
        }
        let secs: i64 = if int_str.is_empty() {
            0
        } else {
            int_str
                .parse()
                .map_err(|_| StampParseError::Invalid(s.to_string()))?
        };
        // Take the first 9 fractional digits, right-padded with zeros.
        let mut nanos_frac: i64 = 0;
        for i in 0..9 {
            nanos_frac *= 10;
            if let Some(d) = frac_str.as_bytes().get(i) {
                nanos_frac += (d - b'0') as i64;
            }
        }
        let total = secs
            .checked_mul(1_000_000_000)
            .and_then(|n| n.checked_add(nanos_frac))
            .ok_or_else(|| StampParseError::Invalid(s.to_string()))?;
        Ok(Stamp::from_nanos(if negative { -total } else { total }))
    }

    #[inline]
    pub const fn as_nanos(self) -> i64 {
        self.nanos
    }

    /// Seconds as `f64`. Convenient for I/O and metrics; do not use for accumulation.
    #[inline]
    pub fn as_seconds(self) -> f64 {
        self.nanos as f64 * 1e-9
    }

    /// Signed span `self - earlier`.
    #[inline]
    pub fn duration_since(self, earlier: Stamp) -> Duration {
        Duration {
            nanos: self.nanos - earlier.nanos,
        }
    }
}

impl fmt::Display for Stamp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Fixed 9-decimal seconds: round-trips with TUM-style files.
        write!(
            f,
            "{}.{:09}",
            self.nanos.div_euclid(1_000_000_000),
            self.nanos.rem_euclid(1_000_000_000)
        )
    }
}

/// Error from [`Stamp::from_seconds_str`].
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum StampParseError {
    #[error("empty timestamp")]
    Empty,
    #[error("invalid decimal seconds: {0:?}")]
    Invalid(String),
}

/// A signed time span in integer nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Duration {
    nanos: i64,
}

impl Duration {
    pub const ZERO: Duration = Duration { nanos: 0 };

    #[inline]
    pub const fn from_nanos(nanos: i64) -> Self {
        Duration { nanos }
    }

    #[inline]
    pub fn from_seconds(seconds: f64) -> Self {
        Duration {
            nanos: (seconds * 1e9).round() as i64,
        }
    }

    #[inline]
    pub const fn as_nanos(self) -> i64 {
        self.nanos
    }

    #[inline]
    pub fn as_seconds(self) -> f64 {
        self.nanos as f64 * 1e-9
    }

    #[inline]
    pub fn is_positive(self) -> bool {
        self.nanos > 0
    }
}

impl std::ops::Add<Duration> for Stamp {
    type Output = Stamp;
    #[inline]
    fn add(self, rhs: Duration) -> Stamp {
        Stamp {
            nanos: self.nanos + rhs.nanos,
        }
    }
}

impl std::ops::Sub<Stamp> for Stamp {
    type Output = Duration;
    #[inline]
    fn sub(self, rhs: Stamp) -> Duration {
        self.duration_since(rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_seconds_str_is_exact_for_epoch_timestamps() {
        // The f64 path cannot represent this exactly; the string path must.
        let s = Stamp::from_seconds_str("1305031910.765238").unwrap();
        assert_eq!(s.as_nanos(), 1_305_031_910_765_238_000);
    }

    #[test]
    fn from_seconds_str_handles_edge_cases() {
        assert_eq!(Stamp::from_seconds_str("0").unwrap().as_nanos(), 0);
        assert_eq!(
            Stamp::from_seconds_str("42").unwrap().as_nanos(),
            42_000_000_000
        );
        // Fewer than 9 fractional digits are right-padded.
        assert_eq!(
            Stamp::from_seconds_str("1.5").unwrap().as_nanos(),
            1_500_000_000
        );
        // More than 9 are truncated toward zero.
        assert_eq!(
            Stamp::from_seconds_str("1.1234567899").unwrap().as_nanos(),
            1_123_456_789
        );
        assert_eq!(
            Stamp::from_seconds_str("-2.25").unwrap().as_nanos(),
            -2_250_000_000
        );
        assert_eq!(
            Stamp::from_seconds_str("  3.0  ").unwrap().as_nanos(),
            3_000_000_000
        );
    }

    #[test]
    fn from_seconds_str_rejects_garbage() {
        assert!(Stamp::from_seconds_str("").is_err());
        assert!(Stamp::from_seconds_str("1e9").is_err());
        assert!(Stamp::from_seconds_str("1_000").is_err());
        assert!(Stamp::from_seconds_str("abc").is_err());
        assert!(Stamp::from_seconds_str(".").is_err());
    }

    #[test]
    fn duration_since_is_signed() {
        let a = Stamp::from_nanos(1_000);
        let b = Stamp::from_nanos(2_500);
        assert_eq!((b - a).as_nanos(), 1_500);
        assert_eq!((a - b).as_nanos(), -1_500);
    }

    #[test]
    fn add_duration_advances_stamp() {
        let s = Stamp::from_nanos(1_000);
        assert_eq!((s + Duration::from_nanos(500)).as_nanos(), 1_500);
    }

    #[test]
    fn display_pads_nanoseconds() {
        assert_eq!(Stamp::from_nanos(1_000_000_042).to_string(), "1.000000042");
    }
}
