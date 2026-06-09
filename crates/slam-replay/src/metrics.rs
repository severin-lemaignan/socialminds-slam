//! Compute metrics for a replay run.
//!
//! Accuracy is scored externally (the `evo`-based harness); this captures the *compute*
//! side that the benchmark report needs (ADR 0005): per-sample processing latency, sustained
//! throughput, and the **real-time factor** (sensor-time span ÷ processing wall-time) — which
//! must stay ≥ 1.0 for online use on the robot.

use std::time::Duration;

/// Processing-side metrics for one run, serialisable to a small JSON document.
#[derive(Debug, Clone)]
pub struct ProcessingMetrics {
    pub system: String,
    pub n_samples: usize,
    pub input_span_s: f64,
    pub processing_wall_s: f64,
    pub throughput_hz: f64,
    pub real_time_factor: f64,
    pub latency_us: LatencyStats,
}

/// Per-sample processing latency, in microseconds.
#[derive(Debug, Clone, Copy, Default)]
pub struct LatencyStats {
    pub p50: f64,
    pub p95: f64,
    pub p99: f64,
    pub max: f64,
    pub mean: f64,
}

impl LatencyStats {
    /// Compute statistics from per-sample latencies (nanoseconds). `percentile` uses the
    /// nearest-rank method on the sorted samples.
    pub fn from_nanos(mut nanos: Vec<u64>) -> LatencyStats {
        if nanos.is_empty() {
            return LatencyStats::default();
        }
        nanos.sort_unstable();
        let n = nanos.len();
        let pct = |p: f64| -> f64 {
            // nearest-rank: rank = ceil(p/100 * n), clamped to [1, n]
            let rank = ((p / 100.0) * n as f64).ceil().max(1.0) as usize;
            nanos[rank.min(n) - 1] as f64 / 1000.0
        };
        let mean = nanos.iter().sum::<u64>() as f64 / n as f64 / 1000.0;
        LatencyStats {
            p50: pct(50.0),
            p95: pct(95.0),
            p99: pct(99.0),
            max: *nanos.last().unwrap() as f64 / 1000.0,
            mean,
        }
    }
}

impl ProcessingMetrics {
    pub fn new(
        system: &str,
        n_samples: usize,
        input_span_s: f64,
        processing_wall: Duration,
        latencies_ns: Vec<u64>,
    ) -> ProcessingMetrics {
        let wall = processing_wall.as_secs_f64();
        ProcessingMetrics {
            system: system.to_string(),
            n_samples,
            input_span_s,
            processing_wall_s: wall,
            throughput_hz: if wall > 0.0 {
                n_samples as f64 / wall
            } else {
                0.0
            },
            real_time_factor: if wall > 0.0 { input_span_s / wall } else { 0.0 },
            latency_us: LatencyStats::from_nanos(latencies_ns),
        }
    }

    /// Serialise to a compact JSON object (no external dependency).
    pub fn to_json(&self) -> String {
        let l = &self.latency_us;
        format!(
            concat!(
                "{{\n",
                "  \"system\": {:?},\n",
                "  \"n_samples\": {},\n",
                "  \"input_span_s\": {:.6},\n",
                "  \"processing_wall_s\": {:.6},\n",
                "  \"throughput_hz\": {:.3},\n",
                "  \"real_time_factor\": {:.3},\n",
                "  \"latency_us\": {{ \"p50\": {:.3}, \"p95\": {:.3}, \"p99\": {:.3}, \"max\": {:.3}, \"mean\": {:.3} }}\n",
                "}}\n"
            ),
            self.system,
            self.n_samples,
            self.input_span_s,
            self.processing_wall_s,
            self.throughput_hz,
            self.real_time_factor,
            l.p50, l.p95, l.p99, l.max, l.mean,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latency_percentiles_nearest_rank() {
        // 1..=100 microseconds (as nanoseconds).
        let nanos: Vec<u64> = (1..=100).map(|x| x * 1000).collect();
        let s = LatencyStats::from_nanos(nanos);
        assert_eq!(s.p50, 50.0);
        assert_eq!(s.p95, 95.0);
        assert_eq!(s.p99, 99.0);
        assert_eq!(s.max, 100.0);
        assert!((s.mean - 50.5).abs() < 1e-9);
    }

    #[test]
    fn empty_is_zeroed() {
        let s = LatencyStats::from_nanos(vec![]);
        assert_eq!(s.p99, 0.0);
        assert_eq!(s.max, 0.0);
    }

    #[test]
    fn real_time_factor_and_throughput() {
        let m = ProcessingMetrics::new("x", 1000, 5.0, Duration::from_millis(10), vec![1000; 1000]);
        // 1000 samples in 0.01 s -> 100 kHz; 5 s of data in 0.01 s -> RTF 500.
        assert!((m.throughput_hz - 100_000.0).abs() < 1.0);
        assert!((m.real_time_factor - 500.0).abs() < 0.1);
    }

    #[test]
    fn json_contains_key_fields() {
        let m = ProcessingMetrics::new("imu", 3, 0.01, Duration::from_micros(50), vec![10, 20, 30]);
        let j = m.to_json();
        assert!(j.contains("\"system\": \"imu\""));
        assert!(j.contains("\"real_time_factor\""));
        assert!(j.contains("\"n_samples\": 3"));
    }
}
