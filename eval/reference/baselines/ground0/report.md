# SLAM benchmark report

| System | Sequence | ATE RMSE (m) | RPE RMSE (m) | Real-time × | Latency p99 (µs) | Peak RSS (MB) |
|---|---|---|---|---|---|---|
| stationary | synthetic (synthetic) | 2.225 ± 0 | nan ± nan | 6.638e+04 ± 3.7e+03 | 0.021 ± 0 | 3.22 ± 0.23 |
| imu_dead_reckoning | synthetic (synthetic) | 0.01429 ± 0 | 0.003843 ± 0 | 5.679e+04 ± 8.7e+03 | 0.04733 ± 0.01 | 3.699 ± 0.0032 |
| stationary | MH_01_easy (euroc) | 5.539 ± 8.9e-16 | nan ± nan | 1.14e+05 ± 6.4e+03 | 0.01467 ± 0.00094 | 7.234 ± 0.048 |
| imu_dead_reckoning | MH_01_easy (euroc) | 6.936e+04 ± 0 | 7.306 ± 8.9e-16 | 6.476e+04 ± 4.2e+03 | 0.055 ± 0.018 | 7.52 ± 0.12 |
| stationary | cafe1-1 (openloris) | 33.27 ± 0 | nan ± nan | 5.01e+04 ± 5.9e+03 | 0.02033 ± 0.0083 | 5.617 ± 0.028 |
| imu_dead_reckoning | cafe1-1 (openloris) | 6419 ± 0 | 11.16 ± 0 | 3.376e+04 ± 2.5e+03 | 0.05433 ± 0.015 | 5.837 ± 0.012 |

