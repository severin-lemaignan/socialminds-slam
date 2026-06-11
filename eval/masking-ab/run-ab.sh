#!/usr/bin/env bash
# Masking A/B (ADR 0015): same engine, same flags as harness.sensor_matrix,
# masked vs unmasked configs. Resumable: a cell with its .tum present is skipped.
set -u
cd "$(dirname "$0")/../.."
# repo root (model paths in configs are repo-relative)
mkdir -p eval/results/masking-ab
BIN=target/release/slam-replay
DATA=data/openloris
OUT=eval/results/masking-ab

run_cell() {
  local seq=$1 tag=$2 cfg=$3
  local tum="$OUT/$seq.$tag.tum"
  [ -f "$tum" ] && { echo "skip $seq.$tag (done)"; return; }
  local gt="$DATA/groundtruth/per-sequence/$seq/groundtruth.txt"
  echo "=== $seq.$tag"
  /usr/bin/time -v "$BIN" --baseline scan-matching-3d \
    --bag "$DATA/$seq.bag" --config "$cfg" \
    --init-pose-from-tum "$gt" \
    --out "$tum.part" --metrics "$OUT/$seq.$tag.metrics.json" \
    >"$OUT/$seq.$tag.log" 2>&1
  local rc=$?
  if [ $rc -eq 0 ]; then mv "$tum.part" "$tum"; else echo "FAILED $seq.$tag (rc=$rc)"; fi
}

for seq in cafe1-1 cafe1-2; do
  run_cell "$seq" depth          configs/ablations/depth.yaml
  run_cell "$seq" depth-masked   configs/ablations/depth-masked.yaml
  run_cell "$seq" odom-depth        configs/ablations/odom-depth.yaml
  run_cell "$seq" odom-depth-masked configs/ablations/odom-depth-masked.yaml
  run_cell "$seq" full           configs/openloris-cafe.yaml
  run_cell "$seq" full-masked    configs/openloris-cafe-masked.yaml
done
run_cell market1-1 market        configs/openloris-market.yaml
run_cell market1-1 market-masked configs/openloris-market-masked.yaml
echo ALL DONE
