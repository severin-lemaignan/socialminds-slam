#!/usr/bin/env bash
# Masking A/B follow-up: confound controls (colour-only, person-only class set)
# and the ADR 0015 gated depth bridges (--depth-updates-pose, --reg-band-tolerance).
# Rebuilds slam-replay first (adds the bridge flags; defaults unchanged) and
# re-verifies one A-arm cell is bit-identical before trusting cross-binary deltas.
set -u
cd "$(dirname "$0")/../.."
mkdir -p eval/results/masking-ab
BIN=target/release/slam-replay
DATA=data/openloris
OUT=eval/results/masking-ab

cargo build --release -p slam-replay --features dynamics || exit 1

run_cell() {
  local seq=$1 tag=$2 cfg=$3; shift 3
  local tum="$OUT/$seq.$tag.tum"
  [ -f "$tum" ] && { echo "skip $seq.$tag (done)"; return; }
  local gt="$DATA/groundtruth/per-sequence/$seq/groundtruth.txt"
  echo "=== $seq.$tag"
  /usr/bin/time -v "$BIN" --baseline scan-matching-3d \
    --bag "$DATA/$seq.bag" --config "$cfg" \
    --init-pose-from-tum "$gt" \
    --out "$tum.part" --metrics "$OUT/$seq.$tag.metrics.json" "$@" \
    >"$OUT/$seq.$tag.log" 2>&1
  local rc=$?
  if [ $rc -eq 0 ]; then mv "$tum.part" "$tum"; else echo "FAILED $seq.$tag (rc=$rc)"; fi
}

# Binary-equivalence check: rerun one existing cell under a new tag and diff.
run_cell cafe1-1 depth.rebuilt configs/ablations/depth.yaml
if cmp -s "$OUT/cafe1-1.depth.tum" "$OUT/cafe1-1.depth.rebuilt.tum"; then
  echo "binary-equivalence OK (cafe1-1.depth bit-identical across rebuild)"
else
  echo "WARNING: rebuilt binary changed the unflagged trajectory"
fi

for seq in cafe1-1 cafe1-2; do
  # Confound controls.
  run_cell "$seq" depth-color          configs/ablations/depth-color.yaml
  run_cell "$seq" depth-masked-person  configs/ablations/depth-masked-person.yaml
  run_cell "$seq" odom-depth-masked-person configs/ablations/odom-depth-masked-person.yaml
  # Gated bridge 1: depth->pose with scans present (ROADMAP: 0.16 -> 3.0 unmasked).
  run_cell "$seq" full-dup             configs/openloris-cafe.yaml        --depth-updates-pose
  run_cell "$seq" full-masked-dup      configs/openloris-cafe-masked.yaml --depth-updates-pose
  run_cell "$seq" full-masked-person-dup configs/ablations/full-masked-person.yaml --depth-updates-pose
  # Gated bridge 2: laser-band depth contribution (ROADMAP: 0.164 -> 0.357 unmasked).
  run_cell "$seq" full-band            configs/openloris-cafe.yaml        --reg-band-tolerance 0.15
  run_cell "$seq" full-masked-band     configs/openloris-cafe-masked.yaml --reg-band-tolerance 0.15
done
run_cell market1-1 market-masked-person configs/ablations/market-masked-person.yaml
echo ALL CONTROLS DONE
