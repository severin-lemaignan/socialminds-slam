"""Guard the committed SotA reference data (eval/reference/sota/) against format rot.

The JSON is hand-extracted from the OpenLORIS-Scene paper (docs/openloris-scene.pdf);
these tests pin its shape and a few spot values so accidental edits are caught.
"""

import json
from pathlib import Path

import pytest

SOTA_JSON = Path(__file__).parent.parent / "reference" / "sota" / "openloris-scene-paper.json"

SCENES = ["office", "corridor", "home", "cafe", "market"]


@pytest.fixture(scope="module")
def sota():
    return json.loads(SOTA_JSON.read_text())


def test_scenes_consistent(sota):
    assert sota["scenes"] == SCENES
    for block in ("per_sequence", "lifelong"):
        for entry in sota[block]["systems"]:
            assert set(entry["cr_pct"]) == set(SCENES), entry["system"]
            assert set(entry["ate_rmse_m"]) == set(SCENES), entry["system"]


def test_value_ranges(sota):
    for block in ("per_sequence", "lifelong"):
        for entry in sota[block]["systems"]:
            for scene in SCENES:
                cr = entry["cr_pct"][scene]
                assert 0.0 <= cr <= 100.0
                ate = entry["ate_rmse_m"][scene]
                # InfiniTAMv2 lifelong market: CR 0% -> no correct poses, ATE undefined
                assert ate is None or ate > 0.0


def test_spot_values(sota):
    """Pin one value per block against the paper (Fig. 2, Fig. 3, Table III)."""
    per_seq = {(s["system"], s["input"]): s for s in sota["per_sequence"]["systems"]}
    vins = per_seq[("VINS-Mono", "fisheye+IMU")]
    assert vins["ate_rmse_m"]["cafe"] == 0.251
    assert vins["cr_pct"]["cafe"] == 95.2

    lifelong = {(s["system"], s["input"]): s for s in sota["lifelong"]["systems"]}
    orb = lifelong[("ORB_SLAM2", "RGB-D")]
    assert orb["cr_pct"]["office"] == 51.6

    reloc = {(s["system"], s["input"]): s for s in sota["relocalization"]["systems"]}
    assert reloc[("DS-SLAM", "RGB-D")]["scores"]["office-1,6"] == 0.994


def test_relocalization_pairs_consistent(sota):
    pairs = set(sota["relocalization"]["pairs"])
    for entry in sota["relocalization"]["systems"]:
        assert set(entry["scores"]) == pairs, entry["system"]
