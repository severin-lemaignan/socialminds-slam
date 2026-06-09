"""Tests for the cache locators (`harness.fetch.locate_*`) — never touch the network.

They run against a fake cache layout under `tmp_path`, mirroring what the fetchers (plus
the operator's `tar -xf` for OpenLORIS bags) leave behind.
"""

from __future__ import annotations

import pytest

from harness import fetch


def _fake_euroc_cache(root, collection="machine_hall", seq="MH_01_easy"):
    """Lay out a cached+extracted EuRoC collection the way `fetch_euroc` leaves it."""
    euroc = root / "euroc"
    (euroc / f"{collection}.zip").parent.mkdir(parents=True)
    (euroc / f"{collection}.zip").touch()
    (euroc / f"{collection}.zip.done").touch()
    extract_dir = euroc / collection
    mav0 = extract_dir / seq / "mav0"
    (mav0 / "imu0").mkdir(parents=True)
    (extract_dir / ".extracted").touch()
    return mav0


def test_locate_euroc_finds_cached_sequence(tmp_path):
    mav0 = _fake_euroc_cache(tmp_path)
    assert fetch.locate_euroc("MH_01_easy", root=tmp_path) == mav0


def test_locate_euroc_refuses_to_download(tmp_path):
    # Nothing cached → a clear error pointing at the make target, not a silent fetch.
    with pytest.raises(FileNotFoundError, match="make data-euroc"):
        fetch.locate_euroc("MH_01_easy", root=tmp_path)


def test_locate_euroc_unknown_sequence(tmp_path):
    with pytest.raises(KeyError):
        fetch.locate_euroc("does_not_exist", root=tmp_path)


def _fake_openloris_cache(root, seq="cafe1-1"):
    ol = root / "openloris"
    ol.mkdir(parents=True)
    bag = ol / f"{seq}.bag"
    bag.touch()
    gt = ol / "groundtruth" / "per-sequence" / seq / "groundtruth.txt"
    gt.parent.mkdir(parents=True)
    gt.touch()
    return bag, gt


def test_locate_openloris_finds_bag_and_groundtruth(tmp_path):
    bag, gt = _fake_openloris_cache(tmp_path)
    assert fetch.locate_openloris("cafe1-1", root=tmp_path) == (bag, gt)


def test_locate_openloris_missing_bag(tmp_path):
    with pytest.raises(FileNotFoundError, match="make data-openloris SCENE=cafe1"):
        fetch.locate_openloris("cafe1-1", root=tmp_path)


def test_locate_openloris_missing_groundtruth(tmp_path):
    (tmp_path / "openloris").mkdir()
    (tmp_path / "openloris" / "cafe1-1.bag").touch()
    with pytest.raises(FileNotFoundError, match="make data-openloris-gt"):
        fetch.locate_openloris("cafe1-1", root=tmp_path)


def test_gather_sequences_reuses_materialized_openloris(tmp_path, monkeypatch):
    # Extraction is expensive (bz2 bags, two passes), so a previously materialised
    # sequence must be reused. The fake bag is empty: any re-extraction would fail.
    from harness import benchmark

    monkeypatch.setenv("SLAM_DATA_DIR", str(tmp_path))
    _fake_openloris_cache(tmp_path)
    mat = tmp_path / "openloris" / "_materialized" / "cafe1-1"
    mat.mkdir(parents=True)
    (mat / "imu.csv").write_text("1.0 0 0 0 0 0 -9.81\n2.5 0 0 0 0 0 -9.81\n")
    (mat / "groundtruth.tum").write_text("1.0 0 0 0 0 0 0 1\n")

    seqs = benchmark.gather_sequences([], ["cafe1-1"], False, tmp_path / "work")
    assert [s.name for s in seqs] == ["cafe1-1"]
    assert seqs[0].imu_csv == mat / "imu.csv"
    assert seqs[0].duration_s == pytest.approx(1.5)
