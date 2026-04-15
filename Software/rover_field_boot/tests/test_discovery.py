"""Tests for survey discovery — card detection, folder walks, and merge."""

import json
from pathlib import Path

import pytest

from rover_field_boot.discovery import (
    SURVEY_KINDS,
    SurveySource,
    discover_surveys,
    find_card,
    load_surveys,
    merge_sources,
)


def _write_survey(path: Path, kind: str, name: str, n_points: int = 2) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    coords = [[float(i), 0.0] for i in range(n_points)]
    doc = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": coords},
                "properties": {
                    "name": name,
                    "survey_kind": kind,
                    "survey_version": 1,
                    "description": f"{name} ({kind})",
                },
            }
        ],
    }
    path.write_text(json.dumps(doc))


# ---- discover_surveys ----


def test_discover_returns_empty_buckets_for_missing_root(tmp_path):
    buckets = discover_surveys(tmp_path / "nonexistent", SurveySource.LOCAL)
    for kind in SURVEY_KINDS:
        assert buckets[kind] == []


def test_discover_groups_by_survey_kind(tmp_path):
    root = tmp_path / "surveys"
    _write_survey(root / "waypoint" / "a.geojson", "waypoint_survey", "wp-a", n_points=6)
    _write_survey(root / "cmp" / "b.geojson", "cmp_survey", "cmp-b", n_points=2)
    _write_survey(root / "multipass" / "c.geojson", "multipass_survey", "mp-c", n_points=50)

    buckets = discover_surveys(root, SurveySource.LOCAL)

    assert len(buckets["waypoint_survey"]) == 1
    assert len(buckets["cmp_survey"]) == 1
    assert len(buckets["multipass_survey"]) == 1
    assert buckets["waypoint_survey"][0].name == "wp-a"
    assert buckets["multipass_survey"][0].n_points == 50
    assert buckets["cmp_survey"][0].source is SurveySource.LOCAL


def test_discover_respects_survey_kind_over_folder(tmp_path):
    """A file placed in the 'wrong' folder still ends up in the right kind
    bucket, because discovery keys off the property not the path."""
    root = tmp_path / "surveys"
    _write_survey(root / "waypoint" / "misfiled.geojson", "cmp_survey", "actually-cmp", 2)

    buckets = discover_surveys(root, SurveySource.LOCAL)
    assert len(buckets["waypoint_survey"]) == 0
    assert len(buckets["cmp_survey"]) == 1


def test_discover_skips_files_without_survey_kind(tmp_path, capsys):
    root = tmp_path / "surveys"
    path = root / "waypoint" / "broken.geojson"
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({"type": "FeatureCollection", "features": []}))

    buckets = discover_surveys(root, SurveySource.LOCAL)
    assert all(len(buckets[k]) == 0 for k in SURVEY_KINDS)
    err = capsys.readouterr().err
    assert "no survey_kind" in err


def test_discover_unknown_kind_still_listed(tmp_path):
    root = tmp_path / "surveys"
    _write_survey(root / "custom" / "x.geojson", "experimental_survey", "x", 2)

    buckets = discover_surveys(root, SurveySource.LOCAL)
    assert "experimental_survey" in buckets
    assert buckets["experimental_survey"][0].name == "x"


# ---- find_card ----


def test_find_card_returns_none_when_no_mount_exists(tmp_path):
    assert find_card([tmp_path / "nonexistent"]) is None


def test_find_card_picks_first_match(tmp_path):
    mount = tmp_path / "media" / "rover"
    card = mount / "NISSE"
    (card / "surveys" / "waypoint").mkdir(parents=True)

    result = find_card([mount])
    assert result == card / "surveys"


def test_find_card_ignores_mounts_without_surveys_dir(tmp_path):
    mount = tmp_path / "media" / "rover"
    (mount / "usb_stick").mkdir(parents=True)  # no surveys/ inside
    assert find_card([mount]) is None


def test_find_card_searches_multiple_candidates(tmp_path):
    first = tmp_path / "media" / "a"
    second = tmp_path / "media" / "b"
    (first / "empty").mkdir(parents=True)
    (second / "CARD" / "surveys").mkdir(parents=True)

    result = find_card([first, second])
    assert result == second / "CARD" / "surveys"


# ---- merge_sources ----


def test_merge_with_no_card_returns_local_only(tmp_path):
    local_root = tmp_path / "surveys"
    _write_survey(local_root / "waypoint" / "x.geojson", "waypoint_survey", "x", 2)

    local = discover_surveys(local_root, SurveySource.LOCAL)
    merged = merge_sources(None, local)

    assert merged["waypoint_survey"][0].source is SurveySource.LOCAL


def test_merge_card_surveys_precede_local(tmp_path):
    card_root = tmp_path / "card" / "surveys"
    local_root = tmp_path / "local" / "surveys"
    _write_survey(card_root / "waypoint" / "c.geojson", "waypoint_survey", "card-wp", 2)
    _write_survey(local_root / "waypoint" / "l.geojson", "waypoint_survey", "local-wp", 2)

    card = discover_surveys(card_root, SurveySource.CARD)
    local = discover_surveys(local_root, SurveySource.LOCAL)
    merged = merge_sources(card, local)

    names = [h.name for h in merged["waypoint_survey"]]
    sources = [h.source for h in merged["waypoint_survey"]]
    assert names == ["card-wp", "local-wp"]
    assert sources == [SurveySource.CARD, SurveySource.LOCAL]


# ---- load_surveys (end-to-end) ----


def test_load_surveys_falls_back_to_local_when_no_card(tmp_path):
    local = tmp_path / "surveys"
    _write_survey(local / "waypoint" / "w.geojson", "waypoint_survey", "w", 3)

    buckets, card_root = load_surveys(local, card_candidates=[tmp_path / "no_card_here"])
    assert card_root is None
    assert len(buckets["waypoint_survey"]) == 1
    assert buckets["waypoint_survey"][0].source is SurveySource.LOCAL


def test_load_surveys_uses_card_when_present(tmp_path):
    local = tmp_path / "local" / "surveys"
    _write_survey(local / "waypoint" / "l.geojson", "waypoint_survey", "l", 2)

    mount = tmp_path / "media" / "rover"
    card = mount / "NISSE"
    _write_survey(card / "surveys" / "cmp" / "c.geojson", "cmp_survey", "c", 2)

    buckets, card_root = load_surveys(local, card_candidates=[mount])
    assert card_root == card / "surveys"
    kinds = {h.source for handles in buckets.values() for h in handles}
    assert SurveySource.CARD in kinds
    assert SurveySource.LOCAL in kinds
