"""Tests for the survey selector — active_file and console paths."""

import json
from pathlib import Path

import pytest

from rover_field_boot.discovery import SurveyHandle, SurveySource
from rover_field_boot.selector import pick_survey


def _handle(path: Path, kind: str, name: str, source=SurveySource.LOCAL) -> SurveyHandle:
    return SurveyHandle(
        name=name, kind=kind, path=path, source=source, n_points=3, description=""
    )


# ---- active_file mode ----


def test_active_file_missing_returns_none(tmp_path):
    buckets = {"waypoint_survey": [_handle(tmp_path / "w.geojson", "waypoint_survey", "w")]}
    assert pick_survey(buckets, mode="active_file", active_file=tmp_path / "nope.txt") is None


def test_active_file_empty_returns_none(tmp_path):
    empty = tmp_path / "active.txt"
    empty.write_text("")
    buckets = {"waypoint_survey": [_handle(tmp_path / "w.geojson", "waypoint_survey", "w")]}
    assert pick_survey(buckets, mode="active_file", active_file=empty) is None


def test_active_file_matches_by_name(tmp_path):
    h = _handle(tmp_path / "w.geojson", "waypoint_survey", "ross_ice_01")
    buckets = {"waypoint_survey": [h]}
    active = tmp_path / "active.txt"
    active.write_text("ross_ice_01\n")
    chosen = pick_survey(buckets, mode="active_file", active_file=active)
    assert chosen is h


def test_active_file_matches_by_filename(tmp_path):
    h = _handle(tmp_path / "w.geojson", "waypoint_survey", "any-name")
    buckets = {"waypoint_survey": [h]}
    active = tmp_path / "active.txt"
    active.write_text("w.geojson")
    chosen = pick_survey(buckets, mode="active_file", active_file=active)
    assert chosen is h


def test_active_file_matches_by_full_path(tmp_path):
    h = _handle(tmp_path / "w.geojson", "waypoint_survey", "n")
    buckets = {"waypoint_survey": [h]}
    active = tmp_path / "active.txt"
    active.write_text(str(tmp_path / "w.geojson"))
    chosen = pick_survey(buckets, mode="active_file", active_file=active)
    assert chosen is h


def test_active_file_no_match_returns_none(tmp_path):
    h = _handle(tmp_path / "w.geojson", "waypoint_survey", "present")
    buckets = {"waypoint_survey": [h]}
    active = tmp_path / "active.txt"
    active.write_text("absent")
    assert pick_survey(buckets, mode="active_file", active_file=active) is None


# ---- console mode ----


def test_console_picks_by_index(monkeypatch, tmp_path, capsys):
    a = _handle(tmp_path / "a.geojson", "waypoint_survey", "a", source=SurveySource.CARD)
    b = _handle(tmp_path / "b.geojson", "cmp_survey", "b")
    buckets = {"waypoint_survey": [a], "cmp_survey": [b]}

    monkeypatch.setattr("builtins.input", lambda prompt="": "2")
    chosen = pick_survey(buckets, mode="console")
    assert chosen is b


def test_console_blank_input_cancels(monkeypatch, tmp_path):
    a = _handle(tmp_path / "a.geojson", "waypoint_survey", "a")
    monkeypatch.setattr("builtins.input", lambda prompt="": "")
    assert pick_survey({"waypoint_survey": [a]}, mode="console") is None


def test_console_invalid_input_returns_none(monkeypatch, tmp_path):
    a = _handle(tmp_path / "a.geojson", "waypoint_survey", "a")
    monkeypatch.setattr("builtins.input", lambda prompt="": "nonsense")
    assert pick_survey({"waypoint_survey": [a]}, mode="console") is None


def test_console_empty_bucket_returns_none(monkeypatch):
    assert pick_survey({}, mode="console") is None


# ---- errors ----


def test_unknown_mode_raises():
    with pytest.raises(ValueError):
        pick_survey({}, mode="nonsense")  # type: ignore[arg-type]
