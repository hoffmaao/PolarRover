"""Tests for the launcher — survey handle to scenario run, end-to-end."""

from pathlib import Path

import pytest

from rover_field_boot.discovery import SurveyHandle, SurveySource
from rover_field_boot.launcher import launch_survey


def test_waypoint_survey_runs_end_to_end(tmp_path):
    repo_root = Path(__file__).resolve().parents[3]
    survey = repo_root / "surveys" / "waypoint" / "ross_ice_waypoint_01.geojson"
    assert survey.is_file(), f"example survey missing: {survey}"

    handle = SurveyHandle(
        name="ross_ice_waypoint_01",
        kind="waypoint_survey",
        path=survey,
        source=SurveySource.LOCAL,
    )

    log_path = tmp_path / "run.jsonl"
    summary = launch_survey(
        handle,
        duration_s=5.0,
        dt_s=0.1,
        log_path=log_path,
    )

    assert summary["controller"] == "waypoint"
    assert log_path.exists()


def test_unknown_kind_raises():
    handle = SurveyHandle(
        name="x",
        kind="mystery_survey",
        path=Path("/nonexistent.geojson"),
        source=SurveySource.LOCAL,
    )
    with pytest.raises(ValueError, match="no controller mapped"):
        launch_survey(handle)
