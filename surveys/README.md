# Surveys

A survey is a pre-authored GeoJSON file that tells the rover where to go and what kind of work to do when it gets there. Each survey declares its kind (waypoint, multipass, CMP, or calibration) in its `survey_kind` property, and the drive code uses that kind to pick the right controller at runtime.

Surveys live in subfolders by kind so that an operator browsing the filesystem can see what's available at a glance. The folder is only a convenience for humans, though — the runner keys off the `survey_kind` property inside the file, not the folder path, so a file in the wrong subfolder still runs correctly with a warning.

## Layout

```
surveys/
  waypoint/          # goal-seeking navigation to sparse waypoints
  multipass/         # dense reference tracks for repeat radar surveys
  cmp/               # two-rover Common Midpoint spreads
  calibration/       # optional rosette or steer-response calibrations
```

## Coordinate system

All coordinates are EPSG:3031 (Antarctic Polar Stereographic). The drive code assumes every survey is projected to this CRS. Authoring surveys in WGS84 lat/lon is supported through the `rover-sim-startup` web UI, which projects to EPSG:3031 on export, but files sitting in this directory should already be projected.

## File format

Each survey is a GeoJSON FeatureCollection. The first feature describes the route itself — a `LineString` for waypoint/multipass or a `Point` with a direction LineString for CMP — and carries the survey metadata in its `properties`:

- `survey_kind` — one of `waypoint_survey`, `multipass_survey`, `cmp_survey`, `calibration_survey`
- `survey_version` — integer; bump when the file format for a kind changes in an incompatible way
- `epsg` — coordinate system declaration (should match the FeatureCollection's `crs`)
- `name`, `description` — free-text labels for the operator
- `params` — kind-specific configuration (start/end spread for CMP, tolerances for waypoint, etc.)

Worked examples live in the three subfolders.

## How the rover selects a survey

On the field rover, the startup flow looks for a removable card first (a USB stick or SD card with a `surveys/` tree on it) and falls back to the local surveys folder if no card is present. Whichever source is chosen, the operator either picks a file from a console menu or points at one explicitly with an `active.txt` selector. The runner loads it, reads `survey_kind`, constructs the matching driver, and transitions the rover from its default teleop state into the autonomous mode for that kind.

The simulator reads from the same directory — the examples in the subfolders here are the canonical inputs for `rover-sim-emu` scenarios and for the test suite. Keeping the field rover and the simulator pointed at the same structure means a survey authored for a field deployment also runs unchanged in simulation, and a survey iterated in simulation drops straight onto a card for deployment.
