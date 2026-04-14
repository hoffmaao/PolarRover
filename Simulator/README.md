# Simulator

The simulator lets us develop, and validate the autonomous drive code for the MTT-154. A scenario file specifies the vehicle configuration, the controller, and the mission; the emulator runs it forward in time with the kinematic model for the rover, a simulated GNSS receiver, and writes a JSONL telemetry log.

The Python code that drives simulation lives under `../Software/` in the `rover_sim`, `rover_drive`, and `rover_sim_emulator` packages. This directory holds the inputs and outputs.

## Example missions

The `examples/` directory contains mission files in GeoJSON format. Coordinates are in EPSG:3031, the Antarctic Polar Stereographic projection. `waypoint_mission.geojson` is a sparse six-point S-curve. `multipass_reftrack.geojson` is a dense reference track for repeat-pass radar surveys; the corresponding controller minimizes cross-track RMS against it. `cmp_mission.geojson` defines a fixed midpoint and a spreading direction for a two-rover Common Midpoint survey.

## Generated outputs

`runs/` holds the JSONL telemetry logs written by the emulator, one line per simulation step with ground truth, estimated state, and the command that was sent. These files are not committed — each run produces its own log. `figures/` and `videos/` hold the plots and MP4 animations rendered from those logs.

## Running a scenario

```bash
rover-sim-emu demo waypoint
rover-sim-emu demo multipass
rover-sim-emu demo cmp
```

Each demo writes a log under `Simulator/runs/` that can be played back through the animation tools in `rover_sim_emulator.viz`.
