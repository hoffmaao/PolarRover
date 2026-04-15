# Simulator

The simulator lets us develop, and validate the autonomous drive code for the MTT-154. A scenario file specifies the vehicle configuration, the controller, and the mission; the emulator runs it forward in time with the kinematic model for the rover, a simulated GNSS receiver, and writes a JSONL telemetry log.

The Python code that drives simulation lives under `../Software/` in the `rover_sim`, `rover_drive`, and `rover_sim_emulator` packages. This directory holds the outputs of simulation runs — telemetry logs, figures, and rendered videos. Surveys themselves live at the repo root under `../surveys/`, where the field rover reads them too, so a survey authored for a deployment also runs unchanged in simulation.

## Surveys

Reference [`../surveys/`](../surveys/) for the library of pre-authored surveys used by both the simulator and the field rover. The folder is organized by survey kind (waypoint, multipass, CMP, calibration). The simulator loads these directly via the `survey_path` field in a scenario config.

## Generated outputs

`runs/` holds the JSONL telemetry logs written by the emulator, one line per simulation step with ground truth, estimated state, and the command that was sent. These files are not committed — each run produces its own log. `figures/` and `videos/` hold the plots and MP4 animations rendered from those logs.

## Running a scenario

```bash
rover-sim-emu demo waypoint
rover-sim-emu demo multipass
rover-sim-emu demo cmp
```

Each demo writes a log under `Simulator/runs/` that can be played back through the animation tools in `rover_sim_emulator.viz`.
