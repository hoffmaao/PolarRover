# Software

The drive software is organized into four packages that separate simulation infrastructure from control logic.

## Packages

`rover_sim` is the foundation layer. It defines the kinematic vehicle models, a simulated GNSS receiver, safety interlocks, mission file loading, and the shared interfaces that the rest of the stack depends on. The primary vehicle model is a bicycle kinematic representation of the MTT-154 pulling an articulated towed accessory. The single free parameter that matters most is the effective arm length from the hitch to the sled's center of resistance — it changes with sled configuration and snow conditions, so we measure it before every mission with a rosette calibration routine.

`rover_drive` contains the autonomous control code. There are five drive modes: teleop for manual or scripted control, waypoint for goal-seeking navigation, multipass for repeat-track radar surveys, linked CMP for two-rover coordinated spreading, and calibration for characterizing the vehicle's steering response. Each mode produces commands through a common `Driver` interface, so the execution backend — whether simulated or real hardware — doesn't need to know which mode is running.

`rover_hardware` bridges the drive code to the MTT-154 CAN bus. It exposes a `HardwareBackend` interface with two concrete implementations: `SingleTrackCANBackend` for a single articulated MTT-154, and `TankCANBackend` for two MTTs coupled side-by-side with differential steering. The confidential 8-byte frame layout lives in a vendor driver outside the public tree; the backend takes a `FrameCoder` by dependency injection, which keeps the bridge logic testable and the wire protocol out of the repository. Wiring diagrams for both configurations live under [`../Electrical/wiring/`](../Electrical/wiring/).

`rover_sim_emulator` is the batch test harness. It steps the vehicle model forward in time, samples the GNSS sensor, calls the active driver, applies the safety filter, and writes JSONL telemetry. A CLI provides `run`, `demo`, and `extract-track` commands.

`rover_sim_startup` is a lightweight FastAPI web application for creating surveys and inspecting telemetry logs. Everything is file-based — no database — because the rover operates disconnected in the field.

`rover_field_boot` is the Jetson-side boot loader. It looks for a survey card at a known mount point, falls back to the local disk surveys folder, lets an operator pick a survey (or reads an `active.txt` selector for unattended operation), and dispatches the matching drive mode. Systemd unit files and an install script live under `rover_field_boot/deploy/`.

`rover_onboard` is the platform layer that sits between the Jetson OS and the autonomy stack. It owns the status broker that connects the drive loop to any operator-facing view of the rover, plus the OLED boot display, the audio/TTS callout layer, the vision pipeline (OAK-D Pro capture and feature segmentation), and a FastAPI web UI that serves live video plus a real-time state panel. Hardware-dependent backends (OLED, TTS, depthai, Flask) are optional extras so the package is still fully importable and testable on a laptop.

## Controllers

Path following is handled by pluggable controllers that all implement the same interface. Pure pursuit is the simplest: it steers toward a lookahead point on the path and is a reasonable default for waypoint navigation. The Stanley controller adds a cross-track correction term that pulls the rover back onto the line, which matters for multipass surveys where path fidelity drives the science. A short-horizon MPC uses the calibrated "bicycle model" to optimize a sequence of steering commands over a prediction window. There is also an MPCC variant for contouring, an LQR for smooth tracking on gentle paths, and several learning-based controllers (GP feedforward, GP-MPC, GP-MPPI, and iterative learning control) that adapt to surface conditions over successive passes.

All of these can be swapped at configuration time. The scenario YAML file specifies which controller to use, and the emulator instantiates it without any code changes.

## State estimation

An Ensemble Kalman Filter with 64 members fuses GNSS observations with the bicycle kinematic model to estimate position, heading, speed, and yaw rate. Observation noise scales with GNSS fix quality, so the filter tightens naturally when RTK is fixed and degrades gracefully during float solutions or dropouts.

## Path planning

Dubins paths compute the shortest feasible route between two oriented poses given the calibrated minimum turn radius. The planner evaluates all six candidate types (LSL, LSR, RSL, RSR, RLR, LRL) and returns the shortest one. This feeds into waypoint approach planning so the rover doesn't attempt turns tighter than it can physically make.

## Installation

```bash
make dev    # editable install of all packages with dev extras
make test   # pytest across all packages
make clean  # remove build artifacts
```

## Deployment on the MTT Jetson

`rover_field_boot/deploy/install.sh` installs the whole stack under `/opt/nisse/` on a Jetson, creates a `nisse` system user, drops the two systemd units into place, and enables them. The boot sequence after install is:

1. `rover-onboard.service` starts after network-online, brings up the status broker, OLED display, audio callouts, and the web UI on port 5000.
2. `rover-field-boot.service` starts after `rover-onboard`, looks for a survey card at `/media/nisse/NISSE/surveys/active.txt`, falls back to `/opt/nisse/surveys/` on disk, and either dispatches the selected survey or exits cleanly so the rover sits in teleop-ready state.

The web UI is the operator's primary interface in the field — an operator joins the rover's WiFi AP fallback, opens `http://<rover-ip>:5000/`, and sees a live OAK-D feed alongside a real-time rover-state panel driven by the StatusBroker.
