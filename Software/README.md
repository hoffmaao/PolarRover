# Software

The drive software is organized into four packages that separate simulation infrastructure from control logic.

## Packages

`rover_sim` is the foundation layer. It defines the kinematic vehicle models, a simulated GNSS receiver, safety interlocks, mission file loading, and the shared interfaces that the rest of the stack depends on. The primary vehicle model is a bicycle kinematic representation of the MTT-154 pulling an articulated towed accessory. The single free parameter that matters most is the effective arm length from the hitch to the sled's center of resistance — it changes with sled configuration and snow conditions, so we measure it before every mission with a rosette calibration routine.

`rover_drive` contains the autonomous control code. There are five drive modes: teleop for manual or scripted control, waypoint for goal-seeking navigation, multipass for repeat-track radar surveys, linked CMP for two-rover coordinated spreading, and calibration for characterizing the vehicle's steering response. Each mode produces commands through a common `Driver` interface, so the execution backend — whether simulated or real hardware — doesn't need to know which mode is running.

`rover_sim_emulator` is the batch test harness. It steps the vehicle model forward in time, samples the GNSS sensor, calls the active driver, applies the safety filter, and writes JSONL telemetry. A CLI provides `run`, `demo`, and `extract-track` commands.

`rover_sim_startup` is a lightweight FastAPI web application for creating missions and inspecting telemetry logs. Everything is file-based — no database — because the rover operates disconnected in the field.

## Controllers

Path following is handled by pluggable controllers that all implement the same interface. Pure pursuit is the simplest: it steers toward a lookahead point on the path and is a reasonable default for waypoint navigation. The Stanley controller adds a cross-track correction term that pulls the rover back onto the line, which matters for multipass surveys where path fidelity drives the science. A short-horizon MPC uses the calibrated bicycle model to optimize a sequence of steering commands over a prediction window. There is also an MPCC variant for contouring, an LQR for smooth tracking on gentle paths, and several learning-based controllers — GP feedforward, GP-MPC, GP-MPPI, and iterative learning control — that adapt to surface conditions over successive passes.

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
