# Nisse

Nisse in danish/norweigan translates to gnome. Gnomes are helpful magical creatures that can also be mischevious making nisse-en and nisse-to the perfect names for two autonomous rovers we have developed for polar radioglaciolgical science. These systems can be used to collect conventional radar data or linked to enable multistatic surveys.


## Why rovers

Polar environments are some of the most logistically challenging research sites to maintain on Earth. The work is physically demanding, which limits who can participate and how much ground-based teams can accomplish in a season. Hazards and access barriers lead to critical data gaps, reducing the reproducibility and spatial reach of field measurements.

An autonomous platform can reduce these barriers and run pre-loaded surveys overnight, repeat radar aquisitions at centimeter precision, and spread two vehicles symmetrically about a midpoint for CMP shots, experiments that are tedious or dangerous to conduct manually.

## Software

The drive software lives under `Software/` and is split into seven Python packages. `rover_sim` includes kinematic vehicle models, a GNSS sensor simulator, safety interlocks, and survey file handling. `rover_drive` defines the drive modes, and path-following controllers. `rover_hardware` is the CAN bridge that carries CommandBus output to the MTT ECU. `rover_sim_emulator` ties the sim modules together in a batch test harness with telemetry logging. `rover_sim_startup` is a small FastAPI app for authoring surveys and viewing logs offline. `rover_field_boot` is the Jetson boot loader that reads a survey from a removable card or local disk and dispatches the matching drive mode. `rover_onboard` is the platform layer on the Jetson that runs the OLED display, audio callouts, vision pipeline, and the operator web UI that serves live video plus a real-time rover-state panel.

See [Software/README.md](Software/README.md) for details on the control architecture.

## Getting started

```bash
cd Software
make dev    # editable install of all four packages
make test   # run the test suite
```

The emulator CLI can run demo scenarios out of the box:

```bash
rover-sim-emu demo waypoint
rover-sim-emu demo multipass
rover-sim-emu demo cmp
```

Example survey files in `surveys/` show the GeoJSON format for waypoint navigation, repeat-track radar lines, and two-rover CMP spreads, organized into one subfolder per survey kind. All coordinates are in EPSG:3031 (Antarctic Polar Stereographic).

## Hardware

The vehicle is an MTT-154 single-track articulated traction unit with a rate-commanded steering cylinder and a 20 km/h speed cap. Positioning comes from a u-blox ZED-F9P dual-band RTK receiver with an L1/L2 mag-mount antenna and an OpenLog Artemis for redundant logging. The two rovers communicate over a WiFi star topology with a GL.iNet GL-AXT1800 base station.

## Repository layout

`Electrical/`, `Instruments/`, and `Mechanical/` contain subsystem documentation and specifications. `Software/` has the full drive stack. `surveys/` is the shared library of pre-authored surveys used by both the simulator and the field rover. `Simulator/` holds generated telemetry, figures, and rendered videos. `Testing/` has the bench test director — the campaign that takes the system from working in simulation to working on the MTT.

## License

See [LICENSE](LICENSE).
