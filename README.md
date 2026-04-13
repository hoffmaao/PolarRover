# PolarRover

PolarRover is the software and hardware designs for two autonomous rovers built at LDEO, University of Kansas and Rice University for polar field science. The vehicles are MTT-154 articulated traction units fitted with dual-band GNSS, towed radar sleds, and onboard computing for linked autonomous surveying. These systems can be used to collect conventional radar data while enabling measurements like coordinated Common Midpoint surveys that require linked autonomy and precise positioning.

This project is supported by the OTIC Paros fund.

## Why rovers

Polar environments are some of the most logistically challenging and expensive research sites on Earth. For every dollar NSF spends on Antarctic science, roughly two go to logistics. The work is physically demanding, which limits who can participate and how much ground a team can cover in a season. Hazards and access barriers lead to critical data gaps, reducing the reproducibility and spatial reach of field measurements.

An autonomous platform can reduce these barriers and run pre-loaded surveys overnight, repeat radar aquisitions at centimeter precision, and spread two vehicles symmetrically about a midpoint for CMP shots, experiments that are tedious or dangerous to conduct manually.

## Software

The drive software lives under `Software/` and is split into four Python packages. `rover_sim` includes kinematic vehicle models, a GNSS sensor simulator, safety interlocks, and mission file handling. `rover_drive` builds on this module with autonomous drive modes, path-following controllers, an Ensemble Kalman Filter for state estimation, and Dubins path planning. `rover_sim_emulator` ties these modules together in a batch test harness with telemetry logging and animation. `rover_sim_startup` is a small FastAPI app for authoring missions and viewing logs offline.

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

Example mission files in `examples/` show the GeoJSON format for waypoint surveys, repeat-track radar lines, and two-rover CMP spreads. All coordinates are in EPSG:3031 (Antarctic Polar Stereographic).

## Hardware

The vehicle is an MTT-154 single-track articulated traction unit with a rate-commanded steering cylinder and a 20 km/h speed cap. Positioning comes from a u-blox ZED-F9P dual-band RTK receiver with an L1/L2 mag-mount antenna and an OpenLog Artemis for redundant logging. The two rovers communicate over a WiFi star topology with a GL.iNet GL-AXT1800 base station.

## Repository layout

`Electrical/`, `Instruments/`, and `Mechanical/` contain subsystem documentation and specifications. `Software/` has the full drive stack. `examples/` has sample mission files.

## License

See [LICENSE](LICENSE).
