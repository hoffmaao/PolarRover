# Electrical

The electrical subsystem encompasses every componenet between the rover battery, the motor controls, rover computer, vision system, GNSS, and data loggers. The MTT-154 supplies the drive battery and runs its own motor controllers, so this documentation focuses on how the compute and sensor payload tap power from the drive system, how the Jetson talks to the MTT ECU over CAN, how the two rovers communicate with each other, and how the harness is laid out so the instrument sled can be unplugged as a unit.

## Power

The MTT-154 carries three hot-swappable lithium-ion batteries that drive the rover motors. An auxiliary tap breaks out from the battery bus through an inline fuse and relay into a wide-input DC-DC converter, which steps the pack voltage down to 5 V for the Jetson Orin Nano and the supporting sensors. The auxiliary rail is isolated from the motor bus on the MTT side, so a compute fault cannot take down the drive.

## Drive data

The Jetson sends commands to the MTT ECU over CAN at ≥5 Hz using a single 8-byte frame. The physical bus runs through a USB-CAN adapter into a Deutsch DT 6-pin connector on the MTT auxiliary port, with 120 Ω terminators at each end. In tank mode each MTT has its own CAN bus, so the Jetson carries two USB-CAN adapters and two independent harnesses. The full wire-level layout for each configuration lives in [`wiring/`](wiring/).

## Drive-system sensors

The drive-system sensors all land on the Jetson over USB. The u-blox ZED-F9P terminates the antenna feed and holds the RTK fix whenever corrections are flowing, providing the absolute position that the state estimator needs. The OAK-D Pro stereo camera, the D500 scanning LiDAR, and the pan-tilt wide-angle camera will eventually support local obstacle awareness to the autonomy code. An OpenLog Artemis on a separate UART gives us redundant telemetry logging that survives a Jetson crash. The GPR is not a drive-system sensor; it lives on the science payload path documented in [Instruments](../Instruments/README.md).

## Radio link

In linked-CMP missions the two rovers coordinate over WiFi anchored by a GL.iNet GL-AXT1800 base station. The link carries formation state between the pair during a spread, and streams differential GNSS corrections from the base receiver to both rovers throughout the survey. The network is deliberately simple. The operator stays at the base and the rovers operate disconnected between file exchanges.

## Parts

The full electrical bill of materials lives in [`../master_parts_list.xlsx`](../master_parts_list.xlsx) under the "Electrical system" section. Each wiring document cross-references parts by their spreadsheet code (E1, E2, …).
