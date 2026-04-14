# Wiring

This directory describes how the onboard electronics connect to the MTT-154 ECU so that a CommandBus produced by the drive code ends up as motion on the vehicle. The two supported configurations are single-drive articulated mode and tank mode with two coupled MTTs, and each has its own wiring document.

The diagrams here inform purchasing decisions, harness length, connector selection, and the internal layout of the sensor/compute enclosure that sits on the instrument sled. Keep them in sync with the parts spreadsheet at `../../master_parts_list.xlsx`.

See [`single_track.md`](single_track.md) for the single-drive articulated configuration and [`tank.md`](tank.md) for the two-MTT tank configuration.
