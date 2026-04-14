# T1 — CAN adapter brings up cleanly

## Purpose

Confirm that a Jetson Orin Nano can run a CAN bus at all using our chosen USB-CAN adapter, that `python-can` can open the interface by name, and that frames round-trip to a second node without errors. This test is the foundation for every hardware-side test that follows — it's also the cheapest and fastest to run, so there's no reason to skip it.

## Prerequisites

The Jetson is flashed with JetPack and boots into Ubuntu. The `rover_hardware` package and its `can` extras install cleanly in a Python virtual environment (`pip install -e Software/rover_hardware[can]`). The software test suite passes. A companion Linux laptop is available — any Ubuntu machine with a USB port will do, since it only needs to run `candump` and `cansend` during the test.

## Parts

The test uses two WWZMDiB USB-CAN adapters (one for the Jetson, one for the laptop — parts list E4), two 120 Ω terminator resistors (E16), roughly half a meter of twisted-pair wire for the CAN bus, and a short USB cable for each adapter.

## Physical setup

Connect the two USB-CAN adapters to each other with the twisted pair: CAN_H on one adapter to CAN_H on the other, CAN_L to CAN_L, ground to ground. Install a 120 Ω terminator across CAN_H and CAN_L at each end of the twisted pair — not in the middle. Plug one adapter into the Jetson and the other into the laptop. Both will appear as USB serial devices, typically `/dev/ttyACM0`.

## Software setup

The WWZMDiB adapter uses the slcan protocol, so we bridge it to a socketcan interface with `slcand`. On the Jetson:

```bash
sudo apt install can-utils
ls /dev/ttyACM*                            # confirm the adapter enumerated
sudo slcand -o -s5 -t hw -S 3000000 /dev/ttyACM0
sudo ip link set up slcan0
ip -details link show slcan0
```

The `-s5` flag selects 250 kbit/s, which matches the default in `rover_hardware.mtt154.single_track.CANInterfaceConfig.bitrate`. The `-S 3000000` is the UART baud rate between the adapter and the Jetson. Repeat the same steps on the laptop so it also has an `slcan0` interface up.

## Procedure

1. **Confirm both interfaces are up.** Run `ip -details link show slcan0` on each node. State should show UP, with bitrate 250000 and no error counters elevated.

2. **Loopback a raw frame.** On the laptop, start `candump slcan0` in a terminal and leave it running. On the Jetson, run `cansend slcan0 123#DEADBEEFCAFEBABE`. The candump window should print a matching frame within milliseconds. Swap roles — send from the laptop, read on the Jetson — and confirm the other direction.

3. **Open the bus from python-can.** On the Jetson, run a short Python snippet that exercises the same path our backend will:
   ```python
   import can
   bus = can.Bus(interface="socketcan", channel="slcan0", bitrate=250000)
   bus.send(can.Message(arbitration_id=0x123, data=b"\xDE\xAD\xBE\xEF\xCA\xFE\xBA\xBE"))
   print(bus.recv(timeout=1.0))  # read back anything the laptop echoes
   bus.shutdown()
   ```
   Run `candump slcan0` on the laptop as before. The Python send should appear on the laptop candump; the laptop can reply with its own cansend to trigger the `recv` on the Jetson.

4. **Open the bus from `SingleTrackCANBackend`.** Construct the backend with a fake FrameCoder that returns a fixed 8-byte payload, open it, call `send(CommandBus(throttle=0.1))`, and observe the frame on the laptop candump. This exercises our wrapper code without any vendor dependency.

## Pass criteria

The test passes if the raw `cansend`/`candump` round-trip works in both directions with no bus errors, the Python `bus.send` path produces frames the listener observes, and `SingleTrackCANBackend.send` produces frames on the bus that match the fake coder's payload bit-for-bit. Error counters on `ip -details link show slcan0` must stay at zero across the full test.

## Failure modes and troubleshooting

If `slcan0` doesn't come up at all, check that `slcand` is actually running (`ps aux | grep slcand`) and that `/dev/ttyACM0` hasn't been claimed by ModemManager — ModemManager likes to grab ACM devices on boot and needs to be either stopped or told to ignore the adapter via a udev rule.

If the interface comes up but no frames are received, the most common cause is termination: either a missing terminator at one end of the twisted pair, or a terminator in the middle instead of at the ends. CAN is tolerant of short unterminated runs but silently fails on longer ones. Swap the terminators across the ends and retry.

If frames are received but error counters climb, bitrate mismatch is the usual cause — confirm both nodes were brought up with `-s5` (250 kbit/s) and not with some default.

If the Python `bus.send` appears to succeed but nothing arrives on the listener, check that python-can is using the same interface name (`slcan0`, not `can0`) and that no stale `slcand` from a previous boot is holding the TTY open.

## Record

Save the output of `ip -details link show slcan0` from both nodes, the candump log file showing the test frames, and a photo of the physical setup. Store these under `Testing/runs/T01_<date>.md` along with a pass/fail line and the names of the operator and observer.
