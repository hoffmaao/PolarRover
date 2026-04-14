# T2 — MTT-154 accepts a crawl command

## Purpose

Confirm that a command frame produced by `SingleTrackCANBackend` using the vendor FrameCoder is something the MTT-154 ECU actually accepts and acts on. This is the single biggest unknown in the current design — everything else in the hardware stack assumes the wire format matches what the vehicle expects, and we haven't yet tested that assumption against a real ECU. A pass here unblocks every subsequent hardware test; a fail sends us back to the vendor coder and the MTT CAN protocol document.

## Prerequisites

T1 must have passed on the same Jetson and adapter. The vendor CAN coder is installed and importable from `rover_drive.vendor.mtt` (or wherever the vendor drivers live). The software test suite passes on the machine in question. An MTT-154 is available with a dev stand — the drive wheels must be either lifted off the ground or chocked so they cannot transmit force to the floor. The MTT-154 owner's manual is on hand for the auxiliary-port pinout and safe-operation checklist.

## Safety

This is the first time our code drives a real motor. Treat it that way. Two people are required for the test: an operator who runs the Jetson and an observer whose only job is the e-stop. Before any command is sent, confirm the mechanical e-stop on the MTT is within arm's reach of the observer and that it's been tested by the observer during this session. No one's hands, feet, or tools may be near the track or articulation cylinder during a command. The test area must be clear of loose cables and obstructions. The first command uses a throttle of 0.05 (roughly 25 cm/s equivalent on the track) and a duration of at most three seconds. Direct battery disconnect is accessible.

## Parts

The test uses everything from T1 plus one MTT-154 on a dev stand, a Deutsch DT 6-pin pigtail (parts list E5) wired to the MTT auxiliary port per the manual, the WWZMDiB adapter and its terminators (E4, E16) tapping the CAN pair, the fuse-and-relay harness (E7), the wide-input DC-DC converter (E6), and a short auxiliary power cable running from the MTT battery tap through the fuse and DC-DC to the Jetson.

## Physical setup

With the MTT powered off, raise or chock the drive wheels and confirm they turn freely. Install the Deutsch DT 6-pin pigtail on the MTT auxiliary port with the pin assignment from the manual — CAN_H, CAN_L, signal ground, switched auxiliary +12 V, and the auxiliary return all land in specific pins and getting this wrong is the fastest way to damage the adapter. Route the CAN twisted pair from the pigtail to the WWZMDiB adapter with a 120 Ω terminator at the adapter end. (The MTT ECU side should already be internally terminated per the vehicle spec — confirm in the manual; if not, install a second terminator on the MTT side of the twisted pair.) Route the switched auxiliary power through the inline fuse, then the relay, then the DC-DC converter, and finally to the Jetson's USB-C power input. Keep the CAN pair away from the power wires to minimize induced noise.

## Software setup

With the Jetson booted, bring up the CAN interface using the same `slcand` procedure from T1 and confirm you can see the ECU's own heartbeat frames arrive on `candump slcan0` — the MTT publishes status messages at several Hz whenever it's powered up, and seeing them is the first positive signal that the wiring is correct. Activate the project's virtual environment and verify that `from rover_drive.vendor.mtt import MTT154FrameCoder` (or whatever the vendor module exposes) imports without error.

## Procedure

1. **Pre-flight.** Confirm wheels lifted or chocked, e-stop within reach of observer, test area clear, observer ready. Confirm that `candump slcan0` is showing ECU heartbeat frames — if not, stop and diagnose before sending anything.

2. **Start the raw log.** In one terminal, run `candump -l slcan0` to write a timestamped log of every frame on the bus. This file is the primary record of the test.

3. **Construct the backend in a script or REPL.** Example:
   ```python
   from rover_hardware import SingleTrackCANBackend
   from rover_hardware.mtt154.single_track import CANInterfaceConfig
   from rover_drive.vendor.mtt import MTT154FrameCoder
   from rover_sim.control.command import CommandBus, Direction

   backend = SingleTrackCANBackend(
       coder=MTT154FrameCoder(),
       config=CANInterfaceConfig(channel="slcan0", bitrate=250_000),
   )
   backend.open()
   ```

4. **Send neutral first.** Send `CommandBus(neutral=True)` at 5 Hz for two seconds. The MTT should stay idle. Confirm no fault frames appear. This proves our command path is reaching the ECU and being accepted without causing motion.

5. **Crawl forward.** With the observer's hand on the e-stop, send `CommandBus(throttle=0.05, direction=Direction.FORWARD)` at 5 Hz for three seconds, then return to `CommandBus(neutral=True)` for two seconds. The drive motor should turn slowly in the forward direction during the crawl window and stop when the neutral command resumes.

6. **Read back feedback.** While the crawl command is active, call `backend.read()` on a loop in a second thread (or poll every 100 ms from the main loop). The returned `VehicleFeedback` should show a non-zero `speed_mps` roughly matching the commanded crawl, a zero or near-zero articulation angle, and no ECU fault flag.

7. **Stop cleanly.** After the three-second crawl window, send neutral commands until the motor has fully stopped. Close the backend: `backend.close()`.

## Pass criteria

The test passes if the drive motor turns in the commanded direction during the crawl window and stops when neutral is commanded, no ECU fault or emergency frames appear in the candump log, and `VehicleFeedback` values read back during the crawl are plausible (non-zero forward speed, no fault flag). A full round-trip from `CommandBus` through the vendor coder to the wire and back into a feedback `VehicleFeedback` is the deliverable.

## Failure modes and troubleshooting

**No motion, no feedback change, no fault.** The frames are probably being sent but with a layout the ECU ignores. Check that the vendor coder produced the expected 8 bytes with `logger.debug` or by printing the payload hex before sending, and compare to the MTT CAN protocol document. It may also indicate that the MTT is in a mode that ignores the auxiliary CAN input — some vehicles require an explicit "autonomous enable" command before they accept motion commands from the bus.

**Fault frame appears immediately.** The coder is producing frames the ECU recognizes but rejects. Look up the fault code in the manual — common causes are out-of-range throttle, reserved bits set nonzero, or a missing checksum byte.

**Motion occurs but in the wrong direction.** The `Direction` encoding in the vendor coder is inverted relative to what the MTT expects. Verify against the protocol document.

**Motion occurs but won't stop.** This is the case we most want to catch early. The observer hits the e-stop immediately. Likely causes are a stuck throttle in the send loop (the script continues commanding crawl rather than falling back to neutral) or a coder bug that encodes neutral with a nonzero throttle byte. Do not re-attempt without reproducing and fixing on a CAN bus with no motor attached.

**Feedback frames never parse.** The coder's `decode_feedback` is either expecting a different arbitration ID than the ECU uses, or the payload structure has changed. Inspect raw bytes in the candump log and compare to the manual.

## Record

Save the `candump -l` log file, the text output of the control-script session (including the `VehicleFeedback` dumps), a short written observation from the observer on whether motion matched what the script commanded, photos of the physical setup, and a video of the crawl window itself. Store these under `Testing/runs/T02_<date>.md` with a pass/fail line and the names of the operator and observer. The candump log is the primary record and should be kept forever — it's the ground truth for the vendor coder.
