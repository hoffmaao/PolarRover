# Testing

Before a rover leaves for the field, it needs to pass several operational tests. This document is the test director for that test procuedure. It names the tests, says what each one retires, and says what it unblocks. Detailed procedures can be written later once the hardware is in hand and we know what "good" looks like at each stage.

The whole campaign presumes that the software stack already passes its test suite in simulation. 

## Stages

The tests group into four logistical stages. The workbench stage needs only a Jetson, cables, and parts on a desk. The shop-floor stage needs access to an MTT-154 with wheels blocked or lifted. The outdoor-ground stage needs an open flat area (parking lot, hangar apron, or flat snow). The environmental stage needs a cold chamber. Within and across stages, the dependency graph runs mostly in a line — the one place parallel work is possible is the vision stack, which can come up on a separate desk while the CAN side is being proved out.

## T1 — CAN adapter brings up cleanly

*Workbench.* The first thing to confirm is that the compute stack can talk to the CAN bus. Setup is one Jetson Orin Nano, the WWZMDiB USB-CAN adapter, a short twisted pair with 120 Ω terminators at each end, and a second CAN node (another adapter on a laptop) to act as a listener. The pass criterion is round-trip frames between the two nodes with no errors in `ip -details link show slcan0`, and `python-can` opening the bus by name. If this test fails, nothing downstream matters — every hardware backend depends on the adapter enumerating cleanly under socketcan. Full procedure in [`t1_can_adapter.md`](t1_can_adapter.md).

## T2 — MTT-154 accepts a crawl command

*Shop floor.* One MTT-154 on a dev stand with the tread taken off, the auxiliary port pigtailed to a Deutsch DT 6-pin, CAN into the WWZMDiB adapter, and the Jetson powered through the fuse-and-DC-DC chain. Running `SingleTrackCANBackend` with the vendor coder, we send a 0.05-throttle forward command at 5 Hz and watch what happens. The pass criterion is the drive motor turning at the commanded direction, no ECU fault, and feedback frames parsing into sane `VehicleFeedback` values. This retires the whole question of whether our wire format matches what the MTT actually expects and unblocks every hardware-side test that follows. Full procedure in [`t2_mtt_crawl.md`](t2_mtt_crawl.md).

## T3 — full command envelope on a single MTT

*Shop floor.* Once T2 passes, we setup the joystick. Setup is the same as T2. We walk through a slow throttle sweep, a brake test, the articulation steer range in both direction modes (open-loop cylinder rate and closed-loop target position), a direction change through the F/R interlock, a neutral command, and the machine and handle e-stops. Each command is held long enough to see the physical response and then released. The pass criterion is that every command produces the expected motion, the interlocks behave the way the safety filter expects, and no flags stay stuck in feedback after release. This unblocks rosette calibration and the autonomous drive modes.

## T4 — rosette calibration on the real vehicle

*Outdoor ground.* Without real calibration on the actual vehicle and sled, the controllers are tuned for a model that doesn't match the hardware they're driving. The MTT starts on flat ground, sled attached in its mission configuration, one GNSS receiver with a valid RTK fix, and the `CalibrationDriver` running. The driver spins the rover through a rosette of steering inputs and logs the resulting GNSS track. The pass criterion is that the fitted arm length, maximum gamma, and steer-to-curvature response are stable across a repeat run of the same routine. The output is the calibration file that every downstream controller and the EnKF depend on.

## T5 — GNSS and EnKF state tracking

*Outdoor ground.* This test confirms that the state estimator converges on a real vehicle. Setup: calibrated MTT, ZED-F9P on the rover with the mag-mount antenna, base station on a surveyed point broadcasting corrections, and EnKF running on the Jetson. We drive the rover manually through a figure-eight for a few minutes. Pass criterion is that the fused track follows the GNSS fixes within the receiver's advertised horizontal accuracy and that heading converges within a few seconds of motion. If this fails, every autonomous mode downstream is flying blind.

## T6 — short autonomous waypoint mission

*Outdoor ground.* Now we string everything together on one vehicle. Setup: calibrated MTT, RTK-fixed GNSS, EnKF already converged from a warm-up lap, and a small waypoint mission (four waypoints, fifty-meter legs, well clear of obstacles and people). We launch `WaypointDriver` and walk alongside the rover with a hand on the e-stop. Pass criterion is mission completion without intervention, each waypoint reached within its tolerance, and a telemetry log that looks like the simulated version of the same mission. This is the first test where the whole autonomy stack is on the line at once.

## T7 — tank-mode dual-CAN fan-out

*Shop floor, then outdoor ground.* If tank mode is in scope for this field season, it gets its own mini-campaign. Setup: two MTTs coupled mechanically, two independent CAN buses, two adapters on the Jetson, `TankCANBackend` wrapping two `SingleTrackCANBackend` instances. The crawl test from T2 and the envelope test from T3 repeat on each side. Then we exercise the differential mixing: straight forward with both moving together, a pure pivot, and a cornered throttle. Pass criterion is that commanded center velocity and yaw rate match observation within the calibration tolerance, and that no ECU sees a frame intended for the other side.

## T8 — vision stack

*Workbench (parallel path).* While the drive-side work is in progress on another bench, we bring the vision sensors up on their own. Setup: Jetson, OAK-D Pro, D500 LiDAR, and the pan-tilt camera all connected over USB. Pass criterion is that each sensor produces data at its expected rate, the OAK-D's onboard classifier returns object detections, the LiDAR scan covers the full 360°, and the pan-tilt responds to commanded angles. No fusion with the drive stack is expected yet.

## T9 — linked CMP over WiFi

*Outdoor ground.* This is the last bench-level test before an actual mission. Setup: two calibrated rovers on hard ground with the GL-AXT1800 between them, a slow CMP formation with a one-meter spread, and formation metrics monitored from the base station. Pass criterion is that the spread grows at the commanded rate, the midpoint stays stationary in the formation frame, and the communication link holds without the formation controller tripping any of its loss-of-link modes.

## T10 — cold chamber dry run

*Environmental.* Before a first cold field test, the enclosure with the compute and sensors inside spends a night in a cold chamber at -20 °C and then boots in place. Pass criterion is clean startup with nothing strange in the logs, RTK fix acquired within a few minutes of cold boot, and thermal equilibrium inside the enclosure settling above the Jetson's minimum operating temperature once the compute is under load. A fail here means either active heating is required or the enclosure design changes.

## What the campaign produces

At the end of the chain we have a calibrated MTT (or a pair), a characterized vision stack, a verified code path from `CommandBus` to the ECU, a validated state estimator, and a set of telemetry logs for every test. The first field trip is then a replay of T5 through T9 on a real survey site with the science payload added, rather than a fight to get the system to move at all.
