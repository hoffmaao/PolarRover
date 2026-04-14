# Instruments

The rover carries two distinct kinds of instruments, and it's worth keeping them straight. Some are there to collect scientific data about the ice. Others are there to help the rover know where it is and what's around it so that it can drive itself safely across an unstructured polar surface. Both kinds sit on the instrument sled behind the articulated hitch and feed the onboard Jetson through USB or serial links, but only the first kind produces a data product that goes into a paper.

## Science payload

### Ground-penetrating radar

The rover tows a miniaturized high-frequency radar transmitter and receiver to sound englacial layering and bed reflectivity along pre-loaded survey lines. Each trace is tagged with the GNSS fix at the moment of transmission — borrowed from the drive-system receiver — which lets us convert two-way travel times into depth profiles and register repeat passes against earlier surveys for temporal change detection.

## Drive system

The drive system is the set of onboard sensors that tell the rover where it is and what's around it — distinct from the mechanical drive of the MTT-154 covered in [Mechanical](../Mechanical/README.md). It combines a precise GNSS for absolute positioning with a vision stack for local hazard awareness. Nothing under this heading is a scientific data product; everything here exists to feed the autonomy stack.

### GNSS

Positioning comes from a pair of u-blox ZED-F9P dual-band RTK receivers, one per rover, plus a third unit acting as the base station that broadcasts differential corrections over the WiFi link. The fix is the absolute-position input to the onboard state estimator, and that estimate is what the Ensemble Kalman Filter and every drive controller downstream depend on. The radar path happens to borrow the same fix stream for timing, but the receiver's primary role is to keep the rover localized while it drives.

### Vision

The cameras and LiDAR on the rover exist to keep it from driving into anything — crevasses, sastrugi, meltwater channels, or the other rover during a CMP spread.

The forward-looking OAK-D Pro is a stereo depth camera. Its Myriad X accelerator runs neural-network hazard classification at the sensor, so the Jetson stays free for path following. Close-range terrain structure comes out of the stereo pair and feeds the obstacle layer of the planner.

A Waveshare D500 scanning LiDAR gives the rover 360° obstacle awareness out to roughly 40 m. At the speeds we actually drive, that corresponds to a planning horizon of tens of seconds — plenty of time for the path-follower to reshape its trajectory around anything that appears inside the lookahead window.

A 5 MP wide-angle (160°) camera sits on a 2-DOF pan-tilt head, giving the rover a roving eye. It can look further down the planned path than the fixed stereo camera reaches, scan overhead for sky obstructions that would cost the RTK fix, or point sideways to maintain visual contact with the partner rover during a CMP spread. The pan-tilt is commanded directly by the autonomy code — there is no teleoperation.

## Parts

Full bill of materials for the instrument payload lives in [`../master_parts_list.xlsx`](../master_parts_list.xlsx) under the "Instrument Build" section. The GPR is the science payload. The drive-system components are I3 and I4 (GNSS receivers and antennas), I1 (OAK-D Pro), I2 (D500 LiDAR), and I5 (pan-tilt wide-angle camera).
