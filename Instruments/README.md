# Instruments

The rover carries two distinct kinds of instruments. Some are there to collect scientific data. Others are there to help the rover drive safely across an unstructured polar surface. Scientific insturments are carried in the instrument sled behind the articulated hinge and remain independent of the rover system. GNSS and optical sensors feed the onboard Jetson through USB or serial link.

## Science payload

### Ground-penetrating radar

The rover tows a very high-frequency radar transmitter and receiver to map englacial layering ice internal sturcture along pre-defined survey waypoints.

## Drive system

The drive system is the set of onboard sensors that tell the rover where it is and what's around it — distinct from the mechanical drive of the MTT-154 covered in [Mechanical](../Mechanical/README.md). It combines a precise GNSS for absolute positioning with a vision stack to image local hazard. Nothing under this heading is a scientific data product.

### GNSS

Positioning comes from a pair of u-blox ZED-F9P dual-band RTK receivers, one per rover, plus a third unit acting as the base station that broadcasts differential corrections over the WiFi link. The fix is the absolute-position input to the onboard state estimator, and that estimate is what the Ensemble Kalman Filter and every drive controller downstream depend on. The radar path happens to borrow the same fix stream for timing, but the receiver's primary role is to keep the rover localized while it drives.

### Vision

The cameras and LiDAR on the rover exist to keep rover users and other equipment around the rover safe.

A Waveshare D500 scanning LiDAR gives the rover 360° obstacle awareness out to roughly 40 m. At the speeds we actually drive, that corresponds to a planning horizon of tens of seconds. A 5 MP wide-angle (160°) camera sits on a 2-DOF pan-tilt head, giving the rover a roving eye. It can look further down the planned path than the fixed stereo camera reaches, scan overhead for sky obstructions that would cost the RTK fix, or point sideways to maintain visual contact with the partner rover during a CMP spread. The pan-tilt is commanded directly by the autonomy code — there is no teleoperation.

## Parts

Full bill of materials for the instrument payload lives in [`../master_parts_list.xlsx`](../master_parts_list.xlsx) under the "Instrument Build" section. The GPR is the science payload. The drive-system components are I3 and I4 (GNSS receivers and antennas), I1 (OAK-D Pro), I2 (D500 LiDAR), and I5 (pan-tilt wide-angle camera).
