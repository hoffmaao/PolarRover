# Enclosure

The enclosure houses the Jetson Orin Nano and the boards that surround it — the ZED-F9P GNSS receiver, the OpenLog Artemis, the DC-DC converter, and the USB-CAN adapter — and couples that compute payload to the MTT-154 instrument sled. The design is deliberately modular: a body that holds the electronics, a lid that seals with a cord gasket, and a separate base plate that mates the enclosure to whatever surface it lives on. Replacing the base plate with a different one is the only change needed to run the same enclosure on the MTT sled or on a bench fixture during early testing.

## Files

Every part is parametric OpenSCAD. Dimensions live in [`params.scad`](params.scad) and every other file includes it, so changing a footprint or a wall thickness there propagates everywhere. The [`brain_box.scad`](brain_box.scad) file renders the body with the cable wall on the back (–X) face and the external camera mount on the front (+X) face. [`lid.scad`](lid.scad) renders the lid with the pan-tilt mount near the front, the LiDAR pedestal near the rear, and the gasket groove and fastener pattern on the underside. [`mount_mtt_sled.scad`](mount_mtt_sled.scad) is the base plate for the MTT sled; [`mount_bench.scad`](mount_bench.scad) is the bench-test base plate with a generic M4 grid. [`components.scad`](components.scad) carries non-printable visualizations of the Jetson, sensors, connectors, and the cable harness running to the MTT — these are used only by the assembly preview and never rendered into an STL. [`assembly.scad`](assembly.scad) centers the enclosure on the plate, populates it with the component stubs, mounts the external cameras, and shows the harness routing back to the MTT ECU.

## Rendering

Install OpenSCAD (`sudo apt install openscad` on Ubuntu, or via the OpenSCAD download page for other platforms), open any `.scad` file, and press F5 for a preview or F6 for a full render. To produce an STL for the slicer, use File → Export → STL. From the command line, `openscad -o brain_box.stl brain_box.scad` does the same thing without the GUI.

## Orientation

The enclosure has an explicit "forward" face. Using the ROS REP-103 convention, +X points in the direction of travel, +Y is to port, and +Z is up. The front face of the enclosure (at `x = ext_w`) carries the external OAK-D Pro mount and looks forward along with the camera. The back face (at `x = 0`) carries all the cable-wall bulkheads — Deutsch DT 6-pin for CAN and auxiliary power, an SMA pass-through for the GNSS antenna, and a silicone-capped USB-C service port — so harnesses route rearward toward the sled interior without interfering with the forward sensor field of view. One side wall carries a small Gortex pressure-equalization vent.

## Internal layout

The Jetson Orin Nano lives in the rear half of the bay on four M2.5 standoffs that match the developer kit's hole pattern, with its own USB ports facing the cable wall so the external-cable runs stay short. The front half of the bay carries a shelf on two printed pillars for the DC-DC converter, the OpenLog Artemis, and the WWZMDiB USB-CAN adapter. The ZED-F9P breakout mounts vertically against the rear cable wall with its SMA jack lined up with the SMA bulkhead. An 18 mm cable channel along one internal wall collects harnesses so the lid can close without pinching anything.

## Camera mounting

The OAK-D Pro stereo camera mounts externally on the front face. The wall there is plain and flat with a four-point M3 bolt pattern (75 × 22 mm) matching the OAK-D Pro flange, and a single grommeted pass-through below the bolt pattern for the camera's USB cable. The camera lives outside the enclosure in its own IP-rated housing; only its cable enters the brain box.

The lid carries the other two vision sensors. The pan-tilt wide-angle camera bolts to a 25 mm square cutout near the front of the lid (offset `pantilt_offset_from_front` from the front edge), which places its swivel axis centered on the forward driving direction with full left-right pan. The D500 LiDAR sits on an integral pedestal near the rear of the lid; the pedestal lifts it above the pan-tilt mast so the 360° scan clears every other sensor on the vehicle, and a side slot lets the LiDAR's cable exit and run back through the service port.

## Mounting interface

The base plate is an interface between two different bolt specs: the enclosure on top, and the MTT sled (or bench) on the bottom. Each face has its own four-corner pattern.

The top face carries the enclosure pattern — four M5 holes at `plate_bolt_dx` × `plate_bolt_dy`, derived from the enclosure footprint so the bolts pass through the enclosure floor under the walls. The enclosure is fastened to the plate from above with flat-head M5 bolts that thread into heat-set inserts pressed into the underside of the enclosure floor.

The bottom face carries the sled pattern (or bench-grid pattern for the bench plate). These holes are larger (M6 by default for the MTT sled) and sit farther out on the plate than the enclosure pattern, so the two sets of holes never land near each other at the corners. The plate gets bolted to the sled from below with the heads recessed into countersinks on the underside.

There are two bolt patterns because the enclosure-to-plate and plate-to-sled interfaces are mechanically independent. That independence is the whole reason the plate exists — you can unbolt the enclosure for bench work without disturbing the sled attachment, and you can swap the plate for a different mount without rewiring the enclosure. Eight holes total per plate, four per face.

## Open questions

The MTT sled mounting pattern in `mount_mtt_sled.scad` is currently a placeholder. The three values at the top of that file — `sled_bolt_dx`, `sled_bolt_dy`, `sled_bolt_hole` — need to come from direct measurement of the sled once we have one on a bench. The DC-DC footprint in `params.scad` assumes the ybbott-branded 10–60 V to 5 V unit on the parts list; a different vendor's module would need its own measured footprint. And the lid's pan-tilt cutout is sized for the Waveshare pan-tilt module specifically — if we end up with a different pan-tilt, the cutout updates along with it.

## Print settings

PETG filament (parts list E22) is the baseline because it holds dimensional tolerance better than PLA at the temperature swings we expect and doesn't get brittle at cold bench temperatures. Print at 0.2 mm layer height with 40 % gyroid infill, three perimeters, and a brim to reduce warping on the larger parts. The lid and the sled base plate both prefer to print flat with the internal features facing up; the brain box prints on its base with no supports needed if the bulkhead cutouts are oriented along the print direction.
