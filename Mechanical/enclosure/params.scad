// Shared parameters for the compute/sensor enclosure.
// All dimensions in millimeters. Change numbers here and every file
// that includes this one updates. Run openscad --preview or use the
// GUI to iterate on geometry.
//
// Coordinate convention (matches ROS REP-103):
//   +X = forward (direction of rover travel)
//   +Y = left
//   +Z = up
//
// The enclosure front face (at x = ext_w) carries the external
// camera mount and looks in the +X direction. The cable wall is
// on the back face (at x = 0) so harnesses run rearward toward
// the sled interior, away from the forward sensor field of view.
// The lid top carries the pan-tilt camera near the front and the
// D500 LiDAR pedestal near the rear.

// ---- component footprints (measured from datasheets) ----

// Jetson Orin Nano Developer Kit with the reference carrier board.
// Footprint includes the mounting hole pattern; height includes the
// stock heatsink + fan. Upgrade the active cooler (parts list E21)
// and this grows by ~10 mm.
jetson_w = 100;   // width  (long edge)
jetson_d = 79;    // depth  (short edge)
jetson_h = 40;    // height with stock cooler
jetson_mount_dx = 90.5;   // hole-to-hole span along jetson_w
jetson_mount_dy = 71.0;   // hole-to-hole span along jetson_d
jetson_mount_hole = 2.6;  // M2.5 through

// SparkFun ZED-F9P breakout (Qwiic SMA, not the shield).
zed_w = 50;
zed_d = 50;
zed_h = 12;   // including SMA jack standing up
zed_mount_dx = 42.5;
zed_mount_dy = 42.5;
zed_mount_hole = 3.2;  // M3 through

// SparkFun OpenLog Artemis.
ola_w = 45;
ola_d = 25;
ola_h = 10;

// DC-DC step-down (10-60 V -> 5 V / 5 A, parts list E6). The module
// footprint varies by vendor; this matches the ybbott unit.
dcdc_w = 65;
dcdc_d = 38;
dcdc_h = 18;

// WWZMDiB USB-CAN adapter in its shell.
uscan_w = 80;
uscan_d = 30;
uscan_h = 15;

// ---- internal bay layout ----

// Margin between any component and the nearest wall.
bay_margin = 5;

// Cable-routing channel along one internal wall.
cable_channel_w = 18;

// Jetson sits in the back half of the bay so its USB ports face the
// cable wall. The small components share a shelf along the front.
shelf_h = 20;   // height of the auxiliary-component shelf above the floor

// ---- wall thicknesses ----

wall_t   = 3.0;   // side walls
floor_t  = 3.5;   // bottom of the brain box (before the mounting plate)
lid_t    = 3.5;   // removable top lid
standoff_d = 6.5; // outer diameter of brass-insert standoffs
standoff_h = 5.0; // component sits this high above the floor

// ---- external features ----

// Deutsch DT 6-pin bulkhead. The mating hole is round; the flange
// bolts through four M3 clearance holes around it.
deutsch_bulkhead_d   = 24.0;   // panel hole diameter
deutsch_flange_bolt_pattern = 35.0;  // bolt-to-bolt on the flange
deutsch_flange_bolt_hole    = 3.3;

// SMA bulkhead for the GNSS antenna feed-through.
sma_bulkhead_d = 7.5;   // for M6 thread
sma_flat_w     = 10.0;  // hex flat across for the panel nut

// Service-access USB-C. Sealed with a silicone cap when not in use.
usbc_cutout_w = 14;
usbc_cutout_h = 9;

// Pressure-equalization vent (Gortex membrane). One M5 hole with an
// O-ring seat for the membrane retainer.
vent_d = 5.5;

// ---- front face: external OAK-D Pro stereo camera mount ----

// The OAK-D Pro has its own IP-rated housing and mounts externally
// on a flat pad with four M3 bolts on a 75 x 22 mm rectangle.
// Cable exits the camera and passes through a sealed grommet
// back into the enclosure.
oak_mount_dx = 75.0;   // bolt span along camera width
oak_mount_dy = 22.0;   // bolt span along camera height
oak_mount_hole = 3.3;  // M3 clearance
oak_cable_grommet_d = 10.0;

// ---- top face: pan-tilt (front) and LiDAR pedestal (rear) ----

// Pan-tilt lives near the front of the lid so its swivel range is
// centered on the forward driving direction.
pantilt_cutout = 25;          // square cutout for the servo hub
pantilt_bolt_span = 30;       // M3 bolts in an X pattern
pantilt_offset_from_front = 35;  // center of cutout from front edge

// D500 LiDAR sits on a pedestal at the rear of the lid. The pedestal
// lifts the sensor above the pan-tilt and the ZED-F9P antenna.
lidar_pedestal_h = 30;
lidar_base_d     = 60;        // pedestal outer diameter
lidar_bolt_pcd   = 44;        // LiDAR bolt pitch circle
lidar_bolt_hole  = 3.3;
lidar_offset_from_rear = 50;  // center of pedestal from rear edge

// ---- lid / gasket ----

// Lid captures a compressed EPDM cord gasket in a groove.
gasket_cord_d   = 3.0;
gasket_groove_d = 2.2;   // narrower than cord so it compresses ~25 %
gasket_groove_w = 3.5;

// Lid fasteners: M3 flat-head into brass heat-set inserts.
lid_bolt_hole      = 3.3;   // clearance through the lid
lid_insert_d       = 4.2;   // heat-set insert outer
lid_insert_depth   = 6.0;
lid_bolt_spacing   = 55;    // spacing around the perimeter

// ---- mounting plate interface ----

// The plate has two bolt patterns — one on top for the enclosure,
// one on the bottom for whatever it's mounting to (MTT sled or bench).
// The two patterns must NOT overlap: enclosure pattern fits inside
// the enclosure footprint, sled/bench pattern sits outside it.
//
// These four values are derived from ext_w / ext_d (see bottom of
// file) so the enclosure bolts pass through its floor near the walls.
// plate_bolt_inset sets how far in from the footprint edges the bolts
// sit — large enough to clear the wall but small enough not to
// conflict with internal components.
plate_bolt_inset = 10;  // mm from enclosure outer edge
plate_bolt_hole  = 5.3; // M5 clearance

// ---- derived external dimensions ----

// Internal bay must hold the Jetson plus the component shelf plus
// margins. Compute it once here so the box body, lid, and base
// plate all agree.
bay_w = jetson_w + cable_channel_w + 2 * bay_margin;
bay_d = max(jetson_d, dcdc_d + ola_d + bay_margin) + 2 * bay_margin;
bay_h = jetson_h + 15;   // headroom above the stock cooler

// Full exterior of the box body (floor is separate).
ext_w = bay_w + 2 * wall_t;
ext_d = bay_d + 2 * wall_t;
ext_h = floor_t + bay_h;

// Derived enclosure bolt pattern — sits inside the footprint so the
// bolts pass through the floor under the walls, not the overhang.
plate_bolt_dx = ext_w - 2 * plate_bolt_inset;
plate_bolt_dy = ext_d - 2 * plate_bolt_inset;

// Rendering resolution.
$fa = 2;
$fs = 0.4;
