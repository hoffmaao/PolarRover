// Visual assembly — centered enclosure on the base plate, forward-
// facing cameras, internal components visible, and a stubbed cable
// harness running to the MTT auxiliary port. For preview only.
//
// Coordinate convention: +X is forward (toward the front of the rover).
// The front face of the box (with the OAK-D camera) points in +X and
// the rear cable wall points in -X.

include <params.scad>
use <brain_box.scad>
use <lid.scad>
use <mount_mtt_sled.scad>
use <mount_bench.scad>
use <components.scad>

// ---- preview knobs ----

// Base plate variant: "sled" or "bench".
base_variant = "sled";

// Exploded-view Z offset. Set to 0 for a closed assembly, raise to
// see the Jetson and the component shelf.
explode_z = 40;

// Show the MTT ECU context and the cable harness running to it.
show_mtt_context = true;

// ---- plate + enclosure + lid ----

plate_w_sled = max(ext_w, 180 + 40);    // matches mount_mtt_sled plate_w logic
plate_w_bench = ext_w + 40;

plate_w_actual = (base_variant == "sled") ? plate_w_sled : plate_w_bench;
plate_d_actual = (base_variant == "sled") ? max(ext_d, 120 + 40) : ext_d + 40;
plate_t_actual = 6.0;

// Center the enclosure on the plate in X and Y.
box_x = (plate_w_actual - ext_w) / 2;
box_y = (plate_d_actual - ext_d) / 2;

module assembly() {
    // base plate at z = 0
    color("lightgray")
        if (base_variant == "sled")
            mount_mtt_sled();
        else
            mount_bench();

    // brain box centered on the plate
    translate([box_x, box_y, plate_t_actual]) {
        color("steelblue", 0.35)
            brain_box();

        // Jetson on its standoffs
        translate([wall_t + bay_margin,
                   wall_t + bay_margin,
                   floor_t + standoff_h])
            jetson_stub();

        // auxiliary component shelf (ZED-F9P, OpenLog, DC-DC, USB-CAN)
        translate([wall_t + bay_margin + jetson_w + bay_margin,
                   wall_t + bay_margin,
                   floor_t + shelf_h])
            component_shelf();

        // OAK-D Pro mounted externally on the front wall
        translate([ext_w, wall_t + bay_d/2 - 48, floor_t + bay_h/2 - 18])
            oak_d_pro_stub();

        // Lid with pan-tilt and LiDAR mounted above
        translate([0, 0, ext_h + explode_z]) {
            color("gray", 0.5)
                lid();

            // pan-tilt head near the front of the lid
            translate([ext_w - pantilt_offset_from_front, ext_d/2, lid_t])
                pantilt_stub();

            // LiDAR sitting on its pedestal at the rear of the lid
            translate([lidar_offset_from_rear, ext_d/2, lid_t + lidar_pedestal_h])
                lidar_stub();
        }

        // Deutsch plug sticking out the back
        translate([-10, wall_t + bay_d/2, floor_t + bay_h/2])
            deutsch_plug_stub();
    }

    // MTT context — stylized ECU a short distance behind the rover
    if (show_mtt_context) {
        translate([-500, -20, -50])
            mtt_ecu_stub();

        // Cable harness from the enclosure back wall down to the ECU
        harness_path();
    }
}

module component_shelf() {
    // Lay out the auxiliary boards in the front half of the bay.
    // Positions are approximate — real layout depends on connector
    // orientations and cable lengths in hardware.
    translate([0, 0, 0])
        dcdc_stub();
    translate([0, dcdc_d + 5, 0])
        openlog_stub();
    translate([dcdc_w + 5, 0, 0])
        uscan_stub();
    // ZED-F9P stands vertically against the rear cable wall so its
    // SMA jack aligns with the SMA bulkhead. Show it separately.
    translate([-zed_w - 5, 5, -shelf_h])
        rotate([0, 0, 0])
            zed_f9p_stub();
}

module harness_path() {
    // Simple approximated path: straight out the rear of the
    // enclosure, then angled down and back toward the ECU stub.
    start_x = box_x - 10;
    start_y = box_y + wall_t + bay_d/2;
    start_z = plate_t_actual + floor_t + bay_h/2;

    // first segment out of the bulkhead
    translate([start_x - 60, start_y, start_z])
        cable_bundle(60, 12, "black");

    // long return run toward the ECU
    hull() {
        translate([start_x - 60, start_y, start_z])
            sphere(d = 12);
        translate([-340, -20 + 160, -24])
            sphere(d = 12);
    }
}

assembly();
