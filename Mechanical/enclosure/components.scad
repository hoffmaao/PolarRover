// Visual-only component stubs for the assembly preview. These are
// NOT printed — they represent the Jetson, sensors, connectors, and
// cables so that the assembly view shows how everything fits and
// routes. Keep them rough — the point is spatial context, not a
// finished rendering.

include <params.scad>

// ---- Jetson Orin Nano Developer Kit ----

module jetson_stub() {
    // Carrier PCB (green) with stock cooler (gray) sitting on top.
    color("darkgreen")
        cube([jetson_w, jetson_d, 2]);
    color("dimgray")
        translate([10, 5, 2])
            cube([jetson_w - 20, jetson_d - 10, jetson_h - 2]);
    // USB / ethernet port block on the rear face of the Jetson
    color("silver")
        translate([-8, 10, 4])
            cube([8, jetson_d - 20, 14]);
}

// ---- small boards ----

module zed_f9p_stub() {
    color("firebrick")
        cube([zed_w, zed_d, 2]);
    // SMA jack on the side
    color("gold")
        translate([-4, zed_d/2 - 3, 2])
            cube([4, 6, 6]);
}

module openlog_stub() {
    color("royalblue")
        cube([ola_w, ola_d, 2]);
}

module dcdc_stub() {
    color("darkorange")
        cube([dcdc_w, dcdc_d, dcdc_h]);
}

module uscan_stub() {
    color("purple")
        cube([uscan_w, uscan_d, uscan_h]);
}

// ---- external sensors ----

module oak_d_pro_stub() {
    // The OAK-D Pro housing is roughly 97 x 37 x 33 mm.
    color("silver")
        cube([8, 97, 37]);      // attached to the front wall
    // Lens barrels
    for (y_off = [20, 77])
        color("black")
            translate([8, y_off, 18])
                rotate([0, 90, 0])
                    cylinder(h = 12, d = 14);
}

module pantilt_stub() {
    // 2-DOF servo head approximated as a small cylinder + camera block.
    color("dimgray")
        cylinder(h = 30, d = 40);
    color("black")
        translate([-20, -5, 25])
            cube([40, 25, 15]);
    // camera lens
    color("darkslategray")
        translate([-20, 3, 33])
            rotate([0, -90, 0])
                cylinder(h = 8, d = 10);
}

module lidar_stub() {
    // D500 LiDAR as a short cylindrical scanner.
    color("gray")
        cylinder(h = 40, d = lidar_base_d - 6);
    color("darkred")
        translate([0, 0, 30])
            cylinder(h = 6, d = lidar_base_d - 4);
}

// ---- connectors and cables ----

module deutsch_plug_stub() {
    // Mated Deutsch DT 6-pin plug protruding out the rear wall.
    color("black")
        rotate([0, 90, 0])
            cylinder(h = 25, d = deutsch_bulkhead_d + 6);
}

module cable_bundle(length, diameter, color_name = "dimgray") {
    // Straight cable segment of given length aimed along +X (you
    // rotate and translate it to route).
    color(color_name)
        rotate([0, 90, 0])
            cylinder(h = length, d = diameter);
}

// ---- MTT-154 context stub ----

module mtt_ecu_stub() {
    // A stylized block standing in for the MTT auxiliary port,
    // mounted to an abstracted sled surface. Used only to show
    // where the cable harness terminates.
    color("tan") {
        // sled surface
        cube([400, 300, 6]);
        // ECU housing
        translate([60, 130, 6])
            cube([100, 60, 40]);
    }
    // Deutsch socket on the ECU housing (facing +X, toward the rover)
    color("black")
        translate([160, 160, 26])
            rotate([0, 90, 0])
                cylinder(h = 15, d = deutsch_bulkhead_d + 6);
}

// ---- the harness that runs from the enclosure to the MTT ----

module harness_enclosure_to_mtt() {
    // Short vertical drop from the rear of the enclosure, then a
    // long run back to the MTT ECU. The shape is approximate; the
    // real harness takes whatever path avoids pinch points on the
    // sled. Useful here only to show that the cable exists and
    // roughly how much length to budget.
    color("black") {
        // straight segment out of the Deutsch bulkhead
        rotate([0, 90, 0])
            cylinder(h = 60, d = 12);
    }
}
