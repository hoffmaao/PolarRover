// Removable lid for the brain box.
//
// Layout on top:
//   Front (+X side) — pan-tilt wide-angle camera mount, centered
//                     across Y. Positioned so the servo axis sits
//                     `pantilt_offset_from_front` from the front edge.
//   Rear  (-X side) — D500 LiDAR pedestal base, same Y centerline.
// Underside carries the gasket groove and the M3 clearance holes
// matching the body's insert pattern.

include <params.scad>

module lid() {
    difference() {
        // solid plate
        cube([ext_w, ext_d, lid_t]);

        // gasket groove on the underside
        translate([wall_t + bay_margin - gasket_groove_w/2,
                   wall_t + bay_margin - gasket_groove_w/2,
                   -0.1])
            gasket_groove(bay_w - 2 * bay_margin + gasket_groove_w,
                          bay_d - 2 * bay_margin + gasket_groove_w);

        // lid fastener clearance holes
        lid_bolt_holes();

        // pan-tilt mount near the FRONT of the lid
        translate([ext_w - pantilt_offset_from_front, ext_d/2, 0])
            pantilt_cutout_features();

        // LiDAR pedestal base mounting holes near the REAR of the lid
        // (the pedestal itself is printed separately and bolts on)
        translate([lidar_offset_from_rear, ext_d/2, 0])
            lidar_pedestal_base_holes();
    }

    // LiDAR pedestal riser — integral with the lid, prints as one piece.
    translate([lidar_offset_from_rear, ext_d/2, lid_t])
        lidar_pedestal();
}

module gasket_groove(w, d) {
    difference() {
        cube([w, d, gasket_groove_d + 0.2]);
        translate([gasket_groove_w, gasket_groove_w, -0.05])
            cube([w - 2 * gasket_groove_w,
                  d - 2 * gasket_groove_w,
                  gasket_groove_d + 0.5]);
    }
}

module lid_bolt_holes() {
    for (x = [lid_bolt_spacing/2 : lid_bolt_spacing : ext_w - lid_bolt_spacing/2])
        for (y = [wall_t/2, ext_d - wall_t/2])
            translate([x, y, -0.1])
                cylinder(h = lid_t + 0.2, d = lid_bolt_hole);

    for (y = [lid_bolt_spacing/2 : lid_bolt_spacing : ext_d - lid_bolt_spacing/2])
        for (x = [wall_t/2, ext_w - wall_t/2])
            translate([x, y, -0.1])
                cylinder(h = lid_t + 0.2, d = lid_bolt_hole);
}

module pantilt_cutout_features() {
    // Square cutout for the servo hub (centered on the pass-through).
    translate([-pantilt_cutout/2, -pantilt_cutout/2, -0.1])
        cube([pantilt_cutout, pantilt_cutout, lid_t + 0.2]);

    // Four M3 bolts in an X pattern around the cutout.
    for (dx = [-1, 1], dy = [-1, 1])
        translate([dx * pantilt_bolt_span/2, dy * pantilt_bolt_span/2, -0.1])
            cylinder(h = lid_t + 0.2, d = 3.3);
}

module lidar_pedestal_base_holes() {
    // Bolt pattern passing through the lid into the pedestal above.
    for (a = [0 : 90 : 359])
        rotate([0, 0, a])
            translate([lidar_bolt_pcd/2, 0, -0.1])
                cylinder(h = lid_t + 0.2, d = lidar_bolt_hole);
}

module lidar_pedestal() {
    // Hollow pedestal that lifts the LiDAR above the pan-tilt mast.
    difference() {
        cylinder(h = lidar_pedestal_h, d = lidar_base_d);
        translate([0, 0, -0.1])
            cylinder(h = lidar_pedestal_h + 0.2, d = lidar_base_d - 6);
        // LiDAR mounting bolts from the top
        for (a = [0 : 90 : 359])
            rotate([0, 0, a])
                translate([lidar_bolt_pcd/2, 0,
                           lidar_pedestal_h - 6])
                    cylinder(h = 7, d = lidar_bolt_hole);
    }

    // Slot for the LiDAR data/power cable to exit the pedestal and
    // run back into the enclosure through the service-USB pass-through.
    translate([-lidar_base_d/2 - 0.5, -4, 5])
        cube([lidar_base_d + 1, 8, 12]);
}

lid();
