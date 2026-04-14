// Main enclosure body — the "brain box."
//
// Orientation (matches params.scad):
//   Back  (x = 0)     — cable wall: Deutsch DT 6-pin, SMA, service USB-C.
//   Front (x = ext_w) — external mount pad for the OAK-D Pro.
//   Top   (z = ext_h) — lid opening (lid in lid.scad).
//   Bottom (z = 0)    — base plate bolt pattern.
//   Sides — sealed, one carries the pressure-equalization vent.

include <params.scad>

module brain_box() {
    difference() {
        // outer shell
        cube([ext_w, ext_d, ext_h]);

        // internal bay (subtracts the hollow)
        translate([wall_t, wall_t, floor_t])
            cube([bay_w, bay_d, bay_h + 1]);

        // gasket groove around the top opening
        translate([wall_t + bay_margin,
                   wall_t + bay_margin,
                   ext_h - gasket_groove_d])
            gasket_groove(bay_w - 2 * bay_margin,
                          bay_d - 2 * bay_margin);

        // lid fastener heat-set insert pockets
        lid_insert_pockets();

        // base plate bolt pattern (floor)
        base_plate_bolt_holes();

        // rear cable wall cutouts (x = 0)
        rear_wall_cutouts();

        // front camera mount pad (x = ext_w)
        front_wall_cutouts();

        // side vent
        side_wall_vent();
    }

    // standoffs for the Jetson carrier board
    translate([wall_t + bay_margin, wall_t + bay_margin, floor_t])
        jetson_standoffs();

    // shelf supports for the auxiliary component shelf
    translate([wall_t + bay_margin + jetson_w + bay_margin,
               wall_t + bay_margin,
               floor_t])
        shelf_supports();
}

module gasket_groove(w, d) {
    difference() {
        translate([-gasket_groove_w/2, -gasket_groove_w/2, 0])
            cube([w + gasket_groove_w, d + gasket_groove_w, gasket_groove_d + 1]);
        cube([w, d, gasket_groove_d + 1]);
    }
}

module lid_insert_pockets() {
    // inserts along the long edges
    for (x = [lid_bolt_spacing/2 : lid_bolt_spacing : ext_w - lid_bolt_spacing/2])
        for (y = [wall_t/2, ext_d - wall_t/2])
            translate([x, y, ext_h - lid_insert_depth])
                cylinder(h = lid_insert_depth + 0.1, d = lid_insert_d);

    // inserts along the short edges
    for (y = [lid_bolt_spacing/2 : lid_bolt_spacing : ext_d - lid_bolt_spacing/2])
        for (x = [wall_t/2, ext_w - wall_t/2])
            translate([x, y, ext_h - lid_insert_depth])
                cylinder(h = lid_insert_depth + 0.1, d = lid_insert_d);
}

module base_plate_bolt_holes() {
    cx = ext_w / 2;
    cy = ext_d / 2;
    for (dx = [-1, 1], dy = [-1, 1])
        translate([cx + dx * plate_bolt_dx / 2,
                   cy + dy * plate_bolt_dy / 2,
                   -0.1])
            cylinder(h = floor_t + 0.2, d = plate_bolt_hole);
}

module rear_wall_cutouts() {
    // Back face: cable wall. Cutouts pierce in +X direction.
    cy = wall_t + bay_d/2;
    cz = floor_t + bay_h/2;

    // Deutsch DT 6-pin (center of the wall, carries CAN + aux power)
    translate([wall_t/2, cy, cz])
        rotate([0, 90, 0]) {
            cylinder(h = wall_t + 2, d = deutsch_bulkhead_d, center = true);
            for (a = [45 : 90 : 359])
                rotate([0, 0, a])
                    translate([0, deutsch_flange_bolt_pattern / sqrt(2) / 2, 0])
                        cylinder(h = wall_t + 2,
                                 d = deutsch_flange_bolt_hole,
                                 center = true);
        }

    // SMA bulkhead (GNSS antenna, above the Deutsch)
    translate([wall_t/2, cy, cz + 32])
        rotate([0, 90, 0])
            cylinder(h = wall_t + 2, d = sma_bulkhead_d, center = true);

    // Service USB-C (below the Deutsch, silicone-capped in use)
    translate([wall_t/2, cy, cz - 28])
        rotate([0, 90, 0])
            cube([wall_t + 2, usbc_cutout_w, usbc_cutout_h], center = true);
}

module front_wall_cutouts() {
    // Front face: OAK-D Pro stereo camera mounts externally on four
    // M3 bolts. Its USB cable passes back through the grommet.
    cy = wall_t + bay_d/2;
    cz = floor_t + bay_h/2;

    translate([ext_w - wall_t/2, cy, cz])
        rotate([0, 90, 0]) {
            // four M3 bolt clearance holes (75 x 22 mm rectangle)
            for (dx = [-1, 1], dy = [-1, 1])
                translate([dx * oak_mount_dx/2, dy * oak_mount_dy/2, 0])
                    cylinder(h = wall_t + 2, d = oak_mount_hole, center = true);

            // USB cable grommet offset below the mount pattern
            translate([0, -oak_mount_dy/2 - 12, 0])
                cylinder(h = wall_t + 2, d = oak_cable_grommet_d, center = true);
        }
}

module side_wall_vent() {
    // Gortex-membrane pressure equalization on the left (+Y) side wall.
    translate([ext_w/2, ext_d - wall_t/2, floor_t + bay_h/2 + 25])
        rotate([90, 0, 0])
            cylinder(h = wall_t + 2, d = vent_d, center = true);
}

module jetson_standoffs() {
    // Four M2.5 standoffs matching the dev kit hole pattern.
    ox = (jetson_w - jetson_mount_dx) / 2;
    oy = (jetson_d - jetson_mount_dy) / 2;
    for (x = [ox, ox + jetson_mount_dx])
        for (y = [oy, oy + jetson_mount_dy])
            translate([x, y, 0])
                difference() {
                    cylinder(h = standoff_h, d = standoff_d);
                    translate([0, 0, -0.1])
                        cylinder(h = standoff_h + 0.2, d = jetson_mount_hole);
                }
}

module shelf_supports() {
    for (y = [0, bay_d - 2 * bay_margin])
        translate([0, y, 0])
            cube([5, 5, shelf_h]);
}

brain_box();
