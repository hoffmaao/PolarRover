// Bench-test base plate. Same four-corner pattern to the enclosure as
// the sled plate, but the bottom face carries a generic grid of M4
// holes on 20 mm centers so the enclosure can be bolted to any
// breadboard, T-slot extrusion, or bench with the right adapter.
// Use this during the T1 / T8 bench tests before the sled is
// available.

include <params.scad>

plate_t    = 6.0;
plate_w    = ext_w + 40;
plate_d    = ext_d + 40;
plate_fillet = 4.0;

grid_pitch = 20;
grid_hole  = 4.3;   // M4 clearance

module mount_bench() {
    difference() {
        filleted_plate(plate_w, plate_d, plate_t, plate_fillet);

        // enclosure attachment pattern (top)
        enclosure_bolt_pattern();

        // bench grid (bottom)
        bench_grid();
    }
}

module filleted_plate(w, d, t, r) {
    linear_extrude(height = t)
        offset(r = r)
            offset(r = -r)
                square([w, d]);
}

module enclosure_bolt_pattern() {
    cx = plate_w / 2;
    cy = plate_d / 2;
    for (dx = [-1, 1], dy = [-1, 1])
        translate([cx + dx * plate_bolt_dx / 2,
                   cy + dy * plate_bolt_dy / 2,
                   -0.1]) {
            cylinder(h = plate_t + 0.2, d = plate_bolt_hole);
            translate([0, 0, plate_t - 3])
                cylinder(h = 3.2, d1 = plate_bolt_hole, d2 = plate_bolt_hole + 5);
        }
}

module bench_grid() {
    // Exclude the central region so we don't drill through the
    // enclosure attachment pocket; place grid holes around the
    // perimeter.
    for (x = [grid_pitch : grid_pitch : plate_w - grid_pitch])
        for (y = [grid_pitch : grid_pitch : plate_d - grid_pitch]) {
            if (x < (plate_w - plate_bolt_dx) / 2 - grid_pitch
                || x > (plate_w + plate_bolt_dx) / 2 + grid_pitch
                || y < (plate_d - plate_bolt_dy) / 2 - grid_pitch
                || y > (plate_d + plate_bolt_dy) / 2 + grid_pitch) {
                translate([x, y, -0.1])
                    cylinder(h = plate_t + 0.2, d = grid_hole);
            }
        }
}

mount_bench();
