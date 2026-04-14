// Base plate that couples the brain box to the MTT-154 instrument
// sled. The top face mirrors the enclosure's four-corner bolt pattern
// (M5, `plate_bolt_dx` x `plate_bolt_dy`). The bottom face carries
// the MTT sled mounting holes — those dimensions are unknown until
// we measure the sled, so the placeholder below uses parametric values
// that should be updated once we have the sled on a bench.

include <params.scad>

// MTT sled mounting pattern — PLACEHOLDER. Update these three values
// from direct measurement before ordering or printing. Keep the bolt
// hole diameter matched to the sled's threaded inserts or hardware.
// These values must sit clearly outside the enclosure's own bolt
// pattern (plate_bolt_dx x plate_bolt_dy) so the two sets of holes
// don't crowd each other at the corners.
sled_bolt_dx      = 220;   // TBD — along sled mounting rail
sled_bolt_dy      = 160;   // TBD — across sled mounting rail
sled_bolt_hole    = 6.6;   // TBD — M6 clearance

// Plate thickness — tuned for vibration damping while keeping mass low.
plate_t = 6.0;

// Corner fillet radius on the plate outline.
plate_fillet = 4.0;

// Overall plate footprint generously exceeds both bolt patterns so
// there's meat around each hole.
plate_w = max(ext_w, sled_bolt_dx + 40);
plate_d = max(ext_d, sled_bolt_dy + 40);

module mount_mtt_sled() {
    difference() {
        filleted_plate(plate_w, plate_d, plate_t, plate_fillet);

        // four enclosure bolt holes (top face) — countersunk M5
        enclosure_bolt_pattern();

        // four sled bolt holes (through the plate) — countersunk on bottom
        sled_bolt_pattern();

        // optional weight-reduction pocket on the underside
        translate([plate_w/2, plate_d/2, -0.1])
            linear_extrude(height = plate_t - 3)
                offset(r = -15)
                    square([plate_w - 30, plate_d - 30], center = true);
    }
}

module filleted_plate(w, d, t, r) {
    linear_extrude(height = t)
        offset(r = r)
            offset(r = -r)
                square([w, d]);
}

module enclosure_bolt_pattern() {
    // The enclosure sits centered on the plate. Holes countersunk
    // from the top so M5 flat-heads sit flush.
    cx = plate_w / 2;
    cy = plate_d / 2;
    for (dx = [-1, 1], dy = [-1, 1])
        translate([cx + dx * plate_bolt_dx / 2,
                   cy + dy * plate_bolt_dy / 2,
                   -0.1]) {
            cylinder(h = plate_t + 0.2, d = plate_bolt_hole);
            // countersink from top (flat-head M5)
            translate([0, 0, plate_t - 3])
                cylinder(h = 3.2, d1 = plate_bolt_hole, d2 = plate_bolt_hole + 5);
        }
}

module sled_bolt_pattern() {
    cx = plate_w / 2;
    cy = plate_d / 2;
    for (dx = [-1, 1], dy = [-1, 1])
        translate([cx + dx * sled_bolt_dx / 2,
                   cy + dy * sled_bolt_dy / 2,
                   -0.1]) {
            cylinder(h = plate_t + 0.2, d = sled_bolt_hole);
            // countersink from bottom
            cylinder(h = 3.2, d1 = sled_bolt_hole + 5, d2 = sled_bolt_hole);
        }
}

mount_mtt_sled();
