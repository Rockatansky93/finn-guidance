use finn_guidance_common::coords;

fn main() {
    // Typical Jamestown area AB line: roughly N-S
    let a_lat = -33.2067;
    let a_lon = 138.6000;
    let b_lat = -33.2167;  // ~1km south
    let b_lon = 138.6000;

    // Point 10m east of the line (should be "right" when traveling A->B = south)
    let p_lat = -33.2117;
    let p_lon = 138.60012; // ~10m east at this latitude

    let bearing_old = coords::bearing(a_lat, a_lon, b_lat, b_lon);
    let bearing_new = coords::bearing_local(a_lat, a_lon, b_lat, b_lon);

    let xtd_old = coords::cross_track_distance(p_lat, p_lon, a_lat, a_lon, b_lat, b_lon);
    let xtd_new = coords::cross_track_distance_local(p_lat, p_lon, a_lat, a_lon, b_lat, b_lon);

    println!("=== Bearing ===");
    println!("  Spherical bearing A->B: {:.6}°", bearing_old);
    println!("  Local     bearing A->B: {:.6}°", bearing_new);
    println!("  Difference:             {:.6}°", bearing_new - bearing_old);

    println!("\n=== XTE (point ~10m east of N-S line) ===");
    println!("  Spherical XTD: {:.4} m", xtd_old);
    println!("  Local     XTD: {:.4} m", xtd_new);
    println!("  Same sign? {}", (xtd_old > 0.0) == (xtd_new > 0.0));

    // Now test E-W line
    let a2_lat = -33.2067;
    let a2_lon = 138.5900;
    let b2_lat = -33.2067;
    let b2_lon = 138.6100; // ~1.7km east

    // Point 10m north of E-W line
    let p2_lat = -33.2066;
    let p2_lon = 138.6000;

    let bearing_old2 = coords::bearing(a2_lat, a2_lon, b2_lat, b2_lon);
    let bearing_new2 = coords::bearing_local(a2_lat, a2_lon, b2_lat, b2_lon);
    let xtd_old2 = coords::cross_track_distance(p2_lat, p2_lon, a2_lat, a2_lon, b2_lat, b2_lon);
    let xtd_new2 = coords::cross_track_distance_local(p2_lat, p2_lon, a2_lat, a2_lon, b2_lat, b2_lon);

    println!("\n=== E-W line ===");
    println!("  Spherical bearing: {:.6}°", bearing_old2);
    println!("  Local     bearing: {:.6}°", bearing_new2);
    println!("  Difference:        {:.6}°", bearing_new2 - bearing_old2);
    println!("  Spherical XTD: {:.4} m", xtd_old2);
    println!("  Local     XTD: {:.4} m", xtd_new2);
    println!("  Same sign? {}", (xtd_old2 > 0.0) == (xtd_new2 > 0.0));
}
