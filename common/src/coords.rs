/// Coordinate math utilities for guidance calculations.
///
/// Uses WGS84 ellipsoid constants. Guidance-critical functions (XTE, bearing)
/// use a local tangent-plane (ENU) projection centred on point A for
/// sub-centimetre accuracy at paddock scale without great-circle artefacts.
/// Haversine is retained for longer-range distance queries.
use std::f64::consts::PI;

const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// WGS84 semi-major axis (equatorial radius) in metres.
const WGS84_A: f64 = 6_378_137.0;
/// WGS84 first eccentricity squared.
const WGS84_E2: f64 = 0.00669437999014;

/// Convert degrees to radians
pub fn deg_to_rad(deg: f64) -> f64 {
    deg * PI / 180.0
}

/// Convert radians to degrees
pub fn rad_to_deg(rad: f64) -> f64 {
    rad * 180.0 / PI
}

/// Haversine distance between two lat/lon points in metres
pub fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = deg_to_rad(lat2 - lat1);
    let dlon = deg_to_rad(lon2 - lon1);
    let lat1_r = deg_to_rad(lat1);
    let lat2_r = deg_to_rad(lat2);

    let a = (dlat / 2.0).sin().powi(2) + lat1_r.cos() * lat2_r.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS_M * c
}

/// Bearing from point 1 to point 2 in degrees (0-360, clockwise from north)
pub fn bearing(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let lat1_r = deg_to_rad(lat1);
    let lat2_r = deg_to_rad(lat2);
    let dlon_r = deg_to_rad(lon2 - lon1);

    let x = dlon_r.sin() * lat2_r.cos();
    let y = lat1_r.cos() * lat2_r.sin() - lat1_r.sin() * lat2_r.cos() * dlon_r.cos();

    (rad_to_deg(x.atan2(y)) + 360.0) % 360.0
}

/// Cross-track distance from a point to the great-circle arc A→B.
/// DEPRECATED for guidance — use `cross_track_distance_local` instead.
/// Retained for non-guidance uses where the spherical model is appropriate.
/// Returns signed distance in metres (negative = left of line, positive = right).
pub fn cross_track_distance(
    point_lat: f64,
    point_lon: f64,
    a_lat: f64,
    a_lon: f64,
    b_lat: f64,
    b_lon: f64,
) -> f64 {
    let dist_a_to_point = haversine_distance(a_lat, a_lon, point_lat, point_lon);
    let bearing_a_to_point = deg_to_rad(bearing(a_lat, a_lon, point_lat, point_lon));
    let bearing_a_to_b = deg_to_rad(bearing(a_lat, a_lon, b_lat, b_lon));

    let xtd =
        (dist_a_to_point / EARTH_RADIUS_M).sin() * (bearing_a_to_point - bearing_a_to_b).sin();

    EARTH_RADIUS_M * xtd.asin()
}

/// Project lat/lon to local East-North metres relative to a reference point.
///
/// Uses a WGS84 tangent-plane approximation. Sub-centimetre accurate out to
/// ~20 km from the reference — more than enough for any paddock.
pub fn to_local_en(ref_lat: f64, ref_lon: f64, lat: f64, lon: f64) -> (f64, f64) {
    let ref_lat_r = deg_to_rad(ref_lat);
    let sin_ref = ref_lat_r.sin();
    let cos_ref = ref_lat_r.cos();
    let dlat = deg_to_rad(lat - ref_lat);
    let dlon = deg_to_rad(lon - ref_lon);

    // Meridional radius of curvature (metres per radian of latitude)
    let m_lat = WGS84_A * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * sin_ref.powi(2)).powf(1.5);
    // Prime-vertical radius of curvature (metres per radian of longitude)
    let n_lon = WGS84_A / (1.0 - WGS84_E2 * sin_ref.powi(2)).sqrt();

    let north = dlat * m_lat;
    let east = dlon * n_lon * cos_ref;
    (east, north)
}

/// Cross-track distance using local tangent-plane (flat-earth) projection.
///
/// Projects A, B, and the point into a local East-North frame centred on A,
/// then computes the signed perpendicular distance from the point to the
/// *infinite* line through A and B. This avoids the great-circle curvature
/// artefacts of the spherical formula and gives a geometrically stable line
/// that doesn't drift with distance from the AB segment.
///
/// Returns signed distance in metres: positive = LEFT of A→B, negative = right.
///
/// NOTE: this is the OPPOSITE sign convention to the deprecated spherical
/// `cross_track_distance` above (which returns positive = right). Guidance
/// uses this local function exclusively, and the entire XTE chain — ab_line.rs,
/// steering.rs, the lightbar — is built on "positive = left". Do not try to
/// reconcile the two conventions without flipping every consumer.
pub fn cross_track_distance_local(
    point_lat: f64,
    point_lon: f64,
    a_lat: f64,
    a_lon: f64,
    b_lat: f64,
    b_lon: f64,
) -> f64 {
    let (pe, pn) = to_local_en(a_lat, a_lon, point_lat, point_lon);
    let (be, bn) = to_local_en(a_lat, a_lon, b_lat, b_lon);

    let line_len = (be * be + bn * bn).sqrt();
    if line_len < 1e-6 {
        return 0.0;
    }

    // 2D cross product (B−A) × (P−A) / |B−A| in the East-North (right-handed)
    // frame: a positive result means P is counter-clockwise from A→B, i.e. to
    // the LEFT of the A→B direction of travel.
    (be * pn - bn * pe) / line_len
}

/// Bearing from A to B computed in the local tangent plane.
///
/// Returns degrees 0–360 clockwise from north, same convention as the
/// spherical `bearing()`. Using the local projection keeps the bearing
/// consistent with `cross_track_distance_local` and avoids any subtle
/// divergence between the two frames.
pub fn bearing_local(a_lat: f64, a_lon: f64, b_lat: f64, b_lon: f64) -> f64 {
    let (de, dn) = to_local_en(a_lat, a_lon, b_lat, b_lon);
    (rad_to_deg(de.atan2(dn)) + 360.0) % 360.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haversine_known_distance() {
        // Jamestown SA to Adelaide SA is roughly 200km
        let dist = haversine_distance(-33.2067, 138.6000, -34.9285, 138.6007);
        assert!((dist - 191_500.0).abs() < 5000.0, "Distance was {dist}m");
    }

    #[test]
    fn test_bearing_north() {
        // Point directly north should give ~0 degrees
        let b = bearing(-33.0, 138.0, -32.0, 138.0);
        assert!(b < 1.0 || b > 359.0, "Bearing was {b}");
    }

    #[test]
    fn test_bearing_local_north() {
        let b = bearing_local(-33.0, 138.0, -32.0, 138.0);
        assert!(b < 1.0 || b > 359.0, "Local bearing was {b}");
    }

    #[test]
    fn test_cross_track_on_line() {
        // A point on the line should have ~0 cross track distance
        let xtd = cross_track_distance(
            -33.5, 138.6, // point (midpoint)
            -33.0, 138.6, // A
            -34.0, 138.6, // B
        );
        assert!(xtd.abs() < 1.0, "XTD was {xtd}m");
    }

    #[test]
    fn test_cross_track_local_on_line() {
        // A point on a N-S line should have ~0 cross track distance
        let xtd = cross_track_distance_local(
            -33.5, 138.6, // point (midpoint)
            -33.0, 138.6, // A
            -34.0, 138.6, // B
        );
        assert!(xtd.abs() < 0.01, "Local XTD was {xtd}m");
    }

    #[test]
    fn test_cross_track_local_left_of_line() {
        // A→B runs north→south here, so a point to the EAST is to the LEFT of
        // the direction of travel → positive XTE.
        let xtd = cross_track_distance_local(
            -33.5, 138.6001, // point slightly east
            -33.0, 138.6, // A (north)
            -34.0, 138.6, // B (south)
        );
        // At -33.5° latitude, 0.0001° longitude ≈ 9.3m
        assert!(
            xtd > 5.0 && xtd < 15.0,
            "Local XTD was {xtd}m, expected ~9m"
        );
    }

    #[test]
    fn test_cross_track_local_beyond_b() {
        // Point well past B (extended line) should still give stable XTE.
        // This is the key advantage over the spherical formula.
        let xtd_mid = cross_track_distance_local(
            -33.5, 138.6001, // midpoint, slightly east
            -33.0, 138.6, // A
            -34.0, 138.6, // B
        );
        let xtd_past = cross_track_distance_local(
            -36.0, 138.6001, // well past B, same east offset
            -33.0, 138.6, // A
            -34.0, 138.6, // B
        );
        // On an infinite line, the XTE at the same longitude offset should be
        // essentially the same regardless of how far along the line we are.
        assert!(
            (xtd_mid - xtd_past).abs() < 0.5,
            "XTD drift: mid={xtd_mid}m, past_B={xtd_past}m"
        );
    }

    #[test]
    fn test_to_local_en_origin() {
        let (e, n) = to_local_en(-33.2, 138.6, -33.2, 138.6);
        assert!(e.abs() < 0.001 && n.abs() < 0.001);
    }
}
