/// Coordinate math utilities for guidance calculations.
///
/// Uses WGS84 ellipsoid for distance calculations.
/// All angles in degrees unless suffixed with _rad.
use std::f64::consts::PI;

const EARTH_RADIUS_M: f64 = 6_371_000.0;

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

/// Cross-track distance from a point to the line defined by A->B.
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
    fn test_cross_track_on_line() {
        // A point on the line should have ~0 cross track distance
        let xtd = cross_track_distance(
            -33.5, 138.6, // point (midpoint)
            -33.0, 138.6, // A
            -34.0, 138.6, // B
        );
        assert!(xtd.abs() < 1.0, "XTD was {xtd}m");
    }
}
