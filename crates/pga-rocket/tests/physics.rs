//! Integration tests for the shipped rocket physics step (no mocked integrator).

use pga_rocket::euclidean_pga::{
    extract_point, motor_from_pose, motor_translation, point, translator,
};
use pga_rocket::sim::{ControlCommand, RocketState, step_rocket, GRAVITY};

const DT: f64 = 1.0 / 120.0;

#[test]
fn freefall_increases_downward_velocity_and_loses_altitude() {
    let mut s = RocketState::at_altitude(100.0);
    s.set_command(ControlCommand {
        throttle: 0.0,
        ..Default::default()
    });
    let y0 = s.altitude();
    let vy0 = s.velocity[1];
    for _ in 0..60 {
        step_rocket(&mut s, DT);
    }
    assert!(
        s.velocity[1] < vy0 - 1.0,
        "expected more downward velocity, vy0={vy0} vy={}",
        s.velocity[1]
    );
    assert!(
        s.altitude() < y0 - 0.5,
        "expected lower altitude, y0={y0} y={}",
        s.altitude()
    );
    // Rough free-fall: Δv ≈ −g t
    let t = 60.0 * DT;
    let expected_dv = -GRAVITY * t;
    assert!(
        (s.velocity[1] - expected_dv).abs() < 2.0,
        "vy should be near {expected_dv}, got {}",
        s.velocity[1]
    );
}

#[test]
fn sustained_thrust_climbs_from_pad() {
    let mut s = RocketState::resting_on_pad();
    let y0 = s.altitude();
    s.set_command(ControlCommand {
        throttle: 1.0,
        ..Default::default()
    });
    // Integrate long enough for T/W=1.5 to leave the pad.
    for _ in 0..600 {
        step_rocket(&mut s, DT);
    }
    assert!(
        s.altitude() > y0 + 5.0,
        "expected climb from pad: y0={y0} y={}",
        s.altitude()
    );
    assert!(
        s.velocity[1] > 1.0,
        "expected positive climb rate, vy={}",
        s.velocity[1]
    );
    assert!(
        s.lowest_foot_y() > 0.5,
        "feet should leave the ground, lowest={}",
        s.lowest_foot_y()
    );
}

#[test]
fn contact_prevents_ground_penetration() {
    // Drop from modest height with zero thrust; feet must not stay below plane.
    let mut s = RocketState::at_altitude(40.0);
    s.set_command(ControlCommand {
        throttle: 0.0,
        ..Default::default()
    });
    for _ in 0..2000 {
        step_rocket(&mut s, DT);
        assert!(
            s.lowest_foot_y() >= -0.05,
            "foot penetrated ground: y={}",
            s.lowest_foot_y()
        );
    }
    // After settling, still non-penetrating and near the pad.
    assert!(
        s.lowest_foot_y() >= -0.05,
        "settled penetration {}",
        s.lowest_foot_y()
    );
    assert!(
        s.altitude() < 30.0,
        "should have fallen toward ground, alt={}",
        s.altitude()
    );
}

#[test]
fn pga_sandwich_and_motor_composition_numeric() {
    // Construct pose via PGA motors and check sandwich product outcome.
    let m = motor_from_pose(2.0, 5.0, -1.0, 0.0, 0.0, 0.0);
    let p = point(1.0, 0.0, 0.0);
    let q = extract_point(&m.sandwich(&p));
    assert!((q[0] - 3.0).abs() < 1e-8, "x={:?}", q);
    assert!((q[1] - 5.0).abs() < 1e-8, "y={:?}", q);
    assert!((q[2] - (-1.0)).abs() < 1e-8, "z={:?}", q);

    let t = translator(0.0, 3.0, 0.0);
    let m2 = t.geo(&m);
    let tr = motor_translation(&m2.normalize_motor());
    assert!((tr[1] - 8.0).abs() < 1e-7, "composed ty={:?}", tr);

    // RocketState pose is this motor; position API must match.
    let mut s = RocketState::resting_on_pad();
    s.motor = m2.normalize_motor();
    let pos = s.position();
    assert!((pos[1] - tr[1]).abs() < 1e-7);
}

#[test]
fn ground_plane_is_pga_element() {
    use pga_rocket::euclidean_pga::{basis, ground_plane};
    let g = ground_plane();
    // y=0 plane → e2 (normal along +Y)
    assert!((g.coeff(basis::E2) - 1.0).abs() < 1e-12);
    assert!(g.coeff(basis::E0).abs() < 1e-12);
    let s = RocketState::resting_on_pad();
    assert!((s.ground.coeff(basis::E2) - 1.0).abs() < 1e-12);
}
