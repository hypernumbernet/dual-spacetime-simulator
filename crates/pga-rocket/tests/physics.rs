//! Integration tests for the shipped rocket physics step (no mocked integrator).

use pga_rocket::euclidean_pga::{
    extract_point, motor_from_pose, motor_translation, point, translator,
};
use pga_rocket::sim::{
    ControlCommand, RocketParams, RocketState, engine_wrench, rcs_wrench, step_rocket, GRAVITY,
};

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

#[test]
fn gimbal_pitch_produces_body_x_torque_and_angular_accel() {
    let mut s = RocketState::at_altitude(200.0);
    s.set_command(ControlCommand {
        throttle: 1.0,
        pitch: 1.0,
        yaw: 0.0,
        roll: 0.0,
    });
    let w = s.engine_wrench_body();
    // Pitch gimbal → force has +Z; r_y < 0 ⇒ τ_x = r_y F_z < 0.
    assert!(
        w.torque[0] < -1000.0,
        "expected large negative pitch torque, got {:?}",
        w.torque
    );
    assert!(
        w.torque[1].abs() < 1.0,
        "pitch TVC should not make roll torque τ_y={}",
        w.torque[1]
    );
    assert!(
        w.torque[2].abs() < 100.0,
        "pitch-only TVC τ_z should be near 0, got {}",
        w.torque[2]
    );

    let omega0 = s.omega;
    step_rocket(&mut s, DT);
    assert!(
        s.omega[0] < omega0[0] - 1e-5,
        "ω_x should decrease under pitch gimbal, was {:?} now {:?}",
        omega0,
        s.omega
    );
}

#[test]
fn gimbal_yaw_produces_body_z_torque() {
    let mut s = RocketState::at_altitude(200.0);
    s.set_command(ControlCommand {
        throttle: 1.0,
        pitch: 0.0,
        yaw: 1.0,
        roll: 0.0,
    });
    let w = s.engine_wrench_body();
    // Yaw about +Z: thrust gains +X; τ_z = −r_y F_x with r_y < 0 ⇒ τ_z > 0 when F_x > 0.
    assert!(
        w.torque[2].abs() > 1000.0,
        "expected large yaw torque about Z, got {:?}",
        w.torque
    );
    assert!(
        w.torque[1].abs() < 1.0,
        "yaw TVC should not spin about Y, τ_y={}",
        w.torque[1]
    );
    assert!(
        w.torque[0].abs() < 100.0,
        "yaw-only TVC τ_x near 0, got {}",
        w.torque[0]
    );
}

#[test]
fn zero_throttle_gimbal_produces_no_engine_wrench() {
    let mut s = RocketState::at_altitude(200.0);
    s.set_command(ControlCommand {
        throttle: 0.0,
        pitch: 1.0,
        yaw: 1.0,
        roll: 0.0,
    });
    let w = s.engine_wrench_body();
    assert!(w.force.iter().all(|c| c.abs() < 1e-12));
    assert!(w.torque.iter().all(|c| c.abs() < 1e-12));

    for _ in 0..30 {
        step_rocket(&mut s, DT);
    }
    // Free-fall only: no propulsive spin from dead engine + gimbal.
    assert!(
        s.omega[0].abs() < 1e-6 && s.omega[1].abs() < 1e-6 && s.omega[2].abs() < 1e-6,
        "expected no angular velocity without thrust/RCS, omega={:?}",
        s.omega
    );
}

#[test]
fn roll_thrusters_pure_couple_spins_about_y() {
    let p = RocketParams::default();
    let w = rcs_wrench(&p, 1.0);
    let expected_ty = 4.0 * p.rcs_radius * p.rcs_thrust;
    assert!(
        (w.torque[1] - expected_ty).abs() < 1e-6,
        "τ_y={} expected {}",
        w.torque[1],
        expected_ty
    );
    assert!(w.force.iter().all(|c| c.abs() < 1e-9));
    assert!(w.torque[0].abs() < 1e-9);
    assert!(w.torque[2].abs() < 1e-9);

    let mut s = RocketState::at_altitude(200.0);
    s.set_command(ControlCommand {
        throttle: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        roll: 1.0,
    });
    let pos0 = s.position();
    for _ in 0..120 {
        step_rocket(&mut s, DT);
    }
    assert!(
        s.omega[1] > 0.05,
        "positive roll should grow ω_y, got {:?}",
        s.omega
    );
    assert!(
        s.omega[0].abs() < 0.02 && s.omega[2].abs() < 0.02,
        "roll RCS should not tip, omega={:?}",
        s.omega
    );
    // Pure couple: CoM horizontal drift from RCS alone should stay tiny.
    let pos = s.position();
    let horiz = ((pos[0] - pos0[0]).powi(2) + (pos[2] - pos0[2]).powi(2)).sqrt();
    assert!(
        horiz < 0.5,
        "pure roll couple should not translate CoM much, horiz drift={horiz}"
    );
}

#[test]
fn engine_lever_arm_magnitude_matches_analytic() {
    let p = RocketParams::default();
    let cmd = ControlCommand {
        throttle: 1.0,
        pitch: 1.0,
        yaw: 0.0,
        roll: 0.0,
    };
    let w = engine_wrench(&p, cmd);
    let expected = p.thrust_application_y * p.max_thrust * p.max_gimbal_angle.sin();
    assert!(
        (w.torque[0] - expected).abs() < 1.0,
        "τ_x={} analytic={}",
        w.torque[0],
        expected
    );
}
