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
    // Drop from modest height with zero thrust; hull/feet must not stay below plane.
    let mut s = RocketState::at_altitude(40.0);
    s.set_command(ControlCommand {
        throttle: 0.0,
        ..Default::default()
    });
    for _ in 0..2000 {
        step_rocket(&mut s, DT);
        assert!(
            s.lowest_probe_y() >= -0.05,
            "hull/foot penetrated ground: y={}",
            s.lowest_probe_y()
        );
    }
    // After settling, still non-penetrating and near the pad.
    assert!(
        s.lowest_probe_y() >= -0.05,
        "settled penetration {}",
        s.lowest_probe_y()
    );
    assert!(
        s.altitude() < 30.0,
        "should have fallen toward ground, alt={}",
        s.altitude()
    );
}

#[test]
fn slight_tilt_on_pad_settles_upright() {
    use pga_rocket::euclidean_pga::motor_body_up_world;

    // Residual post-touchdown lean must not freeze: rest-share normals on the
    // lower feet should produce a restoring torque back toward all-four-feet.
    let mut s = RocketState::resting_on_pad();
    let com_y = s.altitude();
    s.motor = motor_from_pose(0.0, com_y, 0.0, 0.10, 0.0, 0.0);
    s.velocity = [0.0, 0.0, 0.0];
    s.omega = [0.0, 0.0, 0.0];
    s.contacting = true;
    s.set_command(ControlCommand::default());

    let tilt0 = {
        let up = motor_body_up_world(&s.motor);
        up[1].clamp(-1.0, 1.0).acos()
    };
    assert!(tilt0 > 0.08, "precondition: start tilted, tilt0={tilt0:.3}");

    for _ in 0..(8 * 120) {
        step_rocket(&mut s, DT);
        assert!(
            s.lowest_probe_y() >= -0.08,
            "penetrated while settling: y={}",
            s.lowest_probe_y()
        );
    }

    let tilt = {
        let up = motor_body_up_world(&s.motor);
        up[1].clamp(-1.0, 1.0).acos()
    };
    assert!(!s.destroyed, "impact={}", s.last_impact_speed);
    assert!(
        tilt < 0.025,
        "expected near-upright after pad settle, tilt={tilt:.4} (was {tilt0:.3})"
    );
    assert!(
        s.omega[0].abs() < 0.05 && s.omega[2].abs() < 0.05,
        "should be nearly at rest angularly, omega={:?}",
        s.omega
    );
}

#[test]
fn tipped_body_hull_contacts_ground_without_penetration() {
    use pga_rocket::euclidean_pga::motor_from_pose;
    use std::f64::consts::FRAC_PI_2;

    // Tip 90° about +X so the body cylinder side faces the ground.
    let mut s = RocketState::at_altitude(25.0);
    s.motor = motor_from_pose(0.0, 25.0, 0.0, FRAC_PI_2, 0.0, 0.0);
    s.velocity = [0.0, 0.0, 0.0];
    s.omega = [0.0, 0.0, 0.0];
    // The ~21 m/s side impact would (correctly) destroy the vehicle; disable
    // destruction to test hull contact mechanics in isolation.
    s.params.crash_impact_speed = 1.0e6;
    s.set_command(ControlCommand::default());

    let mut saw_body = false;
    for _ in 0..2500 {
        step_rocket(&mut s, DT);
        if s.body_contacting {
            saw_body = true;
        }
        assert!(
            s.lowest_probe_y() >= -0.08,
            "tipped hull penetrated: y={}",
            s.lowest_probe_y()
        );
    }
    assert!(
        saw_body,
        "expected body hull contact while lying/tipping onto the ground"
    );
    assert!(
        s.altitude() < 20.0,
        "should settle lower after tipping, alt={}",
        s.altitude()
    );
}

#[test]
fn restitution_causes_bounce_on_hard_impact() {
    let mut soft = RocketState::at_altitude(30.0);
    soft.params.restitution = 0.0;
    soft.params.crash_impact_speed = 1.0e6;
    soft.set_command(ControlCommand::default());

    let mut bouncy = RocketState::at_altitude(30.0);
    bouncy.params.restitution = 0.75;
    bouncy.params.crash_impact_speed = 1.0e6;
    bouncy.set_command(ControlCommand::default());

    let mut max_up_soft = 0.0_f64;
    let mut max_up_bouncy = 0.0_f64;
    for _ in 0..2000 {
        step_rocket(&mut soft, DT);
        step_rocket(&mut bouncy, DT);
        max_up_soft = max_up_soft.max(soft.velocity[1]);
        max_up_bouncy = max_up_bouncy.max(bouncy.velocity[1]);
    }
    assert!(
        max_up_bouncy > 3.0,
        "elastic drop should rebound upward, vy_max={max_up_bouncy}"
    );
    assert!(
        max_up_bouncy > max_up_soft + 2.0,
        "higher restitution should bounce more: soft_up={max_up_soft} bouncy_up={max_up_bouncy}"
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
fn grounded_roll_rcs_fires_and_friction_opposes_spin() {
    let mut s = RocketState::resting_on_pad();
    assert!(s.contacting);

    // RCS is available on the pad.
    s.set_command(ControlCommand {
        throttle: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        roll: 1.0,
    });
    assert!((s.command.roll - 1.0).abs() < 1e-12);
    let rcs = s.rcs_wrench_body();
    assert!(
        rcs.torque[1] > 1000.0,
        "RCS must produce +τ_y on the pad, got {:?}",
        rcs.torque
    );

    // Spin up briefly with low friction, then high friction should dump ω_y.
    s.params.friction_mu = 0.05;
    for _ in 0..180 {
        step_rocket(&mut s, DT);
    }
    let omega_spun = s.omega[1].abs();
    assert!(
        omega_spun > 0.02,
        "low-μ pad + RCS should allow some spin, ω_y={}",
        s.omega[1]
    );

    s.set_command(ControlCommand::default());
    s.params.friction_mu = 1.2;
    let before = s.omega[1].abs();
    for _ in 0..240 {
        step_rocket(&mut s, DT);
    }
    assert!(
        s.omega[1].abs() < before * 0.5,
        "foot friction should damp roll spin: before={before} after={}",
        s.omega[1].abs()
    );
    assert!(
        s.omega[1].abs() < 0.15,
        "spin should largely settle under high μ, ω_y={}",
        s.omega[1]
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
    let expected = p.nozzle_exit_y() * p.max_thrust * p.max_gimbal_angle.sin();
    assert!(
        (w.torque[0] - expected).abs() < 1e-9,
        "τ_x={} analytic={}",
        w.torque[0],
        expected
    );
}

#[test]
fn hard_ground_impact_destroys_vehicle() {
    let mut s = RocketState::at_altitude(80.0);
    s.set_command(ControlCommand::default());
    let mut destroyed = false;
    for _ in 0..4000 {
        step_rocket(&mut s, DT);
        if s.destroyed {
            destroyed = true;
            break;
        }
    }
    assert!(destroyed, "expected destruction on hard impact");
    assert!(
        s.last_impact_speed >= s.params.crash_impact_speed,
        "impact={} threshold={}",
        s.last_impact_speed,
        s.params.crash_impact_speed
    );
    assert!(s.explosion_age >= 0.0);
}

/// Very fast falls cross the contact band in a single step; the tunneling
/// guard in the hard-projection pass must still destroy the vehicle.
#[test]
fn very_high_drop_still_destroys_vehicle() {
    for alt in [300.0, 500.0] {
        let mut s = RocketState::at_altitude(alt);
        s.set_command(ControlCommand::default());
        let mut destroyed = false;
        for _ in 0..12000 {
            step_rocket(&mut s, DT);
            if s.destroyed {
                destroyed = true;
                break;
            }
        }
        assert!(destroyed, "expected destruction dropping from {alt} m");
        assert!(
            s.last_impact_speed >= s.params.crash_impact_speed,
            "impact={} threshold={}",
            s.last_impact_speed,
            s.params.crash_impact_speed
        );
    }
}

#[test]
fn soft_ground_contact_does_not_destroy() {
    let mut s = RocketState::at_altitude(3.0);
    s.velocity = [0.0, -1.5, 0.0];
    s.set_command(ControlCommand::default());
    for _ in 0..800 {
        step_rocket(&mut s, DT);
        assert!(
            !s.destroyed,
            "soft landing should survive, vy={} impact={}",
            s.velocity[1],
            s.last_impact_speed
        );
    }
}

#[test]
fn direct_high_speed_impact_triggers_explosion() {
    let mut s = RocketState::at_altitude(20.0);
    s.velocity = [0.0, -12.0, 0.0];
    s.set_command(ControlCommand::default());
    for _ in 0..500 {
        step_rocket(&mut s, DT);
        if s.destroyed {
            break;
        }
    }
    assert!(s.destroyed, "12 m/s descent should exceed 10 m/s crash threshold");
    assert!(s.last_impact_speed >= 10.0);
}
