//! Integration tests for automatic landing autopilot.

use pga_rocket::euclidean_pga::{motor_body_up_world, motor_from_pose};
use pga_rocket::landing::LandingAutopilot;
use pga_rocket::sim::{RocketState, step_rocket};

const DT: f64 = 1.0 / 120.0;

fn tilt_of(state: &RocketState) -> f64 {
    let up = motor_body_up_world(&state.motor);
    up[1].clamp(-1.0, 1.0).acos()
}

fn run_landing(mut state: RocketState, steps: usize) -> (RocketState, LandingAutopilot) {
    let mut ap = LandingAutopilot::default();
    ap.enabled = true;
    for _ in 0..steps {
        let cmd = ap.update(&state, DT);
        state.set_command(cmd);
        step_rocket(&mut state, DT);
    }
    (state, ap)
}

fn run_landing_impulse(mut state: RocketState, steps: usize) -> (RocketState, LandingAutopilot, f64) {
    let mut ap = LandingAutopilot::default();
    ap.enabled = true;
    let mut impulse = 0.0;
    let mut coast_steps = 0usize;
    for _ in 0..steps {
        let cmd = ap.update(&state, DT);
        impulse += cmd.throttle * DT;
        if cmd.throttle < 0.05 {
            coast_steps += 1;
        }
        state.set_command(cmd);
        step_rocket(&mut state, DT);
        if ap.complete {
            break;
        }
    }
    assert!(
        coast_steps > 60,
        "expected a free-fall coast phase, coast_steps={coast_steps}"
    );
    (state, ap, impulse)
}

#[test]
fn autopilot_uprights_tilted_hover() {
    let mut state = RocketState::at_altitude(60.0);
    state.motor = motor_from_pose(0.0, 60.0, 0.0, 0.35, 0.0, 0.0);
    state.contacting = false;

    let (state, ap) = run_landing(state, 8 * 120);
    let tilt = tilt_of(&state);

    assert!(tilt < 0.12, "expected near-vertical attitude, tilt={tilt:.3} rad");
    assert!(state.altitude() > 10.0);
    assert!(!ap.complete);
    assert!(ap.attitude_locked || tilt < 0.10);
}

#[test]
fn autopilot_uprights_from_roll() {
    let mut state = RocketState::at_altitude(55.0);
    state.motor = motor_from_pose(0.0, 55.0, 0.0, 0.0, 0.0, 0.55);
    state.contacting = false;

    let (state, _) = run_landing(state, 10 * 120);
    let tilt = tilt_of(&state);
    assert!(tilt < 0.12, "expected upright from roll, tilt={tilt:.3}");
}

#[test]
fn autopilot_uprights_combined_attitude() {
    let mut state = RocketState::at_altitude(55.0);
    state.motor = motor_from_pose(0.0, 55.0, 0.0, 0.45, 0.25, 0.35);
    state.velocity = [1.0, -0.5, -0.8];
    state.contacting = false;

    let (state, _) = run_landing(state, 12 * 120);
    let tilt = tilt_of(&state);
    assert!(tilt < 0.15, "expected upright from combined tilt, tilt={tilt:.3}");
    assert!(state.altitude() > 8.0);
}

#[test]
fn autopilot_lands_gently_from_altitude() {
    let mut state = RocketState::at_altitude(22.0);
    state.velocity = [0.0, -0.8, 0.0];
    state.contacting = false;

    let (state, ap) = run_landing(state, 70 * 120);
    let tilt = tilt_of(&state);

    assert!(
        state.contacting,
        "expected ground contact, lowest_y={}",
        state.lowest_foot_y()
    );
    assert!(
        ap.complete || (tilt < 0.15 && state.velocity[1].abs() < 1.2),
        "expected gentle landing, complete={} tilt={tilt:.3} vy={}",
        ap.complete,
        state.velocity[1]
    );
    assert!(state.lowest_probe_y() >= -0.05);
}

#[test]
fn autopilot_recovers_tilt_then_lands() {
    let mut state = RocketState::at_altitude(40.0);
    state.motor = motor_from_pose(0.0, 40.0, 0.0, 0.40, 0.15, 0.20);
    state.velocity = [0.5, -0.5, -0.3];
    state.contacting = false;

    let (state, ap) = run_landing(state, 90 * 120);
    let tilt = tilt_of(&state);
    assert!(
        state.contacting || ap.attitude_locked,
        "expected landing or upright lock, h={} tilt={tilt:.3} locked={}",
        state.lowest_foot_y(),
        ap.attitude_locked
    );
    assert!(tilt < 0.25, "should not remain heavily tilted, tilt={tilt:.3}");
    assert!(!state.body_contacting || state.contacting);
}

#[test]
fn autopilot_coasts_then_lands_from_high_altitude() {
    // CoM 80 m ⇒ foot clearance ≈ 63 m: long enough for a real suicide-burn coast.
    let mut state = RocketState::at_altitude(80.0);
    state.velocity = [0.0, -1.0, 0.0];
    state.contacting = false;

    let (state, ap, impulse) = run_landing_impulse(state, 45 * 120);
    let tilt = tilt_of(&state);

    assert!(
        state.contacting || ap.complete,
        "expected landing, h={} tilt={tilt:.3}",
        state.lowest_foot_y()
    );
    assert!(tilt < 0.2, "tilt={tilt:.3}");
    // Hovering down the whole way for ~25 s would cost ≳ 8 throttle·s; coast+burn is far less.
    assert!(
        impulse < 6.5,
        "expected fuel-efficient impulse, got {impulse:.2} throttle·s"
    );
}

#[test]
fn landing_toggle_rearms() {
    let mut ap = LandingAutopilot::default();
    ap.enabled = true;
    ap.complete = true;
    ap.toggle();
    assert!(!ap.enabled);
    ap.toggle();
    assert!(ap.enabled);
    assert!(!ap.complete);
}
