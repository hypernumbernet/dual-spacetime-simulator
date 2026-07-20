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
    // Attitude-only check: Moon mode (no drag) so hover recovery is not slowed by aero.
    state.moon_mode = true;
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
    // The autopilot may lean deliberately mid-descent to kill drift, so assert
    // the outcome — an intact upright touchdown — rather than transient tilt.
    let mut state = RocketState::at_altitude(55.0);
    state.motor = motor_from_pose(0.0, 55.0, 0.0, 0.0, 0.0, 0.55);
    state.contacting = false;

    let (state, ap) = run_landing(state, 45 * 120);
    let tilt = tilt_of(&state);
    assert!(!state.destroyed, "impact={}", state.last_impact_speed);
    assert!(
        ap.complete,
        "expected touchdown from roll tilt, tilt={tilt:.3} h={}",
        state.lowest_foot_y()
    );
    assert!(tilt < 0.12, "expected upright at rest, tilt={tilt:.3}");
}

#[test]
fn autopilot_uprights_combined_attitude() {
    let mut state = RocketState::at_altitude(55.0);
    state.motor = motor_from_pose(0.0, 55.0, 0.0, 0.45, 0.25, 0.35);
    state.velocity = [1.0, -0.5, -0.8];
    state.contacting = false;

    let (state, ap) = run_landing(state, 45 * 120);
    let tilt = tilt_of(&state);
    assert!(!state.destroyed, "impact={}", state.last_impact_speed);
    assert!(
        ap.complete,
        "expected touchdown from combined tilt, tilt={tilt:.3} h={}",
        state.lowest_foot_y()
    );
    assert!(tilt < 0.12, "expected upright at rest, tilt={tilt:.3}");
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
fn autopilot_survives_sideways_attitude() {
    // 90° tilt at moderate altitude used to end in a crash; the flip + brake
    // channels must recover it to an intact upright touchdown.
    let mut state = RocketState::at_altitude(60.0);
    state.motor = motor_from_pose(0.0, 60.0, 0.0, 1.57, 0.0, 0.0);
    state.velocity = [0.0, -2.0, 0.0];
    state.contacting = false;

    let (state, ap) = run_landing(state, 60 * 120);
    assert!(
        !state.destroyed,
        "sideways start must not explode, impact={}",
        state.last_impact_speed
    );
    assert!(ap.complete, "expected touchdown, h={}", state.lowest_foot_y());
}

#[test]
fn autopilot_survives_inverted_attitude() {
    // Fully inverted needs a fast flip, an idle coast, and an early hard brake.
    // (Below ~150 m CoM there is no survivable trajectory at T/W = 3.)
    let mut state = RocketState::at_altitude(170.0);
    state.motor = motor_from_pose(0.0, 170.0, 0.0, 3.1, 0.0, 0.0);
    state.contacting = false;

    let (state, ap) = run_landing(state, 60 * 120);
    assert!(
        !state.destroyed,
        "inverted start must not explode, impact={}",
        state.last_impact_speed
    );
    assert!(ap.complete, "expected touchdown, h={}", state.lowest_foot_y());
}

#[test]
fn autopilot_survives_fast_fall_with_tilt() {
    // A tilted fast fall must brake on the envelope even before the attitude
    // has settled (the old attitude-phase gate caused free-fall into the pad).
    let mut state = RocketState::at_altitude(80.0);
    state.motor = motor_from_pose(0.0, 80.0, 0.0, 0.5, 0.0, 0.0);
    state.velocity = [0.0, -30.0, 0.0];
    state.contacting = false;

    let (state, ap) = run_landing(state, 45 * 120);
    assert!(
        !state.destroyed,
        "fast tilted fall must not explode, impact={}",
        state.last_impact_speed
    );
    assert!(ap.complete, "expected touchdown, h={}", state.lowest_foot_y());
}

#[test]
fn autopilot_survives_tumbling_entry() {
    let mut state = RocketState::at_altitude(100.0);
    state.motor = motor_from_pose(0.0, 100.0, 0.0, 1.2, 0.5, 0.8);
    state.velocity = [3.0, -8.0, -3.0];
    state.omega = [0.8, 0.4, -0.7];
    state.contacting = false;

    let (state, ap) = run_landing(state, 60 * 120);
    assert!(
        !state.destroyed,
        "tumbling entry must not explode, impact={}",
        state.last_impact_speed
    );
    assert!(ap.complete, "expected touchdown, h={}", state.lowest_foot_y());
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

#[test]
fn l_mode_brake_upright_counter_limits_pendulum() {
    let mut state = RocketState::at_altitude(60.0);
    state.motor = motor_from_pose(0.0, 60.0, 0.0, 0.25, 0.0, 0.0);
    state.velocity = [6.0, -4.0, -3.0];
    state.contacting = false;

    let mut ap = LandingAutopilot::default();
    ap.enabled = true;

    let mut sway_reversals = 0u32;
    let mut prev_wx_sign = 0i32;
    let mut saw_upright_counter = false;
    let mut max_throttle_on_envelope = 0.0_f64;

    for _ in 0..(45 * 120) {
        let cmd = ap.update(&state, DT);
        let tilt = tilt_of(&state);
        let vh = (state.velocity[0] * state.velocity[0]
            + state.velocity[2] * state.velocity[2])
            .sqrt();

        let s = if state.omega[0] > 0.05 {
            1
        } else if state.omega[0] < -0.05 {
            -1
        } else {
            0
        };
        if s != 0 && prev_wx_sign != 0 && s != prev_wx_sign && tilt > 0.04 {
            sway_reversals += 1;
        }
        if s != 0 {
            prev_wx_sign = s;
        }

        // Upright counter: brake done, still recovering attitude.
        if vh < 3.0 && tilt > 0.06 && tilt < 0.35 {
            saw_upright_counter = true;
        }

        // Descent must not wait for attitude settle.
        if state.lowest_foot_y() < 50.0 && cmd.throttle > max_throttle_on_envelope {
            max_throttle_on_envelope = cmd.throttle;
        }

        state.set_command(cmd);
        step_rocket(&mut state, DT);
        if ap.complete {
            break;
        }
    }

    assert!(
        saw_upright_counter,
        "expected brake→upright counter phase during lateral decel"
    );
    assert!(
        sway_reversals <= 12,
        "pendulum reversals should stay bounded, got {sway_reversals}"
    );
    assert!(
        max_throttle_on_envelope > 0.5,
        "descent must not wait for attitude settle, max_thr={max_throttle_on_envelope}"
    );
}

#[test]
fn l_mode_lateral_decel_lands_intact() {
    let mut state = RocketState::at_altitude(55.0);
    state.motor = motor_from_pose(0.0, 55.0, 0.0, 0.30, 0.10, 0.0);
    state.velocity = [5.0, -3.0, 2.0];
    state.contacting = false;

    let (state, ap) = run_landing(state, 50 * 120);
    assert!(!state.destroyed, "impact={}", state.last_impact_speed);
    assert!(ap.complete, "expected touchdown, h={}", state.lowest_foot_y());
    assert!(tilt_of(&state) < 0.15, "expected near-upright rest");
}
