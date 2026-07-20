//! Integration tests for T-key target-pad autopilot (simultaneous climb + go).

use pga_rocket::euclidean_pga::motor_from_pose;
use pga_rocket::sim::{RocketState, step_rocket};
use pga_rocket::target_landing::{
    inside_target_pad, CLIMB_ALT_M, TargetLandingAutopilot, TargetPhase, TARGET_PAD_HALF_M,
};

const DT: f64 = 1.0 / 120.0;

fn run_target(
    mut state: RocketState,
    target: [f64; 2],
    steps: usize,
) -> (RocketState, TargetLandingAutopilot, f64) {
    let mut ap = TargetLandingAutopilot::default();
    ap.enabled = true;
    let mut max_alt = state.altitude();
    for _ in 0..steps {
        if state.destroyed || ap.complete {
            break;
        }
        let cmd = ap.update(&state, target, DT);
        state.set_command(cmd);
        step_rocket(&mut state, DT);
        max_alt = max_alt.max(state.altitude());
    }
    (state, ap, max_alt)
}

#[test]
fn target_toggle_rearms() {
    let mut ap = TargetLandingAutopilot::default();
    ap.enabled = true;
    ap.complete = true;
    ap.toggle();
    assert!(!ap.enabled);
    ap.toggle();
    assert!(ap.enabled);
    assert!(!ap.complete);
}

#[test]
fn low_altitude_climbs_above_500m_and_lands_on_target() {
    // Start on the launch pad; T mark ~500 m downrange on +X.
    let state = RocketState::resting_on_pad();
    let target = [500.0, 0.0];
    let (state, ap, max_alt) = run_target(state, target, 8 * 60 * 120);

    assert!(
        !state.destroyed,
        "must not explode, impact={}",
        state.last_impact_speed
    );
    assert!(
        max_alt >= CLIMB_ALT_M * 0.96,
        "must roughly reach ~{CLIMB_ALT_M} m before terminal descent, max_alt={max_alt}"
    );
    assert!(
        ap.complete || (state.contacting && state.lowest_foot_y() < 1.0),
        "expected landing complete, phase={:?} h={} complete={}",
        ap.phase,
        state.lowest_foot_y(),
        ap.complete
    );
    let p = state.position();
    assert!(
        inside_target_pad(p, target),
        "should land on painted target pad (±{TARGET_PAD_HALF_M} m), pos=({:.1},{:.1})",
        p[0],
        p[2]
    );
}

/// While still below 500 m, the rocket must already be translating toward the pad
/// (climb and go are simultaneous, not sequential).
#[test]
fn climb_and_translate_happen_together() {
    let mut state = RocketState::resting_on_pad();
    let target = [500.0, 0.0];
    let mut ap = TargetLandingAutopilot::default();
    ap.enabled = true;

    let x0 = state.position()[0];
    let mut max_x_below_gate = x0;
    let mut min_alt_when_x_over_80 = f64::INFINITY;
    let mut saw_progress_below_500 = false;

    for _ in 0..(3 * 60 * 120) {
        if state.destroyed || ap.complete {
            break;
        }
        let cmd = ap.update(&state, target, DT);
        state.set_command(cmd);
        step_rocket(&mut state, DT);

        let p = state.position();
        if p[1] < CLIMB_ALT_M {
            max_x_below_gate = max_x_below_gate.max(p[0]);
            if p[0] - x0 > 80.0 {
                saw_progress_below_500 = true;
                min_alt_when_x_over_80 = min_alt_when_x_over_80.min(p[1]);
            }
        } else if saw_progress_below_500 {
            break;
        }
    }

    assert!(
        saw_progress_below_500,
        "must move downrange while still below {CLIMB_ALT_M} m (max_x_below={max_x_below_gate:.1})"
    );
    assert!(
        min_alt_when_x_over_80 < CLIMB_ALT_M,
        "horizontal progress should start before clearing 500 m, alt={min_alt_when_x_over_80:.1}"
    );
    assert!(!state.destroyed, "impact={}", state.last_impact_speed);
}

#[test]
fn high_altitude_skips_climb_label_and_lands() {
    let mut state = RocketState::at_altitude(600.0);
    state.velocity = [0.0, 0.0, 0.0];
    state.contacting = false;
    let target = [400.0, 0.0];

    let mut ap = TargetLandingAutopilot::default();
    ap.enabled = true;
    let _ = ap.update(&state, target, DT);
    assert_eq!(ap.phase, TargetPhase::Cruise);

    let mut max_alt = state.altitude();
    for _ in 0..(8 * 60 * 120) {
        if state.destroyed || ap.complete {
            break;
        }
        let cmd = ap.update(&state, target, DT);
        state.set_command(cmd);
        step_rocket(&mut state, DT);
        max_alt = max_alt.max(state.altitude());
    }

    assert!(!state.destroyed, "impact={}", state.last_impact_speed);
    assert!(max_alt < 900.0, "unexpected high loft max_alt={max_alt}");
    assert!(
        ap.complete || state.contacting,
        "expected landing, phase={:?} h={}",
        ap.phase,
        state.lowest_foot_y()
    );
    let p = state.position();
    assert!(
        inside_target_pad(p, target),
        "should land on painted target pad, pos=({:.1},{:.1})",
        p[0],
        p[2]
    );
}

#[test]
fn starts_near_target_still_clears_500m_when_low() {
    // Already above the pad horizontally but low altitude: must loft above 500 m
    // before terminal descent (may still translate slightly).
    let mut state = RocketState::resting_on_pad();
    state.motor = motor_from_pose(500.0, state.position()[1], 0.0, 0.0, 0.0, 0.0);
    let target = [500.0, 0.0];

    let mut ap = TargetLandingAutopilot::default();
    ap.enabled = true;
    let mut max_alt = state.altitude();
    for _ in 0..(4 * 60 * 120) {
        if state.destroyed || ap.complete {
            break;
        }
        let cmd = ap.update(&state, target, DT);
        state.set_command(cmd);
        step_rocket(&mut state, DT);
        max_alt = max_alt.max(state.altitude());
    }

    assert!(
        max_alt >= CLIMB_ALT_M * 0.96,
        "must roughly reach ~500 m before landing, max_alt={max_alt}"
    );
    assert!(!state.destroyed, "impact={}", state.last_impact_speed);
}

#[test]
fn target_descend_completes_without_hovering() {
    let state = RocketState::resting_on_pad();
    let target = [500.0, 0.0];
    let (state, ap, _) = run_target(state, target, 8 * 60 * 120);

    assert!(!state.destroyed, "impact={}", state.last_impact_speed);
    assert!(
        ap.complete,
        "must latch complete after pad settle, phase={:?} contacting={} h={}",
        ap.phase,
        state.contacting,
        state.lowest_foot_y()
    );
    assert!(
        state.contacting,
        "must rest on ground after complete, h={}",
        state.lowest_foot_y()
    );
    assert!(
        state.last_impact_speed < 4.0,
        "touchdown should be soft, impact={}",
        state.last_impact_speed
    );
    let up_y = pga_rocket::euclidean_pga::world_up_in_body(&state.motor)[1];
    assert!(
        up_y > (std::f64::consts::PI - 0.12).cos(),
        "should rest nearly upright, cos(tilt)={up_y}"
    );
}
