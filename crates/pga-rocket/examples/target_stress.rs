//! Stress harness for the T-key target autopilot: several start poses vs targets
//! from 500 m to 8 km. Reports flight time, max altitude, climb throttle, and
//! descent attitude sway. Pass a scenario name for a 0.25 s trace.

use pga_rocket::euclidean_pga::{motor_body_up_world, motor_from_pose};
use pga_rocket::sim::{RocketState, step_rocket};
use pga_rocket::target_landing::{
    inside_target_pad, TargetLandingAutopilot, TargetPhase,
};

const DT: f64 = 1.0 / 120.0;
/// Long-range (6 km) full-throttle cruise needs a longer wall-clock budget.
const MAX_T: f64 = 400.0;

struct Scenario {
    name: &'static str,
    start: [f64; 2],
    alt: Option<f64>,
    target: [f64; 2],
}

fn main() {
    let trace_name = std::env::args().nth(1);
    let scenarios = vec![
        Scenario { name: "pad_500x", start: [0.0, 0.0], alt: None, target: [500.0, 0.0] },
        Scenario { name: "pad_500diag", start: [0.0, 0.0], alt: None, target: [354.0, 354.0] },
        Scenario { name: "pad_800x", start: [0.0, 0.0], alt: None, target: [800.0, 0.0] },
        Scenario { name: "pad_overhead", start: [500.0, 0.0], alt: None, target: [500.0, 0.0] },
        Scenario { name: "high_600_off400", start: [0.0, 0.0], alt: Some(600.0), target: [400.0, 0.0] },
        Scenario { name: "mid_250_500x", start: [0.0, 0.0], alt: Some(250.0), target: [500.0, 0.0] },
        // Long-range full-throttle airplane cruise at LONG_CRUISE_ALT_M (~520 m; range ≳ 1.5 km).
        Scenario { name: "pad_6000x", start: [0.0, 0.0], alt: None, target: [6000.0, 0.0] },
        Scenario { name: "pad_8000x", start: [0.0, 0.0], alt: None, target: [8000.0, 0.0] },
    ];

    let mut fails = 0;
    for sc in &scenarios {
        let mut state = match sc.alt {
            Some(a) => {
                let mut s = RocketState::at_altitude(a);
                s.contacting = false;
                s
            }
            None => RocketState::resting_on_pad(),
        };
        let y0 = state.position()[1];
        state.motor = motor_from_pose(sc.start[0], y0, sc.start[1], 0.0, 0.0, 0.0);

        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;

        let tracing = trace_name.as_deref() == Some(sc.name);
        let mut next_trace = 0.0;
        let mut t = 0.0;
        let mut max_alt = state.altitude();
        let mut impulse = 0.0;
        // Climb metrics: mean throttle while below 480 m and ascending.
        let mut climb_thr_sum = 0.0;
        let mut climb_steps = 0u32;
        // Descent sway metrics: tilt stats + zero-crossings of tilt-plane swing.
        let mut t_arm = f64::NAN;
        let mut descend_started = false;
        let mut max_tilt_descent: f64 = 0.0;
        let mut sway_reversals = 0u32;
        let mut prev_wx_sign = 0i8;
        while t < MAX_T {
            let cmd = ap.update(&state, sc.target, DT);
            impulse += cmd.throttle * DT;

            let pos = state.position();
            let up = motor_body_up_world(&state.motor);
            let tilt = up[1].clamp(-1.0, 1.0).acos();
            if pos[1] < 480.0 && state.velocity[1] > 0.5 && ap.phase == TargetPhase::Climb {
                climb_thr_sum += cmd.throttle;
                climb_steps += 1;
            }
            if ap.phase == TargetPhase::Descend && !state.contacting {
                if !descend_started {
                    t_arm = t;
                }
                descend_started = true;
                max_tilt_descent = max_tilt_descent.max(tilt);
                // Count pendulum reversals: sign flips of pitch-plane body rate
                // while meaningfully tilted.
                let s = if state.omega[0] > 0.05 { 1 } else if state.omega[0] < -0.05 { -1 } else { 0 };
                if s != 0 && prev_wx_sign != 0 && s != prev_wx_sign && tilt > 0.03 {
                    sway_reversals += 1;
                }
                if s != 0 {
                    prev_wx_sign = s;
                }
            }

            if tracing && t >= next_trace {
                println!(
                    "t={:6.2} x={:7.1} z={:7.1} alt={:7.1} vy={:7.2} vh={:6.2} tilt={:5.3} w=({:5.2},{:5.2},{:5.2}) thr={:4.2} {:?}",
                    t, pos[0], pos[2], pos[1], state.velocity[1],
                    (state.velocity[0] * state.velocity[0] + state.velocity[2] * state.velocity[2]).sqrt(),
                    tilt, state.omega[0], state.omega[1], state.omega[2],
                    cmd.throttle, ap.phase
                );
                next_trace += 0.25;
            }

            state.set_command(cmd);
            step_rocket(&mut state, DT);
            t += DT;
            max_alt = max_alt.max(state.altitude());
            if state.destroyed || ap.complete {
                break;
            }
        }

        let p = state.position();
        let on_target = inside_target_pad(p, sc.target);
        let center_err = (p[0] - sc.target[0]).abs().max((p[2] - sc.target[1]).abs());
        // Survival: painted-pad touchdown + complete (center err is tuning-only).
        let ok = !state.destroyed && ap.complete && on_target;
        if !ok {
            fails += 1;
        }
        let mean_climb_thr = if climb_steps > 0 { climb_thr_sum / climb_steps as f64 } else { 0.0 };
        println!(
            "{:<16} {} t={:6.1}s arm={:6.1}s max_alt={:6.1} climb_thr={:4.2} impulse={:6.1} desc_tilt_max={:5.3} sway_rev={:3} pos=({:6.1},{:6.1}) err={:5.1} {}{}",
            sc.name,
            if state.destroyed { "DESTROYED" } else if ap.complete { "landed   " } else { "timeout  " },
            t,
            t_arm,
            max_alt,
            mean_climb_thr,
            impulse,
            if descend_started { max_tilt_descent } else { f64::NAN },
            sway_reversals,
            p[0], p[2],
            center_err,
            if on_target { "on-pad" } else { "OFF-PAD" },
            if ok { "" } else { "  <-- FAIL" },
        );
    }
    println!("\n{} / {} scenarios failed", fails, scenarios.len());
}
