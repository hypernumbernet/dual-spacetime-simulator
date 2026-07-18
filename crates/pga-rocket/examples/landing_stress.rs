//! Stress harness: run the landing autopilot from many bad initial attitudes
//! and report which scenarios end in destruction. Not shipped as a test yet.

use pga_rocket::euclidean_pga::{motor_body_up_world, motor_from_pose};
use pga_rocket::landing::LandingAutopilot;
use pga_rocket::sim::{RocketState, step_rocket};

const DT: f64 = 1.0 / 120.0;
const MAX_T: f64 = 120.0;

struct Scenario {
    name: &'static str,
    alt: f64,
    pitch: f64,
    yaw: f64,
    roll: f64,
    vel: [f64; 3],
    omega: [f64; 3],
}

fn main() {
    let trace_name = std::env::args().nth(1);
    if trace_name.as_deref() == Some("--bench") {
        bench_update();
        return;
    }
    let scenarios = vec![
        Scenario { name: "upright_low", alt: 22.0, pitch: 0.0, yaw: 0.0, roll: 0.0, vel: [0.0, -0.8, 0.0], omega: [0.0; 3] },
        Scenario { name: "tilt35_60m", alt: 60.0, pitch: 0.35, yaw: 0.0, roll: 0.0, vel: [0.0, 0.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "tilt60_50m", alt: 50.0, pitch: 0.60, yaw: 0.0, roll: 0.0, vel: [0.0, -3.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "tilt90_60m", alt: 60.0, pitch: 1.57, yaw: 0.0, roll: 0.0, vel: [0.0, -2.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "tilt90_100m", alt: 100.0, pitch: 1.57, yaw: 0.0, roll: 0.0, vel: [0.0, -5.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "tilt120_80m", alt: 80.0, pitch: 2.1, yaw: 0.0, roll: 0.0, vel: [0.0, -2.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "inverted_120m", alt: 120.0, pitch: 3.1, yaw: 0.0, roll: 0.0, vel: [0.0, 0.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "inverted_160m", alt: 160.0, pitch: 3.1, yaw: 0.0, roll: 0.0, vel: [0.0, 0.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "inverted_200m", alt: 200.0, pitch: 3.1, yaw: 0.0, roll: 0.0, vel: [0.0, -5.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "tilt45_spin", alt: 70.0, pitch: 0.8, yaw: 0.3, roll: 0.4, vel: [2.0, -4.0, -1.5], omega: [0.4, 0.6, -0.3] },
        Scenario { name: "fastfall_40m", alt: 40.0, pitch: 0.3, yaw: 0.0, roll: 0.0, vel: [0.0, -18.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "fastfall_80m", alt: 80.0, pitch: 0.5, yaw: 0.0, roll: 0.0, vel: [0.0, -30.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "fastfall_150m", alt: 150.0, pitch: 0.9, yaw: 0.2, roll: 0.1, vel: [4.0, -40.0, 2.0], omega: [0.2, 0.0, -0.2] },
        Scenario { name: "lateral_fast", alt: 60.0, pitch: 0.4, yaw: 0.0, roll: 0.0, vel: [15.0, -5.0, -10.0], omega: [0.0; 3] },
        Scenario { name: "tumble_100m", alt: 100.0, pitch: 1.2, yaw: 0.5, roll: 0.8, vel: [3.0, -8.0, -3.0], omega: [0.8, 0.4, -0.7] },
        Scenario { name: "tilt90_30m", alt: 30.0, pitch: 1.57, yaw: 0.0, roll: 0.0, vel: [0.0, -1.0, 0.0], omega: [0.0; 3] },
        Scenario { name: "tilt60_25m", alt: 25.0, pitch: 0.6, yaw: 0.0, roll: 0.0, vel: [0.0, -5.0, 0.0], omega: [0.0; 3] },
    ];

    let mut fails = 0;
    for sc in &scenarios {
        let mut state = RocketState::at_altitude(sc.alt);
        state.motor = motor_from_pose(0.0, sc.alt, 0.0, sc.pitch, sc.yaw, sc.roll);
        state.velocity = sc.vel;
        state.omega = sc.omega;
        state.contacting = false;

        let mut ap = LandingAutopilot::default();
        ap.enabled = true;
        let mut impulse = 0.0;
        let mut t = 0.0;
        let mut min_h_precontact = f64::INFINITY;
        let tracing = trace_name.as_deref() == Some(sc.name);
        let mut next_trace = 0.0;
        while t < MAX_T {
            let cmd = ap.update(&state, DT);
            impulse += cmd.throttle * DT;
            if tracing && t >= next_trace {
                let up = motor_body_up_world(&state.motor);
                let tilt = up[1].clamp(-1.0, 1.0).acos();
                println!(
                    "t={:6.2} h={:8.2} vy={:7.2} vx={:6.2} vz={:6.2} tilt={:5.2} w=({:5.2},{:5.2},{:5.2}) thr={:4.2} p={:5.2} y={:5.2} contact={}",
                    t, state.lowest_foot_y(), state.velocity[1], state.velocity[0], state.velocity[2],
                    tilt, state.omega[0], state.omega[1], state.omega[2],
                    cmd.throttle, cmd.pitch, cmd.yaw, state.contacting as u8
                );
                next_trace += 0.25;
            }
            state.set_command(cmd);
            step_rocket(&mut state, DT);
            t += DT;
            if !state.contacting {
                let h = state.lowest_foot_y();
                if h < min_h_precontact {
                    min_h_precontact = h;
                }
            }
            if state.destroyed || ap.complete {
                break;
            }
        }
        let up = motor_body_up_world(&state.motor);
        let tilt = up[1].clamp(-1.0, 1.0).acos();
        let ok = !state.destroyed && ap.complete;
        if !ok {
            fails += 1;
        }
        println!(
            "{:<16} {} t={:6.1}s impulse={:6.2} tilt={:5.2} vy={:6.2} h={:7.2} impact={:5.2} {}",
            sc.name,
            if state.destroyed { "DESTROYED" } else if ap.complete { "landed   " } else { "timeout  " },
            t,
            impulse,
            tilt,
            state.velocity[1],
            state.lowest_foot_y(),
            state.last_impact_speed,
            if ok { "" } else { "  <-- FAIL" },
        );
    }
    println!("\n{} / {} scenarios failed", fails, scenarios.len());
}

/// Micro-benchmark of `LandingAutopilot::update` in three attitude regimes.
fn bench_update() {
    let cases = [
        ("upright ", 0.0),
        ("lean 0.6", 0.6),
        ("flip 2.5", 2.5),
    ];
    for (name, pitch) in cases {
        let mut state = RocketState::at_altitude(80.0);
        state.motor = motor_from_pose(0.0, 80.0, 0.0, pitch, 0.0, 0.0);
        state.velocity = [3.0, -10.0, 2.0];
        state.contacting = false;
        let mut ap = LandingAutopilot::default();
        ap.enabled = true;
        let n = 200_000u32;
        let mut acc = 0.0;
        let start = std::time::Instant::now();
        for _ in 0..n {
            acc += ap.update(&state, 1.0 / 120.0).throttle;
        }
        let ns = start.elapsed().as_nanos() as f64 / n as f64;
        println!("{name}  {ns:7.0} ns/update  (checksum {acc:.1})");
    }
}
