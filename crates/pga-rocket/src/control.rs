//! Keyboard → engine/attitude command mapping (pure, testable without window I/O).

use crate::sim::ControlCommand;

/// Time (s) for F-key full-throttle ramp from 0 → 1.
pub const FULL_THROTTLE_RAMP_S: f64 = 0.5;

/// Snapshot of held control keys for one frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct KeySnapshot {
    pub thrust_up: bool,
    pub thrust_down: bool,
    /// Hold F: ramp throttle to full in [`FULL_THROTTLE_RAMP_S`] from zero.
    pub thrust_full: bool,
    pub pitch_up: bool,
    pub pitch_down: bool,
    pub yaw_left: bool,
    pub yaw_right: bool,
    pub roll_left: bool,
    pub roll_right: bool,
    pub reset: bool,
    /// Toggle automatic landing mode (L key, edge-triggered).
    pub toggle_landing: bool,
    /// Toggle target-pad autopilot (T key, edge-triggered).
    pub toggle_target_landing: bool,
}

/// Maps key state into incremental control updates.
///
/// - Space: raise throttle
/// - F: ramp to full throttle in 500 ms (from zero)
/// - Left Ctrl / C: lower throttle
/// - W/S: pitch (main-engine gimbal about body +X; needs thrust)
/// - Q/E: yaw (main-engine gimbal about body +Z; needs thrust)
/// - A/D: roll (four center RCS thrusters about body +Y; works on the pad too)
/// - R: signal reset (caller applies)
/// - L: signal landing toggle (caller applies)
/// - T: signal target-pad landing toggle (caller applies)
#[derive(Clone, Debug)]
pub struct ControlMapper {
    /// Throttle units per second while Space / Ctrl thrust keys held.
    pub throttle_rate: f64,
    /// Current command (persistent throttle).
    pub command: ControlCommand,
}

impl Default for ControlMapper {
    fn default() -> Self {
        Self {
            throttle_rate: 0.45,
            command: ControlCommand::default(),
        }
    }
}

impl ControlMapper {
    /// Apply held keys over `dt` and return the resulting command.
    pub fn apply(&mut self, keys: &KeySnapshot, dt: f64) -> ControlCommand {
        // Throttle: Space up, F full ramp, Ctrl/C down; release leaves last value.
        let mut thr = self.command.throttle;
        if keys.thrust_full {
            // 0 → 1 in FULL_THROTTLE_RAMP_S; partial start reaches 1 sooner.
            thr += dt / FULL_THROTTLE_RAMP_S;
        }
        if keys.thrust_up {
            thr += self.throttle_rate * dt;
        }
        if keys.thrust_down {
            thr -= self.throttle_rate * dt;
        }
        thr = thr.clamp(0.0, 1.0);

        // Attitude is momentary (spring to zero when keys released).
        let pitch = axis(keys.pitch_up, keys.pitch_down);
        let yaw = axis(keys.yaw_right, keys.yaw_left);
        let roll = axis(keys.roll_right, keys.roll_left);

        self.command = ControlCommand {
            throttle: thr,
            pitch,
            yaw,
            roll,
        }
        .clamp();
        self.command
    }

    /// Zero throttle and attitude.
    #[allow(dead_code)]
    pub fn cut_engines(&mut self) {
        self.command = ControlCommand::default();
    }
}

fn axis(pos: bool, neg: bool) -> f64 {
    match (pos, neg) {
        (true, false) => 1.0,
        (false, true) => -1.0,
        _ => 0.0,
    }
}

/// Build a key snapshot from boolean flags (mirrors winit KeyCode mapping).
pub fn map_keys(
    space: bool,
    thrust_down: bool,
    f: bool,
    w: bool,
    s: bool,
    a: bool,
    d: bool,
    q: bool,
    e: bool,
    r: bool,
    l: bool,
    t: bool,
) -> KeySnapshot {
    KeySnapshot {
        thrust_up: space,
        thrust_down,
        thrust_full: f,
        pitch_up: w,
        pitch_down: s,
        // A/D ↔ Q/E swapped relative to classic FPS layout: A/D roll, Q/E yaw.
        yaw_left: q,
        yaw_right: e,
        roll_left: a,
        roll_right: d,
        reset: r,
        toggle_landing: l,
        toggle_target_landing: t,
    }
}
