//! Keyboard → engine/attitude command mapping (pure, testable without window I/O).

use crate::sim::ControlCommand;

/// Time (s) for F / C latch ramps: 0 → 1 (F) or 1 → 0 (C) from the far end.
pub const THROTTLE_LATCH_RAMP_S: f64 = 0.2;

/// Backward-compatible alias for the full-throttle ramp duration.
pub const FULL_THROTTLE_RAMP_S: f64 = THROTTLE_LATCH_RAMP_S;

/// One-shot throttle latch started by F (full) or C (cut). Continues after key release.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ThrottleLatch {
    #[default]
    None,
    /// Ramp toward 1.0 at `1 / THROTTLE_LATCH_RAMP_S` per second.
    ToFull,
    /// Ramp toward 0.0 at the same rate.
    ToZero,
}

/// Snapshot of held / edge-triggered control keys for one frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct KeySnapshot {
    /// Space held: raise throttle while held.
    pub thrust_up: bool,
    /// Ctrl held: lower throttle while held.
    pub thrust_down: bool,
    /// F just pressed: start latched ramp to full (continues after release).
    pub thrust_full: bool,
    /// C just pressed: start latched ramp to zero (continues after release).
    pub thrust_cut: bool,
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
/// - Space: raise throttle while held
/// - Ctrl: lower throttle while held
/// - F: latch ramp to full in [`THROTTLE_LATCH_RAMP_S`] (no hold needed)
/// - C: latch ramp to off in the same duration (no hold needed)
/// - W/S: pitch (main-engine gimbal about body +X; needs thrust)
/// - Q/E: yaw (main-engine gimbal about body +Z; needs thrust)
/// - A/D: roll (four center RCS thrusters about body +Y; works on the pad too)
/// - R: signal reset (caller applies)
/// - L: signal landing toggle (caller applies)
/// - T: signal target-pad landing toggle (caller applies)
#[derive(Clone, Debug)]
pub struct ControlMapper {
    /// Throttle units per second while Space / Ctrl held.
    pub throttle_rate: f64,
    /// Current command (persistent throttle).
    pub command: ControlCommand,
    /// Active one-shot F/C ramp (continues after key release).
    pub throttle_latch: ThrottleLatch,
}

impl Default for ControlMapper {
    fn default() -> Self {
        Self {
            throttle_rate: 0.45,
            command: ControlCommand::default(),
            throttle_latch: ThrottleLatch::None,
        }
    }
}

impl ControlMapper {
    /// Apply held keys over `dt` and return the resulting command.
    pub fn apply(&mut self, keys: &KeySnapshot, dt: f64) -> ControlCommand {
        // Edge-triggered latches (F / C): start ramp; the other latch cancels this one.
        if keys.thrust_full {
            self.throttle_latch = ThrottleLatch::ToFull;
        }
        if keys.thrust_cut {
            self.throttle_latch = ThrottleLatch::ToZero;
        }

        let mut thr = self.command.throttle;
        let ramp = dt / THROTTLE_LATCH_RAMP_S.max(1e-6);

        match self.throttle_latch {
            ThrottleLatch::ToFull => {
                thr += ramp;
                if thr >= 1.0 {
                    thr = 1.0;
                    self.throttle_latch = ThrottleLatch::None;
                }
            }
            ThrottleLatch::ToZero => {
                thr -= ramp;
                if thr <= 0.0 {
                    thr = 0.0;
                    self.throttle_latch = ThrottleLatch::None;
                }
            }
            ThrottleLatch::None => {}
        }

        // Hold keys: Space up, Ctrl down (release leaves last value).
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

    /// Zero throttle and attitude; clear any F/C latch.
    #[allow(dead_code)]
    pub fn cut_engines(&mut self) {
        self.command = ControlCommand::default();
        self.throttle_latch = ThrottleLatch::None;
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
///
/// `f` / `c` are edge presses (full / cut latch); `space` / `thrust_down` are holds.
pub fn map_keys(
    space: bool,
    thrust_down: bool,
    f: bool,
    c: bool,
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
        thrust_cut: c,
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
