//! Keyboard → engine/attitude command mapping (pure, testable without window I/O).

use crate::sim::ControlCommand;

/// Time (s) for F / C latch ramps across a full 0 ↔ 1 span.
pub const THROTTLE_LATCH_RAMP_S: f64 = 0.2;

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
    /// Toggle Moon mode (M key, edge-triggered).
    pub toggle_moon_mode: bool,
}

/// Maps key state into incremental control updates.
///
/// - Space / Ctrl: hold to raise / lower throttle
/// - F / C: edge-triggered latch ramps (full / cut) over [`THROTTLE_LATCH_RAMP_S`]
/// - W/S pitch, Q/E yaw, A/D roll; R / L / T / M are edge signals for the app
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
    /// Apply held / edge keys over `dt` and return the resulting command.
    pub fn apply(&mut self, keys: &KeySnapshot, dt: f64) -> ControlCommand {
        // Edge latches: C wins if both fire in the same frame.
        if keys.thrust_full {
            self.throttle_latch = ThrottleLatch::ToFull;
        }
        if keys.thrust_cut {
            self.throttle_latch = ThrottleLatch::ToZero;
        }
        // Opposite hold cancels a latch so Space/Ctrl stay predictable.
        if keys.thrust_up && self.throttle_latch == ThrottleLatch::ToZero {
            self.throttle_latch = ThrottleLatch::None;
        }
        if keys.thrust_down && self.throttle_latch == ThrottleLatch::ToFull {
            self.throttle_latch = ThrottleLatch::None;
        }

        let mut thr = self.command.throttle;
        thr = step_throttle_latch(&mut self.throttle_latch, thr, dt);

        if keys.thrust_up {
            thr += self.throttle_rate * dt;
        }
        if keys.thrust_down {
            thr -= self.throttle_rate * dt;
        }
        thr = thr.clamp(0.0, 1.0);

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

    /// Take over throttle from an external source (L/T autopilot). Clears F/C latches.
    pub fn adopt_throttle(&mut self, throttle: f64) {
        self.command.throttle = throttle.clamp(0.0, 1.0);
        self.throttle_latch = ThrottleLatch::None;
    }
}

/// Advance a latch toward 0 or 1; clears the latch when the target is reached.
fn step_throttle_latch(latch: &mut ThrottleLatch, thr: f64, dt: f64) -> f64 {
    let target = match *latch {
        ThrottleLatch::ToFull => 1.0,
        ThrottleLatch::ToZero => 0.0,
        ThrottleLatch::None => return thr,
    };
    let rate = 1.0 / THROTTLE_LATCH_RAMP_S.max(1e-6);
    let next = if target >= thr {
        (thr + rate * dt).min(target)
    } else {
        (thr - rate * dt).max(target)
    };
    if (next - target).abs() < 1e-12 {
        *latch = ThrottleLatch::None;
    }
    next
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
    m: bool,
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
        toggle_moon_mode: m,
    }
}

#[cfg(test)]
mod unit_tests {
    use super::*;

    #[test]
    fn latch_step_hits_target_and_clears() {
        let mut latch = ThrottleLatch::ToFull;
        let thr = step_throttle_latch(&mut latch, 0.0, THROTTLE_LATCH_RAMP_S);
        assert!((thr - 1.0).abs() < 1e-12);
        assert_eq!(latch, ThrottleLatch::None);
    }
}
