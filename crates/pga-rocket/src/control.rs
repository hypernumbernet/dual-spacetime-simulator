//! Keyboard → engine/attitude command mapping (pure, testable without window I/O).

use crate::sim::ControlCommand;

/// Snapshot of held control keys for one frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct KeySnapshot {
    pub thrust_up: bool,
    pub thrust_down: bool,
    pub pitch_up: bool,
    pub pitch_down: bool,
    pub yaw_left: bool,
    pub yaw_right: bool,
    pub roll_left: bool,
    pub roll_right: bool,
    pub reset: bool,
}

/// Maps key state into incremental control updates.
///
/// - Space: raise throttle
/// - Left Ctrl / C: lower throttle
/// - W/S: pitch
/// - A/D: roll
/// - Q/E: yaw
/// - R: signal reset (caller applies)
#[derive(Clone, Debug)]
pub struct ControlMapper {
    /// Throttle units per second while thrust keys held.
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
        // Throttle: hold Space to increase, Ctrl/C to decrease; release leaves last value.
        let mut thr = self.command.throttle;
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
    w: bool,
    s: bool,
    a: bool,
    d: bool,
    q: bool,
    e: bool,
    r: bool,
) -> KeySnapshot {
    KeySnapshot {
        thrust_up: space,
        thrust_down,
        pitch_up: w,
        pitch_down: s,
        // A/D ↔ Q/E swapped relative to classic FPS layout: A/D roll, Q/E yaw.
        yaw_left: q,
        yaw_right: e,
        roll_left: a,
        roll_right: d,
        reset: r,
    }
}
