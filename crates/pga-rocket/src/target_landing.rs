//! Target-pad autopilot (T key): lofted-waypoint transit, then land.
//!
//! Guidance aims at a world-space lofted pad waypoint (conceptually a PGA point
//! above the T mark). Free displacement from the CoM drives **simultaneous**
//! climb and downrange translate until the altitude gate, then horizontal
//! cruise with no further climb command. Terminal descent reuses
//! [`LandingAutopilot`] with pad-square success (Chebyshev half-extent).
//!
//! Attitude uses the same inverse sandwich transports as the L lander
//! (`motor_inverse_rotate_vector` / `world_up_in_body`).

use crate::euclidean_pga::{motor_inverse_rotate_vector, world_up_in_body};
use crate::landing::{
    axis_angle_from_cross, clamp_tilt, on_pad_square, saturate, LandingAutopilot, PAD_HALF_M,
};
use crate::sim::{ControlCommand, GRAVITY, RocketState};

/// Nominal loft altitude (m, CoM). Roughly reaching this is enough for the gate.
pub const CLIMB_ALT_M: f64 = 500.0;
/// Soft floor for “about 500 m”: hand-off / no-climb once CoM is at least this high.
const GATE_ALT_MIN: f64 = 480.0;
/// Target pad half-extent (m) — same painted square as the mesh pad.
pub const TARGET_PAD_HALF_M: f64 = PAD_HALF_M;
/// Chebyshev margin outside the pad when starting terminal descent (m).
const PAD_APPROACH_MARGIN_M: f64 = 8.0;
/// Outer approach band for slowing horizontal speed (m, Chebyshev).
const PAD_APPROACH_BAND_M: f64 = TARGET_PAD_HALF_M + PAD_APPROACH_MARGIN_M * 3.0;
/// Max lean cone half-angle during transit (rad).
const LEAN_TRANSIT_MAX: f64 = 0.38;
/// Max commanded ground speed during transit (m/s).
const V_CRUISE_MAX: f64 = 40.0;
/// Climb rate cap (m/s).
const VY_CLIMB_MAX: f64 = 38.0;
/// Minimum planned time-to-go for the sync schedule (s).
const T_SYNC_MIN: f64 = 10.0;
/// Attitude √-profile planning accel (rad/s²).
const ALPHA_PLAN: f64 = 0.5;
const OMEGA_MAX: f64 = 1.5;
const KP_ATT: f64 = 1.8;
const KD_ATT: f64 = 2.4;
const KD_ROLL: f64 = 1.6;
/// cos(TILT_AIM) ≈ cos(1.05) — flip regime without calling acos each frame.
const COS_TILT_AIM: f64 = 0.497571;

/// Guidance phase while the T-key autopilot is armed.
///
/// `Climb` and `Cruise` share the same transit controller; the split is HUD-only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TargetPhase {
    /// Below the altitude gate — climbing **and** translating together.
    Climb,
    /// Gate cleared — no climb command; finish the horizontal leg.
    Cruise,
    /// Terminal descent onto the pad.
    Descend,
}

/// Autopilot that lands on a world-XZ pad mark (T key).
#[derive(Clone, Debug)]
pub struct TargetLandingAutopilot {
    pub enabled: bool,
    pub complete: bool,
    pub phase: TargetPhase,
    /// Nested lander used only in [`TargetPhase::Descend`] (pre-tuned gains).
    lander: LandingAutopilot,
}

impl Default for TargetLandingAutopilot {
    fn default() -> Self {
        Self {
            enabled: false,
            complete: false,
            phase: TargetPhase::Climb,
            lander: LandingAutopilot::for_target_pad(),
        }
    }
}

impl TargetLandingAutopilot {
    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
        if self.enabled {
            self.complete = false;
            self.phase = TargetPhase::Climb;
            self.lander.disable();
        } else {
            self.lander.disable();
        }
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.complete = false;
        self.phase = TargetPhase::Climb;
        self.lander.disable();
    }

    pub fn status_label(&self) -> &'static str {
        if !self.enabled {
            "off"
        } else if self.complete {
            "complete"
        } else {
            match self.phase {
                TargetPhase::Climb => "climb+go",
                TargetPhase::Cruise => "cruise",
                TargetPhase::Descend => "descend",
            }
        }
    }

    pub fn update(
        &mut self,
        state: &RocketState,
        target_xz: [f64; 2],
        dt: f64,
    ) -> ControlCommand {
        if !self.enabled || self.complete || state.destroyed {
            return ControlCommand::default();
        }

        let pos = state.position();
        let alt = pos[1];
        let cheby = chebyshev_xz(pos, target_xz);

        // HUD phase only — both use the same transit controller.
        if self.phase != TargetPhase::Descend {
            self.phase = if alt >= GATE_ALT_MIN {
                TargetPhase::Cruise
            } else {
                TargetPhase::Climb
            };
        }

        // Hand-off: lofted + over pad approach. No velocity gates above the gate.
        if self.phase != TargetPhase::Descend
            && alt >= GATE_ALT_MIN
            && cheby <= TARGET_PAD_HALF_M + PAD_APPROACH_MARGIN_M
        {
            self.phase = TargetPhase::Descend;
            self.lander.arm();
        }

        match self.phase {
            TargetPhase::Climb | TargetPhase::Cruise => transit_command(state, target_xz, pos),
            TargetPhase::Descend => {
                let cmd = self.lander.update_with_target(state, Some(target_xz), dt);
                self.complete = self.lander.complete;
                cmd
            }
        }
    }
}

/// True when CoM XZ lies inside the target pad square.
#[inline]
pub fn inside_target_pad(pos: [f64; 3], target_xz: [f64; 2]) -> bool {
    on_pad_square(pos, target_xz)
}

#[inline]
fn chebyshev_xz(pos: [f64; 3], target_xz: [f64; 2]) -> f64 {
    (pos[0] - target_xz[0])
        .abs()
        .max((pos[2] - target_xz[1]).abs())
}

/// Kill residual climb rate only (never command positive vy once lofted).
#[inline]
fn kill_climb_vy(vy: f64) -> f64 {
    if vy > 0.0 {
        (-0.35 * vy).max(-12.0)
    } else {
        0.0
    }
}

/// Simultaneous climb + translate toward the lofted pad waypoint.
fn transit_command(state: &RocketState, target_xz: [f64; 2], pos: [f64; 3]) -> ControlCommand {
    let lofted = pos[1] >= GATE_ALT_MIN;
    // Horizontal displacement to pad; vertical only while still below the gate.
    let dx = target_xz[0] - pos[0];
    let dz = target_xz[1] - pos[2];
    let dy = if lofted {
        0.0
    } else {
        (CLIMB_ALT_M - pos[1]).max(0.0)
    };
    let range_sq = dx * dx + dz * dz;
    let range = range_sq.sqrt();
    let cheby = dx.abs().max(dz.abs());

    let vx = state.velocity[0];
    let vy = state.velocity[1];
    let vz = state.velocity[2];
    let vh_sq = vx * vx + vz * vz;
    let vh = vh_sq.sqrt();

    // Shared time-to-go (climb leg only below gate).
    let t_h = if range > 1.0 { range / V_CRUISE_MAX } else { 0.0 };
    let t_v = if dy > 1.0 { dy / VY_CLIMB_MAX } else { 0.0 };
    let t_sync = t_h.max(t_v).max(T_SYNC_MIN);

    let terminal = lofted && cheby <= PAD_APPROACH_BAND_M;

    let (v_des_h, v_des_y) = if terminal {
        let v_h = if cheby <= TARGET_PAD_HALF_M {
            0.0
        } else {
            (0.04 * (cheby - TARGET_PAD_HALF_M * 0.5)).min(2.0)
        };
        (v_h, kill_climb_vy(vy))
    } else if lofted {
        ((range / t_sync).min(V_CRUISE_MAX), kill_climb_vy(vy))
    } else {
        (
            (range / t_sync).min(V_CRUISE_MAX),
            if dy > 0.0 {
                (dy / t_sync).min(VY_CLIMB_MAX)
            } else {
                0.0
            },
        )
    };

    let inv_range = if range > 1e-3 { 1.0 / range } else { 0.0 };
    let ux = dx * inv_range;
    let uz = dz * inv_range;

    let v_approach = vx * ux + vz * uz;
    let v_cmd_h = if v_approach < -0.5 {
        0.0
    } else if v_approach > v_des_h + 1.5 {
        v_des_h * 0.3
    } else {
        v_des_h
    };

    let (k_v, k_p) = if terminal {
        (0.18, 0.006)
    } else {
        (0.14, 0.012)
    };
    let ax = k_v * (ux * v_cmd_h - vx) + k_p * dx;
    let ay = 0.12 * (v_des_y - vy);
    let az = k_v * (uz * v_cmd_h - vz) + k_p * dz;

    let lean_max = if terminal {
        if vh > 8.0 {
            0.28
        } else if vh > 3.0 {
            0.18
        } else {
            0.12
        }
    } else if !lofted && dy > 30.0 {
        let climb_frac = (dy / (dy + range + 1.0)).clamp(0.0, 1.0);
        LEAN_TRANSIT_MAX * (1.0 - 0.40 * climb_frac)
    } else if vh > v_des_h + 10.0 {
        0.32
    } else {
        LEAN_TRANSIT_MAX
    };

    let desired = clamp_tilt([ax, 1.0 + 0.02 * ay, az], lean_max);
    let (pitch, yaw, roll, up_y) = attitude_toward(state, desired);

    let mass = state.params.mass;
    let hover = mass * GRAVITY / state.params.max_thrust;
    let hover_cmd = (hover / up_y.max(0.40)).clamp(0.0, 0.98);

    // Mild body-Y rate damping only when leaning enough that world vy ≠ body y.
    // Near upright, world vy is enough (saves an inverse sandwich transport).
    let v_damp = if up_y < 0.92 {
        motor_inverse_rotate_vector(&state.motor, state.velocity)[1]
    } else {
        vy
    };

    let kv_thr = if lofted { 0.08 } else { 0.06 };
    let mut throttle = hover_cmd + kv_thr * (v_des_y - vy) - 0.03 * v_damp.clamp(-5.0, 5.0);

    if lofted && vy > 1.0 {
        throttle = throttle.min(hover_cmd * 0.75);
    }

    let effort = pitch.abs() + yaw.abs() + 0.35 * roll.abs();
    if effort > 0.04 {
        throttle = throttle.max(0.10 + 0.28 * effort);
    }
    if state.contacting {
        throttle = throttle.max(hover_cmd * 1.45).max(0.60);
    }

    let floor = if lofted {
        if vy > 2.0 {
            hover_cmd * 0.45
        } else {
            hover_cmd * 0.70
        }
    } else {
        hover_cmd * 0.62
    };
    throttle = throttle.max(floor);

    ControlCommand {
        throttle: throttle.clamp(0.0, 1.0),
        pitch,
        yaw,
        roll,
    }
    .clamp()
}

/// Attitude PD toward a world-frame desired body +Y via PGA inverse transport.
fn attitude_toward(state: &RocketState, desired_world: [f64; 3]) -> (f64, f64, f64, f64) {
    let up_body = world_up_in_body(&state.motor);
    let up_y = up_body[1].clamp(-1.0, 1.0);

    // Flip first when severely tilted (cos test avoids acos on the hot path).
    let (axis, angle) = if up_y < COS_TILT_AIM {
        axis_angle_from_cross([up_body[2], 0.0, -up_body[0]], up_y)
    } else {
        let d = motor_inverse_rotate_vector(&state.motor, desired_world);
        axis_angle_from_cross([d[2], 0.0, -d[0]], d[1].clamp(-1.0, 1.0))
    };

    let omega = state.omega;
    let w_mag = (KP_ATT * angle)
        .min((2.0 * ALPHA_PLAN * angle).sqrt())
        .min(OMEGA_MAX);
    let pitch = saturate(KD_ATT * (omega[0] - axis[0] * w_mag));
    let yaw = saturate(KD_ATT * (omega[2] - axis[2] * w_mag));
    let roll = saturate(-KD_ROLL * omega[1]);
    (pitch, yaw, roll, up_y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn toggle_enables_and_disables() {
        let mut ap = TargetLandingAutopilot::default();
        ap.toggle();
        assert!(ap.enabled);
        assert!(!ap.complete);
        ap.disable();
        assert!(!ap.enabled);
    }

    #[test]
    fn high_altitude_labels_cruise_on_first_update() {
        let mut state = RocketState::at_altitude(600.0);
        state.contacting = false;
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        ap.phase = TargetPhase::Climb;
        let _ = ap.update(&state, [500.0, 0.0], 1.0 / 120.0);
        assert_eq!(ap.phase, TargetPhase::Cruise);
    }

    #[test]
    fn low_altitude_transit_leans_toward_target() {
        let mut state = RocketState::at_altitude(50.0);
        state.contacting = false;
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let cmd = ap.update(&state, [500.0, 0.0], 1.0 / 120.0);
        assert_eq!(ap.phase, TargetPhase::Climb);
        assert!(cmd.throttle > 0.3, "transit needs thrust, thr={}", cmd.throttle);
        assert!(
            cmd.pitch.abs() + cmd.yaw.abs() > 0.02,
            "expected lean toward target, pitch={} yaw={}",
            cmd.pitch,
            cmd.yaw
        );
    }

    #[test]
    fn pad_square_uses_chebyshev_half_extent() {
        assert!(inside_target_pad([500.0, 10.0, 0.0], [500.0, 0.0]));
        assert!(inside_target_pad([500.0 + TARGET_PAD_HALF_M, 0.0, 0.0], [500.0, 0.0]));
        assert!(!inside_target_pad(
            [500.0 + TARGET_PAD_HALF_M + 0.1, 0.0, 0.0],
            [500.0, 0.0]
        ));
    }

    #[test]
    fn kill_climb_never_commands_positive() {
        assert!(kill_climb_vy(10.0) < 0.0);
        assert_eq!(kill_climb_vy(-3.0), 0.0);
        assert_eq!(kill_climb_vy(0.0), 0.0);
    }
}
