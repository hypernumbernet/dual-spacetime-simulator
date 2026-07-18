//! Target-pad autopilot (T key): burn–coast loft, powered cruise, then land.
//!
//! Vertical plan is burn-then-coast: near-full throttle (leaning downrange to
//! build horizontal speed at the same time) until the ballistic apogee clears
//! the loft target, an upright straightening leg still under full thrust, then
//! a thrust-free coast over the top — gravity kills the climb. The powered
//! cruise leg fades in as the ascent dies and flies a deceleration-limited
//! approach speed profile to the pad. Terminal descent reuses
//! [`LandingAutopilot`] with pad-square success (Chebyshev half-extent), armed
//! only once over the pad approach with drift and attitude quiet enough for
//! the lander's gentler gains.
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
/// Max lean cone half-angle during transit (rad).
const LEAN_TRANSIT_MAX: f64 = 0.38;
/// Lean cap during the full-throttle ascent burn (rad): keeps the climb fast
/// while still building downrange speed (3g·sin ≈ 8.7 m/s² lateral).
const LEAN_BURN_MAX: f64 = 0.30;
/// Deeper lean allowed for the horizontal deceleration leg (rad).
const LEAN_DECEL_MAX: f64 = 0.55;
/// Max commanded ground speed during transit (m/s).
const V_CRUISE_MAX: f64 = 45.0;
/// Ballistic apogee the climb burn aims for (m). Cut thrust once the coast
/// alone clears this, so the loft tops out just above [`CLIMB_ALT_M`].
const APOGEE_TARGET_M: f64 = CLIMB_ALT_M + 30.0;
/// Predicted-apogee point where the burn straightens up (m). The last leg of
/// the burn flies upright so cutoff happens with no lean and no rates — the
/// uprighting happens on full thrust (where apogee is still being commanded),
/// not on the coast, whose recovery thrust would blow the apogee past plan.
/// At full thrust d(apogee)/dt ≈ vy·(1 + a_net/g) ≈ 200 m/s, so this margin
/// buys ≈ 1.2 s of straightening.
const APOGEE_STRAIGHTEN_M: f64 = APOGEE_TARGET_M - 250.0;
/// Near-full climb throttle (gimbal authority scales with thrust, so no
/// reserve is needed — full thrust is also max attitude authority).
const THR_CLIMB_BURN: f64 = 0.97;
/// Horizontal deceleration assumed by the approach speed profile (m/s²).
/// The braking lean at vertical-neutral throttle delivers ≈ g·tan(0.55) ≈ 6,
/// so this keeps a ~25% margin on top of the anticipation term.
const A_DEC_H: f64 = 4.5;
/// Stand-off subtracted from range in the approach profile (m) so the
/// commanded speed is already small when terminal descent arms.
const V_APPROACH_OFF_M: f64 = 20.0;
/// Downrange speed built during the ascent burn (m/s). Ballistic coast keeps
/// whatever vh burnout leaves — there is no cheap way to shed it until the
/// powered cruise leg, so build only what the cruise brake can absorb.
const V_CLIMB_H_MAX: f64 = 24.0;
/// Max drift allowed when arming terminal descent (m/s): a fast overflight
/// must brake on the powered cruise leg first, not hand the lander a sprint.
const VH_HANDOFF_MAX: f64 = 12.0;
/// Attitude quiet gates for the hand-off (rad/s, cos(rad)). Arming mid-swing
/// hands the lander a rotation its gentler √-profile cannot stop before the
/// stack sails through upright into a pro-drift lean.
const OMEGA_HANDOFF_MAX: f64 = 0.15;
/// Loose: station-keeping legitimately leans ~0.3 rad; the rate gate is the
/// real mid-swing detector.
const COS_TILT_HANDOFF: f64 = 0.939; // cos(0.35)
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

        // Hand-off: lofted + over pad approach + drift/attitude the lander can
        // absorb. (A fast overflight keeps transiting — its braking lean
        // decelerates, overshoot flips the approach direction, and it re-arms
        // coming back; likewise a mid-swing attitude settles under the
        // transit PD first.)
        let vh = (state.velocity[0] * state.velocity[0]
            + state.velocity[2] * state.velocity[2])
            .sqrt();
        let om = state.omega;
        let om_pitch_yaw_sq = om[0] * om[0] + om[2] * om[2];
        if self.phase != TargetPhase::Descend
            && alt >= GATE_ALT_MIN
            && cheby <= TARGET_PAD_HALF_M + PAD_APPROACH_MARGIN_M
            && vh <= VH_HANDOFF_MAX
            && om_pitch_yaw_sq <= OMEGA_HANDOFF_MAX * OMEGA_HANDOFF_MAX
            && world_up_in_body(&state.motor)[1] >= COS_TILT_HANDOFF
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
///
/// Vertical plan is burn-then-coast: near-full throttle until the ballistic
/// apogee (`alt + vy²/2g`, conserved in vacuum coast) clears the loft target,
/// then thrust drops to attitude-authority only and gravity tops out the climb
/// — no engine time is spent killing the ascent.
fn transit_command(state: &RocketState, target_xz: [f64; 2], pos: [f64; 3]) -> ControlCommand {
    let lofted = pos[1] >= GATE_ALT_MIN;
    let dx = target_xz[0] - pos[0];
    let dz = target_xz[1] - pos[2];
    let range = (dx * dx + dz * dz).sqrt();

    let vx = state.velocity[0];
    let vy = state.velocity[1];
    let vz = state.velocity[2];
    let vh = (vx * vx + vz * vz).sqrt();

    let apogee = pos[1] + vy.max(0.0) * vy.max(0.0) / (2.0 * GRAVITY);
    let burn_up = !lofted && apogee < APOGEE_TARGET_M;
    // Powered-cruise weight: 1 at vy ≤ +3 (hover/translate), 0 at vy ≥ +8
    // (ballistic coast). The continuous blend matters: a hard vy threshold
    // slides on its own boundary — the aim flips upright/lean every frame,
    // the resulting gimbal effort holds ~hover thrust, and the vehicle rides
    // the boundary upward at constant vy indefinitely.
    let cruise_w = if burn_up {
        0.0
    } else {
        (1.0 - (vy - 3.0) / 5.0).clamp(0.0, 1.0)
    };
    let ballistic = !burn_up && cruise_w < 1.0;

    let inv_range = if range > 1e-3 { 1.0 / range } else { 0.0 };
    let ux = dx * inv_range;
    let uz = dz * inv_range;
    let v_approach = vx * ux + vz * uz;

    // Near-pad station keeping is deliberately gentle: a crawl speed command
    // and small lean caps. Chasing residual error with deep hover-thrust leans
    // re-energizes the drift through attitude lag (limit cycle) and keeps the
    // hand-off quiet-gates from ever passing.
    let terminal = lofted && range <= 55.0;

    // Approach speed profile toward the pad (deceleration-limited, capped).
    // The anticipation term discounts the distance the attitude-reversal lag
    // eats before braking actually bites (~1.5 s at current closing speed);
    // without it a hot approach detects the overspeed too late and sails
    // ~100 m past the pad.
    let v_des_h = if terminal {
        (0.12 * (range - 6.0)).clamp(0.0, 10.0)
    } else {
        let range_eff =
            (range - V_APPROACH_OFF_M - 1.5 * v_approach.max(0.0)).max(0.0);
        (2.0 * A_DEC_H * range_eff).sqrt().min(V_CRUISE_MAX)
    };
    let mut v_cmd_h = if v_approach < -0.5 {
        0.0
    } else if v_approach > v_des_h + 1.5 {
        v_des_h * 0.3
    } else {
        v_des_h
    };
    if burn_up || ballistic {
        v_cmd_h = v_cmd_h.min(V_CLIMB_H_MAX);
    }

    // Ballistic legs steer by velocity only: chasing the position term keeps
    // the lean saturated well past the wanted burnout vh.
    let k_p = if burn_up { 0.0 } else { 0.012 * cruise_w };
    let ax = 0.14 * (ux * v_cmd_h - vx) + k_p * dx;
    let az = 0.14 * (uz * v_cmd_h - vz) + k_p * dz;

    let lean_max = if burn_up {
        LEAN_BURN_MAX
    } else if terminal {
        // Speed-proportional: hot entries still get real braking lean, but the
        // near-rest crawl stays gentle — deep leans at walking pace pendulum
        // through attitude lag.
        (0.10 + 0.04 * vh).clamp(0.12, 0.50)
    } else if v_approach > v_des_h + 3.0 || v_approach < v_des_h - 5.0 {
        // Far off the speed profile in either direction: deep lean (braking or
        // catching up); hover_cmd below compensates 1/cos.
        LEAN_DECEL_MAX
    } else {
        LEAN_TRANSIT_MAX
    };

    // Straightening burn aims strictly upright: cutoff then happens with no
    // lean and no rates, so the coast needs almost no recovery thrust. The
    // ballistic blend fades the lean out toward the same upright aim — a
    // stable, already-achieved aim lets the attitude effort (and with it the
    // authority throttle) decay to ~0, keeping the coast truly ballistic,
    // while near-zero thrust with a leaning aim would tumble the stack.
    let straighten = burn_up && apogee >= APOGEE_STRAIGHTEN_M;
    let aim_w = if burn_up { 1.0 } else { cruise_w };
    let desired = if straighten {
        [0.0, 1.0, 0.0]
    } else {
        clamp_tilt([aim_w * ax, 1.0, aim_w * az], lean_max)
    };
    let (pitch, yaw, roll, up_y) = attitude_toward(state, desired);

    let mass = state.params.mass;
    let hover = mass * GRAVITY / state.params.max_thrust;
    let hover_cmd = (hover / up_y.max(0.40)).clamp(0.0, 0.98);

    let mut throttle = if burn_up {
        THR_CLIMB_BURN
    } else {
        // Altitude-hold cruise: PD on residual vertical rate with mild body-Y
        // damping (only sandwich-transport when leaning enough to matter).
        // Scaled by the cruise weight: gravity kills the remaining climb for
        // free, so the powered hold fades in only as the ascent dies.
        let v_damp = if up_y < 0.92 {
            motor_inverse_rotate_vector(&state.motor, state.velocity)[1]
        } else {
            vy
        };
        let v_des_y = kill_climb_vy(vy);
        let base = hover_cmd + 0.08 * (v_des_y - vy) - 0.03 * v_damp.clamp(-5.0, 5.0);
        cruise_w * base.max(hover_cmd * 0.70)
    };

    let effort = pitch.abs() + yaw.abs() + 0.35 * roll.abs();
    if ballistic || burn_up {
        // Rate-kill bursts only while ballistic. Chasing the last bit of
        // tracking lag costs sustained lift (0.15 throttle ≈ half of gravity)
        // and blows the apogee; the upright cutoff leaves little to kill.
        throttle = throttle.max((0.9 * (effort - 0.15).max(0.0)).min(0.35));
    } else if effort > 0.04 {
        throttle = throttle.max(0.10 + 0.28 * effort);
    }
    if state.contacting {
        throttle = throttle.max(hover_cmd * 1.45).max(0.60);
    }

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
