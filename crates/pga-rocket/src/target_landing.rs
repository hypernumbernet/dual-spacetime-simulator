//! Target-pad autopilot (T key): burn–coast loft, airplane-style cruise, then land.
//!
//! Vertical plan is burn-then-coast: near-full throttle (leaning downrange to
//! build horizontal speed at the same time) until the ballistic apogee clears
//! the loft target, an upright straightening leg still under full thrust, then
//! a thrust-free coast over the top — gravity kills the climb.
//!
//! Far-pad cruise is airplane-like: deep lean of the body +Y thrust axis toward
//! the T-mark free vector, with **no hard speed cap**. Closing speed is governed
//! only by a braking envelope `v_stop = √(2 a r)` — accelerate while below it,
//! reverse-lean while above it.
//!
//! **Long-range airplane cruise (range ≳ 1.5 km):** full-throttle go toward T;
//! altitude is the elevator. Pitch commands
//! `cos(θ) ≈ hover·(1+a_cmd/g)` — nose-up when low, **nose-down dive lean** when
//! high or lofting through the ~800 m hold (flip gate stays low so dive is not
//! fought). Soft long-range weight blends the hold target 520→800 m; a hard
//! airplane-range floor keeps full-T pitch hold after a fast downrange burn.
//! Closer in, control hands to the normal envelope cruise / terminal lander.
//!
//! Attitude uses the same inverse sandwich transports as the L lander
//! (`motor_inverse_rotate_vector` / `world_up_in_body`). Desired thrust is a
//! free vector from CoM toward the lofted pad waypoint, `clamp_tilt`-ed into
//! the cruise lean cone.

use crate::euclidean_pga::{motor_inverse_rotate_vector, world_up_in_body};
use crate::fuzzy::{
    cruise_brake_weight, long_range_go_aim, long_range_hold_cos, long_range_weight,
    LONG_CRUISE_ALT_M,
};
use crate::landing::{
    axis_angle_from_cross, chebyshev_xz, clamp_tilt, on_pad_square, saturate, LandingAutopilot,
    PAD_HALF_M,
};
use crate::sim::{ControlCommand, GRAVITY, RocketState};

/// Nominal loft altitude (m, CoM). Roughly reaching this is enough for the gate.
pub const CLIMB_ALT_M: f64 = 500.0;
/// Soft floor for “about 500 m”: hand-off / no-climb once CoM is at least this high.
const GATE_ALT_MIN: f64 = 480.0;
/// Target pad half-extent (m) — same painted square as the mesh pad.
pub const TARGET_PAD_HALF_M: f64 = PAD_HALF_M;

// --- Hand-off into terminal lander -------------------------------------------
/// Max Chebyshev offset (m) to arm Descend — must already be over the pad
/// (lander will not walk in near the ground).
const HANDOFF_CHEBY_MAX_M: f64 = 22.0;
/// Max horizontal speed (m/s) when arming Descend. Keep low so the lander is
/// not handed a lateral sprint into the upright commit.
const VH_HANDOFF_MAX: f64 = 6.5;
/// Max pitch/yaw rate (rad/s) when arming Descend.
const OMEGA_HANDOFF_MAX: f64 = 0.12;
/// Min body-up · world-up when arming (~0.32 rad tilt).
const COS_TILT_HANDOFF: f64 = 0.95;

// --- Transit lean / envelope -------------------------------------------------
/// Lean cap during the full-throttle ascent burn (rad).
const LEAN_BURN_MAX: f64 = 0.30;
/// Far-cruise lean cap (rad, ~46°). Below flip gate; stable high-alt transit.
const LEAN_CRUISE: f64 = 0.80;
/// Mid-range lean while still on the envelope but closer in (rad).
const LEAN_MID: f64 = 0.45;
/// Range (m) where deep airplane cruise takes over.
const RANGE_FAR_M: f64 = 80.0;
/// Range (m) for near-pad station-keeping before hand-off.
const RANGE_TERMINAL_M: f64 = 55.0;
/// Soft ceiling above the altitude gate (m). Once lofted, prefer a slight
/// sink rather than riding thrust upward past this.
const CRUISE_ALT_CAP: f64 = GATE_ALT_MIN + 40.0;
/// Ballistic apogee the climb burn aims for (m). Cut thrust once the coast
/// alone clears this, so the loft tops out just above [`CLIMB_ALT_M`].
const APOGEE_TARGET_M: f64 = CLIMB_ALT_M + 30.0;
/// Near-full climb throttle (gimbal authority scales with thrust, so no
/// reserve is needed — full thrust is also max attitude authority).
const THR_CLIMB_BURN: f64 = 0.97;
/// Long-range (≥5 km) full-throttle cruise / altitude-hold.
const THR_LONG: f64 = 0.97;
/// Lean cap for airplane cruise (rad). cos(1.45)≈0.12 — deep nose-down dive at
/// full T (hover cos≈0.33). Must stay at/above [`COS_TILT_AIM_AIR`] flip gate.
const LEAN_LONG_MAX: f64 = 1.45;
/// Flip-recover only when nearly inverted during airplane cruise (cos < this).
/// Normal COS_TILT_AIM (0.30) would fight a legitimate dive lean / nose-down.
const COS_TILT_AIM_AIR: f64 = 0.10;
/// Above this horizontal range: free accelerate (no reverse-lean). Below it,
/// envelope braking with **hysteresis** (no go↔brake chatter / sway).
const FREE_GO_RANGE_M: f64 = 5200.0;
/// Airplane full-T + pitch elevator while range is still large. Independent of
/// the soft long-range weight so a fast downrange burn that closes to ~3 km
/// does not fall back into ballistic thr-cut / upright straighten.
const LONG_AIRPLANE_RANGE_M: f64 = 1500.0;
/// Enter reverse-lean when `v_approach − v_stop` exceeds this (m/s).
const BRAKE_LATCH_ENTER: f64 = 10.0;
/// Hold reverse-lean until `v_approach − v_stop` falls below this (m/s).
const BRAKE_LATCH_EXIT: f64 = -25.0;
/// Horizontal deceleration assumed by the braking envelope (m/s²).
/// Conservative vs peak g·tan(LEAN_CRUISE) so attitude-flip lag still fits
/// inside the remaining range (starts reverse lean earlier).
const A_DEC_H: f64 = 3.5;
/// Stand-off subtracted from range in the envelope (m) so speed is already
/// small when terminal descent arms.
const V_APPROACH_OFF_M: f64 = 30.0;
/// Attitude-reversal lag anticipation (s) folded into the envelope distance.
const T_ATT_LAG_S: f64 = 2.0;
/// Downrange speed built during the ascent burn (m/s). Ballistic coast keeps
/// whatever vh burnout leaves — cruise then accelerates freely on the envelope.
const V_CLIMB_H_MAX: f64 = 28.0;
/// Attitude √-profile planning accel (rad/s²).
const ALPHA_PLAN: f64 = 0.5;
const OMEGA_MAX: f64 = 1.15;
const KP_ATT: f64 = 2.0;
const KD_ATT: f64 = 3.0;
const KD_ROLL: f64 = 2.0;
/// Flip only when past the commanded cruise lean (well beyond LEAN_CRUISE).
const COS_TILT_AIM: f64 = 0.30; // ~72.5°
/// Pitch/yaw rate (rad/s) above which attitude is pure rate-kill.
const OMEGA_RATE_KILL: f64 = 0.80;
/// Vertical component of the free-vector aim (dimensionless relative to |horiz|).
/// Keeps the thrust axis from going fully horizontal.
const AIM_Y_BIAS: f64 = 1.0;

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
    /// Latched reverse-lean brake (hysteresis) — kills go↔brake sway.
    brake_latched: bool,
}

impl Default for TargetLandingAutopilot {
    fn default() -> Self {
        Self {
            enabled: false,
            complete: false,
            phase: TargetPhase::Climb,
            lander: LandingAutopilot::for_target_pad(),
            brake_latched: false,
        }
    }
}

impl TargetLandingAutopilot {
    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
        if self.enabled {
            self.complete = false;
            self.phase = TargetPhase::Climb;
            self.brake_latched = false;
            self.lander.disable();
        } else {
            self.lander.disable();
        }
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.complete = false;
        self.phase = TargetPhase::Climb;
        self.brake_latched = false;
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

    /// HUD helper: long-range full-throttle cruise is active (range ≳ 5 km).
    pub fn is_long_range_cruise(&self, pos: [f64; 3], target_xz: [f64; 2]) -> bool {
        if !self.enabled || self.complete || self.phase == TargetPhase::Descend {
            return false;
        }
        let dx = target_xz[0] - pos[0];
        let dz = target_xz[1] - pos[2];
        let range = (dx * dx + dz * dz).sqrt();
        long_range_weight(range) > 0.5
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
            && cheby <= HANDOFF_CHEBY_MAX_M
            && vh <= VH_HANDOFF_MAX
            && om_pitch_yaw_sq <= OMEGA_HANDOFF_MAX * OMEGA_HANDOFF_MAX
            && world_up_in_body(&state.motor)[1] >= COS_TILT_HANDOFF
        {
            self.phase = TargetPhase::Descend;
            self.lander.arm();
        }

        match self.phase {
            TargetPhase::Climb | TargetPhase::Cruise => {
                let (cmd, brake) =
                    transit_command(state, target_xz, pos, self.brake_latched);
                self.brake_latched = brake;
                cmd
            }
            TargetPhase::Descend => {
                self.brake_latched = false;
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

/// Kill residual climb rate only (never command positive vy once lofted).
#[inline]
fn kill_climb_vy(vy: f64) -> f64 {
    if vy > 0.0 {
        (-0.35 * vy).max(-12.0)
    } else {
        0.0
    }
}

/// Vertical rate target for lofted cruise / far burn (altitude-hold).
/// Never commands climb; above [`CRUISE_ALT_CAP`] asks for a gentle sink.
#[inline]
fn cruise_v_des_y(alt: f64, vy: f64) -> f64 {
    let sink = if alt > CRUISE_ALT_CAP {
        // Bleed excess altitude while translating (~1–3 m/s sink).
        (-0.04 * (alt - CRUISE_ALT_CAP)).clamp(-3.0, -0.5)
    } else {
        0.0
    };
    if vy > sink {
        // Stronger than kill_climb_vy so residual climb from lateral burns dies.
        (sink - 0.55 * (vy - sink)).max(-10.0)
    } else {
        sink
    }
}

/// Braking-envelope max approach speed (m/s). **No hard cruise speed cap** —
/// the rocket accelerates freely until this distance-limited stop speed.
#[inline]
fn envelope_v_stop(range: f64, v_approach: f64) -> f64 {
    let range_eff =
        (range - V_APPROACH_OFF_M - T_ATT_LAG_S * v_approach.max(0.0)).max(0.0);
    (2.0 * A_DEC_H * range_eff).sqrt()
}

/// Simultaneous climb + airplane-style translate toward the lofted pad waypoint.
///
/// Vertical plan is burn-then-coast: near-full throttle until the ballistic
/// apogee (`alt + vy²/2g`, conserved in vacuum coast) clears the loft target,
/// then thrust drops and gravity tops out the climb. Far cruise aims body +Y
/// along the free vector to the pad with a deep lean cone; speed is only
/// limited by the braking envelope (accelerate / reverse-brake).
///
/// When horizontal range ≳ [`FREE_GO_RANGE_M`]: **airplane cruise** — full
/// throttle toward the pad; altitude held only by pitch/lean (elevator).
/// Closer in, envelope reverse-lean uses `brake_latched` hysteresis.
///
/// Returns `(command, updated_brake_latch)`.
fn transit_command(
    state: &RocketState,
    target_xz: [f64; 2],
    pos: [f64; 3],
    brake_latched: bool,
) -> (ControlCommand, bool) {
    let lofted = pos[1] >= GATE_ALT_MIN;
    let dx = target_xz[0] - pos[0];
    let dz = target_xz[1] - pos[2];
    let range = (dx * dx + dz * dz).sqrt();
    let mu_long = long_range_weight(range);

    let vx = state.velocity[0];
    let vy = state.velocity[1];
    let vz = state.velocity[2];
    let vh = (vx * vx + vz * vz).sqrt();

    let mass = state.params.mass;
    let hover = mass * GRAVITY / state.params.max_thrust;

    // Long-range loft aims near cruise hold (~800 m). Slightly under so full-T
    // airplane climb into the hold band does not balloon past 1 km.
    let apogee_target = APOGEE_TARGET_M
        + mu_long * ((LONG_CRUISE_ALT_M - 40.0) - APOGEE_TARGET_M);
    let straighten_m = apogee_target - 180.0;

    let apogee = pos[1] + vy.max(0.0) * vy.max(0.0) / (2.0 * GRAVITY);
    // Large remaining range ⇒ airplane mode (full T, pitch elevator), even if
    // the soft long-range membership has already faded after a fast burn.
    let long_airplane = range >= LONG_AIRPLANE_RANGE_M;
    // Airplane ascent: stay powered until the altitude gate. Short hops still
    // use ballistic apogee cut so they do not over-burn.
    let burn_up = !lofted
        && if long_airplane {
            true
        } else {
            apogee < apogee_target
        };
    // Powered-cruise weight: 1 at vy ≤ +3 (hover/translate), 0 at vy ≥ +8
    // (ballistic coast). Continuous blend avoids upright/lean chatter on the
    // boundary that would hold hover thrust forever.
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

    // Near-pad station keeping: quiet enough for hand-off (no deep lean chase).
    let terminal = lofted && range <= RANGE_TERMINAL_M;

    let v_stop = envelope_v_stop(range, v_approach);
    // Climb burn: modest horizontal build; free accel on cruise; creep near pad.
    let v_cmd_h = if burn_up || ballistic {
        v_stop.min(V_CLIMB_H_MAX)
    } else if terminal {
        (0.12 * (range - 6.0)).clamp(0.0, 10.0)
    } else {
        v_stop
    };

    let need_x = ux * v_cmd_h - vx;
    let need_z = uz * v_cmd_h - vz;

    // Far cruise: airplane lean along the free vector to T, or reverse-brake
    // when above the envelope. No velocity-PD flip — aim is direction-stable.
    let far = lofted && cruise_w > 0.75 && !terminal && range > RANGE_FAR_M;
    let far_or_overshoot = far
        || (lofted
            && cruise_w > 0.75
            && !terminal
            && range > RANGE_TERMINAL_M
            && v_approach < -1.5);

    // Free-go band: far enough that reverse-lean is unnecessary (and looks bad).
    let free_go = !terminal && !burn_up && lofted && range >= FREE_GO_RANGE_M;
    // Long altitude-hold style: soft membership **or** still outside the
    // airplane range floor (keeps pitch elevator after a fast range close).
    let long_alt =
        !terminal && !burn_up && lofted && (mu_long > 0.05 || long_airplane);
    // Blend altitude target 520→800 m. While in airplane range, hold full
    // cruise altitude even if mu_long has faded.
    let w_alt = if long_airplane {
        1.0
    } else {
        mu_long
    };
    let alt_hold = CRUISE_ALT_CAP + w_alt * (LONG_CRUISE_ALT_M - CRUISE_ALT_CAP);

    // Brake latch: free_go / airplane cruise never reverse-lean (go + pitch
    // elevator). Inside terminal, drop latch so reverse-lean does not fight
    // station-keeping over the pad.
    let delta_v = v_approach - v_stop;
    let overshoot = v_approach < -1.5 && range > RANGE_TERMINAL_M;
    let mut brake = brake_latched;
    if free_go || long_airplane || terminal {
        brake = false;
    } else if overshoot || delta_v > BRAKE_LATCH_ENTER {
        brake = true;
    } else if delta_v < BRAKE_LATCH_EXIT {
        brake = false;
    }

    let (desired_raw, lean_max, deep, force_full_thr) = if burn_up && long_airplane {
        // Long-range ascent is airplane mode: full-T toward T with pitch
        // elevator to `alt_hold`. No upright straighten loft.
        let cos_up = long_range_hold_cos(pos[1], alt_hold, vy, hover);
        (
            long_range_go_aim(ux, uz, cos_up),
            LEAN_LONG_MAX,
            true,
            true,
        )
    } else if burn_up {
        // Short/mid ascent: modest lean, then upright straighten for coast.
        let lean = LEAN_BURN_MAX + mu_long * (0.90 - LEAN_BURN_MAX);
        let y_bias = AIM_Y_BIAS - mu_long * 0.55;
        let k_h = 0.14 + mu_long * 0.35;
        (
            [k_h * need_x, y_bias.max(0.40), k_h * need_z],
            lean.min(LEAN_LONG_MAX),
            mu_long > 0.35,
            false,
        )
    } else if terminal {
        // Blend toward upright as we enter the hand-off box (vh quiet).
        // Keep this modest: aggressive free anti-v / deep lean overshoots,
        // holds rates/tilt outside Descend gates, or hands the lander a
        // lateral sprint. Mid-range brake envelope already kills most vh.
        let settle = range <= HANDOFF_CHEBY_MAX_M && vh <= VH_HANDOFF_MAX + 3.0;
        let k = if settle { 0.06 } else { 0.10 };
        let lean = if settle {
            (0.05 + 0.015 * vh).clamp(0.06, 0.18)
        } else {
            (0.08 + 0.025 * vh).clamp(0.10, 0.28)
        };
        ([k * need_x, AIM_Y_BIAS, k * need_z], lean, false, false)
    } else if free_go || (long_alt && !brake) {
        // Full-throttle airplane go + pitch elevator altitude hold.
        // Priority: keep thrusting toward T; altitude via nose-up / nose-down
        // lean only (no throttle cut). Dive lean is intentional when high.
        let cos_up = long_range_hold_cos(pos[1], alt_hold, vy, hover);
        (
            long_range_go_aim(ux, uz, cos_up),
            LEAN_LONG_MAX,
            true,
            // Full T whenever this airplane hold is active (not only free_go),
            // so pitch — not thr — owns altitude across the long-range shoulder.
            true,
        )
    } else if far_or_overshoot {
        // Near/mid-pad airplane cruise (cruise_w high). Reverse-lean uses latch.
        let s = vh.max(1.0);
        if brake {
            ([-vx / s, AIM_Y_BIAS, -vz / s], LEAN_CRUISE, true, false)
        } else {
            let w_mid = cruise_brake_weight(delta_v, BRAKE_LATCH_ENTER, BRAKE_LATCH_EXIT);
            if w_mid > 0.35 {
                ([0.08 * need_x, AIM_Y_BIAS, 0.08 * need_z], LEAN_MID, false, false)
            } else {
                ([ux, AIM_Y_BIAS, uz], LEAN_CRUISE, true, false)
            }
        }
    } else {
        // Mid-range / ballistic: shallower cone; reverse only when latched.
        let s = vh.max(1.0);
        if brake && lofted && !terminal {
            ([-vx / s, AIM_Y_BIAS, -vz / s], LEAN_MID, false, false)
        } else {
            (
                [ux + 0.05 * need_x, AIM_Y_BIAS, uz + 0.05 * need_z],
                LEAN_MID,
                false,
                false,
            )
        }
    };

    // Straightening burn aims strictly upright: cutoff then happens with no
    // lean and no rates, so the coast needs almost no recovery thrust.
    // Skip on airplane-range ascent (pitch elevator owns altitude).
    let straighten = burn_up && !long_airplane && apogee >= straighten_m;
    // Deep airplane lean must not be faded by cruise_w (half-open aim = sway).
    let aim_w = if burn_up || deep {
        1.0
    } else {
        cruise_w
    };
    let desired = if straighten {
        [0.0, 1.0, 0.0]
    } else {
        clamp_tilt(
            [aim_w * desired_raw[0], desired_raw[1], aim_w * desired_raw[2]],
            lean_max,
        )
    };
    // Airplane / deep-lean cruise: low flip gate so nose-down dive is tracked
    // instead of fought as "inverted recovery".
    let flip_cos = if force_full_thr || deep {
        COS_TILT_AIM_AIR
    } else {
        COS_TILT_AIM
    };
    let (pitch, yaw, roll, up_y) = attitude_toward(state, desired, flip_cos);

    // Near-pad altitude-hold throttle (not used for free-go airplane cruise).
    let upy_floor = if deep { 0.45 } else { 0.40 };
    let hover_cmd = (hover / up_y.max(upy_floor)).clamp(0.0, 0.95);

    let mut throttle = if burn_up {
        THR_CLIMB_BURN
    } else if force_full_thr {
        // Airplane mode: **always full throttle**. Altitude is pitch/lean only.
        // Reverse-lean brake phase also keeps full T for deceleration authority.
        THR_LONG
    } else {
        let v_damp = if up_y < 0.92 {
            motor_inverse_rotate_vector(&state.motor, state.velocity)[1]
        } else {
            vy
        };
        let v_des_y = if lofted {
            cruise_v_des_y(pos[1], vy)
        } else {
            kill_climb_vy(vy)
        };
        let kv = if lofted { 0.12 } else { 0.08 };
        let base = hover_cmd + kv * (v_des_y - vy) - 0.03 * v_damp.clamp(-5.0, 5.0);
        cruise_w * base.max(hover_cmd * 0.65)
    };

    let effort = pitch.abs() + yaw.abs() + 0.35 * roll.abs();
    if force_full_thr {
        // Never demote airplane full-T to hover/cos — pitch owns altitude.
        throttle = THR_LONG;
    } else if deep && up_y < 0.80 {
        // Near-pad airplane: track hover/cos so vertical net accel ≈ 0.
        throttle = throttle.clamp(hover_cmd * 0.85, (hover_cmd + 0.08).min(0.92));
    } else if deep {
        // Still rotating into lean: gimbal only — never a climb burn.
        throttle = throttle.max(0.35 + 0.20 * effort).clamp(0.35, 0.55);
    } else if ballistic || burn_up {
        throttle = throttle.max((0.9 * (effort - 0.15).max(0.0)).min(0.35));
    } else if effort > 0.04 {
        throttle = throttle.max(0.10 + 0.28 * effort);
    }
    if state.contacting {
        throttle = throttle.max(hover_cmd * 1.45).max(0.60);
    }

    let cmd = ControlCommand {
        throttle: throttle.clamp(0.0, 1.0),
        pitch,
        yaw,
        roll,
    }
    .clamp();
    (cmd, brake)
}

/// Attitude PD toward a world-frame desired body +Y via PGA inverse transport.
///
/// `flip_cos`: if body-up·world-up falls below this, command pure upright
/// recovery (inverted / tumble). Airplane cruise passes a low gate so deep
/// dive lean is tracked instead of fought.
fn attitude_toward(
    state: &RocketState,
    desired_world: [f64; 3],
    flip_cos: f64,
) -> (f64, f64, f64, f64) {
    let up_body = world_up_in_body(&state.motor);
    let up_y = up_body[1].clamp(-1.0, 1.0);
    let omega = state.omega;
    let omega_xy = (omega[0] * omega[0] + omega[2] * omega[2]).sqrt();

    // Flip only past the commanded lean cone (near-inverted), not mid-recovery.
    let (axis, angle) = if up_y < flip_cos {
        axis_angle_from_cross([up_body[2], 0.0, -up_body[0]], up_y)
    } else {
        let d = motor_inverse_rotate_vector(&state.motor, desired_world);
        axis_angle_from_cross([d[2], 0.0, -d[0]], d[1].clamp(-1.0, 1.0))
    };

    // High residual rate: kill spin first so a lean-direction change cannot
    // carry the body through upright into continuous tumble.
    let w_mag = if omega_xy > OMEGA_RATE_KILL {
        0.0
    } else {
        (KP_ATT * angle)
            .min((2.0 * ALPHA_PLAN * angle).sqrt())
            .min(OMEGA_MAX)
            .min((OMEGA_MAX - 0.4 * omega_xy).max(0.0))
    };
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

    #[test]
    fn envelope_has_no_hard_speed_cap() {
        // At 400 m remaining with low closing speed, stop speed exceeds the old 45 m/s cap.
        let v = envelope_v_stop(400.0, 10.0);
        assert!(
            v > 45.0,
            "envelope should allow >45 m/s at long range, got {v}"
        );
        let v_far = envelope_v_stop(800.0, 0.0);
        assert!(v_far > v, "longer range must allow higher stop speed");
    }

    #[test]
    fn far_cruise_leans_toward_target_when_underspeed() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [20.0, 0.0, 0.0]; // well under envelope at 500 m range
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let cmd = ap.update(&state, [500.0, 0.0], 1.0 / 120.0);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        // Target on +X → pitch gimbal (about body +X) should be non-trivial lean.
        assert!(
            cmd.pitch.abs() + cmd.yaw.abs() > 0.05,
            "expected airplane go lean, pitch={} yaw={} thr={}",
            cmd.pitch,
            cmd.yaw,
            cmd.throttle
        );
    }

    #[test]
    fn far_cruise_brakes_when_overspeed() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        // Closing very fast at short remaining range → above envelope.
        state.velocity = [80.0, 0.0, 0.0];
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        // Pad only 120 m ahead: v_stop ≈ √(2·5.5·(120-25-1.5·80)) ≈ small.
        let cmd = ap.update(&state, [120.0, 0.0], 1.0 / 120.0);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        assert!(
            cmd.pitch.abs() + cmd.yaw.abs() > 0.05,
            "expected brake lean when overspeed, pitch={} yaw={}",
            cmd.pitch,
            cmd.yaw
        );
    }

    #[test]
    fn long_range_full_throttle_near_800m() {
        // 6 km out, already at long-cruise altitude → full throttle, not hover.
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M);
        state.contacting = false;
        state.velocity = [40.0, 0.0, 0.0];
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let target = [6000.0, 0.0];
        let cmd = ap.update(&state, target, 1.0 / 120.0);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        assert!(
            cmd.throttle > 0.9,
            "long-range cruise must be near full throttle, thr={}",
            cmd.throttle
        );
        assert!(
            ap.is_long_range_cruise(state.position(), target),
            "expected long-range flag"
        );
        // Airplane lean toward +X target.
        assert!(
            cmd.pitch.abs() + cmd.yaw.abs() > 0.05,
            "expected go lean, pitch={} yaw={}",
            cmd.pitch,
            cmd.yaw
        );
    }

    #[test]
    fn short_range_not_long_cruise() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let target = [500.0, 0.0];
        let _ = ap.update(&state, target, 1.0 / 120.0);
        assert!(!ap.is_long_range_cruise(state.position(), target));
    }

}
