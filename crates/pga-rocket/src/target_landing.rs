//! Target-pad autopilot (T key): loft, cruise to pad, terminal land.
//!
//! **Short / mid range:** burn–coast loft to ~500 m (apogee cut + optional
//! upright straighten), then physics-predicted horizontal braking: reverse lean
//! when `range_eff ≤ d_flip + d_burn` (thrust/mass, lean, drag, attitude lag).
//! No hard speed cap.
//!
//! **Airplane range (≳ 1.5 km):** full throttle toward T while outside the
//! predicted stop distance; altitude is pitch only (`long_range_hold_cos` /
//! `long_range_go_aim`). Hold ~800 m. Same stop-distance gate hands off to
//! reverse lean. Inside ~55 m: physics-predicted settle (attitude / vh / cheby
//! time-to-clear) drives lean and throttle until hard AND gates arm Descend via
//! [`LandingAutopilot::update_target_descend`] (physics closed-loop suicide burn).
//!
//! Attitude: PGA inverse sandwich (`motor_inverse_rotate_vector` /
//! `world_up_in_body`), desired thrust `clamp_tilt`-ed into the lean cone.

use crate::euclidean_pga::{motor_inverse_rotate_vector, world_up_in_body};
use crate::fuzzy::{long_range_go_aim, long_range_hold_cos, long_range_weight, LONG_CRUISE_ALT_M};
use crate::landing::{
    axis_angle_from_cross, chebyshev_xz, clamp_tilt, on_target_success_square, saturate,
    LandingAutopilot, PAD_HALF_M,
};
use crate::sim::{ControlCommand, GRAVITY, RocketState};

/// Nominal loft altitude (m, CoM). Roughly reaching this is enough for the gate.
pub const CLIMB_ALT_M: f64 = 500.0;
/// Soft floor for “about 500 m”: hand-off / no-climb once CoM is at least this high.
const GATE_ALT_MIN: f64 = 480.0;
/// Painted target pad half-extent (m) — matches the mesh / shader pad mark.
pub const TARGET_PAD_HALF_M: f64 = PAD_HALF_M;

// --- Hand-off into terminal lander -------------------------------------------
/// Max Chebyshev offset (m) to arm Descend — must already be over the pad
/// (lander will not walk in near the ground).
const HANDOFF_CHEBY_MAX_M: f64 = 10.0;
/// Max horizontal speed (m/s) when arming Descend. Keep low so the lander is
/// not handed a lateral sprint into the upright commit.
const VH_HANDOFF_MAX: f64 = 4.0;
/// Max pitch/yaw rate (rad/s) when arming Descend.
const OMEGA_HANDOFF_MAX: f64 = 0.12;
/// Min body-up · world-up when arming (~0.32 rad tilt).
const COS_TILT_HANDOFF: f64 = 0.95;
/// Hand-off AND gates must hold this long (s) before arming Descend — kills
/// one-frame chatter at the pad edge.
const HANDOFF_SETTLE_MIN_S: f64 = 0.40;

// --- Transit lean / envelope -------------------------------------------------
/// Lean cap during the full-throttle ascent burn (rad).
const LEAN_BURN_MAX: f64 = 0.30;
/// Soft ceiling for non-airplane reverse-brake lean (rad).
const LEAN_BRAKE_MAX: f64 = 0.90;
/// Conservative lean for stop-distance planning only (≈ prior cruise plan).
const LEAN_BRAKE_PLAN: f64 = 0.80;
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
/// Near-full throttle for climb burn and airplane cruise (gimbal authority
/// scales with thrust — full T is also max attitude authority).
const THR_FULL: f64 = 0.97;
/// Lean cap for airplane cruise (rad). cos(1.45)≈0.12 — matches dive floor in
/// [`long_range_hold_cos`]. Must stay at/above [`COS_TILT_AIM_AIR`].
const LEAN_LONG_MAX: f64 = 1.45;
/// Flip-recover only when nearly inverted in airplane / deep-lean mode.
/// Normal [`COS_TILT_AIM`] (0.30) would fight a legitimate nose-down dive.
const COS_TILT_AIM_AIR: f64 = 0.10;
/// Horizontal range (m) above which transit prefers airplane mode: full-T go +
/// pitch elevator while outside the predicted stop distance.
const LONG_AIRPLANE_RANGE_M: f64 = 1500.0;
/// Geometric hysteresis (m) when releasing latched reverse lean — must exceed
/// hand-off cheby so go↔brake does not chatter at the pad edge.
const BRAKE_RELEASE_MARGIN_M: f64 = HANDOFF_CHEBY_MAX_M;
/// Downrange speed built during the ascent burn (m/s). Ballistic coast keeps
/// whatever vh burnout leaves — cruise then accelerates freely on the envelope.
const V_CLIMB_H_MAX: f64 = 28.0;
/// Attitude √-profile planning accel (rad/s²).
const ALPHA_PLAN: f64 = 0.5;
const OMEGA_MAX: f64 = 1.15;
const KP_ATT: f64 = 2.0;
const KD_ATT: f64 = 3.0;
const KD_ROLL: f64 = 2.0;
/// Flip only when past the commanded lean cone (near-inverted), not mid-recovery.
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
    /// Nested lander used only in [`TargetPhase::Descend`] (physics closed-loop vertical).
    lander: LandingAutopilot,
    /// Latched reverse-lean brake (hysteresis) — kills go↔brake sway.
    brake_latched: bool,
    /// Terminal-pad brake blend latch (separate from mid-range [`brake_latched`]).
    terminal_brake_latched: bool,
    /// Seconds the hand-off AND gates have been continuously satisfied.
    handoff_settle_s: f64,
}

impl Default for TargetLandingAutopilot {
    fn default() -> Self {
        Self {
            enabled: false,
            complete: false,
            phase: TargetPhase::Climb,
            lander: LandingAutopilot::for_target_pad(),
            brake_latched: false,
            terminal_brake_latched: false,
            handoff_settle_s: 0.0,
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
            self.terminal_brake_latched = false;
            self.handoff_settle_s = 0.0;
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
        self.terminal_brake_latched = false;
        self.handoff_settle_s = 0.0;
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

    /// HUD helper: airplane full-T + pitch-elevator cruise is active.
    pub fn is_long_range_cruise(&self, pos: [f64; 3], target_xz: [f64; 2]) -> bool {
        if !self.enabled || self.complete || self.phase == TargetPhase::Descend {
            return false;
        }
        let dx = target_xz[0] - pos[0];
        let dz = target_xz[1] - pos[2];
        let range = (dx * dx + dz * dz).sqrt();
        range >= LONG_AIRPLANE_RANGE_M && !self.brake_latched
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
            let dx = target_xz[0] - pos[0];
            let dz = target_xz[1] - pos[2];
            let range = (dx * dx + dz * dz).sqrt();
            let near_handoff = range <= RANGE_TERMINAL_M
                || cheby <= HANDOFF_CHEBY_MAX_M + 20.0;
            // Do not drop back to Climb (full-T re-loft) while settling over the pad.
            let gate = if near_handoff {
                GATE_ALT_MIN - 40.0
            } else {
                GATE_ALT_MIN
            };
            self.phase = if alt >= gate {
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
        let v_cheby_handoff = chebyshev_closing_rate(pos, target_xz, state.velocity);
        let handoff_ready = self.phase != TargetPhase::Descend
            && alt >= GATE_ALT_MIN
            && cheby <= HANDOFF_CHEBY_MAX_M
            && vh <= VH_HANDOFF_MAX
            && v_cheby_handoff > -0.25
            && (cheby <= HANDOFF_CHEBY_MAX_M * 0.60
                || (v_cheby_handoff > 0.22 && vh <= VH_HANDOFF_MAX * 0.70))
            && om_pitch_yaw_sq <= OMEGA_HANDOFF_MAX * OMEGA_HANDOFF_MAX
            && world_up_in_body(&state.motor)[1] >= COS_TILT_HANDOFF;
        if handoff_ready {
            self.handoff_settle_s += dt;
        } else {
            self.handoff_settle_s = 0.0;
        }
        if handoff_ready && self.handoff_settle_s >= HANDOFF_SETTLE_MIN_S {
            self.phase = TargetPhase::Descend;
            self.lander.arm_from_transit(state);
            self.terminal_brake_latched = false;
            self.handoff_settle_s = 0.0;
        }

        match self.phase {
            TargetPhase::Climb | TargetPhase::Cruise => {
                let (cmd, brake, terminal_brake) = transit_command(
                    state,
                    target_xz,
                    pos,
                    self.brake_latched,
                    self.terminal_brake_latched,
                );
                self.brake_latched = brake;
                self.terminal_brake_latched = terminal_brake;
                cmd
            }
            TargetPhase::Descend => {
                self.brake_latched = false;
                self.terminal_brake_latched = false;
                let cmd = self.lander.update_target_descend(state, target_xz, dt);
                self.complete = self.lander.complete;
                cmd
            }
        }
    }
}

/// True when CoM XZ lies inside the T-mode success box (inner target, not painted pad).
#[inline]
pub fn inside_target_pad(pos: [f64; 3], target_xz: [f64; 2]) -> bool {
    on_target_success_square(pos, target_xz)
}

/// Throttle regime for lateral propulsion planning.
#[derive(Clone, Copy, Debug)]
enum LateralThrMode {
    /// Vertical-neutral reverse lean: `a_lat ≈ g·tan(θ)`.
    VerticalNeutral,
    /// Full-T go/brake: `a_lat = (T/m)·thr·sin(θ)`.
    #[allow(dead_code)]
    FullThrottle,
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

/// Shared per-frame inputs for stop-distance prediction and its inverse.
#[derive(Clone, Copy, Debug)]
struct HorizontalBrakePlan {
    beta: f64,
    a_prop: f64,
    v_end: f64,
    t_flip_go: f64,
    d_stop: f64,
}

impl HorizontalBrakePlan {
    fn evaluate(
        state: &RocketState,
        mass: f64,
        max_thrust: f64,
        ux: f64,
        uz: f64,
        vh: f64,
        v_approach: f64,
        in_airplane_range: bool,
        wind_approach: f64,
    ) -> Self {
        let beta = if state.moon_mode {
            0.0
        } else {
            state.params.air_drag_k / mass.max(1e-6)
        };
        let brake_lean = LEAN_BRAKE_PLAN;
        let a_prop = lateral_accel_for_lean(
            brake_lean,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        let v_end = VH_HANDOFF_MAX;
        let v_closing = (v_approach - wind_approach).max(0.0);
        let flip_brake = brake_flip_angle(state, ux, uz, vh, brake_lean);
        let t_flip_brake = brake_flip_time(flip_brake);
        let go_lean = if in_airplane_range {
            LEAN_LONG_MAX
        } else {
            LEAN_BRAKE_PLAN
        };
        let t_flip_go = if (go_lean - brake_lean).abs() < 1e-9 {
            t_flip_brake
        } else {
            brake_flip_time(brake_flip_angle(state, ux, uz, vh, go_lean))
        };
        let d_stop = predicted_stop_distance(v_closing, v_end, a_prop, beta, t_flip_brake);
        Self {
            beta,
            a_prop,
            v_end,
            t_flip_go,
            d_stop,
        }
    }
}

/// Braking distance along approach axis from `v` to `v_end` with propulsion `a_prop`
/// and quadratic drag coefficient `β = k/m` (Earth drag helps deceleration).
#[inline]
fn horizontal_burn_distance(v: f64, v_end: f64, a_prop: f64, beta: f64) -> f64 {
    let v = v.max(0.0);
    let v_end = v_end.max(0.0).min(v);
    if v <= v_end + 1e-6 || a_prop <= 1e-9 {
        return 0.0;
    }
    if beta <= 1e-12 {
        return (v * v - v_end * v_end) / (2.0 * a_prop);
    }
    let num = a_prop + beta * v * v;
    let den = a_prop + beta * v_end * v_end;
    if num <= den {
        return 0.0;
    }
    (0.5 / beta) * (num / den).ln()
}

/// Angle (rad) from current body-up to the commanded reverse-lean brake axis.
fn brake_flip_angle(state: &RocketState, ux: f64, uz: f64, vh: f64, lean_max: f64) -> f64 {
    let desired_raw = if vh > 0.5 {
        let s = vh.max(1.0);
        let vx = state.velocity[0];
        let vz = state.velocity[2];
        [-vx / s, AIM_Y_BIAS, -vz / s]
    } else {
        [ux, AIM_Y_BIAS, uz]
    };
    let desired = clamp_tilt(desired_raw, lean_max);
    let len = (desired[0] * desired[0] + desired[1] * desired[1] + desired[2] * desired[2]).sqrt();
    if len <= 1e-9 {
        return 0.0;
    }
    let d = motor_inverse_rotate_vector(
        &state.motor,
        [desired[0] / len, desired[1] / len, desired[2] / len],
    );
    let up_y = d[1].clamp(-1.0, 1.0);
    let (_, angle) = axis_angle_from_cross([d[2], 0.0, -d[0]], up_y);
    angle.max(0.0)
}

/// Conservative √-profile time to rotate into reverse lean (s).
#[inline]
fn brake_flip_time(angle: f64) -> f64 {
    if angle <= 1e-6 {
        return 0.0;
    }
    let t_sqrt = (2.0 * angle / ALPHA_PLAN).sqrt();
    let t_linear = angle / OMEGA_MAX;
    t_sqrt.max(t_linear)
}

/// Predicted horizontal stop distance (m): attitude flip coast + propulsive burn.
#[inline]
fn predicted_stop_distance(
    v_approach: f64,
    v_end: f64,
    a_prop: f64,
    beta: f64,
    t_flip: f64,
) -> f64 {
    let v = v_approach.max(0.0);
    v * t_flip + horizontal_burn_distance(v, v_end, a_prop, beta)
}

/// Max approach speed (m/s) that still fits in `range_eff` before braking.
///
/// Vacuum seed from `v·t + (v²−v_end²)/(2a) = range`, then monotone bisection
/// (≤16 steps) so the result matches `predicted_stop_distance ≤ range_eff`.
#[inline]
fn allowed_approach_speed(
    range_eff: f64,
    v_end: f64,
    a_prop: f64,
    beta: f64,
    t_flip: f64,
) -> f64 {
    if range_eff <= 1e-6 {
        return v_end;
    }
    let disc = a_prop * a_prop * t_flip * t_flip + 2.0 * a_prop * range_eff + v_end * v_end;
    let mut hi = if disc > 0.0 {
        (-a_prop * t_flip + disc.sqrt()).max(v_end + 1.0)
    } else {
        v_end + 1.0
    };
    while predicted_stop_distance(hi, v_end, a_prop, beta, t_flip) < range_eff && hi < 900.0 {
        hi *= 1.5;
    }
    let mut lo = v_end;
    for _ in 0..16 {
        let mid = 0.5 * (lo + hi);
        if predicted_stop_distance(mid, v_end, a_prop, beta, t_flip) <= range_eff {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// Physics-predicted time (s) until hand-off AND gates clear.
#[derive(Clone, Copy, Debug)]
struct HandoffSettlePlan {
    t_att: f64,
    t_vh: f64,
    t_pos: f64,
    t_settle: f64,
}

impl HandoffSettlePlan {
    fn cleared(&self) -> bool {
        self.t_settle <= 1e-3
    }

    fn evaluate(
        state: &RocketState,
        pos: [f64; 3],
        target_xz: [f64; 2],
        vh: f64,
        v_cheby: f64,
        lean_cmd: f64,
        beta: f64,
    ) -> Self {
        let up_y = world_up_in_body(&state.motor)[1];
        let om = state.omega;
        let omega_py = (om[0] * om[0] + om[2] * om[2]).sqrt();
        let t_att = predicted_attitude_handoff_time(up_y, omega_py);

        let a_lat = lateral_accel_for_lean(
            lean_cmd,
            LateralThrMode::VerticalNeutral,
            state.params.mass,
            state.params.max_thrust,
        );
        let t_vh = if vh <= VH_HANDOFF_MAX {
            0.0
        } else {
            predicted_decel_time(vh, VH_HANDOFF_MAX, a_lat, beta)
        };

        let cheby = chebyshev_xz(pos, target_xz);
        let t_pos = if cheby <= HANDOFF_CHEBY_MAX_M {
            0.0
        } else {
            let delta = cheby - HANDOFF_CHEBY_MAX_M;
            predicted_chebyshev_settle_time(delta, v_cheby, vh, a_lat)
        };

        let t_settle = t_att.max(t_vh).max(t_pos);
        Self {
            t_att,
            t_vh,
            t_pos,
            t_settle,
        }
    }
}

/// Horizontal propulsive accel (m/s²) at `lean` rad for the given throttle regime.
#[inline]
fn lateral_accel_for_lean(
    lean: f64,
    mode: LateralThrMode,
    mass: f64,
    max_thrust: f64,
) -> f64 {
    let lean = lean.max(0.0);
    match mode {
        LateralThrMode::VerticalNeutral => (GRAVITY * lean.tan()).max(0.15),
        LateralThrMode::FullThrottle => {
            (THR_FULL * max_thrust / mass.max(1e-6) * lean.sin()).max(0.15)
        }
    }
}

/// Lean (rad) needed for lateral accel `a_req` under the given throttle regime.
#[inline]
fn lean_for_lateral_accel(
    a_req: f64,
    mode: LateralThrMode,
    mass: f64,
    max_thrust: f64,
    lean_cap: f64,
) -> f64 {
    let a = a_req.max(0.0);
    if a <= 1e-9 {
        return 0.06;
    }
    let lean = match mode {
        LateralThrMode::VerticalNeutral => (a / GRAVITY).atan(),
        LateralThrMode::FullThrottle => {
            let am = THR_FULL * max_thrust / mass.max(1e-6);
            (a / am).asin()
        }
    };
    lean.clamp(0.06, lean_cap)
}

/// Signed rate (m/s) at which Chebyshev pad offset is shrinking (negative ⇒ diverging).
#[inline]
fn chebyshev_closing_rate(pos: [f64; 3], target_xz: [f64; 2], vel: [f64; 3]) -> f64 {
    let ex = pos[0] - target_xz[0];
    let ez = pos[2] - target_xz[1];
    if ex.abs() >= ez.abs() {
        if ex.abs() <= 1e-6 {
            0.0
        } else {
            -ex.signum() * vel[0]
        }
    } else if ez.abs() <= 1e-6 {
        0.0
    } else {
        -ez.signum() * vel[2]
    }
}

/// Time (s) to shrink Chebyshev offset by `delta` (m) with closing/diverging rate and lateral accel.
#[inline]
fn predicted_chebyshev_settle_time(delta: f64, v_cheby: f64, vh: f64, a_lat: f64) -> f64 {
    if delta <= 1e-6 {
        return 0.0;
    }
    let a = a_lat.max(0.15);
    if v_cheby > 0.2 {
        return predicted_position_time(delta, v_cheby, a);
    }
    // Overshoot / diverging: stop lateral speed then close the gap.
    let v_stop = vh.max((-v_cheby).max(0.0));
    let t_stop = if v_stop > 1e-3 {
        predicted_decel_time(v_stop, 0.0, a, 0.0)
    } else {
        0.0
    };
    t_stop + (2.0 * delta / a).sqrt()
}

/// Time (s) to reach hand-off tilt and pitch/yaw rate from current state.
#[inline]
fn predicted_attitude_handoff_time(up_y: f64, omega_py: f64) -> f64 {
    let mut t: f64 = 0.0;
    if up_y < COS_TILT_HANDOFF {
        let theta = up_y.clamp(-1.0, 1.0).acos();
        let theta_handoff = COS_TILT_HANDOFF.acos();
        let angle = (theta - theta_handoff).max(0.0);
        t = t.max(brake_flip_time(angle));
    }
    if omega_py > OMEGA_HANDOFF_MAX {
        let excess = omega_py - OMEGA_HANDOFF_MAX;
        let t_omega = (2.0 * excess / ALPHA_PLAN)
            .sqrt()
            .max(excess / OMEGA_MAX);
        t = t.max(t_omega);
    }
    t
}

/// Time (s) to decelerate horizontal speed from `v` to `v_end` at `a_prop` (drag helps).
#[inline]
fn predicted_decel_time(v: f64, v_end: f64, a_prop: f64, beta: f64) -> f64 {
    let v = v.max(0.0);
    let v_end = v_end.max(0.0).min(v);
    let dv = v - v_end;
    if dv <= 1e-6 || a_prop <= 1e-9 {
        return 0.0;
    }
    if beta <= 1e-12 {
        return dv / a_prop;
    }
    // ∫ dv/(a + βv²) from v_end to v
    let scale = (a_prop * beta).sqrt();
    let atan_hi = (v * beta / a_prop).sqrt().atan();
    let atan_lo = (v_end * beta / a_prop).sqrt().atan();
    ((atan_hi - atan_lo) / scale).max(0.0)
}

/// Time (s) to close a Chebyshev gap `delta` (m) with closing speed and lateral accel.
#[inline]
fn predicted_position_time(delta: f64, v_close: f64, a_lat: f64) -> f64 {
    if delta <= 1e-6 {
        return 0.0;
    }
    let a = a_lat.max(0.1);
    let v = v_close.max(0.0);
    // delta ≈ v·t + 0.5·a·t²  ⇒  t = (−v + sqrt(v² + 2·a·delta)) / a
    let disc = v * v + 2.0 * a * delta;
    if disc <= 0.0 {
        return (2.0 * delta / a).sqrt();
    }
    ((-v + disc.sqrt()) / a).max((2.0 * delta / a).sqrt())
}

/// Physics brake gate with geometric latch hysteresis.
fn update_brake_latch(
    brake_latched: bool,
    terminal: bool,
    range_eff: f64,
    d_stop: f64,
    v_approach: f64,
) -> bool {
    if terminal {
        return false;
    }
    let overshoot = v_approach < -1.5 && range_eff > 0.0;
    if overshoot {
        return true;
    }
    if brake_latched {
        range_eff <= d_stop + BRAKE_RELEASE_MARGIN_M
    } else {
        range_eff <= d_stop
    }
}

/// Continuous brake blend for terminal pad settle (0 = seek, 1 = full reverse).
/// Returns `(weight, latched)` with hysteresis so go↔brake does not chatter.
fn terminal_brake_blend(
    v_cheby: f64,
    vh: f64,
    v_approach: f64,
    cheby: f64,
    latched: bool,
) -> (f64, bool) {
    let mut score = 0.0_f64;
    if v_cheby < 0.0 {
        score += (-v_cheby / 2.5).min(1.0);
    }
    if v_approach < 0.0 {
        score += (-v_approach / 2.0).min(0.8);
    }
    if cheby <= HANDOFF_CHEBY_MAX_M && vh > VH_HANDOFF_MAX * 0.82 {
        score += ((vh - VH_HANDOFF_MAX * 0.82) / VH_HANDOFF_MAX).min(1.0);
    }
    score = score.clamp(0.0, 1.5);

    if latched {
        let release = score < 0.15 && v_cheby > 0.05 && vh < VH_HANDOFF_MAX * 0.80;
        let w = if release {
            (score / 0.15 * 0.2).clamp(0.0, 0.2)
        } else {
            (0.30 + 0.70 * score.min(1.0)).clamp(0.30, 1.0)
        };
        (w, !release)
    } else {
        let w = if score > 0.08 {
            (0.15 + 0.85 * (score / 1.0).min(1.0)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let engage = w > 0.55;
        (w, engage)
    }
}

/// Continuous damped settle toward the hand-off box (no discrete go/brake modes).
fn terminal_settle_aim(
    hp: HandoffSettlePlan,
    ux: f64,
    uz: f64,
    need_x: f64,
    need_z: f64,
    vx: f64,
    vz: f64,
    vh: f64,
    cheby: f64,
    v_cheby: f64,
    v_approach: f64,
    mass: f64,
    max_thrust: f64,
    brake_mode: LateralThrMode,
    terminal_brake_latched: bool,
) -> ([f64; 3], f64, bool) {
    let pos_urgency = (hp.t_pos / 3.0).clamp(0.0, 1.0);
    let vh_urgency = (hp.t_vh / 3.0).clamp(0.0, 1.0);
    let settle_urgency = pos_urgency.max(vh_urgency);
    let quiet = hp.cleared() || hp.t_settle < 0.35;

    let (brake_w, new_latch) =
        terminal_brake_blend(v_cheby, vh, v_approach, cheby, terminal_brake_latched);

    // Shrink gains inside the hand-off box to avoid limit cycles at the edge.
    let inside_frac = ((HANDOFF_CHEBY_MAX_M - cheby) / HANDOFF_CHEBY_MAX_M).clamp(0.0, 1.0);
    let gain_scale = 0.40 + 0.60 * (1.0 - inside_frac);

    let (k_pos, k_vel) = if quiet {
        (0.07 + 0.08 * pos_urgency, 0.40 + 0.20 * pos_urgency)
    } else if cheby > HANDOFF_CHEBY_MAX_M {
        (
            (0.14 + 0.28 * settle_urgency).clamp(0.14, 0.38),
            (0.55 + 0.32 * settle_urgency).clamp(0.55, 0.72),
        )
    } else {
        (
            (0.10 + 0.22 * settle_urgency).clamp(0.10, 0.28),
            (0.50 + 0.28 * settle_urgency).clamp(0.50, 0.65),
        )
    };

    let dir_bias = if cheby <= HANDOFF_CHEBY_MAX_M {
        0.18 + 0.12 * inside_frac
    } else {
        0.32 + 0.08 * (1.0 - (cheby - HANDOFF_CHEBY_MAX_M).min(30.0) / 30.0)
    };

    let v_ref = vh.max(1.5);
    let mut aim_x = dir_bias * ux + gain_scale * (k_pos * need_x - k_vel * vx / v_ref);
    let mut aim_z = dir_bias * uz + gain_scale * (k_pos * need_z - k_vel * vz / v_ref);

    // Blend toward velocity-opposing aim (continuous, not a hard mode switch).
    let s = vh.max(0.9);
    aim_x = (1.0 - brake_w) * aim_x + brake_w * (-vx / s);
    aim_z = (1.0 - brake_w) * aim_z + brake_w * (-vz / s);

    // Upright bias while attitude settle dominates (replaces att_dominant branch).
    let att_blend = if hp.t_att > 0.15 {
        (hp.t_att / 2.5).clamp(0.0, 0.55) * (1.0 - brake_w * 0.6)
    } else {
        0.0
    };
    aim_x *= 1.0 - att_blend * 0.85;
    aim_z *= 1.0 - att_blend * 0.85;

    let a_req_x = gain_scale * (k_pos * need_x - k_vel * vx) + brake_w * 0.55 * (-vx);
    let a_req_z = gain_scale * (k_pos * need_z - k_vel * vz) + brake_w * 0.55 * (-vz);
    let a_lat = (a_req_x * a_req_x + a_req_z * a_req_z).sqrt();
    let overshoot_boost = if v_cheby < -0.3 {
        (-v_cheby).min(vh) * 0.35 + 0.3
    } else {
        0.0
    };

    let lean_cap = if cheby <= HANDOFF_CHEBY_MAX_M {
        (0.07 + 0.014 * cheby).clamp(0.06, 0.20)
    } else {
        (0.10 + 0.022 * (cheby - HANDOFF_CHEBY_MAX_M).min(25.0) + 0.18 * settle_urgency)
            .clamp(0.10, LEAN_BRAKE_MAX * 0.65)
    };

    let lean = lean_for_lateral_accel(
        a_lat.max(overshoot_boost),
        brake_mode,
        mass,
        max_thrust,
        lean_cap,
    );

    ([aim_x, AIM_Y_BIAS, aim_z], lean, new_latch)
}

/// Full-T airplane aim: pitch elevator holds `alt_hold` while leaning to pad.
#[inline]
fn airplane_hold_aim(
    ux: f64,
    uz: f64,
    alt: f64,
    alt_hold: f64,
    vy: f64,
    hover: f64,
) -> ([f64; 3], f64, bool, bool) {
    let cos_up = long_range_hold_cos(alt, alt_hold, vy, hover);
    (
        long_range_go_aim(ux, uz, cos_up),
        LEAN_LONG_MAX,
        true,
        true,
    )
}

/// Climb + translate toward the lofted pad, or airplane cruise when far out.
///
/// Short range: burn until ballistic apogee clears the loft, optional upright
/// straighten, then envelope go/brake. Airplane range: full T + pitch elevator
/// (see [`airplane_hold_aim`]). Returns `(command, updated_brake_latch, terminal_brake_latch)`.
fn transit_command(
    state: &RocketState,
    target_xz: [f64; 2],
    pos: [f64; 3],
    brake_latched: bool,
    terminal_brake_latched: bool,
) -> (ControlCommand, bool, bool) {
    let dx = target_xz[0] - pos[0];
    let dz = target_xz[1] - pos[2];
    let range = (dx * dx + dz * dz).sqrt();
    let cheby = chebyshev_xz(pos, target_xz);
    let near_handoff = range <= RANGE_TERMINAL_M || cheby <= HANDOFF_CHEBY_MAX_M + 20.0;
    let lofted = pos[1] >= GATE_ALT_MIN
        || (near_handoff && pos[1] >= GATE_ALT_MIN - 40.0);
    let mu_long = long_range_weight(range);

    let vx = state.velocity[0];
    let vy = state.velocity[1];
    let vz = state.velocity[2];
    let vh = (vx * vx + vz * vz).sqrt();

    let mass = state.params.mass;
    let max_thrust = state.params.max_thrust;
    let hover = mass * GRAVITY / max_thrust;
    let brake_mode = LateralThrMode::VerticalNeutral;
    let a_brake_max = lateral_accel_for_lean(
        LEAN_BRAKE_PLAN,
        brake_mode,
        mass,
        max_thrust,
    );

    // Short-hop loft apogee (blends toward cruise alt with mu_long). Airplane
    // mode ignores this and stays powered until the altitude gate.
    let apogee_target =
        APOGEE_TARGET_M + mu_long * ((LONG_CRUISE_ALT_M - 40.0) - APOGEE_TARGET_M);
    let straighten_m = apogee_target - 180.0;

    let apogee = pos[1] + vy.max(0.0) * vy.max(0.0) / (2.0 * GRAVITY);
    let in_airplane_range = range >= LONG_AIRPLANE_RANGE_M;
    let burn_up = !lofted
        && !(near_handoff && pos[1] >= GATE_ALT_MIN - 40.0)
        && (in_airplane_range || apogee < apogee_target);
    // Powered-cruise weight: 1 at vy ≤ +3, 0 at vy ≥ +8 (ballistic coast).
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
    let v_cheby = chebyshev_closing_rate(pos, target_xz, state.velocity);

    let terminal = lofted && range <= RANGE_TERMINAL_M;
    let range_eff = (range - RANGE_TERMINAL_M).max(0.0);

    let plan = (!terminal).then(|| {
        HorizontalBrakePlan::evaluate(
            state,
            mass,
            max_thrust,
            ux,
            uz,
            vh,
            v_approach,
            in_airplane_range,
            0.0, // future: wind dot approach axis
        )
    });
    let beta = if state.moon_mode {
        0.0
    } else {
        state.params.air_drag_k / mass.max(1e-6)
    };
    let handoff_plan = if lofted && (terminal || brake_latched || cheby <= HANDOFF_CHEBY_MAX_M + 35.0)
    {
        let lean_cmd = if brake_latched {
            LEAN_BRAKE_MAX
        } else {
            lean_for_lateral_accel(
                a_brake_max * 0.65,
                brake_mode,
                mass,
                max_thrust,
                LEAN_BRAKE_MAX,
            )
        };
        Some(HandoffSettlePlan::evaluate(
            state,
            pos,
            target_xz,
            vh,
            v_cheby,
            lean_cmd,
            beta,
        ))
    } else {
        None
    };
    let brake = if terminal {
        false
    } else {
        let p = plan.unwrap();
        update_brake_latch(brake_latched, terminal, range_eff, p.d_stop, v_approach)
    };
    let airplane_go = in_airplane_range && !brake;

    let v_allow = if terminal {
        let delta = (cheby - HANDOFF_CHEBY_MAX_M).max(0.0);
        let t_flip = brake_flip_time(0.15);
        let v_phys = if delta > 0.5 {
            allowed_approach_speed(delta, 0.0, a_brake_max, beta, t_flip)
        } else {
            VH_HANDOFF_MAX * 0.45
        };
        let outside_handoff = cheby > HANDOFF_CHEBY_MAX_M + 1.0;
        let v_cap = if cheby > HANDOFF_CHEBY_MAX_M + 8.0 {
            (0.20 * (cheby - HANDOFF_CHEBY_MAX_M).max(1.0) + 1.0).min(VH_HANDOFF_MAX * 0.60)
        } else if outside_handoff {
            2.5_f64
        } else {
            VH_HANDOFF_MAX * 0.75
        };
        v_phys
            .min(0.10 * (range - 4.0).max(0.0))
            .min(v_cap)
            .clamp(0.0, VH_HANDOFF_MAX)
    } else {
        let p = plan.unwrap();
        let v = allowed_approach_speed(
            range_eff,
            p.v_end,
            p.a_prop,
            p.beta,
            p.t_flip_go,
        );
        if burn_up || ballistic {
            v.min(V_CLIMB_H_MAX)
        } else {
            v
        }
    };

    let need_x = ux * v_allow - vx;
    let need_z = uz * v_allow - vz;

    let far = lofted && cruise_w > 0.75 && !terminal && range > RANGE_FAR_M;
    let far_or_overshoot = far
        || (lofted
            && cruise_w > 0.75
            && !terminal
            && range > RANGE_TERMINAL_M
            && v_approach < -1.5);

    // Airplane range: hold cruise altitude. Closer in: blend 520→800 m.
    let alt_hold = if in_airplane_range {
        LONG_CRUISE_ALT_M
    } else {
        CRUISE_ALT_CAP + mu_long * (LONG_CRUISE_ALT_M - CRUISE_ALT_CAP)
    };

    // Aim regime (mutually exclusive by range / loft state).
    let (desired_raw, lean_max, deep, force_full_thr, terminal_brake_out) = if airplane_go {
        // Full-T go + pitch elevator (ascent and cruise share one law).
        let (d, l, de, f) = airplane_hold_aim(ux, uz, pos[1], alt_hold, vy, hover);
        (d, l, de, f, terminal_brake_latched)
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
            terminal_brake_latched,
        )
    } else if terminal {
        let hp = handoff_plan.unwrap();
        let (desired_raw, lean, new_terminal_brake) = terminal_settle_aim(
            hp,
            ux,
            uz,
            need_x,
            need_z,
            vx,
            vz,
            vh,
            cheby,
            v_cheby,
            v_approach,
            mass,
            max_thrust,
            brake_mode,
            terminal_brake_latched,
        );
        (desired_raw, lean, false, false, new_terminal_brake)
    } else if far_or_overshoot {
        // Mid/long range: discrete go or reverse-brake aim.
        let s = vh.max(1.0);
        let brake_lean = lean_for_lateral_accel(
            a_brake_max * 0.85,
            brake_mode,
            mass,
            max_thrust,
            LEAN_BRAKE_MAX,
        );
        if brake {
            (
                [-vx / s, AIM_Y_BIAS, -vz / s],
                brake_lean,
                true,
                false,
                terminal_brake_latched,
            )
        } else {
            (
                [ux, AIM_Y_BIAS, uz],
                brake_lean,
                true,
                false,
                terminal_brake_latched,
            )
        }
    } else {
        // Ballistic / shallow mid: reverse only when latched.
        let s = vh.max(1.0);
        let cruise_lean = lean_for_lateral_accel(
            a_brake_max * 0.55,
            brake_mode,
            mass,
            max_thrust,
            LEAN_BRAKE_MAX,
        );
        if brake && lofted && !terminal {
            (
                [-vx / s, AIM_Y_BIAS, -vz / s],
                cruise_lean,
                false,
                false,
                terminal_brake_latched,
            )
        } else {
            (
                [ux + 0.05 * need_x, AIM_Y_BIAS, uz + 0.05 * need_z],
                cruise_lean,
                false,
                false,
                terminal_brake_latched,
            )
        }
    };

    // Upright straighten only on short-hop burn (airplane owns altitude by pitch).
    let straighten = burn_up && !in_airplane_range && apogee >= straighten_m;
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
    // Deep / airplane lean: low flip gate so nose-down is tracked, not "recovered".
    let flip_cos = if force_full_thr || deep {
        COS_TILT_AIM_AIR
    } else {
        COS_TILT_AIM
    };
    let (pitch, yaw, roll, up_y) = attitude_toward(state, desired, flip_cos);

    let upy_floor = if deep { 0.45 } else { 0.40 };
    let hover_cmd = (hover / up_y.max(upy_floor)).clamp(0.0, 0.95);

    let mut throttle = if force_full_thr || burn_up {
        // Airplane + short climb: near-full T (pitch owns altitude when airplane).
        THR_FULL
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
    let hp = handoff_plan;
    if force_full_thr {
        // Do not demote airplane full-T to hover/cos — pitch owns altitude.
        throttle = THR_FULL;
    } else if deep {
        // Reverse lean: vertical-neutral hover/cos + modest effort torque.
        let t_neutral = hover_cmd;
        throttle = (t_neutral + 0.06 * effort).clamp(t_neutral * 0.94, t_neutral + 0.05);
    } else if terminal {
        let p = hp.unwrap();
        let quiet = p.cleared() || p.t_settle < 0.35;
        let att_blend = (p.t_att / 2.5).clamp(0.0, 0.55);
        let motion_blend = (p.t_pos.max(p.t_vh) / 3.5).clamp(0.0, 0.45);
        let t_neutral = hover_cmd;
        let t_quiet = (t_neutral * 0.92).clamp(t_neutral * 0.88, t_neutral + 0.04);
        let t_att = (t_neutral * (0.90 + 0.06 * att_blend) + 0.18 * att_blend * effort)
            .clamp(t_neutral * 0.88, 0.90);
        let t_motion = (t_neutral * (0.94 - 0.06 * motion_blend))
            .clamp(t_neutral * 0.86, t_neutral + 0.05);
        let t_target = (1.0 - att_blend) * t_motion + att_blend * t_att;
        throttle = if quiet {
            throttle.clamp(t_quiet, t_quiet + 0.03)
        } else {
            throttle.clamp(t_target * 0.92, (t_target + 0.08).min(0.85))
        };
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
    (cmd, brake, terminal_brake_out)
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
    use crate::landing::TARGET_SUCCESS_HALF_M;

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
        assert!(inside_target_pad(
            [500.0 + TARGET_SUCCESS_HALF_M, 0.0, 0.0],
            [500.0, 0.0]
        ));
        assert!(!inside_target_pad(
            [500.0 + TARGET_SUCCESS_HALF_M + 0.1, 0.0, 0.0],
            [500.0, 0.0]
        ));
        // Painted pad is wider than the success box.
        assert!(
            TARGET_PAD_HALF_M > TARGET_SUCCESS_HALF_M,
            "visual pad should exceed success box"
        );
    }

    #[test]
    fn vertical_neutral_lateral_accel_matches_tan() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let lean = 0.5;
        let a = lateral_accel_for_lean(
            lean,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        assert!((a - GRAVITY * lean.tan()).abs() < 1e-9);
    }

    #[test]
    fn full_throttle_lateral_accel_matches_plant() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let lean = 0.5;
        let a = lateral_accel_for_lean(lean, LateralThrMode::FullThrottle, mass, max_thrust);
        let expected = THR_FULL * max_thrust / mass * lean.sin();
        assert!((a - expected).abs() < 1e-9);
    }

    #[test]
    fn kill_climb_never_commands_positive() {
        assert!(kill_climb_vy(10.0) < 0.0);
        assert_eq!(kill_climb_vy(-3.0), 0.0);
        assert_eq!(kill_climb_vy(0.0), 0.0);
    }

    #[test]
    fn vacuum_burn_distance_matches_kinematics() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let a = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        let d = horizontal_burn_distance(40.0, VH_HANDOFF_MAX, a, 0.0);
        let expected = (40.0 * 40.0 - VH_HANDOFF_MAX * VH_HANDOFF_MAX) / (2.0 * a);
        assert!((d - expected).abs() < 1e-6, "d={d} expected={expected}");
    }

    #[test]
    fn drag_shortens_burn_distance() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let a = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        let d_vac = horizontal_burn_distance(60.0, VH_HANDOFF_MAX, a, 0.0);
        let d_drag = horizontal_burn_distance(60.0, VH_HANDOFF_MAX, a, 0.001);
        assert!(
            d_drag < d_vac,
            "drag should help braking: vac={d_vac} drag={d_drag}"
        );
    }

    #[test]
    fn lower_thrust_increases_stop_distance() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [50.0, 0.0, 0.0];
        let mass = state.params.mass;
        let max_thrust = state.params.max_thrust;
        let a_hi = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::FullThrottle,
            mass,
            max_thrust,
        );
        state.params.max_thrust *= 0.5;
        let beta = state.params.air_drag_k / state.params.mass;
        let a_lo = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::FullThrottle,
            state.params.mass,
            state.params.max_thrust,
        );
        let d_hi = predicted_stop_distance(50.0, VH_HANDOFF_MAX, a_hi, beta, 0.5);
        let d_lo = predicted_stop_distance(50.0, VH_HANDOFF_MAX, a_lo, beta, 0.5);
        assert!(d_lo > d_hi, "weaker thrust needs longer stop: hi={d_hi} lo={d_lo}");
        assert!(a_lo < a_hi, "full-T lateral accel should scale with thrust");
    }

    #[test]
    fn allowed_speed_inverts_stop_distance_vacuum() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let a = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        let t = 0.5;
        let range = 400.0;
        let v = allowed_approach_speed(range, VH_HANDOFF_MAX, a, 0.0, t);
        let d = predicted_stop_distance(v, VH_HANDOFF_MAX, a, 0.0, t);
        assert!(
            (d - range).abs() < 0.5,
            "v={v} d={d} range={range}"
        );
    }

    #[test]
    fn allowed_speed_grows_with_range() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let a = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        let v_near = allowed_approach_speed(200.0, VH_HANDOFF_MAX, a, 0.0, 0.5);
        let v_far = allowed_approach_speed(800.0, VH_HANDOFF_MAX, a, 0.0, 0.5);
        assert!(v_far > v_near, "v_near={v_near} v_far={v_far}");
    }

    #[test]
    fn predicted_decel_time_drag_matches_integral() {
        let a = 8.0;
        let beta = 0.001;
        let v = 30.0;
        let v_end = 6.5;
        let t = predicted_decel_time(v, v_end, a, beta);
        let scale = (a * beta).sqrt();
        let expected = ((v * beta / a).sqrt().atan() - (v_end * beta / a).sqrt().atan()) / scale;
        assert!((t - expected).abs() < 1e-9, "t={t} expected={expected}");
    }

    #[test]
    fn airplane_brakes_when_inside_predicted_stop() {
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M);
        state.contacting = false;
        state.velocity = [90.0, 0.0, 0.0];
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        // Fast close inside physics stop distance — must leave full-T airplane go.
        let target = [600.0, 0.0];
        let cmd = ap.update(&state, target, 1.0 / 120.0);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        assert!(
            !ap.is_long_range_cruise(state.position(), target),
            "braking must drop airplane HUD flag"
        );
        assert!(
            cmd.throttle < 0.92,
            "expected brake throttle not full-T, thr={}",
            cmd.throttle
        );
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
        // Closing very fast at short remaining range → inside predicted stop distance.
        state.velocity = [80.0, 0.0, 0.0];
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
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
    fn handoff_settle_time_zero_when_already_ready() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.motor = crate::euclidean_pga::motor_from_pose(500.0, 500.0, 0.0, 0.0, 0.0, 0.0);
        state.velocity = [2.0, 0.0, 0.0];
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        let plan = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            2.0,
            v_cheby,
            LEAN_BRAKE_MAX,
            0.0,
        );
        assert!(
            plan.t_settle < 0.5,
            "near-pad quiet state should be nearly cleared, t={}",
            plan.t_settle
        );
    }

    #[test]
    fn handoff_settle_time_positive_when_tilted() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.motor = crate::euclidean_pga::motor_from_pose(500.0, 500.0, 0.0, 0.35, 0.0, 0.0);
        state.velocity = [2.0, 0.0, 0.0];
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        let plan = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            2.0,
            v_cheby,
            LEAN_BRAKE_MAX,
            0.0,
        );
        assert!(
            plan.t_att > 0.1,
            "tilted body should need attitude settle time, t_att={}",
            plan.t_att
        );
    }

    #[test]
    fn handoff_settle_time_positive_when_fast() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.motor = crate::euclidean_pga::motor_from_pose(500.0, 500.0, 0.0, 0.0, 0.0, 0.0);
        state.velocity = [20.0, 0.0, 0.0];
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        let plan = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            20.0,
            v_cheby,
            LEAN_BRAKE_MAX,
            0.0,
        );
        assert!(
            plan.t_vh > 0.5,
            "fast horizontal speed should need decel time, t_vh={}",
            plan.t_vh
        );
    }

    #[test]
    fn handoff_settle_time_positive_when_overshooting() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.motor = crate::euclidean_pga::motor_from_pose(540.0, 500.0, 0.0, 0.0, 0.0, 0.0);
        state.velocity = [6.0, 0.0, 0.0];
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        assert!(v_cheby < 0.0, "past pad should diverge, v_cheby={v_cheby}");
        let plan = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            6.0,
            v_cheby,
            LEAN_BRAKE_MAX,
            0.0,
        );
        assert!(
            plan.t_pos > 1.0,
            "overshoot should predict long position settle, t_pos={}",
            plan.t_pos
        );
    }

    #[test]
    fn chebyshev_closing_rate_tracks_worst_axis() {
        let pos = [510.0, 500.0, 3.0];
        let target = [500.0, 0.0];
        let v_close_x = chebyshev_closing_rate(pos, target, [-2.0, 0.0, 0.0]);
        assert!(v_close_x > 0.0, "moving toward pad on X should close, got {v_close_x}");
        let v_div = chebyshev_closing_rate(pos, target, [2.0, 0.0, 0.0]);
        assert!(v_div < 0.0, "moving away on X should diverge, got {v_div}");
    }

    #[test]
    fn deep_lean_uses_vertical_neutral_not_starvation_cap() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [8.0, 0.0, 0.0];
        state.motor = crate::euclidean_pga::motor_from_pose(520.0, 500.0, 0.0, 0.05, 0.0, 0.0);
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let cmd = ap.update(&state, [500.0, 0.0], 1.0 / 120.0);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        assert!(
            cmd.throttle > 0.32,
            "terminal brake should keep torque headroom, thr={}",
            cmd.throttle
        );
        assert!(
            cmd.throttle < 0.80,
            "terminal brake must stay near vertical-neutral, thr={}",
            cmd.throttle
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
