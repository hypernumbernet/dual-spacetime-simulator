//! Target-pad autopilot (T key): loft, cruise to pad, terminal land.
//!
//! **Climb (below altitude gate):** full-throttle upright liftoff, then an open-loop
//! pitch program that tilts toward the pad — no MPC, no velocity-feedback lean.
//!
//! **Cruise (gate cleared):** receding-horizon MPC over a simplified 3DOF rollout
//! picks among cruise, brake, coast, sink, and (when far) airplane-hold candidates
//! every other frame. The predictor couples translate (gravity + thrust + quadratic
//! drag + lean lag). A parallel closed-form `d_stop` gate latches reverse-lean
//! braking with geometric hysteresis.
//!
//! **Airplane range (horizontal ≳ 1.5 km):** full throttle toward T while outside
//! `d_stop`; altitude is pitch only ([`long_range_hold_cos`] /
//! [`long_range_go_aim`]). Hold [`LONG_CRUISE_ALT_M`] (~520 m, same band as the
//! short-hop cruise cap). The climb apogee target soft-blends toward 480 m via
//! [`long_range_weight`]. Same stop-distance gate hands off to reverse lean. Inside
//! the terminal-settle envelope (enter ~90 m, exit ~140 m): sequenced
//! Brake → Upright → Trim drives lean and throttle while sinking toward
//! [`HANDOFF_ALT_M`] (~300 m). Hard AND gates (position / attitude only) arm
//! [`TargetPhase::Descend`] via [`LandingAutopilot::update_target_descend`]
//! (closed-loop suicide burn). Above [`h_freefall_m`] (Earth 6000 m / Moon 10000 m),
//! transit flies a nose-down **dive** (full-T acceleration toward ground / pad)
//! under the speed envelope; lateral steering when `range > alt`, otherwise pure
//! vertical dive. Overspeed flips upright and brakes via [`freefall_v_cap`]
//! (safe descent speed is highest priority).
//!
//! Attitude: PGA inverse sandwich ([`motor_inverse_rotate_vector`] /
//! [`world_up_in_body`]), desired thrust tracked without an upright flip fight
//! during dive, then rate-limited (aim slew + gimbal actuator) before the attitude PD.

use crate::euclidean_pga::{motor_inverse_rotate_vector, world_up_in_body};
use crate::fuzzy::{
    blend_vec3, careful_aggression, careful_terminal_latch, cruise_brake_hardness,
    freefall_overspeed_mu, long_range_go_aim, long_range_hold_cos, long_range_weight, ramp,
    ramp_down, settle_aim_blend, settle_brake_lean_scale, settle_lean_freedom, settle_motion_scale,
    settle_trim_rate_gate, slew_throttle, CruiseThrottleFuzzy, FreefallThrottleFuzzy,
    CAREFUL_NEAR_M, CAREFUL_RANGE_M, CAREFUL_TERMINAL_ENTER_M, LONG_CRUISE_ALT_M,
};
use crate::landing::{
    axis_angle_from_cross, chebyshev_xz, clamp_tilt, high_alt_dive_throttle_gate,
    high_alt_freefall_desired_aim, h_freefall_m, on_pad_square, saturate, LandingAutopilot,
    PAD_HALF_M,
};
use crate::sim::{
    air_drag_k_at_altitude, effective_air_drag_beta, ControlCommand, GRAVITY, RocketState,
};

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
const HANDOFF_SETTLE_MIN_S: f64 = 0.25;
/// Allowed touchdown drift (m) when already centered: bounds `vh · t_drift`.
const HANDOFF_DRIFT_NEAR_M: f64 = 9.0;
/// Allowed drift (m) while still closing — larger, because the predicted-miss
/// term below cancels most of the along-track component.
const HANDOFF_DRIFT_CLOSING_M: f64 = 12.0;
/// Max predicted touchdown miss (m) for the closing-branch arm (half of the
/// ±12 m inner guidance box, leaving room for cross-track drift).
const HANDOFF_MISS_MAX_M: f64 = 6.0;
/// Target altitude (m) during terminal settle — sink while trimming position/attitude.
pub const HANDOFF_ALT_M: f64 = 300.0;
/// Near-handoff loft floor (m) — stay in Cruise while settling over the pad.
const HANDOFF_ALT_MIN_M: f64 = 260.0;
/// Chebyshev (m) within which near-handoff altitude gate applies.
const NEAR_HANDOFF_CHEBY_M: f64 = HANDOFF_CHEBY_MAX_M + 20.0;
/// Chebyshev (m) beyond which terminal latch may release on range exit.
const TERMINAL_EXIT_CHEBY_M: f64 = HANDOFF_CHEBY_MAX_M + 35.0;

// --- Terminal settle (Brake → Upright → Trim) --------------------------------
/// Upright attitude must hold this long (s) before terminal position trim.
const UPRIGHT_STABLE_MIN_S: f64 = 0.45;
/// Shorter upright hold when already quiet (no attitude recovery pending).
const UPRIGHT_STABLE_QUIET_S: f64 = 0.25;
/// Max pitch/yaw rate (rad/s) allowed before leaving Upright for Trim.
const OMEGA_TRIM_ENTER: f64 = OMEGA_HANDOFF_MAX * 0.50;
/// Inside this Chebyshev (m) with quiet vh, Trim holds upright (no chase).
const TRIM_DEADZONE_CHEBY_M: f64 = HANDOFF_CHEBY_MAX_M * 0.60;
// Unscaled bases; runtime values multiply by distance-dependent aggression.
const TRIM_LEAN_CAP_BASE: f64 = 0.15;
const TRIM_LEAN_NEAR_BASE: f64 = 0.032;
const TRIM_LEAN_STRICT_BASE: f64 = 0.018;
const TRIM_V_CREEP_PER_M_BASE: f64 = 0.26;
const TRIM_V_CREEP_MAX_BASE: f64 = 6.00;
const TRIM_V_CREEP_MIN_BASE: f64 = 1.3;
const CAREFUL_BRAKE_LEAN_SOFT_BASE: f64 = 0.22;

// --- Transit lean / envelope -------------------------------------------------
/// Lean cap during the full-throttle ascent burn (rad) — MPC rollout / legacy label.
const LEAN_BURN_MAX: f64 = 0.30;
/// Altitude (m) before opening lateral lean — stay upright through liftoff.
const CLIMB_CLEAR_ALT_M: f64 = 25.0;
/// Max lean (rad) at the end of the climb pitch program (short range).
const LEAN_CLIMB_MAX: f64 = 0.30;
/// Lean cap for airplane cruise (rad). cos(1.45)≈0.12 — matches dive floor in
/// [`long_range_hold_cos`]. Must stay at/above [`COS_TILT_AIM_AIR`].
const LEAN_LONG_MAX: f64 = 1.45;
/// Reverse-brake lean ceiling (rad) — matches [`LEAN_LONG_MAX`] so cruise→brake
/// does not tighten the attitude cone at engagement.
const LEAN_BRAKE_MAX: f64 = LEAN_LONG_MAX;
/// Legacy go-side lean reference (mid-range cruise go flip planning).
const LEAN_BRAKE_PLAN: f64 = 0.80;
/// Range (m) where deep airplane cruise takes over.
const RANGE_FAR_M: f64 = 80.0;
/// Soft ceiling above the altitude gate (m). Once lofted, prefer a slight
/// sink rather than riding thrust upward past this.
const CRUISE_ALT_CAP: f64 = GATE_ALT_MIN + 40.0;
/// Near-full throttle for climb burn and airplane cruise (gimbal authority
/// scales with thrust — full T is also max attitude authority).
const THR_FULL: f64 = 0.97;
/// Flip-recover only when nearly inverted in airplane / deep-lean mode.
/// Normal [`COS_TILT_AIM`] (0.30) would fight a legitimate nose-down dive.
const COS_TILT_AIM_AIR: f64 = 0.10;
/// Horizontal range (m) above which transit prefers airplane mode: full-T go +
/// pitch elevator while outside the predicted stop distance.
const LONG_AIRPLANE_RANGE_M: f64 = 1500.0;
/// Flip-recover gate for freefall dive — allow full nose-down (do not fight invert).
const COS_TILT_AIM_FF: f64 = -1.01;
/// Geometric hysteresis (m) when releasing latched reverse lean — must exceed
/// hand-off cheby so go↔brake does not chatter at the pad edge.
const BRAKE_RELEASE_MARGIN_M: f64 = HANDOFF_CHEBY_MAX_M;
/// Extra range (m) to engage reverse lean before the nominal stop distance.
const BRAKE_ENGAGE_MARGIN_M: f64 = 25.0;
/// Horizontal speed (m/s) above which mid-range braking uses full-T lateral accel.
const VH_BRAKE_FULL_THR: f64 = 20.0;
/// Soft shoulder: below this vh, reverse-brake authority fades toward settle.
const VH_BRAKE_SOFT: f64 = 6.0;
/// Hard shoulder: at/above this vh, reverse brake runs at full lean / full-T.
const VH_BRAKE_HARD: f64 = 22.0;
/// Main-engine throttle spool-up rate (0→1 in ~0.9 s) — matches Descend actuator.
const THROTTLE_SPOOL_UP: f64 = 1.1;
/// Faster spool when GNC requests a large step (airplane / brake engagement).
const THROTTLE_SPOOL_UP_EMERGENCY: f64 = 4.0;
/// Main-engine throttle spool-down rate (1→0 in ~0.4 s).
const THROTTLE_SPOOL_DOWN: f64 = 2.5;
/// Quiet reverse-brake lean floor (rad) once horizontal speed is bled off.
const LEAN_BRAKE_SOFT: f64 = 0.22;
/// Downrange speed built during the ascent burn (m/s). Ballistic coast keeps
/// whatever vh burnout leaves — cruise then accelerates freely on the envelope.
const V_CLIMB_H_MAX: f64 = 28.0;
/// Attitude √-profile planning accel (rad/s²).
const ALPHA_PLAN: f64 = 0.70;
const OMEGA_MAX: f64 = 1.35;
const KP_ATT: f64 = 2.0;
const KD_ATT: f64 = 3.0;
const KD_ROLL: f64 = 2.0;
/// Flip only when past the commanded lean cone (near-inverted), not mid-recovery.
const COS_TILT_AIM: f64 = 0.30; // ~72.5°
/// Pitch/yaw rate (rad/s) above which attitude is pure rate-kill.
const OMEGA_RATE_KILL: f64 = 0.80;
/// Relaxed rate-kill threshold during latched mid-range reverse lean.
const OMEGA_RATE_KILL_BRAKE: f64 = 1.10;
/// Vertical component of the free-vector aim (dimensionless relative to |horiz|).
/// Keeps the thrust axis from going fully horizontal.
const AIM_Y_BIAS: f64 = 1.0;
/// Below this horizontal speed (m/s), anti-velocity aim uses the filtered aim
/// azimuth instead of instantaneous velocity (prevents 180° flip at low vh).
const VH_AIM_AZIMUTH_HOLD: f64 = 8.0;
/// Aim slew rate (rad/s) floor — soft brake / terminal settle / upright.
const AIM_SLEW_SOFT: f64 = 1.0;
/// Aim slew rate (rad/s) ceiling — hard reverse-brake / deep airplane lean.
const AIM_SLEW_HARD: f64 = 3.0;
/// Gimbal command slew (fraction of full deflection per second).
/// Caps bang-bang pitch/yaw from saturated rate-PD so the nozzle does not chatter.
const GIMBAL_SLEW_RATE: f64 = 5.0;

/// Anti-velocity brake aim: +Y component so unit horizontal gives tilt ≈ `lean_cap`.
///
/// [`clamp_tilt`] only reduces tilt; shallow y-bias must not cap the cone below
/// [`LEAN_BRAKE_MAX`].
#[inline]
fn brake_aim_y_bias(lean_cap: f64) -> f64 {
    AIM_Y_BIAS.min(1.0 / lean_cap.max(0.05).tan())
}

/// Hardness-scaled lean cap and attitude PD mode for latched cruise reverse brake.
#[inline]
fn brake_exec_from_hardness(hardness: f64) -> (f64, bool, bool) {
    let h = hardness.clamp(0.0, 1.0);
    let lean_cap = LEAN_BRAKE_SOFT + h * (LEAN_BRAKE_MAX - LEAN_BRAKE_SOFT);
    let aggressive_att = h > 0.55;
    let soft_att = h < 0.40;
    (lean_cap, aggressive_att, soft_att)
}

/// Mid-range cruise reverse-brake aim and lean cap from horizontal kinematics.
#[derive(Clone, Copy, Debug)]
struct CruiseBrakeCommand {
    hardness: f64,
    lean_cap: f64,
    aim: [f64; 3],
    aggressive_att: bool,
    soft_att: bool,
}

/// Unit-length world-frame vector, or `None` if degenerate.
#[inline]
fn normalize_vec3(v: [f64; 3]) -> Option<[f64; 3]> {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-9 {
        None
    } else {
        Some([v[0] / len, v[1] / len, v[2] / len])
    }
}

/// Angle (rad) between two unit vectors via dot product.
#[inline]
fn unit_angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] * b[0] + a[1] * b[1] + a[2] * b[2])
        .clamp(-1.0, 1.0)
        .acos()
}

/// Spherical linear interpolation between unit vectors `a` and `b` (t in [0, 1]).
#[inline]
fn slerp_unit(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    let t = t.clamp(0.0, 1.0);
    let dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).clamp(-1.0, 1.0);
    if dot > 1.0 - 1e-8 {
        return b;
    }
    // Antipodal: pick any perpendicular and rotate toward it (avoids a 180° snap).
    let b = if dot < -1.0 + 1e-8 {
        let ortho = if a[1].abs() < 0.9 {
            normalize_vec3([-a[2], 0.0, a[0]]).unwrap_or([1.0, 0.0, 0.0])
        } else {
            normalize_vec3([1.0, 0.0, 0.0]).unwrap_or([0.0, 0.0, 1.0])
        };
        ortho
    } else {
        b
    };
    let dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).clamp(-1.0, 1.0);
    let omega = dot.acos();
    let sin_omega = omega.sin();
    if sin_omega < 1e-8 {
        return b;
    }
    let wa = ((1.0 - t) * omega).sin() / sin_omega;
    let wb = (t * omega).sin() / sin_omega;
    normalize_vec3([
        wa * a[0] + wb * b[0],
        wa * a[1] + wb * b[1],
        wa * a[2] + wb * b[2],
    ])
    .unwrap_or(b)
}

/// Rate-limited slew of a world-frame thrust aim unit vector toward `target`.
#[inline]
fn slew_aim_world(current: [f64; 3], target: [f64; 3], dt: f64, max_rate: f64) -> [f64; 3] {
    let dt = dt.max(0.0);
    let rate = max_rate.max(0.0);
    let Some(cur) = normalize_vec3(current) else {
        return normalize_vec3(target).unwrap_or([0.0, 1.0, 0.0]);
    };
    let Some(tgt) = normalize_vec3(target) else {
        return cur;
    };
    let angle = unit_angle(cur, tgt);
    let max_step = rate * dt;
    if angle <= max_step || max_step <= 1e-12 {
        return tgt;
    }
    slerp_unit(cur, tgt, max_step / angle)
}

/// Horizontal anti-velocity direction; below [`VH_AIM_AZIMUTH_HOLD`] uses filtered aim.
#[inline]
fn brake_anti_horizontal(vx: f64, vz: f64, vh: f64, aim_filtered: [f64; 3]) -> (f64, f64) {
    if vh < VH_AIM_AZIMUTH_HOLD {
        let hx = aim_filtered[0];
        let hz = aim_filtered[2];
        let h_len = (hx * hx + hz * hz).sqrt();
        if h_len > 0.05 {
            return (hx / h_len, hz / h_len);
        }
    }
    let s = vh.max(1.0);
    (-vx / s, -vz / s)
}

/// Slew rate (rad/s) for transit aim filtering — continuous soft→hard by authority.
#[inline]
fn aim_slew_rate(
    brake: bool,
    brake_hardness: f64,
    deep: bool,
    terminal: bool,
    terminal_phase: Option<TerminalSettlePhase>,
) -> f64 {
    if matches!(
        terminal_phase,
        Some(TerminalSettlePhase::Upright | TerminalSettlePhase::Trim)
    ) {
        return AIM_SLEW_SOFT;
    }
    let authority = if terminal {
        0.0
    } else if deep {
        1.0
    } else if brake {
        brake_hardness.clamp(0.0, 1.0)
    } else {
        0.45
    };
    AIM_SLEW_SOFT + authority * (AIM_SLEW_HARD - AIM_SLEW_SOFT)
}

/// Apply aim slew filter; on sync, snap to the first target.
#[inline]
fn filter_and_slew_aim(
    aim_filtered: &mut [f64; 3],
    aim_filter_sync: &mut bool,
    target: [f64; 3],
    dt: f64,
    max_rate: f64,
) -> [f64; 3] {
    if *aim_filter_sync {
        *aim_filtered = normalize_vec3(target).unwrap_or([0.0, 1.0, 0.0]);
        *aim_filter_sync = false;
        return *aim_filtered;
    }
    let out = slew_aim_world(*aim_filtered, target, dt, max_rate);
    *aim_filtered = out;
    out
}

/// Rate-limit a signed command in [-1, 1] toward `target`.
#[inline]
fn slew_command_axis(current: f64, target: f64, dt: f64, rate: f64) -> f64 {
    let target = target.clamp(-1.0, 1.0);
    let current = current.clamp(-1.0, 1.0);
    let max_step = rate.max(0.0) * dt.max(0.0);
    let delta = (target - current).clamp(-max_step, max_step);
    (current + delta).clamp(-1.0, 1.0)
}

#[inline]
fn cruise_brake_command(
    vx: f64,
    vz: f64,
    vh: f64,
    v_approach: f64,
    aim_filtered: [f64; 3],
) -> CruiseBrakeCommand {
    let hardness = cruise_brake_hardness(vh, v_approach, VH_BRAKE_SOFT, VH_BRAKE_HARD);
    let (lean_cap, aggressive_att, soft_att) = brake_exec_from_hardness(hardness);
    let y_bias = brake_aim_y_bias(lean_cap);
    let (ax, az) = brake_anti_horizontal(vx, vz, vh, aim_filtered);
    let anti = [ax, y_bias, az];
    let upright = [0.0, 1.0, 0.0];
    let aim = blend_vec3(upright, anti, 0.30 + 0.70 * hardness);
    CruiseBrakeCommand {
        hardness,
        lean_cap,
        aim,
        aggressive_att,
        soft_att,
    }
}

/// Lateral thrust regime for stop-distance planning and brake execution.
#[inline]
fn brake_lateral_mode(in_airplane_range: bool, vh: f64, moon_mode: bool) -> LateralThrMode {
    if in_airplane_range || vh > VH_BRAKE_FULL_THR || moon_mode {
        LateralThrMode::FullThrottle
    } else {
        LateralThrMode::VerticalNeutral
    }
}

/// Whether reverse lean should run at full throttle (not hover/cos capped).
///
/// Requires fuzzy hardness so low-speed latched brake does not keep punching
/// full-T after deceleration is done.
#[inline]
fn brake_force_full_throttle(
    in_airplane_range: bool,
    vh: f64,
    moon_mode: bool,
    hardness: f64,
) -> bool {
    hardness > 0.55 && (in_airplane_range || vh > VH_BRAKE_FULL_THR || moon_mode)
}

#[inline]
fn careful(x: f64, aggression: f64) -> f64 {
    x * aggression
}

/// Pad approach: latched terminal envelope or already over the pad box.
#[inline]
fn near_handoff_zone(terminal_latched: bool, cheby: f64) -> bool {
    terminal_latched || cheby <= NEAR_HANDOFF_CHEBY_M
}

/// Ballistic apogee (m) if thrust cuts now at current altitude / vertical speed.
#[inline]
fn ballistic_apogee(alt: f64, vy: f64) -> f64 {
    alt + vy.max(0.0).powi(2) / (2.0 * GRAVITY)
}

/// Loft gate cleared: at altitude, near-handoff soft floor, or ballistic apogee
/// already reaches [`CLIMB_ALT_M`] (500 m target).
#[inline]
fn transit_lofted(alt: f64, vy: f64, near_handoff: bool) -> bool {
    alt >= GATE_ALT_MIN
        || (near_handoff && alt >= HANDOFF_ALT_MIN_M)
        || ballistic_apogee(alt, vy) >= CLIMB_ALT_M
}

/// Blend pad-near curve with outer Chebyshev shoulder (creep / v-cap shared).
#[inline]
fn cheby_near_pad_blend(cheby: f64, pad_near: f64, outer: f64) -> f64 {
    let mu_pad = ramp_down(cheby, 12.0, 22.0);
    mu_pad * pad_near + (1.0 - mu_pad) * outer
}

/// Continuous creep-speed cap vs Chebyshev offset (closer → slower).
#[inline]
fn cheby_creep_cap(cheby: f64) -> f64 {
    // Pad-near slope keeps creep just under the closing-branch hand-off vh
    // bound (`HANDOFF_DRIFT_CLOSING_M / t_drift`) so Descend arms on the fly.
    let pad_near = (0.45 + 0.12 * cheby).min(2.60);
    let outer = 4.50 + ramp(cheby, HANDOFF_CHEBY_MAX_M, 50.0) * 2.00;
    cheby_near_pad_blend(cheby, pad_near, outer)
}

/// Continuous speed ceiling base vs Chebyshev offset (closer → slower).
#[inline]
fn cheby_v_cap_base(cheby: f64) -> f64 {
    let pad_near = (0.40 + 0.12 * cheby).min(2.40);
    let outer = 4.80 + ramp(cheby, HANDOFF_CHEBY_MAX_M, 50.0) * 2.00;
    cheby_near_pad_blend(cheby, pad_near, outer)
}

/// Horizontal creep speed (m/s) for Trim / careful `v_allow`.
#[inline]
fn trim_creep_speed(cheby: f64, aggression: f64) -> f64 {
    let mut v = (careful(TRIM_V_CREEP_PER_M_BASE, aggression) * cheby).clamp(
        careful(TRIM_V_CREEP_MIN_BASE, aggression),
        careful(TRIM_V_CREEP_MAX_BASE, aggression),
    );
    v = v.min(careful(cheby_creep_cap(cheby), aggression));
    v
}

/// Soft speed ceiling (m/s) while inside the careful / terminal envelope.
#[inline]
fn terminal_v_cap(cheby: f64, aggression: f64) -> f64 {
    careful(cheby_v_cap_base(cheby), aggression)
}

/// Brake lean cap: mild demand stays shallow; hard demand can still open fully.
#[inline]
fn careful_brake_lean_cap(soft: f64, demand_shaped: f64, aggression: f64) -> f64 {
    let open = (demand_shaped * demand_shaped).clamp(0.0, 1.0);
    let hi_frac = aggression + (1.0 - aggression) * open;
    soft + open * (LEAN_BRAKE_MAX * hi_frac - soft).max(0.0)
}

/// Guidance phase while the T-key autopilot is armed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TargetPhase {
    /// Below the altitude gate — full-T upright liftoff + open-loop pitch program.
    Climb,
    /// Gate cleared — no climb command; finish the horizontal leg.
    Cruise,
    /// Terminal descent onto the pad.
    Descend,
}

/// Sub-phase within cruise terminal settle (careful envelope): brake → upright → trim.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
enum TerminalSettlePhase {
    /// Reverse lean to kill horizontal speed / overshoot.
    #[default]
    Brake,
    /// Pure upright recovery — no position PD (kills pendulum sway).
    Upright,
    /// Small position trim once upright and stable.
    Trim,
}

/// Autopilot that lands on a world-XZ pad mark (T key).
#[derive(Clone, Debug)]
pub struct TargetLandingAutopilot {
    pub enabled: bool,
    pub complete: bool,
    pub phase: TargetPhase,
    /// Nested lander armed at transit hand-off; runs
    /// [`LandingAutopilot::update_target_descend`] (closed-loop suicide burn +
    /// shared pad-seek attitude geometry).
    lander: LandingAutopilot,
    /// Latched reverse-lean brake (hysteresis) — kills go↔brake sway.
    brake_latched: bool,
    /// Terminal settle envelope latch (enter ~90 m, exit ~140 m).
    terminal_latched: bool,
    /// Terminal-pad brake blend latch (separate from mid-range [`brake_latched`]).
    terminal_brake_latched: bool,
    /// Seconds the hand-off AND gates have been continuously satisfied.
    handoff_settle_s: f64,
    /// Terminal settle sub-phase (Brake / Upright / Trim).
    terminal_settle_phase: TerminalSettlePhase,
    /// Seconds upright gates have held during [`TerminalSettlePhase::Upright`].
    upright_stable_s: f64,
    /// Held MPC candidate between replans (receding-horizon hysteresis).
    mpc_hold: TransitCandidate,
    mpc_hold_counter: u32,
    /// Delivered throttle state (lags the GNC setpoint via [`slew_throttle`]).
    throttle_actuator: f64,
    /// Re-sync actuator from vehicle command on arm / enable.
    throttle_actuator_sync: bool,
    /// Rate-limited world-frame thrust aim (unit vector).
    aim_filtered: [f64; 3],
    /// Re-sync aim filter on arm / reset.
    aim_filter_sync: bool,
    /// Delivered gimbal commands (lags GNC pitch/yaw/roll via slew).
    gimbal_actuator: [f64; 3],
    /// Re-sync gimbal actuator from vehicle command on arm / enable.
    gimbal_actuator_sync: bool,
}

impl Default for TargetLandingAutopilot {
    fn default() -> Self {
        Self {
            enabled: false,
            complete: false,
            phase: TargetPhase::Climb,
            lander: LandingAutopilot::for_target_pad(),
            brake_latched: false,
            terminal_latched: false,
            terminal_brake_latched: false,
            handoff_settle_s: 0.0,
            terminal_settle_phase: TerminalSettlePhase::Brake,
            upright_stable_s: 0.0,
            mpc_hold: TransitCandidate::CruiseGo,
            mpc_hold_counter: MPC_REPLAN_EVERY,
            throttle_actuator: 0.0,
            throttle_actuator_sync: true,
            aim_filtered: [0.0, 1.0, 0.0],
            aim_filter_sync: true,
            gimbal_actuator: [0.0, 0.0, 0.0],
            gimbal_actuator_sync: true,
        }
    }
}

impl TargetLandingAutopilot {
    fn reset_terminal_settle(&mut self) {
        self.terminal_brake_latched = false;
        self.handoff_settle_s = 0.0;
        self.terminal_settle_phase = TerminalSettlePhase::Brake;
        self.upright_stable_s = 0.0;
    }

    fn reset_transit_latches(&mut self) {
        self.brake_latched = false;
        self.terminal_latched = false;
        self.mpc_hold = TransitCandidate::CruiseGo;
        self.mpc_hold_counter = MPC_REPLAN_EVERY;
        self.reset_terminal_settle();
        self.throttle_actuator_sync = true;
        self.aim_filter_sync = true;
        self.gimbal_actuator_sync = true;
    }

    fn finalize_cruise_throttle(
        &mut self,
        target: f64,
        dt: f64,
        state: &RocketState,
    ) -> f64 {
        if self.throttle_actuator_sync {
            self.throttle_actuator = state.command.throttle.clamp(0.0, 1.0);
            self.throttle_actuator_sync = false;
        }
        let target = target.clamp(0.0, 1.0);
        let spool_up = if target - self.throttle_actuator > 0.35 {
            THROTTLE_SPOOL_UP_EMERGENCY
        } else {
            THROTTLE_SPOOL_UP
        };
        self.throttle_actuator = slew_throttle(
            self.throttle_actuator,
            target,
            dt,
            spool_up,
            THROTTLE_SPOOL_DOWN,
        );
        self.throttle_actuator
    }

    /// Rate-limit gimbal commands so saturated attitude PD cannot bang-bang the nozzle.
    fn finalize_gimbal(
        &mut self,
        pitch: f64,
        yaw: f64,
        roll: f64,
        dt: f64,
        state: &RocketState,
    ) -> (f64, f64, f64) {
        if self.gimbal_actuator_sync {
            self.gimbal_actuator = [
                state.command.pitch.clamp(-1.0, 1.0),
                state.command.yaw.clamp(-1.0, 1.0),
                state.command.roll.clamp(-1.0, 1.0),
            ];
            self.gimbal_actuator_sync = false;
        }
        self.gimbal_actuator[0] =
            slew_command_axis(self.gimbal_actuator[0], pitch, dt, GIMBAL_SLEW_RATE);
        self.gimbal_actuator[1] =
            slew_command_axis(self.gimbal_actuator[1], yaw, dt, GIMBAL_SLEW_RATE);
        self.gimbal_actuator[2] =
            slew_command_axis(self.gimbal_actuator[2], roll, dt, GIMBAL_SLEW_RATE);
        (
            self.gimbal_actuator[0],
            self.gimbal_actuator[1],
            self.gimbal_actuator[2],
        )
    }

    /// Spool throttle + gimbal actuators onto a GNC command (Climb / Cruise).
    fn apply_actuators(
        &mut self,
        cmd: &mut ControlCommand,
        dt: f64,
        state: &RocketState,
    ) {
        cmd.throttle = self.finalize_cruise_throttle(cmd.throttle, dt, state);
        let (pitch, yaw, roll) =
            self.finalize_gimbal(cmd.pitch, cmd.yaw, cmd.roll, dt, state);
        cmd.pitch = pitch;
        cmd.yaw = yaw;
        cmd.roll = roll;
    }

    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
        if self.enabled {
            self.complete = false;
            self.phase = TargetPhase::Climb;
            self.reset_transit_latches();
            self.lander.disable();
        } else {
            self.lander.disable();
        }
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.complete = false;
        self.phase = TargetPhase::Climb;
        self.reset_transit_latches();
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

    /// HUD helper: airplane full-T + pitch-elevator cruise is active
    /// (range ≳ [`LONG_AIRPLANE_RANGE_M`], not brake-latched).
    pub fn is_long_range_cruise(&self, pos: [f64; 3], target_xz: [f64; 2]) -> bool {
        if !self.enabled
            || self.complete
            || self.phase == TargetPhase::Descend
            || self.phase == TargetPhase::Climb
        {
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
        let vy = state.velocity[1];
        let cheby = chebyshev_xz(pos, target_xz);
        let dx = target_xz[0] - pos[0];
        let dz = target_xz[1] - pos[2];
        let range = (dx * dx + dz * dz).sqrt();
        if self.phase != TargetPhase::Descend {
            let was_terminal = self.terminal_latched;
            let near_handoff = near_handoff_zone(self.terminal_latched, cheby);
            self.terminal_latched = careful_terminal_latch(
                self.terminal_latched,
                range,
                cheby,
                transit_lofted(alt, vy, near_handoff),
                TERMINAL_EXIT_CHEBY_M,
            );
            if was_terminal && !self.terminal_latched {
                self.reset_terminal_settle();
            }

            // Climb → Cruise once altitude or ballistic apogee clears the 500 m target.
            // Do not drop back to Climb (full-T re-loft) while settling over the pad.
            let near_handoff = near_handoff_zone(self.terminal_latched, cheby);
            self.phase = if transit_lofted(alt, vy, near_handoff) {
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
        // Drift budget: hand-off drift persists through the unpowered coast
        // (the lander's burn then trims it), so end miss ≈ cheby − vh·t_drift.
        // The coast time scales with hand-off altitude.
        let t_drift = (2.0 * alt.max(0.0) / GRAVITY).sqrt().clamp(8.0, 16.0);
        let miss_pred = (cheby - v_cheby_handoff * t_drift).abs();
        let handoff_ready = self.phase == TargetPhase::Cruise
            && cheby <= HANDOFF_CHEBY_MAX_M
            && vh <= VH_HANDOFF_MAX
            && v_cheby_handoff > -0.25
            && ((cheby <= HANDOFF_CHEBY_MAX_M * 0.60 && vh <= HANDOFF_DRIFT_NEAR_M / t_drift)
                || (v_cheby_handoff > 0.12
                    && vh <= HANDOFF_DRIFT_CLOSING_M / t_drift
                    && miss_pred <= HANDOFF_MISS_MAX_M))
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
            self.reset_terminal_settle();
        }

        if self.phase != TargetPhase::Descend && alt >= h_freefall_m(state.moon_mode) {
            let mut cmd = high_alt_freefall_to_pad(state, target_xz);
            self.apply_actuators(&mut cmd, dt, state);
            return cmd;
        }

        match self.phase {
            TargetPhase::Climb => {
                let mut cmd = climb_command(state, target_xz, pos);
                self.apply_actuators(&mut cmd, dt, state);
                cmd
            }
            TargetPhase::Cruise => {
                let (mut cmd, brake, terminal_brake, settle_phase, upright_s, mpc_hold, mpc_counter) =
                    transit_command(
                        state,
                        target_xz,
                        pos,
                        self.brake_latched,
                        self.terminal_latched,
                        self.terminal_brake_latched,
                        self.terminal_settle_phase,
                        self.upright_stable_s,
                        self.mpc_hold,
                        self.mpc_hold_counter,
                        dt,
                        &mut self.aim_filtered,
                        &mut self.aim_filter_sync,
                    );
                self.apply_actuators(&mut cmd, dt, state);
                self.brake_latched = brake;
                self.terminal_brake_latched = terminal_brake;
                self.terminal_settle_phase = settle_phase;
                self.upright_stable_s = upright_s;
                self.mpc_hold = mpc_hold;
                self.mpc_hold_counter = mpc_counter;
                cmd
            }
            TargetPhase::Descend => {
                self.brake_latched = false;
                self.reset_terminal_settle();
                let cmd = self.lander.update_target_descend(state, target_xz, dt);
                self.complete = self.lander.complete;
                cmd
            }
        }
    }
}

/// True when CoM XZ lies inside the painted target platform (complete region).
#[inline]
pub fn inside_target_pad(pos: [f64; 3], target_xz: [f64; 2]) -> bool {
    on_pad_square(pos, target_xz)
}

/// Throttle regime for lateral propulsion planning.
#[derive(Clone, Copy, Debug)]
enum LateralThrMode {
    /// Vertical-neutral reverse lean: `a_lat ≈ g·tan(θ)`.
    VerticalNeutral,
    /// Full-T go/brake: `a_lat = (T/m)·thr·sin(θ)`.
    FullThrottle,
}

// --- Short-horizon MPC (transit only) ----------------------------------------
const MPC_DT: f64 = 0.10;
const MPC_HORIZON_NEAR: f64 = 8.0;
const MPC_HORIZON_MID: f64 = 10.0;
const MPC_HORIZON_FAR: f64 = 12.0;
const MPC_REPLAN_EVERY: u32 = 2;
const MPC_COST_HYSTERESIS: f64 = 2.5;
const W_MPC_GATE: f64 = 55.0;
const W_MPC_OVERLOFT: f64 = 0.45;
const W_MPC_RANGE: f64 = 0.07;
const W_MPC_TIME: f64 = 0.015;
const W_MPC_OVERSHOOT: f64 = 16.0;
const W_MPC_HANDOFF: f64 = 18.0;
/// Range (m) below which MPC hand-off cost is boosted toward terminal entry.
const MPC_HANDOFF_BOOST_RANGE_M: f64 = 200.0;
/// Hand-off weight multiplier at the pad (× at [`MPC_HANDOFF_BOOST_RANGE_M`] = 1).
const MPC_HANDOFF_BOOST_MAX: f64 = 2.5;
const W_MPC_IMPULSE: f64 = 0.12;

/// High-level transit action evaluated by the receding-horizon MPC sampler.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
enum TransitCandidate {
    /// Full-thrust loft toward the pad while below the altitude gate.
    LoftGo,
    /// Long-range full-T + pitch elevator (hold [`LONG_CRUISE_ALT_M`] ≈ 520 m).
    AirplaneHold,
    /// Mid-range powered cruise toward the pad.
    #[default]
    CruiseGo,
    /// Reverse lean to kill approach speed / overshoot.
    Brake,
    /// Ballistic coast with upright aim.
    Coast,
    /// Bleed excess altitude while translating.
    SinkGo,
}

/// Simplified 3DOF state for transit rollouts (position, velocity, lean lag).
#[derive(Clone, Copy, Debug)]
struct TransitPredictorState {
    pos: [f64; 3],
    vel: [f64; 3],
    lean_angle: f64,
    lean_dir_x: f64,
    lean_dir_z: f64,
}

/// Terminal metrics from one candidate rollout.
#[derive(Clone, Copy, Debug)]
struct TransitRolloutMetrics {
    max_alt: f64,
    range_end: f64,
    v_approach_end: f64,
    impulse: f64,
    handoff_penalty: f64,
}

/// Per-candidate thrust / aim parameters for the predictor.
#[derive(Clone, Copy, Debug)]
struct CandidateParams {
    aim: [f64; 3],
    lean_max: f64,
    thr: f64,
    mode: LateralThrMode,
    coast: bool,
    deep: bool,
    force_full_thr: bool,
}

/// MPC output mapped into the existing attitude/throttle pipeline.
#[derive(Clone, Copy, Debug)]
struct TransitMpcPlan {
    desired_raw: [f64; 3],
    lean_max: f64,
    deep: bool,
    force_full_thr: bool,
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
/// Never commands climb; above [`CRUISE_ALT_CAP`] (non-terminal) or
/// [`HANDOFF_ALT_M`] (terminal settle) asks for a gentle sink.
/// `terminal`: inside the settle envelope, sink toward [`HANDOFF_ALT_M`]
/// while Brake → Upright → Trim adjusts position and attitude in parallel.
#[inline]
fn cruise_v_des_y(alt: f64, vy: f64, terminal: bool) -> f64 {
    let sink = if terminal {
        if alt > HANDOFF_ALT_M {
            // Bleed toward hand-off altitude (~1–8 m/s) during terminal settle.
            (-0.08 * (alt - HANDOFF_ALT_M)).clamp(-8.0, -0.8)
        } else {
            0.0
        }
    } else if alt > CRUISE_ALT_CAP {
        // Bleed excess altitude while translating (~1–8 m/s sink; deep only
        // when returning from long-range cruise altitude). Gain sets the
        // bleed time constant (~12 s) — at 0.04 the tail alone took ~50 s
        // and the HUD sat in "cruise" sinking centimeters per frame.
        (-0.08 * (alt - CRUISE_ALT_CAP)).clamp(-8.0, -0.8)
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
            effective_air_drag_beta(state)
        };
        let brake_mode = brake_lateral_mode(in_airplane_range, vh, state.moon_mode);
        let brake_lean = LEAN_BRAKE_MAX;
        let a_prop = lateral_accel_for_lean(
            brake_lean,
            brake_mode,
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
        [-vx / s, brake_aim_y_bias(lean_max), -vz / s]
    } else {
        [ux, brake_aim_y_bias(lean_max), uz]
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

/// Angle (rad) from current body-up to a world-frame thrust aim (PGA motor frame).
fn go_flip_angle(state: &RocketState, desired: [f64; 3]) -> f64 {
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
    engage_margin: f64,
) -> f64 {
    let range_budget = (range_eff - engage_margin).max(0.0);
    if range_budget <= 1e-6 {
        return v_end;
    }
    let disc = a_prop * a_prop * t_flip * t_flip + 2.0 * a_prop * range_budget + v_end * v_end;
    let mut hi = if disc > 0.0 {
        (-a_prop * t_flip + disc.sqrt()).max(v_end + 1.0)
    } else {
        v_end + 1.0
    };
    while predicted_stop_distance(hi, v_end, a_prop, beta, t_flip) < range_budget && hi < 900.0 {
        hi *= 1.5;
    }
    let mut lo = v_end;
    for _ in 0..16 {
        let mid = 0.5 * (lo + hi);
        if predicted_stop_distance(mid, v_end, a_prop, beta, t_flip) <= range_budget {
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

#[inline]
fn mpc_horizon_s(range: f64) -> f64 {
    if range >= LONG_AIRPLANE_RANGE_M {
        MPC_HORIZON_FAR
    } else if range >= RANGE_FAR_M {
        MPC_HORIZON_MID
    } else {
        MPC_HORIZON_NEAR
    }
}

#[inline]
fn predictor_drag_accel(vel: [f64; 3], k: f64, mass: f64) -> [f64; 3] {
    let vmag_sq = vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2];
    if vmag_sq <= 1e-12 {
        return [0.0, 0.0, 0.0];
    }
    let vmag = vmag_sq.sqrt();
    let c = -k * vmag / mass.max(1e-6);
    [c * vel[0], c * vel[1], c * vel[2]]
}

#[inline]
fn predictor_thrust_accel(
    aim: [f64; 3],
    lean_max: f64,
    thr: f64,
    mode: LateralThrMode,
    mass: f64,
    max_thrust: f64,
) -> [f64; 3] {
    if thr <= 1e-6 {
        return [0.0, 0.0, 0.0];
    }
    let d = clamp_tilt(aim, lean_max);
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len <= 1e-9 {
        return [0.0, 0.0, 0.0];
    }
    let u = [d[0] / len, d[1] / len, d[2] / len];
    match mode {
        LateralThrMode::FullThrottle => {
            let am = thr * max_thrust / mass.max(1e-6);
            [am * u[0], am * u[1], am * u[2]]
        }
        LateralThrMode::VerticalNeutral => {
            let horiz = (u[0] * u[0] + u[2] * u[2]).sqrt();
            if horiz <= 1e-9 {
                [0.0, 0.0, 0.0]
            } else {
                let lean = horiz.clamp(0.0, 0.999).asin();
                let a_lat = (GRAVITY * lean.tan()).max(0.0);
                let scale = a_lat / horiz;
                [scale * u[0], 0.0, scale * u[2]]
            }
        }
    }
}

fn predictor_init(state: &RocketState, pos: [f64; 3]) -> TransitPredictorState {
    let up = world_up_in_body(&state.motor);
    let up_y = up[1].clamp(-1.0, 1.0);
    let lean_angle = up_y.acos();
    let horiz = (up[0] * up[0] + up[2] * up[2]).sqrt();
    let (lean_dir_x, lean_dir_z) = if horiz > 1e-6 {
        (up[0] / horiz, up[2] / horiz)
    } else {
        (0.0, 0.0)
    };
    TransitPredictorState {
        pos,
        vel: state.velocity,
        lean_angle,
        lean_dir_x,
        lean_dir_z,
    }
}

fn predictor_target_lean(aim: [f64; 3], lean_max: f64) -> (f64, f64, f64) {
    let d = clamp_tilt(aim, lean_max);
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len <= 1e-9 {
        return (0.0, 0.0, 0.0);
    }
    let u = [d[0] / len, d[1] / len, d[2] / len];
    let horiz = (u[0] * u[0] + u[2] * u[2]).sqrt();
    let angle = horiz.clamp(0.0, 0.999).asin();
    if horiz > 1e-9 {
        (angle, u[0] / horiz, u[2] / horiz)
    } else {
        (angle, 0.0, 0.0)
    }
}

fn predictor_step(
    st: &mut TransitPredictorState,
    params: CandidateParams,
    mass: f64,
    max_thrust: f64,
    k_drag: f64,
    dt: f64,
) -> f64 {
    let (tgt_angle, tgt_dx, tgt_dz) = predictor_target_lean(params.aim, params.lean_max);
    let tau = brake_flip_time((st.lean_angle - tgt_angle).abs().max(1e-4)).max(0.35);
    let alpha = (dt / tau).clamp(0.0, 1.0);
    st.lean_angle += alpha * (tgt_angle - st.lean_angle);
    if tgt_angle > 1e-4 {
        st.lean_dir_x += alpha * (tgt_dx - st.lean_dir_x);
        st.lean_dir_z += alpha * (tgt_dz - st.lean_dir_z);
    }

    let sin_l = st.lean_angle.sin();
    let cos_l = st.lean_angle.cos();
    let thrust_aim = [
        st.lean_dir_x * sin_l,
        cos_l,
        st.lean_dir_z * sin_l,
    ];
    let a_thr = if params.coast {
        [0.0, 0.0, 0.0]
    } else {
        predictor_thrust_accel(
            thrust_aim,
            params.lean_max,
            params.thr,
            params.mode,
            mass,
            max_thrust,
        )
    };
    let a_drag = predictor_drag_accel(
        st.vel,
        air_drag_k_at_altitude(k_drag, st.pos[1]),
        mass,
    );
    let ax = a_thr[0] + a_drag[0];
    let ay = a_thr[1] + a_drag[1] - GRAVITY;
    let az = a_thr[2] + a_drag[2];

    st.vel[0] += ax * dt;
    st.vel[1] += ay * dt;
    st.vel[2] += az * dt;
    st.pos[0] += st.vel[0] * dt;
    st.pos[1] += st.vel[1] * dt;
    st.pos[2] += st.vel[2] * dt;
    if st.pos[1] < 0.0 {
        st.pos[1] = 0.0;
        if st.vel[1] < 0.0 {
            st.vel[1] = 0.0;
        }
    }
    if params.coast {
        0.0
    } else {
        params.thr * dt
    }
}

fn rollout_metrics(
    st: &TransitPredictorState,
    target_xz: [f64; 2],
    ux: f64,
    uz: f64,
    approach_range: f64,
) -> TransitRolloutMetrics {
    let dx = target_xz[0] - st.pos[0];
    let dz = target_xz[1] - st.pos[2];
    let range_end = (dx * dx + dz * dz).sqrt();
    let cheby_end = chebyshev_xz(st.pos, target_xz);
    let vh_end = (st.vel[0] * st.vel[0] + st.vel[2] * st.vel[2]).sqrt();
    let v_approach_end = st.vel[0] * ux + st.vel[2] * uz;
    let mut handoff_penalty = 0.0;
    if st.pos[1] < GATE_ALT_MIN {
        handoff_penalty += (GATE_ALT_MIN - st.pos[1]).powi(2);
    }
    if cheby_end > HANDOFF_CHEBY_MAX_M {
        handoff_penalty += (cheby_end - HANDOFF_CHEBY_MAX_M).powi(2);
    }
    if vh_end > VH_HANDOFF_MAX {
        handoff_penalty += (vh_end - VH_HANDOFF_MAX).powi(2);
    }
    if v_approach_end < -0.5 {
        handoff_penalty += (-v_approach_end).powi(2);
    }
    // Drift / miss / diverging terms mirror the hard hand-off AND gates — only
    // inside the terminal envelope so mid-range MPC ranking stays stable.
    if approach_range <= CAREFUL_TERMINAL_ENTER_M {
        let v_cheby_end = chebyshev_closing_rate(st.pos, target_xz, st.vel);
        let t_drift = (2.0 * st.pos[1].max(0.0) / GRAVITY).sqrt().clamp(8.0, 16.0);
        let miss_pred = (cheby_end - v_cheby_end * t_drift).abs();
        if v_cheby_end < -0.25 {
            handoff_penalty += (-v_cheby_end - 0.25).powi(2);
        }
        let vh_drift_max = HANDOFF_DRIFT_CLOSING_M / t_drift;
        if vh_end > vh_drift_max {
            handoff_penalty += (vh_end - vh_drift_max).powi(2);
        }
        if miss_pred > HANDOFF_MISS_MAX_M {
            handoff_penalty += (miss_pred - HANDOFF_MISS_MAX_M).powi(2);
        }
    }
    TransitRolloutMetrics {
        max_alt: st.pos[1],
        range_end,
        v_approach_end,
        impulse: 0.0,
        handoff_penalty,
    }
}

fn transit_rollout(
    init: TransitPredictorState,
    params: CandidateParams,
    target_xz: [f64; 2],
    ux: f64,
    uz: f64,
    mass: f64,
    max_thrust: f64,
    k_drag: f64,
    horizon: f64,
    approach_range: f64,
) -> TransitRolloutMetrics {
    let steps = (horizon / MPC_DT).ceil() as u32;
    let mut st = init;
    let mut max_alt = st.pos[1];
    let mut impulse = 0.0;
    for _ in 0..steps {
        impulse += predictor_step(&mut st, params, mass, max_thrust, k_drag, MPC_DT);
        max_alt = max_alt.max(st.pos[1]);
    }
    let mut m = rollout_metrics(&st, target_xz, ux, uz, approach_range);
    m.max_alt = max_alt;
    m.impulse = impulse;
    m
}

fn mpc_rollout_cost(
    metrics: TransitRolloutMetrics,
    lofted: bool,
    alt_cap: f64,
    horizon: f64,
    needs_gate: bool,
    approach_range: f64,
) -> f64 {
    let mut cost = 0.0;
    if needs_gate && metrics.max_alt < GATE_ALT_MIN {
        cost += W_MPC_GATE * (GATE_ALT_MIN - metrics.max_alt).powi(2);
    }
    if metrics.max_alt > alt_cap {
        cost += W_MPC_OVERLOFT * (metrics.max_alt - alt_cap).powi(2);
    }
    cost += W_MPC_RANGE * metrics.range_end;
    cost += W_MPC_TIME * horizon;
    if metrics.v_approach_end < 0.0 {
        cost += W_MPC_OVERSHOOT * (-metrics.v_approach_end).powi(2);
    }
    let handoff_boost = 1.0
        + (MPC_HANDOFF_BOOST_MAX - 1.0)
            * ramp(
                MPC_HANDOFF_BOOST_RANGE_M - approach_range.max(0.0),
                0.0,
                MPC_HANDOFF_BOOST_RANGE_M,
            );
    cost += W_MPC_HANDOFF * handoff_boost * metrics.handoff_penalty;
    cost += W_MPC_IMPULSE * metrics.impulse;
    if lofted && metrics.max_alt < GATE_ALT_MIN - 5.0 {
        cost += W_MPC_GATE * 0.25;
    }
    cost
}

fn candidate_params(
    candidate: TransitCandidate,
    ux: f64,
    uz: f64,
    vx: f64,
    vz: f64,
    vh: f64,
    alt: f64,
    alt_hold: f64,
    vy: f64,
    hover: f64,
    mu_long: f64,
    in_airplane_range: bool,
    lofted: bool,
    moon_mode: bool,
    mass: f64,
    max_thrust: f64,
    a_brake_max: f64,
    brake_mode: LateralThrMode,
) -> Option<CandidateParams> {
    match candidate {
        TransitCandidate::LoftGo => {
            if lofted {
                return None;
            }
            let lean = LEAN_BURN_MAX + mu_long * (0.90 - LEAN_BURN_MAX);
            let y_bias = AIM_Y_BIAS - mu_long * 0.55;
            let k_h = 0.14 + mu_long * 0.35;
            Some(CandidateParams {
                aim: [k_h * ux, y_bias.max(0.40), k_h * uz],
                lean_max: lean.min(LEAN_LONG_MAX),
                thr: THR_FULL,
                mode: if in_airplane_range {
                    LateralThrMode::FullThrottle
                } else {
                    LateralThrMode::FullThrottle
                },
                coast: false,
                deep: mu_long > 0.35,
                force_full_thr: true,
            })
        }
        TransitCandidate::AirplaneHold => {
            if !in_airplane_range {
                return None;
            }
            let cos_up = long_range_hold_cos(alt, alt_hold, vy, hover);
            let aim = long_range_go_aim(ux, uz, cos_up);
            Some(CandidateParams {
                aim,
                lean_max: LEAN_LONG_MAX,
                thr: THR_FULL,
                mode: LateralThrMode::FullThrottle,
                coast: false,
                deep: true,
                force_full_thr: true,
            })
        }
        TransitCandidate::CruiseGo => {
            let go_lean = lean_for_lateral_accel(
                a_brake_max * 0.35,
                brake_mode,
                mass,
                max_thrust,
                LEAN_BRAKE_MAX * 0.55,
            );
            Some(CandidateParams {
                aim: [ux, AIM_Y_BIAS, uz],
                lean_max: go_lean,
                thr: hover.clamp(0.35, 0.85),
                mode: LateralThrMode::VerticalNeutral,
                coast: false,
                deep: false,
                force_full_thr: false,
            })
        }
        TransitCandidate::Brake => {
            let v_approach = vx * ux + vz * uz;
            // MPC brake uses instantaneous anti-v only — filtered azimuth is for
            // the live command path, not the open-loop predictor.
            let cmd = cruise_brake_command(vx, vz, vh, v_approach, [0.0, 1.0, 0.0]);
            let full_thr = brake_force_full_throttle(
                in_airplane_range,
                vh,
                moon_mode,
                cmd.hardness,
            );
            let exec_mode = if cmd.hardness > 0.45 {
                brake_mode
            } else {
                LateralThrMode::VerticalNeutral
            };
            Some(CandidateParams {
                aim: cmd.aim,
                lean_max: cmd.lean_cap,
                thr: if full_thr {
                    THR_FULL
                } else {
                    hover.clamp(0.55, 0.95)
                },
                mode: exec_mode,
                coast: false,
                deep: cmd.hardness > 0.25,
                force_full_thr: full_thr,
            })
        }
        TransitCandidate::Coast => Some(CandidateParams {
            aim: [0.0, 1.0, 0.0],
            lean_max: 0.05,
            thr: 0.0,
            mode: LateralThrMode::VerticalNeutral,
            coast: true,
            deep: false,
            force_full_thr: false,
        }),
        TransitCandidate::SinkGo => {
            if alt <= CRUISE_ALT_CAP {
                return None;
            }
            let sink_bias = (-0.35 * (alt - CRUISE_ALT_CAP) / 40.0).clamp(-0.55, -0.15);
            Some(CandidateParams {
                aim: [0.65 * ux, AIM_Y_BIAS + sink_bias, 0.65 * uz],
                lean_max: 0.35,
                thr: hover.clamp(0.40, 0.80),
                mode: LateralThrMode::VerticalNeutral,
                coast: false,
                deep: false,
                force_full_thr: false,
            })
        }
    }
}

fn candidate_to_plan(params: CandidateParams) -> TransitMpcPlan {
    TransitMpcPlan {
        desired_raw: params.aim,
        lean_max: params.lean_max,
        deep: params.deep,
        force_full_thr: params.force_full_thr,
    }
}

fn transit_mpc_select(
    state: &RocketState,
    pos: [f64; 3],
    target_xz: [f64; 2],
    ux: f64,
    uz: f64,
    vx: f64,
    vz: f64,
    vh: f64,
    range: f64,
    alt_hold: f64,
    hover: f64,
    mu_long: f64,
    in_airplane_range: bool,
    lofted: bool,
    ballistic: bool,
    brake_latched: bool,
    brake_now: bool,
    hold: TransitCandidate,
    hold_counter: u32,
) -> (TransitMpcPlan, TransitCandidate, u32) {
    let mass = state.params.mass;
    let max_thrust = state.params.max_thrust;
    let k_drag = if state.moon_mode {
        0.0
    } else {
        state.params.air_drag_k
    };
    let brake_mode = brake_lateral_mode(in_airplane_range, vh, state.moon_mode);
    let a_brake_max = lateral_accel_for_lean(
        LEAN_BRAKE_MAX,
        brake_mode,
        mass,
        max_thrust,
    );
    let horizon = mpc_horizon_s(range);
    let alt_cap = if in_airplane_range {
        LONG_CRUISE_ALT_M + 20.0
    } else {
        CRUISE_ALT_CAP
    };
    let needs_gate = !lofted;
    let init = predictor_init(state, pos);

    let airplane_go = in_airplane_range && !brake_latched && !brake_now;
    let candidates: &[TransitCandidate] = if brake_now {
        &[TransitCandidate::Brake]
    } else if airplane_go {
        &[TransitCandidate::AirplaneHold, TransitCandidate::Brake]
    } else if !lofted {
        &[
            TransitCandidate::LoftGo,
            TransitCandidate::CruiseGo,
            TransitCandidate::Brake,
            TransitCandidate::Coast,
            TransitCandidate::SinkGo,
        ]
    } else {
        &[
            TransitCandidate::CruiseGo,
            TransitCandidate::Brake,
            TransitCandidate::Coast,
            TransitCandidate::SinkGo,
            TransitCandidate::AirplaneHold,
        ]
    };

    let mut best = hold;
    let mut best_cost = f64::INFINITY;
    let mut best_plan = candidate_to_plan(CandidateParams {
        aim: [ux, AIM_Y_BIAS, uz],
        lean_max: 0.35,
        thr: hover,
        mode: LateralThrMode::VerticalNeutral,
        coast: false,
        deep: false,
        force_full_thr: false,
    });

    for &cand in candidates {
        let Some(params) = candidate_params(
            cand,
            ux,
            uz,
            vx,
            vz,
            vh,
            pos[1],
            alt_hold,
            state.velocity[1],
            hover,
            mu_long,
            in_airplane_range,
            lofted,
            state.moon_mode,
            mass,
            max_thrust,
            a_brake_max,
            brake_mode,
        ) else {
            continue;
        };
        let metrics = transit_rollout(
            init,
            params,
            target_xz,
            ux,
            uz,
            mass,
            max_thrust,
            k_drag,
            horizon,
            range,
        );
        let mut cost = mpc_rollout_cost(metrics, lofted, alt_cap, horizon, needs_gate, range);
        if cand == TransitCandidate::Coast && !ballistic {
            cost += 25.0;
        }
        if cand == TransitCandidate::Coast && range > RANGE_FAR_M {
            cost += 35.0;
        }
        if brake_latched && cand == TransitCandidate::Brake {
            cost -= MPC_COST_HYSTERESIS;
        }
        if cand == hold {
            cost -= MPC_COST_HYSTERESIS * 0.5;
        }
        if cost < best_cost {
            best_cost = cost;
            best = cand;
            best_plan = candidate_to_plan(params);
        }
    }

    let replan = hold_counter >= MPC_REPLAN_EVERY;
    let out_cand = if replan { best } else { hold };
    let out_counter = if replan { 0 } else { hold_counter + 1 };

    let plan = if let Some(params) = candidate_params(
        out_cand,
        ux,
        uz,
        vx,
        vz,
        vh,
        pos[1],
        alt_hold,
        state.velocity[1],
        hover,
        mu_long,
        in_airplane_range,
        lofted,
        state.moon_mode,
        mass,
        max_thrust,
        a_brake_max,
        brake_mode,
    ) {
        candidate_to_plan(params)
    } else {
        best_plan
    };

    (plan, out_cand, out_counter)
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
    let lean_cap = lean_cap.max(0.0);
    // Allow shallow careful lean below the usual 0.06 floor when cap is lower.
    let floor = 0.06_f64.min(lean_cap);
    let a = a_req.max(0.0);
    if a <= 1e-9 {
        return floor;
    }
    let lean = match mode {
        LateralThrMode::VerticalNeutral => (a / GRAVITY).atan(),
        LateralThrMode::FullThrottle => {
            let am = THR_FULL * max_thrust / mass.max(1e-6);
            // Demand can exceed full-T authority (hot terminal entry) — asin
            // of >1 is NaN, so saturate at the flat-out quarter turn.
            (a / am).min(1.0).asin()
        }
    };
    lean.clamp(floor, lean_cap.max(floor))
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
        range_eff <= d_stop + BRAKE_ENGAGE_MARGIN_M
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
    vh_hot: f64,
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
    if vh > vh_hot {
        // Inbound but too fast for this offset — open reverse lean.
        score += ((vh - vh_hot) / 1.5).min(1.0);
    }
    score = score.clamp(0.0, 1.5);

    if latched {
        let release = score < 0.15 && v_cheby > 0.05 && vh < VH_HANDOFF_MAX * 0.80;
        let w = if release {
            (score / 0.15 * 0.15).clamp(0.0, 0.15)
        } else {
            // Mild latch stays mild — no 0.30 floor that forces deep lean.
            (0.12 + 0.88 * score.min(1.0)).clamp(0.12, 1.0)
        };
        (w, !release)
    } else {
        let w = if score > 0.08 {
            (0.10 + 0.90 * (score / 1.0).min(1.0)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let engage = w > 0.55;
        (w, engage)
    }
}

/// 0..1: how hard horizontal deceleration is needed (mild → hard).
#[inline]
fn brake_decel_demand(vh: f64, v_cheby: f64, v_approach: f64, t_vh: f64) -> f64 {
    let v_quiet = VH_HANDOFF_MAX * 0.55;
    let mut d = 0.0_f64;
    if vh > v_quiet {
        // ~4 m/s excess → mild; ~12 m/s → hard
        d = d.max(((vh - v_quiet) / (VH_HANDOFF_MAX * 2.5)).clamp(0.0, 1.0));
    }
    if v_cheby < -0.2 {
        d = d.max(((-v_cheby - 0.2) / 5.0).clamp(0.0, 1.0));
    }
    if v_approach < -1.0 {
        d = d.max(((-v_approach - 1.0) / 8.0).clamp(0.0, 1.0));
    }
    d = d.max((t_vh / 4.5).clamp(0.0, 1.0));
    d.clamp(0.0, 1.0)
}

/// Output of one terminal-settle aim step.
#[derive(Clone, Copy, Debug)]
struct TerminalSettleOutput {
    desired_raw: [f64; 3],
    lean_max: f64,
    terminal_brake_latch: bool,
    phase: TerminalSettlePhase,
    upright_stable_s: f64,
}

/// "Too fast for here" speed: creep target plus margin, so a hot arrival
/// brakes instead of sailing across the pad into the hand-off gate.
#[inline]
fn terminal_vh_hot(cheby: f64, aggression: f64) -> f64 {
    (2.0 * trim_creep_speed(cheby, aggression) + 0.8).min(VH_HANDOFF_MAX * 1.35)
}

#[inline]
fn terminal_needs_brake(
    brake_w: f64,
    brake_latched: bool,
    v_cheby: f64,
    vh: f64,
    v_approach: f64,
    vh_hot: f64,
) -> bool {
    // Terminal is always inside [`CAREFUL_RANGE_M`]: prefer Trim unless clearly too fast.
    vh > vh_hot
        || v_cheby < -0.70
        || v_approach < -2.5
        || (brake_latched && brake_w > 0.55 && vh > VH_HANDOFF_MAX)
}

/// Advance terminal settle sub-phase (Brake → Upright → Trim loop).
fn update_terminal_settle_phase(
    phase: TerminalSettlePhase,
    upright_stable_s: f64,
    dt: f64,
    up_y: f64,
    omega_py: f64,
    brake_w: f64,
    brake_latched: bool,
    v_cheby: f64,
    vh: f64,
    v_approach: f64,
    t_att: f64,
    vh_hot: f64,
) -> (TerminalSettlePhase, f64) {
    // Stricter than hand-off: Trim must not start while still rocking.
    let needs_upright = up_y < COS_TILT_HANDOFF
        || omega_py > OMEGA_TRIM_ENTER
        || t_att > 0.05;
    let needs_brake =
        terminal_needs_brake(brake_w, brake_latched, v_cheby, vh, v_approach, vh_hot);

    match phase {
        TerminalSettlePhase::Brake => {
            // Always damp brake lean through Upright — never skip to Trim.
            if needs_brake {
                (TerminalSettlePhase::Brake, 0.0)
            } else {
                (TerminalSettlePhase::Upright, 0.0)
            }
        }
        TerminalSettlePhase::Upright => {
            if needs_brake {
                (TerminalSettlePhase::Brake, 0.0)
            } else if needs_upright {
                (TerminalSettlePhase::Upright, 0.0)
            } else {
                let stable = upright_stable_s + dt;
                let upright_min = if t_att <= 0.02 && omega_py <= OMEGA_TRIM_ENTER {
                    UPRIGHT_STABLE_QUIET_S
                } else {
                    UPRIGHT_STABLE_MIN_S
                };
                if stable >= upright_min {
                    (TerminalSettlePhase::Trim, 0.0)
                } else {
                    (TerminalSettlePhase::Upright, stable)
                }
            }
        }
        TerminalSettlePhase::Trim => {
            // Hysteresis: only leave Trim for clear attitude upset, not micro-tilt.
            let upset = up_y < COS_TILT_HANDOFF - 0.02 || omega_py > OMEGA_HANDOFF_MAX;
            if upset {
                (TerminalSettlePhase::Upright, 0.0)
            } else if needs_brake {
                (TerminalSettlePhase::Brake, 0.0)
            } else {
                (TerminalSettlePhase::Trim, upright_stable_s)
            }
        }
    }
}

/// Brake-phase aim: reverse lean scaled by deceleration demand (mild → hard).
fn terminal_brake_aim(
    ux: f64,
    uz: f64,
    vx: f64,
    vz: f64,
    vh: f64,
    v_cheby: f64,
    v_approach: f64,
    cheby: f64,
    hp: HandoffSettlePlan,
    need_x: f64,
    need_z: f64,
    mass: f64,
    max_thrust: f64,
    brake_mode: LateralThrMode,
    brake_w: f64,
    freedom: f64,
    aggression: f64,
    aim_filtered: [f64; 3],
) -> ([f64; 3], f64) {
    let pos_urgency = (hp.t_pos / 3.0).clamp(0.0, 1.0);
    let vh_urgency = (hp.t_vh / 3.0).clamp(0.0, 1.0);
    let settle_urgency = pos_urgency.max(vh_urgency);
    let inside_frac = ((HANDOFF_CHEBY_MAX_M - cheby) / HANDOFF_CHEBY_MAX_M).clamp(0.0, 1.0);
    let gain_scale = 0.40 + 0.60 * (1.0 - inside_frac);

    // Physics demand (not latch floor): mild excess → small lean; hard → full.
    let demand = brake_decel_demand(vh, v_cheby, v_approach, hp.t_vh).max(brake_w * 0.55);
    let demand_shaped = (demand * demand).clamp(0.0, 1.0);

    let (k_pos, k_vel) = if cheby > HANDOFF_CHEBY_MAX_M {
        (
            (0.12 + 0.22 * settle_urgency * demand).clamp(0.12, 0.32),
            (0.50 + 0.28 * settle_urgency).clamp(0.50, 0.72),
        )
    } else {
        (
            (0.08 + 0.16 * settle_urgency * demand).clamp(0.08, 0.22),
            (0.48 + 0.24 * settle_urgency).clamp(0.48, 0.65),
        )
    };

    let dir_bias = if cheby <= HANDOFF_CHEBY_MAX_M {
        0.12 + 0.10 * inside_frac
    } else {
        0.24 + 0.08 * (1.0 - (cheby - HANDOFF_CHEBY_MAX_M).min(30.0) / 30.0)
    };

    let v_ref = vh.max(1.5);
    let mut aim_x = dir_bias * ux + gain_scale * (k_pos * need_x - k_vel * vx / v_ref);
    let mut aim_z = dir_bias * uz + gain_scale * (k_pos * need_z - k_vel * vz / v_ref);

    // Blend toward velocity-opposing aim only as hard as demand requires.
    let motion = settle_motion_scale(freedom);
    let anti_w =
        (brake_w * (0.35 + 0.65 * demand_shaped) * motion).clamp(0.0, 1.0);
    let (anti_x, anti_z) = brake_anti_horizontal(vx, vz, vh, aim_filtered);
    aim_x = (1.0 - anti_w) * aim_x + anti_w * anti_x;
    aim_z = (1.0 - anti_w) * aim_z + anti_w * anti_z;

    let a_req_x = gain_scale * (k_pos * need_x - k_vel * vx) + anti_w * 0.45 * (-vx);
    let a_req_z = gain_scale * (k_pos * need_z - k_vel * vz) + anti_w * 0.45 * (-vz);
    let a_lat = (a_req_x * a_req_x + a_req_z * a_req_z).sqrt();
    let overshoot_boost = if v_cheby < -0.3 {
        (-v_cheby).min(vh) * 0.25 * demand_shaped
    } else {
        0.0
    };

    // Terminal settle is always inside the careful envelope.
    let soft0 = careful(0.05, aggression);
    let soft = (soft0 + careful(0.004, aggression) * cheby)
        .clamp(soft0, careful(CAREFUL_BRAKE_LEAN_SOFT_BASE, aggression));
    let lean_cap = careful_brake_lean_cap(soft, demand_shaped, aggression);
    let a_scale = careful(0.12 + 0.55 * demand_shaped, aggression);
    let a_cmd = (a_lat.max(overshoot_boost) * a_scale)
        .max(careful(0.05, aggression) * demand_shaped);
    let effective_cap =
        lean_cap.clamp(careful(0.05, aggression), LEAN_BRAKE_MAX) * settle_brake_lean_scale(freedom);
    let lean = lean_for_lateral_accel(
        a_cmd,
        brake_mode,
        mass,
        max_thrust,
        effective_cap,
    );

    ([aim_x, AIM_Y_BIAS, aim_z], lean)
}

/// Trim-phase aim: slow creep with tiny lean — never chase transit `v_allow`.
fn terminal_trim_aim(
    ux: f64,
    uz: f64,
    vx: f64,
    vz: f64,
    vh: f64,
    cheby: f64,
    v_cheby: f64,
    omega_py: f64,
    freedom: f64,
    aggression: f64,
) -> ([f64; 3], f64) {
    // Quiet and already in the hand-off box: hold upright. Must sit below the
    // centered arm branch's vh bound — holding upright at a higher vh would
    // freeze the deceleration right where arming needs it quiet.
    if cheby <= TRIM_DEADZONE_CHEBY_M
        && vh <= VH_HANDOFF_MAX * 0.12
        && v_cheby > -0.08
    {
        return ([0.0, 1.0, 0.0], careful(0.02, aggression));
    }

    let v_creep = trim_creep_speed(cheby, aggression);
    let err_vx = ux * v_creep - vx;
    let err_vz = uz * v_creep - vz;

    let dist_scale = (cheby / CAREFUL_RANGE_M).clamp(0.10, 1.0);
    let rate_gate_base = (1.0 - (omega_py / OMEGA_HANDOFF_MAX).clamp(0.0, 1.0)).powi(2);
    let rate_gate = settle_trim_rate_gate(rate_gate_base, freedom);
    let lean_near = careful(TRIM_LEAN_NEAR_BASE, aggression);
    let lean_far = careful(TRIM_LEAN_CAP_BASE, aggression);
    let lean_dist = lean_near + (lean_far - lean_near) * dist_scale;
    let lean_strict = careful(TRIM_LEAN_STRICT_BASE, aggression);
    let lean_cap = (lean_strict + (lean_dist - lean_strict) * freedom) * rate_gate;

    let gain_scale = settle_motion_scale(freedom);
    let k_vel = careful(0.38 + 0.12 * dist_scale, aggression) * gain_scale;
    let k_pos = careful(0.008 * dist_scale, aggression) * gain_scale;
    let a_req_x = k_pos * ux * cheby + k_vel * err_vx;
    let a_req_z = k_pos * uz * cheby + k_vel * err_vz;
    let a_lat = (a_req_x * a_req_x + a_req_z * a_req_z).sqrt();
    let lean = (a_lat / GRAVITY).atan().clamp(0.0, lean_cap);

    let aim_scale = careful(0.06 + 0.14 * dist_scale, aggression);
    let v_ref = vh.max(0.6);
    let pos_aim = [
        aim_scale * (ux * 0.30 + err_vx / v_ref),
        AIM_Y_BIAS,
        aim_scale * (uz * 0.30 + err_vz / v_ref),
    ];
    let upright = [0.0, 1.0, 0.0];
    let blended = blend_vec3(upright, pos_aim, settle_aim_blend(freedom));

    (blended, lean)
}

/// Sequenced terminal settle: Brake → Upright → Trim (no simultaneous pos+att blend).
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
    phase: TerminalSettlePhase,
    upright_stable_s: f64,
    dt: f64,
    up_y: f64,
    omega_py: f64,
    aggression: f64,
    aim_filtered: [f64; 3],
) -> TerminalSettleOutput {
    let vh_hot = terminal_vh_hot(cheby, aggression);
    let (brake_w, new_latch) =
        terminal_brake_blend(v_cheby, vh, v_approach, cheby, terminal_brake_latched, vh_hot);

    let freedom = settle_lean_freedom(vh);

    let (phase, upright_stable_s) = update_terminal_settle_phase(
        phase,
        upright_stable_s,
        dt,
        up_y,
        omega_py,
        brake_w,
        new_latch,
        v_cheby,
        vh,
        v_approach,
        hp.t_att,
        vh_hot,
    );

    let (desired_raw, lean_max) = match phase {
        TerminalSettlePhase::Brake => {
            let (d, lean) = terminal_brake_aim(
                ux,
                uz,
                vx,
                vz,
                vh,
                v_cheby,
                v_approach,
                cheby,
                hp,
                need_x,
                need_z,
                mass,
                max_thrust,
                brake_mode,
                brake_w,
                freedom,
                aggression,
                aim_filtered,
            );
            (d, lean)
        }
        TerminalSettlePhase::Upright => ([0.0, 1.0, 0.0], 0.0),
        TerminalSettlePhase::Trim => {
            let (d, lean) =
                terminal_trim_aim(ux, uz, vx, vz, vh, cheby, v_cheby, omega_py, freedom, aggression);
            (d, lean)
        }
    };

    TerminalSettleOutput {
        desired_raw,
        lean_max,
        terminal_brake_latch: new_latch,
        phase,
        upright_stable_s,
    }
}

/// Full-T airplane aim: pitch elevator holds `alt_hold` while leaning to pad.
#[inline]
#[allow(dead_code)]
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

/// Hermite smoothstep on `[0, 1]`.
#[inline]
fn smoothstep01(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Terminal-settle vertical setpoint (local law before fuzzy arbitration).
fn terminal_settle_throttle(
    phase: TerminalSettlePhase,
    quiet: bool,
    hover: f64,
    _hover_cmd: f64,
    up_y: f64,
    effort: f64,
    t_hold: f64,
    t_neutral: f64,
    t_motion: f64,
) -> f64 {
    match phase {
        TerminalSettlePhase::Upright => {
            let tilt_cap = hover * (0.92 + 0.06 * up_y.clamp(0.70, 1.0));
            (hover + 0.08 * effort).clamp(tilt_cap * 0.96, (hover + 0.04).min(0.78))
        }
        TerminalSettlePhase::Trim => {
            let t_trim = hover * (0.96 + 0.03 * up_y.clamp(0.90, 1.0));
            if quiet {
                t_hold.clamp(t_trim * 0.97, t_trim + 0.02)
            } else {
                t_hold.clamp(t_trim * 0.95, t_trim + 0.03)
            }
        }
        TerminalSettlePhase::Brake => {
            if quiet {
                t_hold.clamp(t_neutral * 0.90, t_neutral * 0.94)
            } else {
                t_hold.clamp(t_motion * 0.92, (t_motion + 0.08).min(0.85))
            }
        }
    }
}

/// World-frame thrust aim and dive-go membership for high-altitude T dive.
///
/// Under the speed envelope: nose-down dive — slant toward the target when
/// `range > alt`, otherwise pure `[0, −1, 0]`. Inside predicted stop distance,
/// fade lateral slant to pure vertical dive. Over the envelope: blend upright
/// (safe speed first). Returns `(desired_aim, mu_dive_go)` where `mu_dive_go`
/// gates full-T dive acceleration (still needs a nose-down attitude gate).
fn high_alt_freefall_guidance(
    state: &RocketState,
    pos: [f64; 3],
    velocity: [f64; 3],
    target_xz: [f64; 2],
) -> ([f64; 3], f64) {
    let alt = pos[1];
    let v_down = (-velocity[1]).max(0.0);
    let dx = target_xz[0] - pos[0];
    let dz = target_xz[1] - pos[2];
    let range = (dx * dx + dz * dz).sqrt();
    let (ux, uz) = if range > 1e-3 {
        (dx / range, dz / range)
    } else {
        (0.0, 0.0)
    };

    let mu_over = freefall_overspeed_mu(v_down, alt, state.moon_mode);

    // Hard priority: steer toward pad when horizontal range exceeds altitude.
    let prioritize_lateral = range > alt;
    let dive_down = [0.0, -1.0, 0.0];
    let steer_aim = high_alt_freefall_desired_aim(pos, target_xz);
    let base_aim = if prioritize_lateral {
        steer_aim
    } else {
        dive_down
    };

    // Predicted stop distance: PGA flip time + propulsive decel (same as cruise).
    let mass = state.params.mass;
    let max_thrust = state.params.max_thrust;
    let beta = if state.moon_mode {
        0.0
    } else {
        effective_air_drag_beta(state)
    };
    let a_prop = lateral_accel_for_lean(
        std::f64::consts::FRAC_PI_2,
        LateralThrMode::FullThrottle,
        mass,
        max_thrust,
    );
    let v_approach = velocity[0] * ux + velocity[2] * uz;
    let v_closing = v_approach.max(0.0);
    let t_flip = if prioritize_lateral && range > 1e-3 {
        brake_flip_time(go_flip_angle(state, steer_aim))
    } else {
        0.0
    };
    let d_stop = predicted_stop_distance(v_closing, VH_HANDOFF_MAX, a_prop, beta, t_flip);

    // Inside stop envelope: fade lateral slant → pure vertical dive (still nose-down).
    let mu_inside = ramp_down(range, d_stop, d_stop + BRAKE_ENGAGE_MARGIN_M);
    let dive_aim = blend_vec3(base_aim, dive_down, mu_inside);
    // Overspeed: upright so vertical thrust can brake on the freefall envelope.
    let desired = blend_vec3(dive_aim, [0.0, 1.0, 0.0], mu_over);
    let mu_dive_go = 1.0 - mu_over;
    (desired, mu_dive_go)
}

/// High-altitude T dive: nose-down full-T acceleration toward the pad / ground.
/// Predicted stop distance collapses lateral slant to pure vertical dive near the
/// pad. The freefall speed envelope uprights and brakes when `v_down` exceeds
/// [`freefall_v_cap`] (safe descent speed is highest priority).
fn high_alt_freefall_to_pad(state: &RocketState, target_xz: [f64; 2]) -> ControlCommand {
    let pos = state.position();
    let alt = pos[1];
    let v_down = (-state.velocity[1]).max(0.0);
    let mass = state.params.mass;
    let max_thrust = state.params.max_thrust;
    let hover = mass * GRAVITY / max_thrust.max(1e-9);

    let (desired, mu_dive_go) =
        high_alt_freefall_guidance(state, pos, state.velocity, target_xz);

    let (pitch, yaw, roll, up_y) =
        attitude_toward(state, desired, COS_TILT_AIM_FF, false, false);

    let effort = pitch.abs() + yaw.abs() + 0.35 * roll.abs();
    let t_auth = if effort < 0.04 {
        0.0
    } else {
        (0.08 + 0.35 * effort).min(hover * 0.55)
    };

    let t_brake = FreefallThrottleFuzzy {
        alt,
        v_down,
        up_y,
        t_auth,
        t_brake_cmd: THR_FULL,
        upy_brake: 0.25,
        moon_mode: state.moon_mode,
    }
    .arbitrate();

    // Under envelope + nose-down: full-T dive acceleration.
    // Over envelope: exponential freefall brake takes over.
    // While flipping: attitude authority only (avoid lofting upright).
    let t_dive = THR_FULL * mu_dive_go * high_alt_dive_throttle_gate(up_y);
    let throttle = t_dive.max(t_brake).max(t_auth);

    ControlCommand {
        throttle,
        pitch,
        yaw,
        roll,
    }
    .clamp()
}

/// Climb-phase guidance: always full throttle, upright through liftoff, then an
/// open-loop pitch program toward the pad (no MPC / velocity-feedback lean).
fn climb_command(
    state: &RocketState,
    target_xz: [f64; 2],
    pos: [f64; 3],
) -> ControlCommand {
    let dx = target_xz[0] - pos[0];
    let dz = target_xz[1] - pos[2];
    let range = (dx * dx + dz * dz).sqrt();
    let alt = pos[1];

    let desired = if state.contacting || alt < CLIMB_CLEAR_ALT_M || range < 1.0 {
        [0.0, 1.0, 0.0]
    } else {
        let inv_range = 1.0 / range;
        let ux = dx * inv_range;
        let uz = dz * inv_range;
        let mu_long = long_range_weight(range);
        let lean_max = LEAN_CLIMB_MAX + mu_long * (0.90 - LEAN_CLIMB_MAX);
        let u = smoothstep01(ramp(alt, CLIMB_CLEAR_ALT_M, GATE_ALT_MIN));
        let lean = u * lean_max.min(LEAN_LONG_MAX);
        clamp_tilt([ux, 1.0, uz], lean)
    };

    // Soft PD for the whole climb pitch program — a hard soft→stiff gate at
    // ~40 m kicked the gimbal while lean was still opening.
    let (pitch, yaw, roll, _) = attitude_toward(state, desired, COS_TILT_AIM, true, false);

    ControlCommand {
        throttle: THR_FULL,
        pitch,
        yaw,
        roll,
    }
    .clamp()
}

/// Transit guidance for Cruise: MPC selection, stop-distance brake latch,
/// and terminal settle inside the careful envelope.
///
/// Short/mid range: receding-horizon MPC among cruise / brake / coast / sink;
/// airplane range (≳ [`LONG_AIRPLANE_RANGE_M`]): full T + pitch elevator at
/// [`LONG_CRUISE_ALT_M`] (see [`airplane_hold_aim`]). Returns command, brake latch,
/// terminal brake latch, MPC hold state, and terminal settle sub-phase.
fn transit_command(
    state: &RocketState,
    target_xz: [f64; 2],
    pos: [f64; 3],
    brake_latched: bool,
    terminal_latched: bool,
    terminal_brake_latched: bool,
    terminal_settle_phase: TerminalSettlePhase,
    upright_stable_s: f64,
    mpc_hold: TransitCandidate,
    mpc_hold_counter: u32,
    dt: f64,
    aim_filtered: &mut [f64; 3],
    aim_filter_sync: &mut bool,
) -> (
    ControlCommand,
    bool,
    bool,
    TerminalSettlePhase,
    f64,
    TransitCandidate,
    u32,
) {
    let dx = target_xz[0] - pos[0];
    let dz = target_xz[1] - pos[2];
    let range = (dx * dx + dz * dz).sqrt();
    let cheby = chebyshev_xz(pos, target_xz);
    let near_handoff = near_handoff_zone(terminal_latched, cheby);
    let vx = state.velocity[0];
    let vy = state.velocity[1];
    let vz = state.velocity[2];
    let lofted = transit_lofted(pos[1], vy, near_handoff);
    let aggression = careful_aggression(range);
    let mu_long = long_range_weight(range);

    let vh = (vx * vx + vz * vz).sqrt();

    let mass = state.params.mass;
    let max_thrust = state.params.max_thrust;
    let hover = mass * GRAVITY / max_thrust;

    let in_airplane_range = range >= LONG_AIRPLANE_RANGE_M;
    let brake_mode = brake_lateral_mode(in_airplane_range, vh, state.moon_mode);
    let a_brake_max = lateral_accel_for_lean(
        LEAN_BRAKE_MAX,
        brake_mode,
        mass,
        max_thrust,
    );
    // Powered-cruise weight: 1 at vy ≤ +3, 0 at vy ≥ +8 (ballistic coast).
    let cruise_w = (1.0 - (vy - 3.0) / 5.0).clamp(0.0, 1.0);
    let ballistic = cruise_w < 1.0;

    let inv_range = if range > 1e-3 { 1.0 / range } else { 0.0 };
    let ux = dx * inv_range;
    let uz = dz * inv_range;
    let v_approach = vx * ux + vz * uz;
    let v_cheby = chebyshev_closing_rate(pos, target_xz, state.velocity);

    let terminal = lofted && terminal_latched;
    let range_eff = (range - CAREFUL_NEAR_M).max(0.0);
    let aim_prev = *aim_filtered;

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
        effective_air_drag_beta(state)
    };
    let handoff_plan = if lofted
        && (terminal || brake_latched || cheby <= TERMINAL_EXIT_CHEBY_M)
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

    let v_allow = if terminal {
        // Terminal == careful envelope: creep speed only.
        trim_creep_speed(cheby.max(1.0), aggression)
            .min(careful(0.20, aggression) * (range - 4.0).max(0.0))
            .min(terminal_v_cap(cheby, aggression))
            .clamp(0.0, VH_HANDOFF_MAX)
    } else {
        let p = plan.unwrap();
        let v = allowed_approach_speed(
            range_eff,
            p.v_end,
            p.a_prop,
            p.beta,
            p.t_flip_go,
            BRAKE_ENGAGE_MARGIN_M,
        );
        if ballistic {
            v.min(V_CLIMB_H_MAX)
        } else {
            v
        }
    };

    let need_x = ux * v_allow - vx;
    let need_z = uz * v_allow - vz;

    // Pitch-elevator altitude target — [`LONG_CRUISE_ALT_M`] at all ranges
    // (short-hop [`CRUISE_ALT_CAP`] matches the long-range hold).
    let alt_hold = if in_airplane_range {
        LONG_CRUISE_ALT_M
    } else {
        CRUISE_ALT_CAP + mu_long * (LONG_CRUISE_ALT_M - CRUISE_ALT_CAP)
    };

    // Aim regime: terminal settle (fixed) or MPC-selected transit.
    let mut terminal_settle_out: Option<TerminalSettleOutput> = None;
    let mut mpc_out_hold = mpc_hold;
    let mut mpc_out_counter = mpc_hold_counter;
    let mut brake_hardness = 0.0;
    let mut cruise_brake: Option<CruiseBrakeCommand> = None;
    let (desired_raw, lean_max, deep, force_full_thr, terminal_brake_out) =
        if terminal {
        let hp = handoff_plan.unwrap();
        let up_y = world_up_in_body(&state.motor)[1];
        let om = state.omega;
        let omega_py = (om[0] * om[0] + om[2] * om[2]).sqrt();
        let out = terminal_settle_aim(
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
            terminal_settle_phase,
            upright_stable_s,
            dt,
            up_y,
            omega_py,
            aggression,
            aim_prev,
        );
        terminal_settle_out = Some(out);
        (
            out.desired_raw,
            out.lean_max,
            false,
            false,
            out.terminal_brake_latch,
        )
    } else {
        let (mut mpc_plan, out_hold, out_counter) = transit_mpc_select(
            state,
            pos,
            target_xz,
            ux,
            uz,
            vx,
            vz,
            vh,
            range,
            alt_hold,
            hover,
            mu_long,
            in_airplane_range,
            lofted,
            ballistic,
            brake_latched,
            brake,
            mpc_hold,
            mpc_hold_counter,
        );
        mpc_out_hold = out_hold;
        mpc_out_counter = out_counter;

        if brake {
            let cmd = cruise_brake_command(vx, vz, vh, v_approach, aim_prev);
            brake_hardness = cmd.hardness;
            cruise_brake = Some(cmd);
            mpc_plan = TransitMpcPlan {
                desired_raw: cmd.aim,
                lean_max: cmd.lean_cap,
                deep: cmd.hardness > 0.25,
                force_full_thr: brake_force_full_throttle(
                    in_airplane_range,
                    vh,
                    state.moon_mode,
                    cmd.hardness,
                ),
            };
            mpc_out_hold = TransitCandidate::Brake;
        }

        if !mpc_plan.force_full_thr && !brake {
            mpc_plan.desired_raw[0] += 0.05 * need_x;
            mpc_plan.desired_raw[2] += 0.05 * need_z;
        }

        (
            mpc_plan.desired_raw,
            mpc_plan.lean_max,
            mpc_plan.deep,
            mpc_plan.force_full_thr,
            terminal_brake_latched,
        )
    };

    // Deep airplane lean must not be faded by cruise_w (half-open aim = sway).
    let aim_w = if deep { 1.0 } else { cruise_w };
    let desired = clamp_tilt(
        [aim_w * desired_raw[0], desired_raw[1], aim_w * desired_raw[2]],
        lean_max,
    );
    let terminal_phase = terminal_settle_out.map(|o| o.phase);
    let slew_rate = aim_slew_rate(brake, brake_hardness, deep, terminal, terminal_phase);
    let desired = filter_and_slew_aim(aim_filtered, aim_filter_sync, desired, dt, slew_rate);
    // Deep / airplane lean: low flip gate so nose-down is tracked, not "recovered".
    // Once brake hardness fades, restore the upright flip gate for settle.
    let flip_cos = if (force_full_thr || deep) && !(brake && brake_hardness < 0.40) {
        COS_TILT_AIM_AIR
    } else {
        COS_TILT_AIM
    };
    // Soft attitude PD while killing brake lean / trimming — less snap overshoot.
    let brake_agg_from_h = cruise_brake.as_ref().map(|c| c.aggressive_att).unwrap_or(false);
    let brake_soft_from_h = cruise_brake.as_ref().map(|c| c.soft_att).unwrap_or(false);
    let soft_att = matches!(
        terminal_settle_out.map(|o| o.phase),
        Some(TerminalSettlePhase::Upright | TerminalSettlePhase::Trim)
    ) || (brake && !terminal && brake_soft_from_h);
    let brake_aggressive_att = brake && !terminal && brake_agg_from_h;
    let (pitch, yaw, roll, up_y) =
        attitude_toward(state, desired, flip_cos, soft_att, brake_aggressive_att);

    let upy_floor = if deep { 0.45 } else { 0.40 };
    let hover_cmd = (hover / up_y.max(upy_floor)).clamp(0.0, 0.95);

    let v_damp = if up_y < 0.92 {
        motor_inverse_rotate_vector(&state.motor, state.velocity)[1]
    } else {
        vy
    };
    let v_des_y = if lofted {
        cruise_v_des_y(pos[1], vy, terminal)
    } else {
        kill_climb_vy(vy)
    };
    let kv = if lofted { 0.12 } else { 0.08 };
    let base = hover_cmd + kv * (v_des_y - vy) - 0.03 * v_damp.clamp(-5.0, 5.0);
    let t_hold = cruise_w * base.max(hover_cmd * 0.65);

    let effort = pitch.abs() + yaw.abs() + 0.35 * roll.abs();
    let t_auth = (0.9 * (effort - 0.15).max(0.0)).min(0.35);

    let climb_cut = if !brake && pos[1] > CRUISE_ALT_CAP + 50.0 && vy > 1.5 {
        (0.04 * (vy - 1.5)).min(0.08)
    } else {
        0.0
    };
    let t_neutral = hover_cmd * (1.0 - climb_cut);
    let t_deep = (t_neutral + 0.08 * effort).clamp(t_neutral * 0.92, t_neutral + 0.12);

    let hp = handoff_plan;
    let t_settle = if terminal {
        let p = hp.unwrap();
        let settle = terminal_settle_out.as_ref().unwrap();
        let quiet = p.cleared() || p.t_settle < 0.35;
        let t_neutral_settle = hover_cmd;
        let motion_blend = (p.t_pos.max(p.t_vh) / 3.5).clamp(0.0, 0.45);
        let t_motion = (t_neutral_settle * (0.94 - 0.06 * motion_blend))
            .clamp(t_neutral_settle * 0.86, t_neutral_settle + 0.05);
        terminal_settle_throttle(
            settle.phase,
            quiet,
            hover,
            hover_cmd,
            up_y,
            effort,
            t_hold,
            t_neutral_settle,
            t_motion,
        )
    } else {
        t_hold
    };

    let t_contact = hover_cmd.mul_add(1.45, 0.0).max(0.60);

    let throttle = CruiseThrottleFuzzy {
        force_full_thr,
        deep,
        terminal,
        ballistic,
        contacting: state.contacting,
        brake,
        brake_hardness,
        vy,
        effort,
        t_hold,
        t_full: THR_FULL,
        t_auth,
        t_deep,
        t_settle,
        t_contact,
    }
    .arbitrate();

    let cmd = ControlCommand {
        throttle: throttle.clamp(0.0, 1.0),
        pitch,
        yaw,
        roll,
    }
    .clamp();
    let out_phase = terminal_settle_out
        .map(|o| o.phase)
        .unwrap_or(terminal_settle_phase);
    let out_upright_s = terminal_settle_out
        .map(|o| o.upright_stable_s)
        .unwrap_or(upright_stable_s);
    (cmd, brake, terminal_brake_out, out_phase, out_upright_s, mpc_out_hold, mpc_out_counter)
}

/// Attitude PD toward a world-frame desired body +Y via PGA inverse transport.
///
/// `flip_cos`: if body-up·world-up falls below this, command pure upright
/// recovery (inverted / tumble). Airplane cruise passes a low gate so deep
/// dive lean is tracked instead of fought.
///
/// `soft`: lower rate command / higher damping for terminal upright settle so
/// the brake→upright snap does not overshoot into a pendulum half-cycle.
fn attitude_toward(
    state: &RocketState,
    desired_world: [f64; 3],
    flip_cos: f64,
    soft: bool,
    brake_aggressive: bool,
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

    let (kp, kd, w_cap, rate_kill) = if soft {
        (KP_ATT * 0.55, KD_ATT * 1.35, OMEGA_MAX * 0.45, OMEGA_RATE_KILL * 0.45)
    } else if brake_aggressive {
        (KP_ATT, KD_ATT, OMEGA_MAX, OMEGA_RATE_KILL_BRAKE)
    } else {
        (KP_ATT, KD_ATT, OMEGA_MAX, OMEGA_RATE_KILL)
    };

    // Soft fade: kill the position-rate command as residual rate approaches
    // `rate_kill`, instead of a hard cut that bang-bangs across the threshold.
    let rate_fade = ramp_down(omega_xy, rate_kill * 0.60, rate_kill);
    let w_cmd = (kp * angle)
        .min((2.0 * ALPHA_PLAN * angle).sqrt())
        .min(w_cap)
        .min((w_cap - 0.4 * omega_xy).max(0.0));
    let w_mag = w_cmd * rate_fade;
    let pitch = saturate(kd * (omega[0] - axis[0] * w_mag));
    let yaw = saturate(kd * (omega[2] - axis[2] * w_mag));
    let roll = saturate(-KD_ROLL * omega[1]);
    (pitch, yaw, roll, up_y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fuzzy::CAREFUL_AGGRESSION_MIN;
    use crate::landing::{H_FREEFALL_EARTH_M, TARGET_SUCCESS_HALF_M};

    const TEST_DT: f64 = 1.0 / 120.0;
    const FF_TEST_ALT: f64 = H_FREEFALL_EARTH_M + 200.0;

    /// Advance the autopilot until the throttle actuator reaches `min_throttle`.
    fn spool_autopilot(
        ap: &mut TargetLandingAutopilot,
        state: &RocketState,
        target: [f64; 2],
        min_throttle: f64,
        max_steps: u32,
    ) -> ControlCommand {
        let mut cmd = ControlCommand::default();
        for _ in 0..max_steps {
            cmd = ap.update(state, target, TEST_DT);
            if cmd.throttle >= min_throttle {
                break;
            }
        }
        cmd
    }

    #[test]
    fn cruise_throttle_slew_limits_step_changes() {
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M);
        state.contacting = false;
        state.velocity = [40.0, 0.0, 0.0];
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let target = [6000.0, 0.0];
        let first = ap.update(&state, target, TEST_DT);
        let second = ap.update(&state, target, TEST_DT);
        assert!(
            (second.throttle - first.throttle).abs() <= THROTTLE_SPOOL_UP_EMERGENCY * TEST_DT + 1e-9,
            "cruise throttle must slew, first={} second={}",
            first.throttle,
            second.throttle
        );
        let spooled = spool_autopilot(&mut ap, &state, target, THR_FULL - 0.05, 60);
        assert!(
            spooled.throttle > 0.9,
            "long-range cruise should reach near full T after spool, thr={}",
            spooled.throttle
        );
    }

    #[test]
    fn slew_aim_world_respects_rate_limit_per_step() {
        let current = [0.0, 1.0, 0.0];
        let target = [1.0, 0.0, 0.0];
        let dt = TEST_DT;
        let rate = 2.0;
        let out = slew_aim_world(current, target, dt, rate);
        let step = unit_angle(current, out);
        assert!(
            step <= rate * dt + 1e-9,
            "slew step {step} exceeds rate*dt={}",
            rate * dt
        );
        assert!(out[1] > 0.0, "slew should move toward target, got {out:?}");
    }

    #[test]
    fn slew_aim_world_antipodal_does_not_snap() {
        let current = [0.0, 1.0, 0.0];
        let target = [0.0, -1.0, 0.0];
        let dt = TEST_DT;
        let rate = AIM_SLEW_HARD;
        let out = slew_aim_world(current, target, dt, rate);
        let step = unit_angle(current, out);
        assert!(
            step <= rate * dt + 1e-6,
            "antipodal slew must not snap, step={step}"
        );
        assert!(
            (out[0] * out[0] + out[1] * out[1] + out[2] * out[2] - 1.0).abs() < 1e-9,
            "result must stay unit, got {out:?}"
        );
    }

    #[test]
    fn filter_and_slew_aim_limits_flip_on_brake_target() {
        let mut filtered = [0.0, 1.0, 0.0];
        let mut sync = false;
        let dt = TEST_DT;
        let rate = AIM_SLEW_SOFT;
        let target_a = normalize_vec3([-1.0, 1.0, 0.0]).unwrap();
        let target_b = normalize_vec3([1.0, 1.0, 0.0]).unwrap();
        let first = filter_and_slew_aim(&mut filtered, &mut sync, target_a, dt, rate);
        let second = filter_and_slew_aim(&mut filtered, &mut sync, target_b, dt, rate);
        let flip_step = unit_angle(first, second);
        assert!(
            flip_step <= rate * dt + 1e-9,
            "180° brake flip must slew, step={flip_step} max={}",
            rate * dt
        );
    }

    #[test]
    fn low_vh_brake_anti_uses_filtered_azimuth() {
        let aim = normalize_vec3([-0.8, 0.6, 0.0]).unwrap();
        let (ax, az) = brake_anti_horizontal(0.1, 0.0, 3.0, aim);
        let h_len = (ax * ax + az * az).sqrt();
        assert!(h_len > 0.9, "filtered azimuth should be unit horizontal, got ({ax},{az})");
        assert!(ax < -0.5, "should keep filtered -X brake direction, ax={ax}");
    }

    #[test]
    fn slew_command_axis_respects_rate_limit() {
        let dt = TEST_DT;
        let rate = GIMBAL_SLEW_RATE;
        let out = slew_command_axis(0.0, 1.0, dt, rate);
        assert!(
            out <= rate * dt + 1e-9,
            "gimbal slew step {out} exceeds rate*dt={}",
            rate * dt
        );
        let flip = slew_command_axis(1.0, -1.0, dt, rate);
        assert!(
            (1.0 - flip) <= rate * dt + 1e-9,
            "gimbal reverse must slew, got {flip}"
        );
    }

    #[test]
    fn gimbal_actuator_limits_saturated_yaw_flips() {
        let mut state = RocketState::resting_on_pad();
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let target = [500.0, 0.0];
        let mut prev_y = 0.0f64;
        let mut hard_flips = 0u32;
        for _ in 0..120 * 40 {
            let cmd = ap.update(&state, target, TEST_DT);
            if prev_y.abs() > 0.85
                && cmd.yaw.abs() > 0.85
                && prev_y.signum() != cmd.yaw.signum()
            {
                hard_flips += 1;
            }
            prev_y = cmd.yaw;
            state.set_command(cmd);
            crate::sim::step_rocket(&mut state, TEST_DT);
            if state.destroyed || ap.complete {
                break;
            }
        }
        assert!(
            hard_flips < 8,
            "saturated yaw must not bang-bang; hard_flips={hard_flips}"
        );
    }

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
    fn high_altitude_cruise_translates_toward_pad() {
        // Far from pad (range > alt), nose-down under envelope → full-T dive.
        let mut state = RocketState::at_altitude(FF_TEST_ALT);
        state.motor = crate::euclidean_pga::motor_from_pose(
            0.0,
            FF_TEST_ALT,
            0.0,
            std::f64::consts::PI,
            0.0,
            0.0,
        );
        state.velocity = [0.0, -15.0, 0.0];
        state.contacting = false;
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        ap.phase = TargetPhase::Cruise;
        let target = [8000.0, 0.0];
        let mut cmd = ControlCommand::default();
        for _ in 0..120 {
            cmd = ap.update(&state, target, 1.0 / 120.0);
        }
        assert!(
            cmd.throttle > 0.85,
            "T dive to pad should burn at high T, thr={}",
            cmd.throttle
        );
    }

    #[test]
    fn high_altitude_freefall_aims_toward_target_when_range_exceeds_alt() {
        // range=8000 > alt=6200 → slant aim toward target (downward component).
        let mut state = RocketState::at_altitude(FF_TEST_ALT);
        state.velocity = [0.0, -15.0, 0.0];
        state.contacting = false;
        let (aim, mu_go) = high_alt_freefall_guidance(
            &state,
            state.position(),
            state.velocity,
            [8000.0, 0.0],
        );
        assert!(
            aim[1] < 0.0,
            "slant aim toward distant pad should point downward, y={}",
            aim[1]
        );
        assert!(aim[0] > 0.5, "should lean toward +X target, x={}", aim[0]);
        assert!(mu_go > 0.9, "under envelope should dive-go, mu={mu_go}");
    }

    #[test]
    fn high_altitude_dive_vertical_inside_d_stop() {
        // 5 m from pad → inside d_stop: pure vertical dive, still dive-go.
        let mut state = RocketState::at_altitude(FF_TEST_ALT);
        state.motor = crate::euclidean_pga::motor_from_pose(
            5.0,
            FF_TEST_ALT,
            0.0,
            std::f64::consts::PI,
            0.0,
            0.0,
        );
        state.velocity = [-3.0, -15.0, 0.0];
        state.contacting = false;
        let (aim, mu_go) = high_alt_freefall_guidance(
            &state,
            state.position(),
            state.velocity,
            [0.0, 0.0],
        );
        assert!(
            mu_go > 0.9,
            "inside d_stop should still dive-go, mu={mu_go}"
        );
        assert!(
            (aim[1] + 1.0).abs() < 0.05,
            "inside d_stop should pure vertical dive, y={}",
            aim[1]
        );
        let cmd = high_alt_freefall_to_pad(&state, [0.0, 0.0]);
        assert!(
            cmd.throttle > 0.85,
            "nose-down dive inside d_stop should full-T, thr={}",
            cmd.throttle
        );
    }

    #[test]
    fn high_altitude_freefall_descend_priority_when_range_below_alt() {
        // range=500 m < alt=6200 m → pure vertical dive.
        let mut state = RocketState::at_altitude(FF_TEST_ALT);
        state.motor = crate::euclidean_pga::motor_from_pose(500.0, FF_TEST_ALT, 0.0, 0.0, 0.0, 0.0);
        state.velocity = [0.0, -15.0, 0.0];
        state.contacting = false;
        let (aim, mu_go) = high_alt_freefall_guidance(
            &state,
            state.position(),
            state.velocity,
            [0.0, 0.0],
        );
        assert!(mu_go > 0.9, "under envelope should dive-go, mu={mu_go}");
        assert!(
            (aim[1] + 1.0).abs() < 1e-9,
            "descend priority should aim nose-down, y={}",
            aim[1]
        );
    }

    #[test]
    fn high_altitude_dive_over_pad_under_envelope() {
        // Over the pad at 6.2 km, under envelope → pure nose-down dive.
        let mut state = RocketState::at_altitude(FF_TEST_ALT);
        state.velocity = [0.0, -10.0, 0.0];
        state.contacting = false;
        let (aim, mu_go) = high_alt_freefall_guidance(
            &state,
            state.position(),
            state.velocity,
            [0.0, 0.0],
        );
        assert!(
            (aim[1] + 1.0).abs() < 1e-9,
            "over pad under envelope should dive nose-down, y={}",
            aim[1]
        );
        assert!(mu_go > 0.9, "over pad under envelope should dive-go, mu={mu_go}");
    }

    #[test]
    fn high_altitude_cruise_brakes_on_speed_cap() {
        // 6200 m → v_cap = 280; 380 m/s is deep overspeed → upright brake.
        let mut state = RocketState::at_altitude(FF_TEST_ALT);
        state.velocity = [0.0, -380.0, 0.0];
        state.contacting = false;
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        ap.phase = TargetPhase::Cruise;
        let target = [8000.0, 0.0];
        let mut cmd = ControlCommand::default();
        for _ in 0..120 {
            cmd = ap.update(&state, target, 1.0 / 120.0);
        }
        assert!(
            cmd.throttle > 0.85,
            "T-mode high-alt fast fall should brake, thr={}",
            cmd.throttle
        );
        // Overspeed aim is upright; under-envelope dive aim is nose-down.
        let (aim_over, _) = high_alt_freefall_guidance(
            &state,
            state.position(),
            state.velocity,
            target,
        );
        let mut slow = RocketState::at_altitude(FF_TEST_ALT);
        slow.velocity = [0.0, -15.0, 0.0];
        slow.contacting = false;
        let (aim_dive, _) = high_alt_freefall_guidance(
            &slow,
            slow.position(),
            slow.velocity,
            target,
        );
        assert!(
            aim_over[1] > aim_dive[1] + 0.5,
            "overspeed should be more upright than dive, over y={} dive y={}",
            aim_over[1],
            aim_dive[1]
        );
    }

    #[test]
    fn apogee_prediction_arms_cruise_before_altitude_gate() {
        let mut state = RocketState::at_altitude(250.0);
        state.contacting = false;
        // vy ≈ 71 m/s → ballistic apogee ≈ 250 + 5041/19.6 > 500 m
        state.velocity[1] = 71.0;
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        ap.phase = TargetPhase::Climb;
        let cmd = ap.update(&state, [500.0, 0.0], 1.0 / 120.0);
        assert_eq!(
            ap.phase,
            TargetPhase::Cruise,
            "predicted 500 m apogee must cut climb early"
        );
        assert!(
            cmd.throttle < THR_FULL,
            "cruise should bleed climb rate, thr={}",
            cmd.throttle
        );
    }

    #[test]
    fn apogee_below_target_stays_in_climb() {
        let mut state = RocketState::at_altitude(250.0);
        state.contacting = false;
        state.velocity[1] = 40.0;
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let cmd = spool_autopilot(&mut ap, &state, [500.0, 0.0], THR_FULL - 0.02, 80);
        assert_eq!(ap.phase, TargetPhase::Climb);
        assert!((cmd.throttle - THR_FULL).abs() < 0.02);
    }

    #[test]
    fn climb_from_pad_is_upright_full_throttle() {
        let state = RocketState::resting_on_pad();
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let cmd = spool_autopilot(&mut ap, &state, [500.0, 0.0], THR_FULL - 0.02, 80);
        assert_eq!(ap.phase, TargetPhase::Climb);
        assert!(
            (cmd.throttle - THR_FULL).abs() < 0.02,
            "climb must be full throttle, thr={}",
            cmd.throttle
        );
        assert!(
            cmd.pitch.abs() + cmd.yaw.abs() < 0.05,
            "pad liftoff must stay upright, pitch={} yaw={}",
            cmd.pitch,
            cmd.yaw
        );
    }

    #[test]
    fn low_altitude_climb_leans_toward_target() {
        let mut state = RocketState::at_altitude(200.0);
        state.contacting = false;
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let cmd = spool_autopilot(&mut ap, &state, [500.0, 0.0], THR_FULL - 0.02, 80);
        assert_eq!(ap.phase, TargetPhase::Climb);
        assert!(
            (cmd.throttle - THR_FULL).abs() < 0.02,
            "climb must be full throttle, thr={}",
            cmd.throttle
        );
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
        // Complete region is the painted pad (±TARGET_PAD_HALF_M), not the inner aim box.
        assert!(inside_target_pad(
            [500.0 + TARGET_PAD_HALF_M, 0.0, 0.0],
            [500.0, 0.0]
        ));
        assert!(!inside_target_pad(
            [500.0 + TARGET_PAD_HALF_M + 0.1, 0.0, 0.0],
            [500.0, 0.0]
        ));
        // Outside the inner aim box but still on the painted pad counts as on-pad.
        assert!(inside_target_pad(
            [500.0 + TARGET_SUCCESS_HALF_M + 0.1, 0.0, 0.0],
            [500.0, 0.0]
        ));
        assert!(
            TARGET_PAD_HALF_M > TARGET_SUCCESS_HALF_M,
            "visual pad should exceed inner guidance box"
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
    fn cruise_brake_full_hardness_reaches_long_range_lean() {
        let cmd = cruise_brake_command(-40.0, 0.0, 40.0, 40.0, [0.0, 1.0, 0.0]);
        assert!(
            (cmd.lean_cap - LEAN_LONG_MAX).abs() < 1e-9,
            "full hardness should open to cruise lean cap, got {}",
            cmd.lean_cap
        );
        assert!(
            (LEAN_BRAKE_MAX - LEAN_LONG_MAX).abs() < 1e-9,
            "brake and cruise lean ceilings should stay aliased"
        );
        let aim = clamp_tilt([-1.0, brake_aim_y_bias(cmd.lean_cap), 0.0], cmd.lean_cap);
        let len = (aim[0] * aim[0] + aim[1] * aim[1] + aim[2] * aim[2]).sqrt();
        let tilt = (aim[1] / len).acos();
        assert!(
            (tilt - LEAN_BRAKE_MAX).abs() < 0.05,
            "full brake aim should reach LEAN_BRAKE_MAX, got tilt={tilt}"
        );
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
        let v = allowed_approach_speed(range, VH_HANDOFF_MAX, a, 0.0, t, 0.0);
        let d = predicted_stop_distance(v, VH_HANDOFF_MAX, a, 0.0, t);
        assert!(
            (d - range).abs() < 0.5,
            "v={v} d={d} range={range}"
        );
    }

    #[test]
    fn allowed_speed_inverts_stop_distance_with_engage_margin() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let a = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        let t = 0.5;
        let range_eff = 400.0;
        let margin = BRAKE_ENGAGE_MARGIN_M;
        let v = allowed_approach_speed(range_eff, VH_HANDOFF_MAX, a, 0.0, t, margin);
        let d = predicted_stop_distance(v, VH_HANDOFF_MAX, a, 0.0, t);
        assert!(
            (d - (range_eff - margin)).abs() < 0.5,
            "v={v} d={d} budget={}",
            range_eff - margin
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
        let v_near = allowed_approach_speed(200.0, VH_HANDOFF_MAX, a, 0.0, 0.5, 0.0);
        let v_far = allowed_approach_speed(800.0, VH_HANDOFF_MAX, a, 0.0, 0.5, 0.0);
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
        // Fast close inside predicted stop distance + engage margin.
        let target = [250.0, 0.0];
        let cmd = spool_autopilot(&mut ap, &state, target, 0.92, 80);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        assert!(
            !ap.is_long_range_cruise(state.position(), target),
            "braking must drop airplane HUD flag"
        );
        assert!(
            cmd.pitch.abs() + cmd.yaw.abs() > 0.05,
            "expected brake lean, pitch={} yaw={}",
            cmd.pitch,
            cmd.yaw
        );
        assert!(
            cmd.throttle > 0.92,
            "high-speed brake must use full-T, thr={}",
            cmd.throttle
        );
    }

    /// Advance autopilot without physics so actuators can leave neutral.
    fn spool_frames(
        ap: &mut TargetLandingAutopilot,
        state: &RocketState,
        target: [f64; 2],
        frames: u32,
    ) -> ControlCommand {
        let mut cmd = ControlCommand::default();
        for _ in 0..frames {
            cmd = ap.update(state, target, TEST_DT);
        }
        cmd
    }

    #[test]
    fn far_cruise_leans_toward_target_when_underspeed() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [20.0, 0.0, 0.0]; // well under envelope at 500 m range
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let cmd = spool_frames(&mut ap, &state, [500.0, 0.0], 12);
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
        let cmd = spool_frames(&mut ap, &state, [120.0, 0.0], 12);
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
        // 6 km out at LONG_CRUISE_ALT_M (~520 m) → full throttle, not hover.
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M);
        state.contacting = false;
        state.velocity = [40.0, 0.0, 0.0];
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let target = [6000.0, 0.0];
        let cmd = spool_autopilot(&mut ap, &state, target, 0.9, 80);
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
            plan.t_vh > 0.15,
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
            plan.t_pos > 0.5,
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
    fn cruise_v_des_y_terminal_sinks_toward_handoff_alt() {
        let sink_high = cruise_v_des_y(520.0, 0.0, true);
        assert!(
            sink_high <= -0.8,
            "terminal settle above HANDOFF_ALT_M should sink, got {sink_high}"
        );
        let hold_at = cruise_v_des_y(HANDOFF_ALT_M, 0.0, true);
        assert!(
            hold_at.abs() < 1e-9,
            "at HANDOFF_ALT_M terminal should hold altitude, got {hold_at}"
        );
        let non_terminal = cruise_v_des_y(530.0, 0.0, false);
        assert!(
            non_terminal <= -0.8,
            "non-terminal above CRUISE_ALT_CAP should still bleed, got {non_terminal}"
        );
    }

    #[test]
    fn deep_lean_uses_vertical_neutral_not_starvation_cap() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [8.0, 0.0, 0.0];
        state.motor = crate::euclidean_pga::motor_from_pose(520.0, 500.0, 0.0, 0.05, 0.0, 0.0);
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let cmd = spool_autopilot(&mut ap, &state, [500.0, 0.0], 0.26, 40);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        // Lower bound leaves room for the terminal-approach sink bias
        // (cruise_v_des_y with terminal=true sinks toward HANDOFF_ALT_M).
        assert!(
            cmd.throttle > 0.26,
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

    #[test]
    fn brake_decel_demand_mild_vs_hard() {
        let mild = brake_decel_demand(3.0, 0.5, 2.0, 0.2);
        let hard = brake_decel_demand(18.0, -3.0, -6.0, 3.0);
        assert!(
            mild < 0.35,
            "quiet approach should be mild demand, got {mild}"
        );
        assert!(
            hard > 0.75,
            "fast overshoot should be hard demand, got {hard}"
        );
        assert!(hard > mild);
    }

    #[test]
    fn terminal_mild_brake_keeps_shallow_lean() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [-2.5, 0.0, 0.0];
        state.motor = crate::euclidean_pga::motor_from_pose(515.0, 500.0, 0.0, 0.0, 0.0, 0.0);
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        let hp = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            2.5,
            v_cheby,
            0.3,
            0.0,
        );
        let (_aim, lean) = terminal_brake_aim(
            -1.0,
            0.0,
            -2.5,
            0.0,
            2.5,
            v_cheby,
            2.5,
            15.0,
            hp,
            1.0,
            0.0,
            state.params.mass,
            state.params.max_thrust,
            LateralThrMode::VerticalNeutral,
            0.25,
            settle_lean_freedom(2.5),
            careful_aggression(80.0),
            [0.0, 1.0, 0.0],
        );
        assert!(
            lean < 0.40,
            "mild terminal brake must not deep-lean, lean={lean}"
        );
    }

    #[test]
    fn terminal_hard_brake_can_use_deep_lean() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [55.0, 0.0, 0.0];
        state.motor = crate::euclidean_pga::motor_from_pose(540.0, 500.0, 0.0, 0.0, 0.0, 0.0);
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        let hp = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            55.0,
            v_cheby,
            LEAN_BRAKE_MAX,
            0.0,
        );
        let (_aim, lean) = terminal_brake_aim(
            -1.0,
            0.0,
            55.0,
            0.0,
            55.0,
            v_cheby,
            -40.0,
            40.0,
            hp,
            -15.0,
            0.0,
            state.params.mass,
            state.params.max_thrust,
            LateralThrMode::VerticalNeutral,
            1.0,
            settle_lean_freedom(55.0),
            careful_aggression(80.0),
            [0.0, 1.0, 0.0],
        );
        assert!(
            lean > 0.55,
            "hard overshoot brake should open deep lean, lean={lean}"
        );
    }

    #[test]
    fn terminal_settle_brake_releases_to_upright_when_tilted() {
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Brake,
            0.0,
            1.0 / 120.0,
            0.88,
            0.05,
            0.0,
            false,
            0.5,
            2.0,
            1.0,
            0.0,
            VH_HANDOFF_MAX * 1.35,
        );
        assert_eq!(phase.0, TerminalSettlePhase::Upright);
    }

    #[test]
    fn terminal_settle_brake_always_enters_upright() {
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Brake,
            0.0,
            1.0 / 120.0,
            0.98,
            0.05,
            0.0,
            false,
            0.5,
            2.0,
            1.0,
            0.0,
            VH_HANDOFF_MAX * 1.35,
        );
        assert_eq!(phase.0, TerminalSettlePhase::Upright);
    }

    #[test]
    fn terminal_settle_upright_holds_before_trim() {
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Upright,
            0.0,
            0.05,
            0.98,
            OMEGA_TRIM_ENTER * 0.5,
            0.0,
            false,
            0.5,
            2.0,
            1.0,
            0.0,
            VH_HANDOFF_MAX * 1.35,
        );
        assert_eq!(phase.0, TerminalSettlePhase::Upright);
        assert!(phase.1 > 0.0, "upright stable timer should accumulate");
    }

    #[test]
    fn terminal_settle_upright_holds_while_rate_above_trim_enter() {
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Upright,
            UPRIGHT_STABLE_MIN_S,
            0.02,
            0.98,
            OMEGA_TRIM_ENTER + 0.01,
            0.0,
            false,
            0.5,
            2.0,
            1.0,
            0.0,
            VH_HANDOFF_MAX * 1.35,
        );
        assert_eq!(phase.0, TerminalSettlePhase::Upright);
    }

    #[test]
    fn terminal_settle_upright_advances_to_trim_after_stable() {
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Upright,
            UPRIGHT_STABLE_QUIET_S - 0.01,
            0.02,
            0.98,
            OMEGA_TRIM_ENTER * 0.5,
            0.0,
            false,
            0.5,
            2.0,
            1.0,
            0.0,
            VH_HANDOFF_MAX * 1.35,
        );
        assert_eq!(phase.0, TerminalSettlePhase::Trim);
    }

    #[test]
    fn terminal_upright_aim_is_pure_vertical() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [-0.8, 0.0, 0.0];
        state.motor = crate::euclidean_pga::motor_from_pose(505.0, 500.0, 0.0, 0.35, 0.0, 0.0);
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        assert!(v_cheby > 0.0, "test setup must close on pad, v_cheby={v_cheby}");
        let hp = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            0.8,
            v_cheby,
            0.4,
            0.0,
        );
        let out = terminal_settle_aim(
            hp,
            -1.0,
            0.0,
            0.5,
            0.0,
            -0.8,
            0.0,
            0.8,
            5.0,
            v_cheby,
            0.8,
            state.params.mass,
            state.params.max_thrust,
            LateralThrMode::VerticalNeutral,
            false,
            TerminalSettlePhase::Upright,
            0.0,
            1.0 / 120.0,
            world_up_in_body(&state.motor)[1],
            0.08,
            careful_aggression(CAREFUL_NEAR_M),
            [0.0, 1.0, 0.0],
        );
        assert_eq!(out.phase, TerminalSettlePhase::Upright);
        assert!(
            out.desired_raw[0].abs() + out.desired_raw[2].abs() < 1e-6,
            "upright phase must not command lateral aim, got {:?}",
            out.desired_raw
        );
        assert!(
            out.lean_max <= 1e-6,
            "upright lean cap should be zero, lean={}",
            out.lean_max
        );
    }

    #[test]
    fn terminal_trim_deadzone_holds_upright() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [-0.3, 0.0, 0.0];
        state.motor = crate::euclidean_pga::motor_from_pose(503.0, 500.0, 0.0, 0.0, 0.0, 0.0);
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        let hp = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            0.3,
            v_cheby,
            0.05,
            0.0,
        );
        let out = terminal_settle_aim(
            hp,
            -1.0,
            0.0,
            0.5,
            0.0,
            -0.3,
            0.0,
            0.3,
            3.0,
            v_cheby,
            0.3,
            state.params.mass,
            state.params.max_thrust,
            LateralThrMode::VerticalNeutral,
            false,
            TerminalSettlePhase::Trim,
            0.0,
            1.0 / 120.0,
            1.0,
            0.02,
            careful_aggression(CAREFUL_NEAR_M),
            [0.0, 1.0, 0.0],
        );
        assert_eq!(out.phase, TerminalSettlePhase::Trim);
        assert!(
            out.desired_raw[0].abs() + out.desired_raw[2].abs() < 1e-6,
            "deadzone trim must hold upright, aim={:?}",
            out.desired_raw
        );
    }

    #[test]
    fn terminal_trim_allows_small_position_lean() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [-0.2, 0.0, 0.0];
        // Outside deadzone so trim must nudge toward the pad.
        state.motor = crate::euclidean_pga::motor_from_pose(512.0, 500.0, 0.0, 0.0, 0.0, 0.0);
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        assert!(v_cheby > 0.0, "test setup must close on pad, v_cheby={v_cheby}");
        let hp = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            0.2,
            v_cheby,
            0.08,
            0.0,
        );
        let out = terminal_settle_aim(
            hp,
            -1.0,
            0.0,
            1.5,
            0.0,
            -0.2,
            0.0,
            0.2,
            12.0,
            v_cheby,
            0.2,
            state.params.mass,
            state.params.max_thrust,
            LateralThrMode::VerticalNeutral,
            false,
            TerminalSettlePhase::Trim,
            0.0,
            1.0 / 120.0,
            1.0,
            0.02,
            careful_aggression(80.0),
            [0.0, 1.0, 0.0],
        );
        assert_eq!(out.phase, TerminalSettlePhase::Trim);
        assert!(
            out.desired_raw[0].abs() > 0.005,
            "trim should nudge toward pad, aim={:?}",
            out.desired_raw
        );
        let agg = careful_aggression(80.0);
        assert!(
            out.lean_max <= careful(TRIM_LEAN_CAP_BASE, agg) + 1e-6,
            "trim lean must stay capped, lean={}",
            out.lean_max
        );
        assert!(
            out.lean_max < careful(TRIM_LEAN_STRICT_BASE, agg) + 0.012,
            "low-vh trim must stay near strict floor, lean={}",
            out.lean_max
        );
    }

    #[test]
    fn terminal_trim_lean_freedom_opens_at_speed() {
        let agg = careful_aggression(80.0);
        let cheby = 30.0;
        let omega = 0.02;
        let (_, lean_slow) = terminal_trim_aim(
            -1.0,
            0.0,
            -0.5,
            0.0,
            4.0,
            cheby,
            4.0,
            omega,
            settle_lean_freedom(4.0),
            agg,
        );
        let (_, lean_fast) = terminal_trim_aim(
            -1.0,
            0.0,
            -30.0,
            0.0,
            55.0,
            cheby,
            55.0,
            omega,
            settle_lean_freedom(55.0),
            agg,
        );
        assert!(
            lean_slow < lean_fast,
            "trim lean must open with vh: slow={lean_slow} fast={lean_fast}"
        );
    }

    #[test]
    fn terminal_brake_freedom_opens_at_speed() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.motor = crate::euclidean_pga::motor_from_pose(530.0, 500.0, 0.0, 0.0, 0.0, 0.0);
        let pos = state.position();
        let v_cheby = chebyshev_closing_rate(pos, [500.0, 0.0], state.velocity);
        let hp = HandoffSettlePlan::evaluate(
            &state,
            pos,
            [500.0, 0.0],
            12.0,
            v_cheby,
            0.5,
            0.0,
        );
        let agg = careful_aggression(80.0);
        state.velocity = [-2.5, 0.0, 0.0];
        let (_, lean_slow) = terminal_brake_aim(
            -1.0,
            0.0,
            -2.5,
            0.0,
            2.5,
            v_cheby,
            2.5,
            30.0,
            hp,
            2.0,
            0.0,
            state.params.mass,
            state.params.max_thrust,
            LateralThrMode::VerticalNeutral,
            0.85,
            settle_lean_freedom(2.5),
            agg,
            [0.0, 1.0, 0.0],
        );
        state.velocity = [-12.0, 0.0, 0.0];
        let (_, lean_fast) = terminal_brake_aim(
            -1.0,
            0.0,
            -55.0,
            0.0,
            55.0,
            v_cheby,
            -40.0,
            30.0,
            hp,
            2.0,
            0.0,
            state.params.mass,
            state.params.max_thrust,
            LateralThrMode::VerticalNeutral,
            0.85,
            settle_lean_freedom(55.0),
            agg,
            [0.0, 1.0, 0.0],
        );
        assert!(
            lean_slow < lean_fast,
            "brake lean must open with vh: slow={lean_slow} fast={lean_fast}"
        );
        assert!(
            lean_fast > 0.35,
            "fast terminal brake should retain decel authority, lean={lean_fast}"
        );
    }

    #[test]
    fn terminal_trim_creep_slower_than_handoff_speed() {
        let agg_near = careful_aggression(CAREFUL_NEAR_M);
        let agg_mid = careful_aggression(80.0);
        let v8 = trim_creep_speed(8.0, agg_near);
        let v30 = trim_creep_speed(30.0, agg_mid);
        // Closing-branch arm bound is HANDOFF_DRIFT_CLOSING_M / t_drift;
        // t_drift ≈ 11.5 s at the ~650 m hand-off altitude band. Creep in the
        // 6–10 m band must sit under it so Descend arms while still closing.
        assert!(
            v8 < HANDOFF_DRIFT_CLOSING_M / 11.5,
            "near-pad creep above closing hand-off gate: {v8}"
        );
        // Hand-off vh gates apply inside the 10 m box; at 30 m the creep may
        // ride modestly above VH_HANDOFF_MAX and bleed off on the way in.
        assert!(v30 < VH_HANDOFF_MAX * 1.25, "careful-range creep too fast: {v30}");
        // Closing-branch hand-off gate must be satisfiable while creeping in
        // the 6–10 m band (see `handoff_ready`), or arming stalls at the rim.
        assert!(v8 > 0.12, "near-pad creep below hand-off closing gate: {v8}");
        assert!(
            trim_creep_speed(8.0, agg_near) < trim_creep_speed(8.0, agg_mid),
            "closer range must creep slower at same cheby"
        );
        assert_eq!(CAREFUL_RANGE_M, 100.0);
        assert!((careful_aggression(CAREFUL_NEAR_M) - CAREFUL_AGGRESSION_MIN).abs() < 1e-9);
    }

    #[test]
    fn terminal_latch_hysteresis() {
        assert!(!careful_terminal_latch(false, 120.0, 50.0, true, TERMINAL_EXIT_CHEBY_M));
        assert!(careful_terminal_latch(false, 85.0, 50.0, true, TERMINAL_EXIT_CHEBY_M));
        assert!(careful_terminal_latch(true, 120.0, 50.0, true, TERMINAL_EXIT_CHEBY_M));
        assert!(!careful_terminal_latch(true, 150.0, 60.0, true, TERMINAL_EXIT_CHEBY_M));
        assert!(careful_terminal_latch(true, 150.0, 30.0, true, TERMINAL_EXIT_CHEBY_M));
    }

    #[test]
    fn terminal_trim_returns_to_upright_when_tilted() {
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Trim,
            0.0,
            1.0 / 120.0,
            0.90,
            0.05,
            0.0,
            false,
            0.5,
            1.0,
            0.5,
            0.0,
            VH_HANDOFF_MAX * 1.35,
        );
        assert_eq!(phase.0, TerminalSettlePhase::Upright);
    }

    #[test]
    fn airplane_brake_plan_uses_full_throttle_lateral_accel() {
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M);
        state.contacting = false;
        state.velocity = [60.0, 0.0, 0.0];
        let mass = state.params.mass;
        let max_thrust = state.params.max_thrust;
        let plan = HorizontalBrakePlan::evaluate(
            &state,
            mass,
            max_thrust,
            1.0,
            0.0,
            60.0,
            60.0,
            true,
            0.0,
        );
        let a_full = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::FullThrottle,
            mass,
            max_thrust,
        );
        assert!(
            (plan.a_prop - a_full).abs() < 1e-9,
            "airplane stop plan must use full-T lateral accel"
        );
    }

    #[test]
    fn mpc_rollout_loft_go_climbs_from_pad() {
        let state = RocketState::resting_on_pad();
        let pos = state.position();
        let init = predictor_init(&state, pos);
        let params = CandidateParams {
            aim: [0.3, 1.0, 0.0],
            lean_max: LEAN_BURN_MAX,
            thr: THR_FULL,
            mode: LateralThrMode::FullThrottle,
            coast: false,
            deep: false,
            force_full_thr: true,
        };
        let m = transit_rollout(
            init,
            params,
            [500.0, 0.0],
            1.0,
            0.0,
            state.params.mass,
            state.params.max_thrust,
            state.params.air_drag_k,
            6.0,
            500.0,
        );
        assert!(
            m.max_alt > pos[1] + 40.0,
            "LoftGo rollout should gain altitude, max_alt={}",
            m.max_alt
        );
    }

    #[test]
    fn mpc_selects_loft_when_below_gate() {
        let state = RocketState::resting_on_pad();
        let pos = state.position();
        let target = [500.0, 0.0];
        let dx = target[0] - pos[0];
        let dz = target[1] - pos[2];
        let range = (dx * dx + dz * dz).sqrt();
        let ux = dx / range;
        let uz = dz / range;
        let (_, cand, _) = transit_mpc_select(
            &state,
            pos,
            target,
            ux,
            uz,
            0.0,
            0.0,
            0.0,
            range,
            CRUISE_ALT_CAP,
            state.params.mass * GRAVITY / state.params.max_thrust,
            0.0,
            false,
            false,
            false,
            false,
            false,
            TransitCandidate::CruiseGo,
            MPC_REPLAN_EVERY,
        );
        assert_eq!(
            cand,
            TransitCandidate::LoftGo,
            "below altitude gate MPC should prefer loft"
        );
    }

    #[test]
    fn mpc_brake_only_when_brake_latched() {
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M);
        state.contacting = false;
        state.velocity = [70.0, 0.0, 0.0];
        let pos = state.position();
        let (_, cand, _) = transit_mpc_select(
            &state,
            pos,
            [6000.0, 0.0],
            1.0,
            0.0,
            70.0,
            0.0,
            70.0,
            6000.0,
            LONG_CRUISE_ALT_M,
            state.params.mass * GRAVITY / state.params.max_thrust,
            0.0,
            true,
            true,
            false,
            true,
            true,
            TransitCandidate::AirplaneHold,
            MPC_REPLAN_EVERY,
        );
        assert_eq!(cand, TransitCandidate::Brake);
    }

    #[test]
    fn vertical_neutral_out_brakes_full_throttle_at_long_range_lean() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let beta = 0.0;
        let a_neutral = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        let a_full = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::FullThrottle,
            mass,
            max_thrust,
        );
        let d_neutral = predicted_stop_distance(50.0, VH_HANDOFF_MAX, a_neutral, beta, 0.5);
        let d_full = predicted_stop_distance(50.0, VH_HANDOFF_MAX, a_full, beta, 0.5);
        assert!(
            d_neutral < d_full,
            "at LEAN_LONG_MAX, vertical-neutral tan(θ) exceeds full-T sin(θ): neutral={d_neutral} full={d_full}"
        );
    }

    #[test]
    fn moon_vacuum_brake_uses_full_lean_on_first_frame() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.moon_mode = true;
        // Fast close inside predicted stop distance (vacuum, no drag cushion).
        state.velocity = [55.0, 0.0, 0.0];
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let target = [200.0, 0.0];
        let cmd = spool_autopilot(&mut ap, &state, target, 0.92, 80);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        assert!(
            cmd.pitch.abs() + cmd.yaw.abs() > LEAN_BRAKE_MAX * 0.55,
            "expected near-max brake lean on engage, pitch={} yaw={}",
            cmd.pitch,
            cmd.yaw
        );
        assert!(
            cmd.throttle > 0.92,
            "moon vacuum brake must use full-T, thr={}",
            cmd.throttle
        );
    }

    #[test]
    fn brake_latch_engages_with_engage_margin() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let a = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        let d_stop = predicted_stop_distance(40.0, VH_HANDOFF_MAX, a, 0.0, 0.5);
        let range_eff = d_stop + BRAKE_ENGAGE_MARGIN_M * 0.5;
        assert!(
            update_brake_latch(false, false, range_eff, d_stop, 40.0),
            "engage margin should latch before nominal d_stop"
        );
        let range_outside = d_stop + BRAKE_ENGAGE_MARGIN_M * 1.5;
        assert!(
            !update_brake_latch(false, false, range_outside, d_stop, 40.0),
            "outside engage margin should stay in go"
        );
    }

    #[test]
    fn low_speed_brake_settles_without_full_throttle() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.moon_mode = true;
        // Mild overshoot at low vh latches brake; hardness must fade (no full-T).
        state.velocity = [-3.0, 0.0, 0.0];
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        let cmd = ap.update(&state, [80.0, 0.0], 1.0 / 120.0);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        let h = cruise_brake_hardness(3.0, -3.0, VH_BRAKE_SOFT, VH_BRAKE_HARD);
        assert!(h < 0.55, "expected soft hardness, got {h}");
        assert!(
            cmd.throttle < 0.85,
            "low-vh brake should leave full-T, thr={}",
            cmd.throttle
        );
        assert!(
            cmd.pitch.abs() + cmd.yaw.abs() < 0.55,
            "low-vh brake lean should be modest, pitch={} yaw={}",
            cmd.pitch,
            cmd.yaw
        );
    }

}
