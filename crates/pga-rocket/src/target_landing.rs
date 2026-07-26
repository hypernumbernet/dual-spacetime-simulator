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
//! the terminal-settle envelope (enter ~140 m, exit ~200 m): sequenced
//! Brake | Align drives lean and throttle while sinking toward
//! [`HANDOFF_ALT_M`] (~300 m). Altitude-scaled AND gates (position / attitude;
//! strict ≤150 m, relaxed ≥600 m via [`handoff_envelope`]) arm
//! [`TargetPhase::Descend`] via [`LandingAutopilot::update_target_descend`]
//! (closed-loop suicide burn). Above [`h_freefall_m`] (Earth 6000 m / Moon 10000 m),
//! transit flies a nose-down **dive** (full-T acceleration toward ground / pad)
//! under the speed envelope; lateral steering when `range > alt + 1000 m`, otherwise pure
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
    ramp_down, settle_aim_blend, settle_freedom_effective,
    settle_lean_auth, settle_lean_freedom, settle_motion_scale, settle_trim_rate_gate,
    settle_urgency, slew_throttle,
    CruiseThrottleFuzzy, FreefallThrottleFuzzy,
    CAREFUL_AGGRESSION_MAX, CAREFUL_NEAR_M, CAREFUL_RANGE_M, CAREFUL_TERMINAL_ENTER_M,
    LONG_CRUISE_ALT_M,
};
use crate::landing::{
    axis_angle_from_cross, chebyshev_xz, clamp_tilt, high_alt_dive_throttle_gate,
    high_alt_freefall_desired_aim, h_freefall_m, on_pad_square, saturate, LandingAutopilot,
    HIGH_ALT_OVERHEAD_BIAS_M,
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
/// Altitude (m) at the low end of the hand-off envelope — strict pad values.
const HANDOFF_ENV_ALT_LO_M: f64 = 150.0;
/// Altitude (m) at the high end — full relaxation for high-altitude arm.
const HANDOFF_ENV_ALT_HI_M: f64 = 600.0;
/// High-altitude ceiling for Chebyshev arm gate (m).
const HANDOFF_CHEBY_MAX_HI_M: f64 = 20.0;
/// High-altitude ceiling for horizontal speed arm gate (m/s).
const VH_HANDOFF_MAX_HI: f64 = 7.0;
/// High-altitude ceiling for pitch/yaw rate arm gate (rad/s).
const OMEGA_HANDOFF_MAX_HI: f64 = 0.20;
/// High-altitude floor for body-up · world-up arm gate (~0.46 rad tilt).
const COS_TILT_HANDOFF_HI: f64 = 0.90;
/// High-altitude drift budget (m) for the near-pad arm branch.
const HANDOFF_DRIFT_NEAR_HI_M: f64 = 16.0;
/// High-altitude drift budget (m) for the closing arm branch.
const HANDOFF_DRIFT_CLOSING_HI_M: f64 = 20.0;
/// High-altitude predicted-miss ceiling (m) for the closing arm branch.
const HANDOFF_MISS_MAX_HI_M: f64 = 12.0;
/// Target altitude (m) during terminal settle — sink while trimming position/attitude.
pub const HANDOFF_ALT_M: f64 = 300.0;
/// Near-handoff loft floor (m) — stay in Cruise while settling over the pad.
const HANDOFF_ALT_MIN_M: f64 = 260.0;
/// Chebyshev (m) within which near-handoff altitude gate applies.
const NEAR_HANDOFF_CHEBY_M: f64 = HANDOFF_CHEBY_MAX_M + 20.0;
/// Chebyshev (m) beyond which terminal latch may release on range exit.
const TERMINAL_EXIT_CHEBY_M: f64 = HANDOFF_CHEBY_MAX_M + 35.0;

/// Altitude-scaled Descend arm thresholds — strict at low altitude, relaxed high up
/// so the terminal lander has more trim budget during the suicide burn.
#[derive(Clone, Copy, Debug, PartialEq)]
struct HandoffEnvelope {
    cheby_max: f64,
    vh_max: f64,
    omega_max: f64,
    cos_tilt_min: f64,
    drift_near_m: f64,
    drift_closing_m: f64,
    miss_max_m: f64,
}

/// Linear hand-off envelope vs CoM altitude (150 m strict → 600 m relaxed).
#[inline]
fn handoff_envelope(alt: f64) -> HandoffEnvelope {
    let u = ramp(alt, HANDOFF_ENV_ALT_LO_M, HANDOFF_ENV_ALT_HI_M);
    HandoffEnvelope {
        cheby_max: HANDOFF_CHEBY_MAX_M + u * (HANDOFF_CHEBY_MAX_HI_M - HANDOFF_CHEBY_MAX_M),
        vh_max: VH_HANDOFF_MAX + u * (VH_HANDOFF_MAX_HI - VH_HANDOFF_MAX),
        omega_max: OMEGA_HANDOFF_MAX + u * (OMEGA_HANDOFF_MAX_HI - OMEGA_HANDOFF_MAX),
        cos_tilt_min: COS_TILT_HANDOFF + u * (COS_TILT_HANDOFF_HI - COS_TILT_HANDOFF),
        drift_near_m: HANDOFF_DRIFT_NEAR_M + u * (HANDOFF_DRIFT_NEAR_HI_M - HANDOFF_DRIFT_NEAR_M),
        drift_closing_m: HANDOFF_DRIFT_CLOSING_M
            + u * (HANDOFF_DRIFT_CLOSING_HI_M - HANDOFF_DRIFT_CLOSING_M),
        miss_max_m: HANDOFF_MISS_MAX_M + u * (HANDOFF_MISS_MAX_HI_M - HANDOFF_MISS_MAX_M),
    }
}

// --- Terminal settle (Brake | Align) -----------------------------------------
/// Attitude recovery horizon (s) for constraint ramp — no fixed upright wait.
const T_ATT_STRICT_S: f64 = 0.8;
/// Inside this Chebyshev (m) with quiet vh, Align holds upright (no chase).
const ALIGN_DEADZONE_CHEBY_M: f64 = HANDOFF_CHEBY_MAX_M * 0.60;
// Unscaled bases; runtime values multiply by distance-dependent aggression.
const TRIM_V_CREEP_PER_M_BASE: f64 = 0.32;
const TRIM_V_CREEP_MAX_BASE: f64 = 6.90;
const TRIM_V_CREEP_MIN_BASE: f64 = 1.5;
/// Pad-near creep cap: intercept + slope·cheby, capped (keeps under hand-off vh gate).
const CREEP_CAP_PAD_INTERCEPT: f64 = 0.45;
const CREEP_CAP_PAD_SLOPE: f64 = 0.12;
const CREEP_CAP_PAD_MAX: f64 = 2.60;
/// Outer creep cap shoulder beyond the pad-near blend (10–50 m Chebyshev).
const CREEP_CAP_OUTER_BASE: f64 = 4.50;
const CREEP_CAP_OUTER_RAMP: f64 = 2.00;

// --- Transit lean / envelope -------------------------------------------------
/// Lean cap during the full-throttle ascent burn (rad) — MPC rollout / legacy label.
const LEAN_BURN_MAX: f64 = 0.30;
/// Altitude (m) before opening lateral lean — stay upright through liftoff.
const CLIMB_CLEAR_ALT_M: f64 = 25.0;
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
/// Fraction of go-side lateral accel credited during attitude flip coast.
const FLIP_COAST_ACCEL_FRAC: f64 = 0.5;
/// Moon vacuum stop-distance pessimism (no drag cushion).
const MOON_DSTOP_SAFETY_FACTOR: f64 = 1.15;
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
    if matches!(terminal_phase, Some(TerminalSettlePhase::Align)) {
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

/// Limit brake lean so full-T reverse thrust still satisfies altitude-hold pitch.
///
/// Matches [`long_range_hold_cos`] / AirplaneHold: caps tilt so vertical thrust
/// does not fall below the cruise altitude schedule while braking.
#[inline]
fn apply_cruise_alt_lean_cap(
    mut cmd: CruiseBrakeCommand,
    alt: f64,
    alt_hold: f64,
    vy: f64,
    hover: f64,
) -> CruiseBrakeCommand {
    let cos_floor = long_range_hold_cos(alt, alt_hold, vy, hover);
    if cmd.lean_cap.cos() >= cos_floor {
        return cmd;
    }
    cmd.lean_cap = cos_floor.acos();
    cmd.aim = clamp_tilt(cmd.aim, cmd.lean_cap);
    cmd
}

/// Latched cruise reverse-brake: anti-v aim + altitude-hold lean cap + MPC plan fields.
#[inline]
fn latched_cruise_brake_plan(
    vx: f64,
    vz: f64,
    vh: f64,
    v_approach: f64,
    aim_filtered: [f64; 3],
    alt: f64,
    alt_hold: f64,
    vy: f64,
    hover: f64,
    in_airplane_range: bool,
    moon_mode: bool,
) -> (CruiseBrakeCommand, TransitMpcPlan) {
    let cmd = apply_cruise_alt_lean_cap(
        cruise_brake_command(vx, vz, vh, v_approach, aim_filtered),
        alt,
        alt_hold,
        vy,
        hover,
    );
    let plan = TransitMpcPlan {
        desired_raw: cmd.aim,
        lean_max: cmd.lean_cap,
        deep: cmd.hardness > 0.25,
        force_full_thr: brake_force_full_throttle(
            in_airplane_range,
            vh,
            moon_mode,
            cmd.hardness,
        ),
    };
    (cmd, plan)
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

/// Soft altitude floor for staying in Cruise near the pad.
///
/// Early terminal latch (armed at hundreds of metres for settle *planning*) must
/// not alone drop Climb into Cruise at [`HANDOFF_ALT_MIN_M`] — that cuts the
/// full-T climb into hover-authority Cruise ("姿勢優先" with no propulsion).
/// Only the near-pad Chebyshev box (optionally with an already-latched settle)
/// softens the loft gate.
#[inline]
fn near_handoff_zone(terminal_latched: bool, cheby: f64) -> bool {
    cheby <= NEAR_HANDOFF_CHEBY_M
        || (terminal_latched && cheby <= TERMINAL_EXIT_CHEBY_M)
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

/// Continuous creep-speed cap vs Chebyshev offset (closer → slower).
#[inline]
fn cheby_creep_cap(cheby: f64) -> f64 {
    // Pad-near slope keeps creep just under the closing-branch hand-off vh
    // bound (`HANDOFF_DRIFT_CLOSING_M / t_drift`) so Descend arms on the fly.
    let pad_near =
        (CREEP_CAP_PAD_INTERCEPT + CREEP_CAP_PAD_SLOPE * cheby).min(CREEP_CAP_PAD_MAX);
    let outer =
        CREEP_CAP_OUTER_BASE + ramp(cheby, HANDOFF_CHEBY_MAX_M, 50.0) * CREEP_CAP_OUTER_RAMP;
    let mu_pad = ramp_down(cheby, 12.0, 22.0);
    mu_pad * pad_near + (1.0 - mu_pad) * outer
}

/// Horizontal creep speed (m/s) for Align position trim.
#[inline]
fn trim_creep_speed(cheby: f64, aggression: f64) -> f64 {
    let v = (TRIM_V_CREEP_PER_M_BASE * cheby)
        .clamp(TRIM_V_CREEP_MIN_BASE, TRIM_V_CREEP_MAX_BASE)
        .min(cheby_creep_cap(cheby));
    careful(v, aggression)
}

/// True when predicted stop distance says braking should begin — also arms terminal settle.
fn terminal_brake_engage(
    state: &RocketState,
    pos: [f64; 3],
    target_xz: [f64; 2],
    range: f64,
    lofted: bool,
) -> bool {
    if !lofted {
        return false;
    }
    let dx = target_xz[0] - pos[0];
    let dz = target_xz[1] - pos[2];
    let range_eff = (range - CAREFUL_NEAR_M).max(0.0);
    let inv_range = if range > 1e-3 { 1.0 / range } else { 0.0 };
    let ux = dx * inv_range;
    let uz = dz * inv_range;
    let vx = state.velocity[0];
    let vz = state.velocity[2];
    let vh = (vx * vx + vz * vz).sqrt();
    let v_approach = vx * ux + vz * uz;
    let in_airplane_range = range >= LONG_AIRPLANE_RANGE_M;
    let mass = state.params.mass;
    let max_thrust = state.params.max_thrust;
    let hover = mass * GRAVITY / max_thrust;
    let mu_long = long_range_weight(range);
    let alt_hold = if in_airplane_range {
        LONG_CRUISE_ALT_M
    } else {
        CRUISE_ALT_CAP + mu_long * (LONG_CRUISE_ALT_M - CRUISE_ALT_CAP)
    };
    let plan = HorizontalBrakePlan::evaluate(
        state,
        mass,
        max_thrust,
        ux,
        uz,
        vh,
        v_approach,
        in_airplane_range,
        0.0,
        pos[1],
        alt_hold,
        state.velocity[1],
        hover,
    );
    range_eff <= plan.d_stop + BRAKE_ENGAGE_MARGIN_M
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

/// Sub-phase within cruise terminal settle (careful envelope): brake | align.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
enum TerminalSettlePhase {
    /// Reverse lean to kill horizontal speed / overshoot.
    #[default]
    Brake,
    /// Continuous position + attitude alignment (constraint × freedom arbitration).
    Align,
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
    /// Terminal settle envelope latch (enter ~300 m / d_stop, exit ~400 m).
    terminal_latched: bool,
    /// True when latched *and* inside the fine-settle box (cheby ≤ [`RANGE_FAR_M`]):
    /// Brake|Align aim + settle throttle. Outer latch keeps mid-range go/brake.
    pad_settle_active: bool,
    /// Terminal-pad brake blend latch (separate from mid-range [`brake_latched`]).
    terminal_brake_latched: bool,
    /// Seconds the hand-off AND gates have been continuously satisfied.
    handoff_settle_s: f64,
    /// Terminal settle sub-phase (Brake / Align).
    terminal_settle_phase: TerminalSettlePhase,
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
            pad_settle_active: false,
            terminal_brake_latched: false,
            handoff_settle_s: 0.0,
            terminal_settle_phase: TerminalSettlePhase::Brake,
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
        self.pad_settle_active = false;
        self.handoff_settle_s = 0.0;
        self.terminal_settle_phase = TerminalSettlePhase::Brake;
    }

    fn reset_transit_latches(&mut self) {
        self.brake_latched = false;
        self.terminal_latched = false;
        self.pad_settle_active = false;
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

    /// Compact HUD / panel label. Cruise sub-regimes stay ≤14 chars so the
    /// narrow left dock (`Target (T)` row) and top HUD do not wrap awkwardly.
    pub fn status_label(&self) -> &'static str {
        if !self.enabled {
            "off"
        } else if self.complete {
            "complete"
        } else {
            match self.phase {
                TargetPhase::Climb => "climb+go",
                TargetPhase::Cruise => self.cruise_status_label(),
                TargetPhase::Descend => "descend",
            }
        }
    }

    /// Cruise soft-regime label (MPC hold / latches / terminal settle).
    #[inline]
    fn cruise_status_label(&self) -> &'static str {
        // Fine settle only — outer terminal latch still runs mid-range go/brake.
        if self.pad_settle_active {
            return match self.terminal_settle_phase {
                TerminalSettlePhase::Brake => "cruise/s-brake",
                TerminalSettlePhase::Align => "cruise/s-align",
            };
        }
        if self.brake_latched {
            return "cruise/brake";
        }
        match self.mpc_hold {
            TransitCandidate::AirplaneHold => "cruise/air",
            TransitCandidate::CruiseGo => "cruise/go",
            TransitCandidate::Brake => "cruise/brake",
            TransitCandidate::Coast => "cruise/coast",
            TransitCandidate::SinkGo => "cruise/sink",
            TransitCandidate::LoftGo => "cruise/loft",
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
            let cruise_lofted = transit_lofted(alt, vy, false);
            let brake_engage = terminal_brake_engage(state, pos, target_xz, range, cruise_lofted);
            self.terminal_latched = careful_terminal_latch(
                self.terminal_latched,
                range,
                cheby,
                cruise_lofted,
                TERMINAL_EXIT_CHEBY_M,
                brake_engage,
            );
            self.pad_settle_active =
                self.terminal_latched && cheby <= RANGE_FAR_M;
            if was_terminal && !self.terminal_latched {
                self.reset_terminal_settle();
                self.pad_settle_active = false;
            } else if !was_terminal && self.terminal_latched {
                let vh_entry = (state.velocity[0] * state.velocity[0]
                    + state.velocity[2] * state.velocity[2])
                    .sqrt();
                let v_cheby_entry =
                    chebyshev_closing_rate(pos, target_xz, state.velocity);
                self.terminal_brake_latched = false;
                self.handoff_settle_s = 0.0;
                self.terminal_settle_phase = initial_terminal_settle_phase(
                    v_cheby_entry,
                    vh_entry,
                    cheby,
                    CAREFUL_AGGRESSION_MAX,
                );
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
        let env = handoff_envelope(alt);
        let handoff_ready = self.phase == TargetPhase::Cruise
            && cheby <= env.cheby_max
            && vh <= env.vh_max
            && v_cheby_handoff > -0.25
            && ((cheby <= env.cheby_max * 0.60 && vh <= env.drift_near_m / t_drift)
                || (v_cheby_handoff > 0.12
                    && vh <= env.drift_closing_m / t_drift
                    && miss_pred <= env.miss_max_m))
            && om_pitch_yaw_sq <= env.omega_max * env.omega_max
            && world_up_in_body(&state.motor)[1] >= env.cos_tilt_min;
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
                let (mut cmd, brake, terminal_brake, settle_phase, mpc_hold, mpc_counter) =
                    transit_command(
                        state,
                        target_xz,
                        pos,
                        self.brake_latched,
                        self.terminal_latched,
                        self.terminal_brake_latched,
                        self.terminal_settle_phase,
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
/// while Brake | Align adjusts position and attitude in parallel.
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
    a_coast: f64,
    v_end: f64,
    t_flip_brake: f64,
    d_stop: f64,
}

/// Brake lean cap matching live cruise altitude-hold authority.
#[inline]
fn brake_plan_lean_cap(alt: f64, alt_hold: f64, vy: f64, hover: f64) -> f64 {
    long_range_hold_cos(alt, alt_hold, vy, hover).acos().min(LEAN_BRAKE_MAX)
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
        alt: f64,
        alt_hold: f64,
        vy: f64,
        hover: f64,
    ) -> Self {
        let beta = if state.moon_mode {
            0.0
        } else {
            effective_air_drag_beta(state)
        };
        let brake_mode = brake_lateral_mode(in_airplane_range, vh, state.moon_mode);
        let lean_cap = brake_plan_lean_cap(alt, alt_hold, vy, hover);
        let brake_lean = lean_cap;
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
        let go_lean_raw = if in_airplane_range {
            LEAN_LONG_MAX
        } else {
            LEAN_BRAKE_PLAN
        };
        let go_lean = go_lean_raw.min(lean_cap);
        let go_mode = if in_airplane_range || vh > VH_BRAKE_FULL_THR || state.moon_mode {
            LateralThrMode::FullThrottle
        } else {
            LateralThrMode::VerticalNeutral
        };
        let a_coast =
            FLIP_COAST_ACCEL_FRAC * lateral_accel_for_lean(go_lean, go_mode, mass, max_thrust);
        let mut d_stop = predicted_stop_distance(
            v_closing,
            v_end,
            a_prop,
            beta,
            t_flip_brake,
            a_coast,
        );
        if state.moon_mode {
            d_stop *= MOON_DSTOP_SAFETY_FACTOR;
        }
        Self {
            beta,
            a_prop,
            a_coast,
            v_end,
            t_flip_brake,
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
///
/// Flip coast includes residual forward thrust during the attitude change:
/// `d_flip = v·t_flip + ½·a_coast·t_flip²`.
#[inline]
fn predicted_stop_distance(
    v_approach: f64,
    v_end: f64,
    a_prop: f64,
    beta: f64,
    t_flip: f64,
    a_coast: f64,
) -> f64 {
    let v = v_approach.max(0.0);
    let t = t_flip.max(0.0);
    let d_flip = v * t + 0.5 * a_coast.max(0.0) * t * t;
    d_flip + horizontal_burn_distance(v, v_end, a_prop, beta)
}

/// Max approach speed (m/s) that still fits in `range_eff` before braking.
///
/// Monotone bisection (≤16 steps) so the result matches
/// `predicted_stop_distance ≤ range_eff` (flip coast + burn).
#[inline]
fn allowed_approach_speed(
    range_eff: f64,
    v_end: f64,
    a_prop: f64,
    beta: f64,
    t_flip: f64,
    a_coast: f64,
    engage_margin: f64,
) -> f64 {
    let range_budget = (range_eff - engage_margin).max(0.0);
    if range_budget <= 1e-6 {
        return v_end;
    }
    let mut hi = v_end + 1.0;
    while predicted_stop_distance(hi, v_end, a_prop, beta, t_flip, a_coast) < range_budget
        && hi < 900.0
    {
        hi *= 1.5;
    }
    let mut lo = v_end;
    for _ in 0..16 {
        let mid = 0.5 * (lo + hi);
        if predicted_stop_distance(mid, v_end, a_prop, beta, t_flip, a_coast) <= range_budget {
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
        let env = handoff_envelope(pos[1]);
        let t_att = predicted_attitude_handoff_time(up_y, omega_py, env.cos_tilt_min, env.omega_max);

        let a_lat = lateral_accel_for_lean(
            lean_cmd,
            LateralThrMode::VerticalNeutral,
            state.params.mass,
            state.params.max_thrust,
        );
        let t_vh = if vh <= env.vh_max {
            0.0
        } else {
            predicted_decel_time(vh, env.vh_max, a_lat, beta)
        };

        let cheby = chebyshev_xz(pos, target_xz);
        let t_pos = if cheby <= env.cheby_max {
            0.0
        } else {
            let delta = cheby - env.cheby_max;
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
            let horiz_sq = u[0] * u[0] + u[2] * u[2];
            if horiz_sq <= 1e-18 {
                [0.0, GRAVITY, 0.0]
            } else {
                let horiz = horiz_sq.sqrt();
                let horiz_c = horiz.min(0.999);
                let a_lat = GRAVITY * horiz_c / (1.0 - horiz_c * horiz_c).max(1e-12).sqrt();
                let scale = a_lat / horiz;
                [scale * u[0], GRAVITY, scale * u[2]]
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
    let env = handoff_envelope(st.pos[1]);
    let mut handoff_penalty = 0.0;
    if st.pos[1] < GATE_ALT_MIN {
        handoff_penalty += (GATE_ALT_MIN - st.pos[1]).powi(2);
    }
    if cheby_end > env.cheby_max {
        handoff_penalty += (cheby_end - env.cheby_max).powi(2);
    }
    if vh_end > env.vh_max {
        handoff_penalty += (vh_end - env.vh_max).powi(2);
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
        let vh_drift_max = env.drift_closing_m / t_drift;
        if vh_end > vh_drift_max {
            handoff_penalty += (vh_end - vh_drift_max).powi(2);
        }
        if miss_pred > env.miss_max_m {
            handoff_penalty += (miss_pred - env.miss_max_m).powi(2);
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
            let (cmd, plan) = latched_cruise_brake_plan(
                vx,
                vz,
                vh,
                v_approach,
                [0.0, 1.0, 0.0],
                alt,
                alt_hold,
                vy,
                hover,
                in_airplane_range,
                moon_mode,
            );
            let exec_mode = if cmd.hardness > 0.45 {
                brake_mode
            } else {
                LateralThrMode::VerticalNeutral
            };
            Some(CandidateParams {
                aim: cmd.aim,
                lean_max: cmd.lean_cap,
                thr: if plan.force_full_thr {
                    THR_FULL
                } else {
                    hover.clamp(0.55, 0.95)
                },
                mode: exec_mode,
                coast: false,
                deep: plan.deep,
                force_full_thr: plan.force_full_thr,
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
    v_approach: f64,
    v_allow: f64,
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
    let speed_limited = !brake_now && v_approach > v_allow + 0.25;
    let candidates: &[TransitCandidate] = if brake_now {
        &[TransitCandidate::Brake]
    } else if speed_limited {
        &[TransitCandidate::Coast, TransitCandidate::Brake]
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
fn predicted_attitude_handoff_time(
    up_y: f64,
    omega_py: f64,
    cos_tilt_min: f64,
    omega_max: f64,
) -> f64 {
    let mut t: f64 = 0.0;
    if up_y < cos_tilt_min {
        let theta = up_y.clamp(-1.0, 1.0).acos();
        let theta_handoff = cos_tilt_min.acos();
        let angle = (theta - theta_handoff).max(0.0);
        t = t.max(brake_flip_time(angle));
    }
    if omega_py > omega_max {
        let excess = omega_py - omega_max;
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
    /// Attitude constraint (0 = free to trim, 1 = upright priority).
    constraint: f64,
}

/// Attitude constraint for Align: upright priority when tilt/rate or `t_att` demand recovery.
#[inline]
fn settle_attitude_constraint(hp: HandoffSettlePlan, up_y: f64, omega_py: f64) -> f64 {
    let t_att_c = smoothstep01(hp.t_att / T_ATT_STRICT_S);
    let tilt_c = ramp_down(up_y, COS_TILT_HANDOFF - 0.08, COS_TILT_HANDOFF);
    let rate_c = ramp(omega_py, OMEGA_HANDOFF_MAX * 0.5, OMEGA_HANDOFF_MAX);
    t_att_c.max(tilt_c).max(rate_c).clamp(0.0, 1.0)
}

/// "Too fast for here" speed: creep target plus margin, so a hot arrival
/// brakes instead of sailing across the pad into the hand-off gate.
#[inline]
fn terminal_vh_hot(v_creep: f64) -> f64 {
    (2.0 * v_creep + 0.8).min(VH_HANDOFF_MAX * 1.35)
}

#[inline]
fn terminal_needs_brake(v_cheby: f64, vh: f64, vh_hot: f64, cheby: f64) -> bool {
    let delta_cheby = (cheby - HANDOFF_CHEBY_MAX_M).max(1.0);
    let a_stop_req = vh * vh / (2.0 * delta_cheby);
    let a_lat_avail = GRAVITY * LEAN_BRAKE_MAX.tan();
    vh > vh_hot || a_stop_req > a_lat_avail || v_cheby < -1.2
}

/// Advance terminal settle sub-phase (Brake | Align).
fn update_terminal_settle_phase(
    phase: TerminalSettlePhase,
    v_cheby: f64,
    vh: f64,
    cheby: f64,
    vh_hot: f64,
) -> TerminalSettlePhase {
    let needs_brake = terminal_needs_brake(v_cheby, vh, vh_hot, cheby);
    match phase {
        TerminalSettlePhase::Brake | TerminalSettlePhase::Align => {
            if needs_brake {
                TerminalSettlePhase::Brake
            } else {
                TerminalSettlePhase::Align
            }
        }
    }
}

/// Pick initial settle sub-phase on envelope entry (quiet → Align, hot → Brake).
#[inline]
fn initial_terminal_settle_phase(v_cheby: f64, vh: f64, cheby: f64, aggression: f64) -> TerminalSettlePhase {
    let vh_hot = terminal_vh_hot(trim_creep_speed(cheby, aggression));
    if terminal_needs_brake(v_cheby, vh, vh_hot, cheby) {
        TerminalSettlePhase::Brake
    } else {
        TerminalSettlePhase::Align
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
    _aggression: f64,
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

    // Demand shapes a_cmd magnitude; lean ceiling is physical LEAN_BRAKE_MAX only
    // (no careful_brake_lean_cap soft roof).
    let a_scale = careful(0.12 + 0.55 * demand_shaped, _aggression);
    let a_cmd = (a_lat.max(overshoot_boost) * a_scale)
        .max(careful(0.05, _aggression) * demand_shaped);
    let lean = lean_for_lateral_accel(
        a_cmd,
        brake_mode,
        mass,
        max_thrust,
        LEAN_BRAKE_MAX,
    );

    ([aim_x, AIM_Y_BIAS, aim_z], lean)
}

/// Align-phase aim: creep with physics lean (ALIGN_LEAN_* artificial caps removed).
fn terminal_align_aim(
    ux: f64,
    uz: f64,
    vx: f64,
    vz: f64,
    vh: f64,
    cheby: f64,
    v_creep: f64,
    v_cheby: f64,
    omega_py: f64,
    lean_auth: f64,
    aggression: f64,
    mass: f64,
    max_thrust: f64,
    brake_mode: LateralThrMode,
) -> ([f64; 3], f64) {
    if cheby <= ALIGN_DEADZONE_CHEBY_M
        && vh <= VH_HANDOFF_MAX * 0.85
        && v_cheby > -0.08
    {
        return ([0.0, 1.0, 0.0], 0.0);
    }

    let err_vx = ux * v_creep - vx;
    let err_vz = uz * v_creep - vz;

    let dist_scale = (cheby / CAREFUL_RANGE_M).clamp(0.10, 1.0);
    let rate_gate_base = (1.0 - (omega_py / OMEGA_HANDOFF_MAX).clamp(0.0, 1.0)).powi(2);
    let rate_gate = settle_trim_rate_gate(rate_gate_base, lean_auth.clamp(0.0, 1.0));

    let motion = settle_motion_scale(lean_auth.clamp(0.0, 1.0));
    let k_vel = careful(0.38 + 0.12 * dist_scale, aggression) * motion * rate_gate;
    let k_pos = careful(0.008 * dist_scale, aggression) * motion * rate_gate;
    let a_req_x = k_pos * ux * cheby + k_vel * err_vx;
    let a_req_z = k_pos * uz * cheby + k_vel * err_vz;
    let a_lat = (a_req_x * a_req_x + a_req_z * a_req_z).sqrt();
    let lean = lean_for_lateral_accel(a_lat, brake_mode, mass, max_thrust, LEAN_BRAKE_MAX);

    let aim_scale = careful(0.06 + 0.14 * dist_scale, aggression);
    let v_ref = vh.max(0.6);
    let pos_aim = [
        aim_scale * (ux * 0.30 + err_vx / v_ref),
        AIM_Y_BIAS,
        aim_scale * (uz * 0.30 + err_vz / v_ref),
    ];
    let upright = [0.0, 1.0, 0.0];
    let blended = blend_vec3(upright, pos_aim, settle_aim_blend(lean_auth.clamp(0.0, 1.0)));

    (blended, lean)
}

/// Terminal settle: Brake | Align with continuous constraint × freedom arbitration.
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
    up_y: f64,
    omega_py: f64,
    aggression: f64,
    aim_filtered: [f64; 3],
) -> TerminalSettleOutput {
    let v_creep = trim_creep_speed(cheby, aggression);
    let vh_hot = terminal_vh_hot(v_creep);
    let (brake_w, new_latch) =
        terminal_brake_blend(v_cheby, vh, v_approach, cheby, terminal_brake_latched, vh_hot);

    let phase = update_terminal_settle_phase(phase, v_cheby, vh, cheby, vh_hot);

    let constraint = settle_attitude_constraint(hp, up_y, omega_py);
    let freedom_eff = settle_freedom_effective(vh, hp.t_pos, hp.t_vh);
    let urgency = settle_urgency(hp.t_pos, hp.t_vh);
    let lean_auth = settle_lean_auth(freedom_eff, constraint, urgency);
    let freedom = settle_lean_freedom(vh);

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
        TerminalSettlePhase::Align => {
            terminal_align_aim(
                ux,
                uz,
                vx,
                vz,
                vh,
                cheby,
                v_creep,
                v_cheby,
                omega_py,
                lean_auth,
                aggression,
                mass,
                max_thrust,
                brake_mode,
            )
        }
    };

    TerminalSettleOutput {
        desired_raw,
        lean_max,
        terminal_brake_latch: new_latch,
        phase,
        constraint,
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
    constraint: f64,
    hover: f64,
    _hover_cmd: f64,
    up_y: f64,
    effort: f64,
    t_hold: f64,
    t_neutral: f64,
    t_motion: f64,
) -> f64 {
    match phase {
        TerminalSettlePhase::Align => {
            let t_align = hover * (0.96 + 0.03 * up_y.clamp(0.90, 1.0));
            let t_upright = hover * (0.92 + 0.06 * up_y.clamp(0.70, 1.0));
            let t_target = t_upright + (1.0 - constraint) * (t_align - t_upright);
            if quiet {
                t_hold.clamp(t_target * 0.97, t_target + 0.02)
            } else {
                let effort_boost = constraint * 0.08 * effort;
                (t_hold + effort_boost).clamp(t_target * 0.95, t_target + 0.03)
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
/// `range > alt + 1000 m`, otherwise pure `[0, −1, 0]`. Inside predicted stop distance,
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

    // Hard priority: steer toward pad when horizontal range exceeds altitude + overhead bias.
    let prioritize_lateral = range > alt + HIGH_ALT_OVERHEAD_BIAS_M;
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
    let d_stop = predicted_stop_distance(
        v_closing,
        VH_HANDOFF_MAX,
        a_prop,
        beta,
        t_flip,
        0.0,
    );

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
        // Same pitch-program ceiling at all ranges (~0.90 rad). Short-range
        // LEAN_CLIMB_MAX / long_range_weight floor removed; dive LEAN_LONG_MAX
        // stays cruise/airplane-only.
        let u = smoothstep01(ramp(alt, CLIMB_CLEAR_ALT_M, GATE_ALT_MIN));
        let lean_cap = LEAN_BURN_MAX + u * (0.90 - LEAN_BURN_MAX);
        let lean = u * lean_cap;
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
    let terminal = lofted && terminal_latched;
    // Inner pad box: full terminal settle; outer latch keeps mid-range brake until cheby ≤ 80 m.
    let fine_settle = terminal && cheby <= RANGE_FAR_M;
    let aggression = if fine_settle {
        CAREFUL_AGGRESSION_MAX
    } else {
        careful_aggression(range)
    };
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

    let range_eff = (range - CAREFUL_NEAR_M).max(0.0);
    let aim_prev = *aim_filtered;

    // Pitch-elevator altitude target — computed before stop-distance plan.
    let alt_hold = if in_airplane_range {
        LONG_CRUISE_ALT_M
    } else {
        CRUISE_ALT_CAP + mu_long * (LONG_CRUISE_ALT_M - CRUISE_ALT_CAP)
    };

    let plan = HorizontalBrakePlan::evaluate(
        state,
        mass,
        max_thrust,
        ux,
        uz,
        vh,
        v_approach,
        in_airplane_range,
        0.0, // future: wind dot approach axis
        pos[1],
        alt_hold,
        vy,
        hover,
    );
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
    let brake = if fine_settle {
        false
    } else {
        update_brake_latch(brake_latched, fine_settle, range_eff, plan.d_stop, v_approach)
    };

    let v_allow = if fine_settle {
        let cheby_eff = (cheby - CAREFUL_NEAR_M).max(0.0);
        allowed_approach_speed(
            cheby_eff,
            VH_HANDOFF_MAX,
            plan.a_prop,
            plan.beta,
            plan.t_flip_brake,
            plan.a_coast,
            BRAKE_ENGAGE_MARGIN_M,
        )
        .clamp(0.0, VH_HANDOFF_MAX)
    } else {
        let v = allowed_approach_speed(
            range_eff,
            plan.v_end,
            plan.a_prop,
            plan.beta,
            plan.t_flip_brake,
            plan.a_coast,
            BRAKE_ENGAGE_MARGIN_M,
        );
        // Ascent-burn downrange cap only — once lofted, cruise/airplane must
        // follow the stop-distance envelope. Applying V_CLIMB_H_MAX whenever
        // vy≳3 (pitch-elevator "ballistic") freezes long-range go at ~28 m/s
        // in perpetual Coast with no lateral lean.
        if ballistic && !lofted {
            v.min(V_CLIMB_H_MAX)
        } else {
            v
        }
    };

    let need_x = ux * v_allow - vx;
    let need_z = uz * v_allow - vz;

    // Aim regime: terminal settle (fixed) or MPC-selected transit.
    let mut terminal_settle_out: Option<TerminalSettleOutput> = None;
    let mut mpc_out_hold = mpc_hold;
    let mut mpc_out_counter = mpc_hold_counter;
    let mut brake_hardness = 0.0;
    let mut cruise_brake: Option<CruiseBrakeCommand> = None;
    let (desired_raw, lean_max, deep, force_full_thr, terminal_brake_out) =
        if fine_settle {
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
    } else if brake {
        let (cmd, mpc_plan) = latched_cruise_brake_plan(
            vx,
            vz,
            vh,
            v_approach,
            aim_prev,
            pos[1],
            alt_hold,
            vy,
            hover,
            in_airplane_range,
            state.moon_mode,
        );
        brake_hardness = cmd.hardness;
        cruise_brake = Some(cmd);
        mpc_out_hold = TransitCandidate::Brake;
        mpc_out_counter = mpc_hold_counter.saturating_add(1);
        (
            mpc_plan.desired_raw,
            mpc_plan.lean_max,
            mpc_plan.deep,
            mpc_plan.force_full_thr,
            terminal_brake_latched,
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
            v_approach,
            v_allow,
            range,
            alt_hold,
            hover,
            mu_long,
            in_airplane_range,
            lofted,
            ballistic,
            brake_latched,
            false,
            mpc_hold,
            mpc_hold_counter,
        );
        mpc_out_hold = out_hold;
        mpc_out_counter = out_counter;

        if !mpc_plan.force_full_thr {
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
    let slew_rate = aim_slew_rate(brake, brake_hardness, deep, fine_settle, terminal_phase);
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
        Some(TerminalSettlePhase::Align)
    ) || (brake && !fine_settle && brake_soft_from_h);
    let brake_aggressive_att = brake && !fine_settle && brake_agg_from_h;
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
        // Sink-to-handoff only inside fine settle — outer terminal latch must
        // not bleed altitude during mid-range go/brake.
        cruise_v_des_y(pos[1], vy, fine_settle)
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
    let t_settle = if fine_settle {
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
            settle.constraint,
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
        terminal: fine_settle,
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
    (cmd, brake, terminal_brake_out, out_phase, mpc_out_hold, mpc_out_counter)
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
    fn status_label_exposes_short_cruise_submodes() {
        let mut ap = TargetLandingAutopilot::default();
        assert_eq!(ap.status_label(), "off");
        ap.enabled = true;
        ap.phase = TargetPhase::Climb;
        assert_eq!(ap.status_label(), "climb+go");
        ap.phase = TargetPhase::Descend;
        assert_eq!(ap.status_label(), "descend");

        ap.phase = TargetPhase::Cruise;
        ap.mpc_hold = TransitCandidate::AirplaneHold;
        assert_eq!(ap.status_label(), "cruise/air");
        ap.mpc_hold = TransitCandidate::CruiseGo;
        assert_eq!(ap.status_label(), "cruise/go");
        ap.mpc_hold = TransitCandidate::Coast;
        assert_eq!(ap.status_label(), "cruise/coast");
        ap.mpc_hold = TransitCandidate::SinkGo;
        assert_eq!(ap.status_label(), "cruise/sink");
        ap.mpc_hold = TransitCandidate::LoftGo;
        assert_eq!(ap.status_label(), "cruise/loft");

        ap.brake_latched = true;
        assert_eq!(ap.status_label(), "cruise/brake");
        ap.brake_latched = false;
        ap.mpc_hold = TransitCandidate::CruiseGo;
        ap.terminal_latched = true;
        // Outer latch alone keeps mid-range labels.
        assert_eq!(ap.status_label(), "cruise/go");
        ap.pad_settle_active = true;
        ap.terminal_settle_phase = TerminalSettlePhase::Brake;
        assert_eq!(ap.status_label(), "cruise/s-brake");
        ap.terminal_settle_phase = TerminalSettlePhase::Align;
        assert_eq!(ap.status_label(), "cruise/s-align");

        for label in [
            "cruise/air",
            "cruise/go",
            "cruise/brake",
            "cruise/coast",
            "cruise/sink",
            "cruise/loft",
            "cruise/s-brake",
            "cruise/s-align",
            "climb+go",
            "descend",
        ] {
            assert!(
                label.len() <= 14,
                "HUD label too wide for narrow panel: {label}"
            );
        }
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
        // Far from pad (range > alt + 1000 m), nose-down under envelope → full-T dive.
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
        // range=8000 > alt=6200 + 1000 → slant aim toward target (downward component).
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
        // range=500 m < alt=6200 + 1000 m → pure vertical dive.
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
    fn high_altitude_freefall_descend_priority_when_range_between_alt_and_bias() {
        // alt=6200, range=6500: alt < range <= alt + 1000 → pure vertical dive.
        let mut state = RocketState::at_altitude(FF_TEST_ALT);
        state.motor = crate::euclidean_pga::motor_from_pose(
            6500.0,
            FF_TEST_ALT,
            0.0,
            0.0,
            0.0,
            0.0,
        );
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
            "range between alt and alt+bias should aim nose-down, y={}",
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
    fn vertical_neutral_predictor_hovers() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let hover = mass * GRAVITY / max_thrust;
        let aim = clamp_tilt([1.0, 1.0, 0.0], 0.4);
        let a = predictor_thrust_accel(
            aim,
            0.4,
            hover,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        assert!(
            (a[1] - GRAVITY).abs() < 1e-9,
            "VerticalNeutral ay should cancel gravity, got {}",
            a[1]
        );
        let upright = predictor_thrust_accel(
            [0.0, 1.0, 0.0],
            0.05,
            hover,
            LateralThrMode::VerticalNeutral,
            mass,
            max_thrust,
        );
        assert!(
            (upright[1] - GRAVITY).abs() < 1e-9,
            "upright VerticalNeutral should hover, got {}",
            upright[1]
        );
    }

    #[test]
    fn cruise_brake_alt_lean_cap_limits_deep_lean_below_hold() {
        let mass = 1000.0;
        let max_thrust = mass * GRAVITY * 3.0;
        let hover = mass * GRAVITY / max_thrust;
        let alt = 400.0;
        let alt_hold = LONG_CRUISE_ALT_M;
        let raw = cruise_brake_command(-50.0, 0.0, 50.0, 50.0, [0.0, 1.0, 0.0]);
        assert!(
            raw.lean_cap > 1.0,
            "full hardness should want deep lean, got {}",
            raw.lean_cap
        );
        let capped = apply_cruise_alt_lean_cap(raw, alt, alt_hold, 0.0, hover);
        let cos_floor = long_range_hold_cos(alt, alt_hold, 0.0, hover);
        let len = (capped.aim[0] * capped.aim[0]
            + capped.aim[1] * capped.aim[1]
            + capped.aim[2] * capped.aim[2])
            .sqrt();
        let cos_aim = capped.aim[1] / len;
        assert!(
            cos_aim >= cos_floor - 1e-6,
            "brake aim must respect altitude hold floor, cos_aim={cos_aim} floor={cos_floor}"
        );
        assert!(
            capped.lean_cap <= cos_floor.acos() + 1e-6,
            "lean cap must shrink below hold band, cap={} floor_angle={}",
            capped.lean_cap,
            cos_floor.acos()
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
        let d_hi = predicted_stop_distance(50.0, VH_HANDOFF_MAX, a_hi, beta, 0.5, 0.0);
        let d_lo = predicted_stop_distance(50.0, VH_HANDOFF_MAX, a_lo, beta, 0.5, 0.0);
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
        let v = allowed_approach_speed(range, VH_HANDOFF_MAX, a, 0.0, t, 0.0, 0.0);
        let d = predicted_stop_distance(v, VH_HANDOFF_MAX, a, 0.0, t, 0.0);
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
        let v = allowed_approach_speed(range_eff, VH_HANDOFF_MAX, a, 0.0, t, 0.0, margin);
        let d = predicted_stop_distance(v, VH_HANDOFF_MAX, a, 0.0, t, 0.0);
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
        let v_near = allowed_approach_speed(200.0, VH_HANDOFF_MAX, a, 0.0, 0.5, 0.0, 0.0);
        let v_far = allowed_approach_speed(800.0, VH_HANDOFF_MAX, a, 0.0, 0.5, 0.0, 0.0);
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
    fn handoff_envelope_strict_at_low_altitude() {
        let env = handoff_envelope(100.0);
        assert_eq!(env.cheby_max, HANDOFF_CHEBY_MAX_M);
        assert_eq!(env.vh_max, VH_HANDOFF_MAX);
        assert_eq!(env.omega_max, OMEGA_HANDOFF_MAX);
        assert_eq!(env.cos_tilt_min, COS_TILT_HANDOFF);
        assert_eq!(env.drift_near_m, HANDOFF_DRIFT_NEAR_M);
        assert_eq!(env.drift_closing_m, HANDOFF_DRIFT_CLOSING_M);
        assert_eq!(env.miss_max_m, HANDOFF_MISS_MAX_M);
    }

    #[test]
    fn handoff_envelope_relaxed_at_high_altitude() {
        let env = handoff_envelope(800.0);
        assert_eq!(env.cheby_max, HANDOFF_CHEBY_MAX_HI_M);
        assert_eq!(env.vh_max, VH_HANDOFF_MAX_HI);
        assert_eq!(env.omega_max, OMEGA_HANDOFF_MAX_HI);
        assert_eq!(env.cos_tilt_min, COS_TILT_HANDOFF_HI);
        assert_eq!(env.drift_near_m, HANDOFF_DRIFT_NEAR_HI_M);
        assert_eq!(env.drift_closing_m, HANDOFF_DRIFT_CLOSING_HI_M);
        assert_eq!(env.miss_max_m, HANDOFF_MISS_MAX_HI_M);
    }

    #[test]
    fn handoff_envelope_monotonic_between_endpoints() {
        let lo = handoff_envelope(HANDOFF_ENV_ALT_LO_M);
        let mid = handoff_envelope(375.0);
        let hi = handoff_envelope(HANDOFF_ENV_ALT_HI_M);
        assert!(mid.cheby_max > lo.cheby_max && mid.cheby_max < hi.cheby_max);
        assert!(mid.vh_max > lo.vh_max && mid.vh_max < hi.vh_max);
        assert!(mid.omega_max > lo.omega_max && mid.omega_max < hi.omega_max);
        assert!(mid.cos_tilt_min < lo.cos_tilt_min && mid.cos_tilt_min > hi.cos_tilt_min);
        assert!(mid.drift_near_m > lo.drift_near_m && mid.drift_near_m < hi.drift_near_m);
        assert!(
            mid.drift_closing_m > lo.drift_closing_m && mid.drift_closing_m < hi.drift_closing_m
        );
        assert!(mid.miss_max_m > lo.miss_max_m && mid.miss_max_m < hi.miss_max_m);
    }

    #[test]
    fn handoff_envelope_allows_wider_gate_at_high_altitude() {
        let low = handoff_envelope(120.0);
        let high = handoff_envelope(650.0);
        assert!(high.cheby_max > low.cheby_max);
        assert!(high.vh_max > low.vh_max);
        // Example: 5 m/s and 15 m cheby pass at high alt but not at low alt.
        assert!(5.0 <= high.vh_max && 5.0 > low.vh_max);
        assert!(15.0 <= high.cheby_max && 15.0 > low.cheby_max);
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
        let mut state = RocketState::at_altitude(120.0);
        state.contacting = false;
        state.motor = crate::euclidean_pga::motor_from_pose(500.0, 120.0, 0.0, 0.35, 0.0, 0.0);
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
    fn terminal_settle_brake_releases_to_align_when_quiet() {
        let vh_hot = terminal_vh_hot(trim_creep_speed(50.0, CAREFUL_AGGRESSION_MAX));
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Brake,
            0.5,
            2.0,
            50.0,
            vh_hot,
        );
        assert_eq!(phase, TerminalSettlePhase::Align);
    }

    #[test]
    fn terminal_settle_brake_always_enters_align() {
        let vh_hot = terminal_vh_hot(trim_creep_speed(50.0, CAREFUL_AGGRESSION_MAX));
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Brake,
            0.5,
            2.0,
            50.0,
            vh_hot,
        );
        assert_eq!(phase, TerminalSettlePhase::Align);
    }

    #[test]
    fn terminal_settle_align_reenters_brake_when_hot() {
        let vh_hot = terminal_vh_hot(trim_creep_speed(50.0, CAREFUL_AGGRESSION_MAX));
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Align,
            0.5,
            VH_HANDOFF_MAX * 2.0,
            50.0,
            vh_hot,
        );
        assert_eq!(phase, TerminalSettlePhase::Brake);
    }

    #[test]
    fn initial_terminal_settle_phase_quiet_starts_align() {
        let phase = initial_terminal_settle_phase(0.5, 2.0, 50.0, CAREFUL_AGGRESSION_MAX);
        assert_eq!(phase, TerminalSettlePhase::Align);
    }

    #[test]
    fn initial_terminal_settle_phase_hot_starts_brake() {
        let phase = initial_terminal_settle_phase(0.5, VH_HANDOFF_MAX * 2.0, 50.0, CAREFUL_AGGRESSION_MAX);
        assert_eq!(phase, TerminalSettlePhase::Brake);
    }

    #[test]
    fn settle_attitude_constraint_high_when_tilted() {
        let hp = HandoffSettlePlan {
            t_att: 0.0,
            t_vh: 0.0,
            t_pos: 0.0,
            t_settle: 0.0,
        };
        let c = settle_attitude_constraint(hp, 0.88, 0.05);
        assert!(c > 0.5, "tilted rocket should constrain align, c={c}");
    }

    #[test]
    fn terminal_align_high_constraint_suppresses_lateral() {
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
            TerminalSettlePhase::Align,
            world_up_in_body(&state.motor)[1],
            0.08,
            careful_aggression(CAREFUL_NEAR_M),
            [0.0, 1.0, 0.0],
        );
        assert_eq!(out.phase, TerminalSettlePhase::Align);
        assert!(
            out.constraint > 0.25,
            "tilted align should be attitude-constrained, c={}",
            out.constraint
        );
    }

    #[test]
    fn terminal_align_deadzone_holds_upright() {
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
            TerminalSettlePhase::Align,
            1.0,
            0.02,
            careful_aggression(CAREFUL_NEAR_M),
            [0.0, 1.0, 0.0],
        );
        assert_eq!(out.phase, TerminalSettlePhase::Align);
        assert!(
            out.desired_raw[0].abs() + out.desired_raw[2].abs() < 1e-6,
            "deadzone align must hold upright, aim={:?}",
            out.desired_raw
        );
    }

    #[test]
    fn terminal_align_allows_small_position_lean() {
        let mut state = RocketState::at_altitude(500.0);
        state.contacting = false;
        state.velocity = [-0.2, 0.0, 0.0];
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
            TerminalSettlePhase::Align,
            1.0,
            0.02,
            careful_aggression(80.0),
            [0.0, 1.0, 0.0],
        );
        assert_eq!(out.phase, TerminalSettlePhase::Align);
        assert!(
            out.desired_raw[0].abs() > 0.005,
            "align should nudge toward pad, aim={:?}",
            out.desired_raw
        );
        assert!(
            out.lean_max > 0.0 && out.lean_max <= LEAN_BRAKE_MAX + 1e-9,
            "align physics lean in (0, LEAN_BRAKE_MAX], lean={}",
            out.lean_max
        );
    }

    #[test]
    fn terminal_align_lean_auth_opens_at_speed() {
        let state = RocketState::at_altitude(500.0);
        let agg = careful_aggression(80.0);
        let cheby = 30.0;
        let omega = 0.02;
        let auth_slow = settle_lean_auth(settle_freedom_effective(4.0, 0.0, 0.0), 0.0, 0.0);
        let auth_fast = settle_lean_auth(settle_freedom_effective(14.0, 0.0, 0.0), 0.0, 0.0);
        let v_creep = trim_creep_speed(cheby, agg);
        let (_, lean_slow) = terminal_align_aim(
            -1.0,
            0.0,
            -0.5,
            0.0,
            4.0,
            cheby,
            v_creep,
            4.0,
            omega,
            auth_slow,
            agg,
            state.params.mass,
            state.params.max_thrust,
            LateralThrMode::VerticalNeutral,
        );
        let (_, lean_fast) = terminal_align_aim(
            -1.0,
            0.0,
            -14.0,
            0.0,
            14.0,
            cheby,
            v_creep,
            14.0,
            omega,
            auth_fast,
            agg,
            state.params.mass,
            state.params.max_thrust,
            LateralThrMode::VerticalNeutral,
        );
        assert!(
            lean_slow <= lean_fast + 1e-9,
            "align lean must not shrink with speed: slow={lean_slow} fast={lean_fast}"
        );
        assert!(lean_fast > 0.0, "align should retain creep lean");
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
    fn terminal_align_creep_slower_than_handoff_speed() {
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
        assert!(!careful_terminal_latch(false, 350.0, 50.0, true, TERMINAL_EXIT_CHEBY_M, false));
        assert!(careful_terminal_latch(false, 280.0, 50.0, true, TERMINAL_EXIT_CHEBY_M, false));
        assert!(careful_terminal_latch(false, 500.0, 50.0, true, TERMINAL_EXIT_CHEBY_M, true));
        assert!(careful_terminal_latch(true, 380.0, 50.0, true, TERMINAL_EXIT_CHEBY_M, false));
        assert!(!careful_terminal_latch(true, 420.0, 60.0, true, TERMINAL_EXIT_CHEBY_M, false));
        assert!(careful_terminal_latch(true, 420.0, 30.0, true, TERMINAL_EXIT_CHEBY_M, false));
    }

    #[test]
    fn terminal_align_constraint_rises_when_tilted() {
        let hp = HandoffSettlePlan {
            t_att: 0.0,
            t_vh: 0.0,
            t_pos: 0.0,
            t_settle: 0.0,
        };
        let phase = update_terminal_settle_phase(
            TerminalSettlePhase::Align,
            0.5,
            1.0,
            50.0,
            terminal_vh_hot(trim_creep_speed(50.0, CAREFUL_AGGRESSION_MAX)),
        );
        assert_eq!(phase, TerminalSettlePhase::Align);
        assert!(
            settle_attitude_constraint(hp, 0.90, 0.05) > 0.3,
            "tilt should raise constraint without leaving Align"
        );
    }

    #[test]
    fn airplane_brake_plan_uses_altitude_capped_lateral_accel() {
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M);
        state.contacting = false;
        state.velocity = [60.0, 0.0, 0.0];
        let mass = state.params.mass;
        let max_thrust = state.params.max_thrust;
        let hover = mass * GRAVITY / max_thrust;
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
            LONG_CRUISE_ALT_M,
            LONG_CRUISE_ALT_M,
            0.0,
            hover,
        );
        let lean_cap = brake_plan_lean_cap(LONG_CRUISE_ALT_M, LONG_CRUISE_ALT_M, 0.0, hover);
        let a_capped = lateral_accel_for_lean(
            lean_cap,
            LateralThrMode::FullThrottle,
            mass,
            max_thrust,
        );
        assert!(
            (plan.a_prop - a_capped).abs() < 1e-9,
            "airplane stop plan must use altitude-hold lean cap"
        );
        let a_full = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::FullThrottle,
            mass,
            max_thrust,
        );
        assert!(
            plan.a_prop <= a_full + 1e-9,
            "capped lateral accel must not exceed full lean authority"
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
            0.0,
            50.0,
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
            70.0,
            50.0,
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
        let d_neutral = predicted_stop_distance(50.0, VH_HANDOFF_MAX, a_neutral, beta, 0.5, 0.0);
        let d_full = predicted_stop_distance(50.0, VH_HANDOFF_MAX, a_full, beta, 0.5, 0.0);
        assert!(
            d_neutral < d_full,
            "at LEAN_LONG_MAX, vertical-neutral tan(θ) exceeds full-T sin(θ): neutral={d_neutral} full={d_full}"
        );
    }

    #[test]
    fn predicted_stop_distance_includes_flip_coast_accel() {
        let v = 50.0;
        let t = 0.8;
        let a = 10.0;
        let a_coast = 6.0;
        let d_plain = predicted_stop_distance(v, VH_HANDOFF_MAX, a, 0.0, t, 0.0);
        let d_coast = predicted_stop_distance(v, VH_HANDOFF_MAX, a, 0.0, t, a_coast);
        let extra = 0.5 * a_coast * t * t;
        assert!(
            (d_coast - d_plain - extra).abs() < 1e-9,
            "flip coast should add 0.5*a*t^2: plain={d_plain} coast={d_coast} extra={extra}"
        );
    }

    #[test]
    fn altitude_capped_lean_increases_stop_distance() {
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M);
        state.contacting = false;
        state.velocity = [55.0, 0.0, 0.0];
        let mass = state.params.mass;
        let max_thrust = state.params.max_thrust;
        let hover = mass * GRAVITY / max_thrust;
        let plan = HorizontalBrakePlan::evaluate(
            &state,
            mass,
            max_thrust,
            1.0,
            0.0,
            55.0,
            55.0,
            true,
            0.0,
            LONG_CRUISE_ALT_M,
            LONG_CRUISE_ALT_M,
            0.0,
            hover,
        );
        let a_full = lateral_accel_for_lean(
            LEAN_BRAKE_MAX,
            LateralThrMode::FullThrottle,
            mass,
            max_thrust,
        );
        assert!(plan.a_prop <= a_full + 1e-9);
        let d_at_capped = predicted_stop_distance(
            55.0,
            VH_HANDOFF_MAX,
            plan.a_prop,
            plan.beta,
            plan.t_flip_brake,
            plan.a_coast,
        );
        assert!((plan.d_stop - d_at_capped).abs() < 1e-6);
        if plan.a_prop < a_full - 1e-9 {
            let d_at_full = predicted_stop_distance(
                55.0,
                VH_HANDOFF_MAX,
                a_full,
                0.0,
                plan.t_flip_brake,
                0.0,
            );
            assert!(
                d_at_capped > d_at_full,
                "capped lean must need longer stop distance"
            );
        }
    }

    #[test]
    fn mpc_speed_limited_prefers_coast_over_airplane() {
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M);
        state.contacting = false;
        state.velocity = [85.0, 0.0, 0.0];
        let pos = state.position();
        let hover = state.params.mass * GRAVITY / state.params.max_thrust;
        let (_, cand, _) = transit_mpc_select(
            &state,
            pos,
            [4000.0, 0.0],
            1.0,
            0.0,
            85.0,
            0.0,
            85.0,
            85.0,
            40.0,
            4000.0,
            LONG_CRUISE_ALT_M,
            hover,
            0.0,
            true,
            true,
            false,
            false,
            false,
            TransitCandidate::AirplaneHold,
            MPC_REPLAN_EVERY,
        );
        assert_eq!(
            cand,
            TransitCandidate::Coast,
            "above v_allow must not keep accelerating in airplane hold"
        );
    }

    #[test]
    fn lofted_airplane_not_capped_by_climb_vh_max() {
        // Pitch-elevator often leaves vy ≳ 3 ("ballistic") while lofted; that
        // must not clamp v_allow to V_CLIMB_H_MAX or long-range go freezes.
        let mut state = RocketState::at_altitude(LONG_CRUISE_ALT_M + 80.0);
        state.contacting = false;
        state.velocity = [20.0, 4.0, 0.0];
        let mut ap = TargetLandingAutopilot::default();
        ap.enabled = true;
        ap.phase = TargetPhase::Cruise;
        let target = [4000.0, 0.0];
        let cmd = spool_autopilot(&mut ap, &state, target, 0.92, 40);
        assert_eq!(ap.phase, TargetPhase::Cruise);
        assert!(
            matches!(ap.status_label(), "cruise/air" | "cruise/go"),
            "lofted long-range must accelerate, label={}",
            ap.status_label()
        );
        assert!(
            cmd.pitch.abs() + cmd.yaw.abs() > 0.15,
            "expected airplane lean, pitch={} yaw={} thr={}",
            cmd.pitch,
            cmd.yaw,
            cmd.throttle
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
        let d_stop = predicted_stop_distance(40.0, VH_HANDOFF_MAX, a, 0.0, 0.5, 0.0);
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
