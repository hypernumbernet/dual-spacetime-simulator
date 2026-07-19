//! Lightweight fuzzy / Takagi–Sugeno helpers for GNC arbitration.
//!
//! Physics schedules (suicide-burn envelope, √h soft touch, √-profile attitude)
//! stay closed-form elsewhere. This module only **blends** local laws and
//! continuous regime weights — never replaces safety latches or geometry.

/// Rising ramp membership: 0 for `x <= lo`, 1 for `x >= hi`, linear in between.
#[inline]
pub fn ramp(x: f64, lo: f64, hi: f64) -> f64 {
    if hi <= lo {
        return if x >= hi { 1.0 } else { 0.0 };
    }
    ((x - lo) / (hi - lo)).clamp(0.0, 1.0)
}

/// Falling ramp: 1 for `x <= hi`, 0 for `x >= lo` when `lo > hi` is swapped —
/// here `x <= lo` → 1, `x >= hi` → 0.
#[inline]
pub fn ramp_down(x: f64, lo: f64, hi: f64) -> f64 {
    1.0 - ramp(x, lo, hi)
}

/// Triangle membership peaking at `b` between feet `a` and `c` (`a < b < c`).
#[inline]
pub fn tri(x: f64, a: f64, b: f64, c: f64) -> f64 {
    if x <= a || x >= c {
        return 0.0;
    }
    if x < b {
        ramp(x, a, b)
    } else {
        ramp_down(x, b, c)
    }
}

/// Trapezoid: 0 outside `[a,d]`, 1 on `[b,c]`, linear shoulders (`a≤b≤c≤d`).
#[inline]
pub fn trap(x: f64, a: f64, b: f64, c: f64, d: f64) -> f64 {
    if x <= a || x >= d {
        return 0.0;
    }
    if x < b {
        ramp(x, a, b)
    } else if x <= c {
        1.0
    } else {
        ramp_down(x, c, d)
    }
}

/// Algebraic product AND (standard for TS weight).
#[inline]
pub fn and(a: f64, b: f64) -> f64 {
    (a * b).clamp(0.0, 1.0)
}

/// Probabilistic OR: `a + b - a*b`.
#[inline]
pub fn or(a: f64, b: f64) -> f64 {
    (a + b - a * b).clamp(0.0, 1.0)
}

/// Normalized weighted average (Takagi–Sugeno defuzzification).
/// Returns `default` if total weight is negligible.
#[inline]
pub fn defuzz_weighted(pairs: &[(f64, f64)], default: f64) -> f64 {
    let mut num = 0.0;
    let mut den = 0.0;
    for &(w, y) in pairs {
        let w = w.max(0.0);
        num += w * y;
        den += w;
    }
    if den < 1e-12 {
        default
    } else {
        num / den
    }
}

/// Soft maximum of positive channel commands with membership weights.
/// When all weights are ~0, returns 0. Prefer for OR-like throttle arbitration
/// that should not average a strong brake with a zero coast.
#[inline]
pub fn weighted_max(pairs: &[(f64, f64)]) -> f64 {
    let mut best: f64 = 0.0;
    for &(w, y) in pairs {
        if w > 1e-12 {
            best = best.max(y * w.min(1.0));
        }
    }
    // Also allow full channel when weight saturates (w≥1).
    for &(w, y) in pairs {
        if w >= 1.0 - 1e-9 {
            best = best.max(y);
        }
    }
    best
}

/// Vertical-channel fuzzy blend for the L-mode lander.
///
/// Local laws (`t_soft`, `t_support`, `t_brake`, `t_auth`, `t_drift`) are computed
/// by the caller. This only decides how to mix soft-terminal vs coast/bang and
/// applies a **hard brake floor** when late on the envelope so attitude never
/// gates a needed suicide burn.
#[derive(Clone, Copy, Debug)]
pub struct LandingThrottleFuzzy {
    pub h: f64,
    pub h_env: f64,
    pub h_need: f64,
    pub v_down: f64,
    pub up_y: f64,
    pub contacting: bool,
    /// Soft √h channel (already includes auth max if desired).
    pub t_soft: f64,
    pub t_support: f64,
    /// Bang-brake magnitude when fully engaged (0..1).
    pub t_brake_cmd: f64,
    pub t_auth: f64,
    pub t_drift: f64,
    /// Height below which soft terminal is preferred (m).
    pub h_terminal: f64,
    /// Height above which coast/suicide is preferred when not terminal (m).
    pub h_coast_enable: f64,
    /// Min up-component to count as soft-capable.
    pub upy_soft: f64,
    /// Min up-component for brake floor.
    pub upy_brake: f64,
    /// Min descent speed for brake floor (m/s).
    pub v_brake_min: f64,
}

impl LandingThrottleFuzzy {
    /// Membership-weighted vertical throttle in [0, 1].
    pub fn arbitrate(self) -> f64 {
        let LandingThrottleFuzzy {
            h,
            h_env,
            h_need,
            v_down,
            up_y,
            contacting,
            t_soft,
            t_support,
            t_brake_cmd,
            t_auth,
            t_drift,
            h_terminal,
            h_coast_enable,
            upy_soft,
            upy_brake,
            v_brake_min,
        } = self;

        // --- Regime gate (matches discrete soft_regime; sharp on purpose) ---
        // Soft blend *across* this gate was tried and regressed lateral hover
        // recovery (partial soft hover + lean → rocket-sled drift). Keep the
        // gate hard; apply fuzzy only to bang-brake *engagement* edges.
        //
        //   use_coast_burn = h_env >= H_COAST || h_need+1 >= h_env
        //   soft = upright && (contacting || h < H_TERMINAL || !use_coast_burn)
        let use_coast_burn =
            h_env >= h_coast_enable || h_need + 1.0 >= h_env;
        let soft_regime =
            up_y >= upy_soft && (contacting || h < h_terminal || !use_coast_burn);

        // Smooth bang engagement (replaces hard AND of three thresholds).
        let mu_can_brake = ramp(up_y, upy_brake - 0.06, upy_brake + 0.02);
        let mu_falling = ramp(v_down, v_brake_min - 0.5, v_brake_min + 0.3);
        let mu_on_curve = ramp(h_need + 0.75 - h_env, -1.0, 0.5);
        let mu_brake = and(and(mu_can_brake, mu_falling), mu_on_curve);
        let t_brake = mu_brake * t_brake_cmd;

        let mut throttle = if soft_regime {
            t_soft.max(t_auth)
        } else {
            t_support.max(t_brake).max(t_auth).max(t_drift)
        };

        // --- Hard safety floor: envelope never gated by attitude / soft path ---
        let hard_late = h_env <= h_need + 0.75 && v_down > v_brake_min && up_y >= upy_brake;
        if hard_late {
            throttle = throttle.max(t_brake_cmd);
        }

        throttle.clamp(0.0, 1.0)
    }
}

// --- T-Descend physics vertical (regime blend + actuator) ---------------------

/// Fuzzy blend for [`crate::landing::physics_pad_vertical_throttle`]: smooths
/// coast ↔ brake ↔ ground-cut transitions instead of hard `if` steps.
#[derive(Clone, Copy, Debug)]
pub struct PhysicsPadThrottleFuzzy {
    pub h_env: f64,
    pub h_need: f64,
    pub v_down: f64,
    pub vy: f64,
    pub vh: f64,
    pub contacting: bool,
    pub vh_touch: f64,
    pub up_y: f64,
    pub t_coast: f64,
    pub t_brake: f64,
    pub t_bang: f64,
    pub t_ground: f64,
    pub t_skid: f64,
    pub t_anti_loft: f64,
    pub t_drift_hold: f64,
    pub h_terminal: f64,
    pub coast_margin: f64,
    pub envelope_late: f64,
    pub v_brake_min: f64,
    pub upy_brake: f64,
}

impl PhysicsPadThrottleFuzzy {
    /// Membership-weighted vertical setpoint in [0, 1].
    pub fn arbitrate(self) -> f64 {
        let PhysicsPadThrottleFuzzy {
            h_env,
            h_need,
            v_down,
            vy,
            vh,
            contacting,
            vh_touch,
            up_y,
            t_coast,
            t_brake,
            t_bang,
            t_ground,
            t_skid,
            t_anti_loft,
            t_drift_hold,
            h_terminal,
            coast_margin,
            envelope_late,
            v_brake_min,
            upy_brake,
        } = self;

        // Coast vs closed-loop brake: shoulder on Δh above the envelope + descent rate.
        let dh = h_env - h_need;
        let mu_coast = and(
            ramp(dh, coast_margin - 4.0, coast_margin + 6.0),
            ramp(v_down, v_brake_min - 1.0, v_brake_min + 0.5),
        );
        let mu_brake = ramp_down(dh, coast_margin - 6.0, coast_margin + 4.0);
        let mut t = defuzz_weighted(
            &[(mu_coast, t_coast), (mu_brake, t_brake)],
            t_brake,
        );

        // Pad skid: contacting with residual horizontal speed.
        let mu_skid = if contacting {
            ramp(vh, vh_touch - 0.4, vh_touch + 1.2)
        } else {
            0.0
        };
        if mu_skid > 1e-6 {
            t = defuzz_weighted(&[(mu_skid, t_skid), (1.0 - mu_skid, t)], t);
        }

        // Ground support: weight on feet — taper thrust off (smooth shoulder on vh).
        let mu_ground = if contacting {
            ramp_down(vh, vh_touch + 0.6, vh_touch - 0.4)
        } else {
            0.0
        };
        if mu_ground > 1e-6 {
            t = defuzz_weighted(&[(mu_ground, t_ground), (1.0 - mu_ground, t)], t);
        }

        // Post-bounce anti-loft near the pad.
        let mu_loft = if h_env < h_terminal {
            ramp(vy, -0.02, 0.30)
        } else {
            0.0
        };
        if mu_loft > 1e-6 {
            t = defuzz_weighted(&[(mu_loft, t_anti_loft), (1.0 - mu_loft, t)], t);
        }

        // Feet just above pad, slow drift: hold thrust off until reground.
        let mu_drift = if !contacting && h_env < 1.2 && vh <= vh_touch {
            ramp_down(v_down, v_brake_min, 0.4)
        } else {
            0.0
        };
        if mu_drift > 1e-6 {
            t = defuzz_weighted(&[(mu_drift, t_drift_hold), (1.0 - mu_drift, t)], t);
        }

        // Hard safety floor when late on the envelope (same doctrine as L-mode fuzzy).
        let hard_late =
            h_env <= h_need + envelope_late && v_down > v_brake_min && up_y >= upy_brake;
        if hard_late {
            let mu_late = ramp(h_need + envelope_late + 1.5 - h_env, -2.0, 1.0);
            t = t.max(mu_late * t_bang + (1.0 - mu_late) * t.min(t_bang));
            t = t.max(t_bang);
        }

        t.clamp(0.0, 1.0)
    }
}

/// Asymmetric main-engine throttle slew (fraction per second).
///
/// Pump-fed stages spool up slower than they throttle down; this models the
/// commanded-throttle → delivered-thrust lag without changing the GNC setpoint law.
#[inline]
pub fn slew_throttle(current: f64, target: f64, dt: f64, spool_up: f64, spool_down: f64) -> f64 {
    let dt = dt.max(0.0);
    let target = target.clamp(0.0, 1.0);
    let current = current.clamp(0.0, 1.0);
    if target > current {
        (current + spool_up * dt).min(target)
    } else {
        (current - spool_down * dt).max(target)
    }
}

// --- Attitude gain scheduling (continuous pad / free field) -------------------

/// Multipliers on base attitude gains / √-profile parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeGainScale {
    pub kp: f64,
    pub kd: f64,
    pub kd_roll: f64,
    pub alpha: f64,
    pub omega_cap: f64,
}

impl AttitudeGainScale {
    pub const FREE: Self = Self {
        kp: 1.0,
        kd: 1.0,
        kd_roll: 1.0,
        alpha: 1.0,
        omega_cap: 1.0,
    };
    /// On-pad, low altitude (former `on_pad && h < 20`).
    pub const NEAR_PAD: Self = Self {
        kp: 1.2,
        kd: 1.25,
        kd_roll: 1.15,
        alpha: 1.25,
        omega_cap: 1.0,
    };
    /// Contacting on pad (former `pad_settle`).
    pub const SETTLE: Self = Self {
        kp: 1.5,
        kd: 1.55,
        kd_roll: 1.45,
        alpha: 1.8,
        omega_cap: 1.15,
    };
}

/// Continuous attitude gain scales from pad contact / height.
///
/// Replaces the three-way stepwise schedule with Takagi–Sugeno blending so
/// gains do not jump at the contact edge or the 20 m height notch.
pub fn attitude_gain_scales(contacting: bool, on_pad: bool, h: f64) -> AttitudeGainScale {
    // Settle owns the schedule when planted on the painted square.
    let w_settle = if contacting && on_pad { 1.0 } else { 0.0 };
    // Near-pad boost while over the square and not yet settled (fade 8→28 m).
    let w_near = if on_pad {
        ramp_down(h, 8.0, 28.0) * (1.0 - w_settle)
    } else {
        0.0
    };
    let w_free = (1.0 - w_settle - w_near).max(0.0);

    let blend = |a: f64, b: f64, c: f64| {
        defuzz_weighted(
            &[(w_settle, a), (w_near, b), (w_free, c)],
            c,
        )
    };
    AttitudeGainScale {
        kp: blend(
            AttitudeGainScale::SETTLE.kp,
            AttitudeGainScale::NEAR_PAD.kp,
            AttitudeGainScale::FREE.kp,
        ),
        kd: blend(
            AttitudeGainScale::SETTLE.kd,
            AttitudeGainScale::NEAR_PAD.kd,
            AttitudeGainScale::FREE.kd,
        ),
        kd_roll: blend(
            AttitudeGainScale::SETTLE.kd_roll,
            AttitudeGainScale::NEAR_PAD.kd_roll,
            AttitudeGainScale::FREE.kd_roll,
        ),
        alpha: blend(
            AttitudeGainScale::SETTLE.alpha,
            AttitudeGainScale::NEAR_PAD.alpha,
            AttitudeGainScale::FREE.alpha,
        ),
        omega_cap: blend(
            AttitudeGainScale::SETTLE.omega_cap,
            AttitudeGainScale::NEAR_PAD.omega_cap,
            AttitudeGainScale::FREE.omega_cap,
        ),
    }
}

// --- Lean cone + desired thrust axis mix (wobble reduction) -------------------

/// Inputs for continuous lean allowance and aim blending (L lander).
#[derive(Clone, Copy, Debug)]
pub struct LeanAimFuzzy {
    pub h: f64,
    pub vh: f64,
    pub vx: f64,
    pub vz: f64,
    pub vy: f64,
    pub v_down: f64,
    pub cheby: f64,
    pub k_lat: f64,
    pub max_lat_tilt: f64,
    pub has_pad: bool,
    pub seeking_center: bool,
    pub terminal_commit: bool,
    pub vh_touch: f64,
    pub lean_max: f64,
    pub lean_seek_max: f64,
    pub lean_terminal_vh: f64,
    pub lean_pad_extra_max: f64,
    pub lat_tilt_gain: f64,
    pub h_terminal: f64,
    pub k_pos: f64,
    pub k_vel: f64,
    /// Pad center XZ when seeking; ignored otherwise.
    pub target_xz: Option<[f64; 2]>,
    pub pos_x: f64,
    pub pos_z: f64,
}

/// Nominal lean cone (rad) before [`brake_safe_lean`] hard cap.
pub fn lean_max_nominal(f: &LeanAimFuzzy) -> f64 {
    let lean_seek = (0.08 + 0.004 * f.cheby + 0.025 * f.vh).min(f.lean_seek_max);
    let lean_term = (0.08 + 0.03 * f.vh).min(f.lean_terminal_vh);
    let lean_high = if f.has_pad {
        (0.10 + 0.03 * f.vh).min(f.lean_seek_max)
    } else {
        (f.max_lat_tilt + f.lat_tilt_gain * f.vh).min(f.lean_max)
    };
    // Extra lean for near-pad drift kill fades with altitude so soft approach
    // does not keep a deep lean (rocket-sled / wobble limit cycle).
    let h_lean_fade = ramp(f.h, 3.0, 18.0);
    let lean_near_drift = f.max_lat_tilt
        + (f.lat_tilt_gain * f.vh).min(f.lean_pad_extra_max) * h_lean_fade;
    let lean_near_quiet = f.max_lat_tilt;

    // Soft regime weights (hard seeking/terminal flags still dominate when set).
    let w_seek = if f.seeking_center { 1.0 } else { 0.0 };
    let w_term = if f.terminal_commit {
        1.0 - w_seek
    } else {
        0.0
    };
    // High vs near-pad free-field (and quiet pad above terminal).
    let mu_high = ramp(f.h, f.h_terminal + 2.0, f.h_terminal + 12.0);
    let mu_near_drift = and(
        and(
            ramp(f.h, 0.8, 1.5),
            ramp(f.vy, -2.0, -1.2), // vy > about −1.5
        ),
        ramp(f.vh, f.vh_touch - 0.5, f.vh_touch + 0.8),
    );
    let w_rest = (1.0_f64 - w_seek - w_term).max(0.0);
    // Cap free-field high lean as we enter the soft band (h ≲ 15 m).
    let lean_high_capped = lean_high * (0.35 + 0.65 * h_lean_fade);
    let w_high = w_rest * mu_high;
    let w_near_drift = w_rest * (1.0 - mu_high) * mu_near_drift;
    let w_near_quiet = (w_rest - w_high - w_near_drift).max(0.0);

    defuzz_weighted(
        &[
            (w_seek, lean_seek),
            (w_term, lean_term),
            (w_high, lean_high_capped),
            (w_near_drift, lean_near_drift),
            (w_near_quiet, lean_near_quiet),
        ],
        lean_high_capped,
    )
}

/// World-frame desired thrust axis (not yet lean-clamped) via TS aim mix.
///
/// Candidates: upright, anti-velocity, soft lateral trim, optional pad position PD.
/// Position seek weight is zero unless `seeking_center` (high-seek / low-commit).
pub fn blend_desired_axis(f: &LeanAimFuzzy) -> [f64; 3] {
    let upright = [0.0, 1.0, 0.0];
    let trim = [-f.k_lat * f.vx, 1.0, -f.k_lat * f.vz];
    // Vertical-dominant anti-velocity (avoids pure horizontal aim).
    let antiv = [-f.vx, 1.0, -f.vz];
    // Free-field braking axis keeps descent component when fast (legacy form).
    let antiv_brake = [-f.vx, f.v_down.max(0.2), -f.vz];

    let pos = if let (true, Some([tx, tz])) = (f.seeking_center, f.target_xz) {
        let ex = tx - f.pos_x;
        let ez = tz - f.pos_z;
        let pos_w = if f.v_down > 8.0 {
            (1.0 - ((f.v_down - 8.0) / 20.0).clamp(0.0, 0.75)).max(0.25)
        } else {
            1.0
        };
        let ax = pos_w * (f.k_pos * ex - f.k_vel * f.vx);
        let az = pos_w * (f.k_pos * ez - f.k_vel * f.vz);
        [ax, 1.0, az]
    } else {
        upright
    };

    // Memberships — continuous stand-ins for the old hard desired() branches.
    let mu_quiet = ramp_down(f.vh, 1.5, 3.2);
    let mu_fast_h = ramp(f.vh, f.vh_touch - 0.5, f.vh_touch + 1.5);
    let mu_low = ramp_down(f.h, f.h_terminal, f.h_terminal + 6.0);
    // Prefer soft trim up through ~2 m/s residual (reduces hover lean-chatter).
    let speed_sq = f.vh * f.vh + f.v_down * f.v_down;
    let mu_slow = ramp_down(speed_sq, 0.12, 4.0);
    let mu_brake_axis = ramp(f.v_down, 2.0, 8.0) * (1.0 - mu_low);
    // Mild descent + modest vh: bias free-field toward upright (wobble kill).
    let mu_hoverish = and(ramp_down(f.v_down, 1.5, 4.0), ramp_down(f.vh, 2.0, 5.0));

    // Pad quiet cruise (high, centered): upright.
    let w_upright_pad = if f.has_pad && !f.seeking_center && !f.terminal_commit {
        mu_quiet
    } else {
        0.0
    };
    // Terminal commit with residual vh: anti-v; quiet terminal: upright.
    let w_upright_term = if f.terminal_commit {
        mu_quiet
    } else {
        0.0
    };
    let w_antiv_term = if f.terminal_commit {
        mu_fast_h
    } else {
        0.0
    };
    // Position seek only while high-seeking.
    let w_pos = if f.seeking_center { 1.0 } else { 0.0 };
    // Free-field / non-seek: trim when low or slow, else anti-v (brake form when falling).
    let w_free = if f.seeking_center || f.terminal_commit {
        0.0
    } else if f.has_pad {
        // Pad path already handled quiet upright / else fall through lightly.
        (1.0 - w_upright_pad).max(0.0)
    } else {
        1.0
    };
    // Low / mild descent: commit upright to break lean↔vh limit cycles.
    let mu_low_commit = ramp_down(f.h, 10.0, 22.0) * ramp_down(f.v_down, 2.5, 7.0);
    let w_upright_free = w_free * or(mu_hoverish * 0.7, mu_low_commit * 0.85);
    let w_trim = w_free * or(mu_low, mu_slow);
    let w_antiv_free =
        (w_free * (1.0 - or(mu_low, mu_slow)) - w_upright_free).max(0.0);
    let w_antiv = w_antiv_term + w_antiv_free * (1.0 - mu_brake_axis);
    let w_antiv_b = w_antiv_free * mu_brake_axis;
    let w_upright = w_upright_pad + w_upright_term + w_upright_free;

    // Weighted sum of axes (not unit-normalized yet — caller clamp_tilt's).
    let mut acc = [0.0_f64; 3];
    let mut den = 0.0_f64;
    let add = |acc: &mut [f64; 3], den: &mut f64, w: f64, v: [f64; 3]| {
        if w > 1e-12 {
            acc[0] += w * v[0];
            acc[1] += w * v[1];
            acc[2] += w * v[2];
            *den += w;
        }
    };
    add(&mut acc, &mut den, w_upright, upright);
    add(&mut acc, &mut den, w_trim, trim);
    add(&mut acc, &mut den, w_antiv, antiv);
    add(&mut acc, &mut den, w_antiv_b, antiv_brake);
    add(&mut acc, &mut den, w_pos, pos);

    if den < 1e-12 {
        upright
    } else {
        [acc[0] / den, acc[1] / den, acc[2] / den]
    }
}

/// Blend factor toward pure world-up flip aim (0 = lean aim, 1 = flip upright).
/// Soft shoulder around `tilt_aim` so the target axis does not snap.
#[inline]
pub fn flip_aim_weight(tilt: f64, tilt_aim: f64) -> f64 {
    ramp(tilt, tilt_aim - 0.18, tilt_aim)
}

/// Linear blend of two world-frame aim vectors (not normalized).
#[inline]
pub fn blend_vec3(a: [f64; 3], b: [f64; 3], w_b: f64) -> [f64; 3] {
    let w = w_b.clamp(0.0, 1.0);
    let u = 1.0 - w;
    [u * a[0] + w * b[0], u * a[1] + w * b[1], u * a[2] + w * b[2]]
}

// --- T-cruise go / brake lean mix --------------------------------------------

/// Continuous mix weight toward **brake** lean (1 = full anti-velocity, 0 = go).
///
/// `delta_v = v_approach - v_stop`. Uses a soft band around the former enter/exit
/// thresholds so go↔brake does not chatter.
#[inline]
pub fn cruise_brake_weight(delta_v: f64, v_brake_enter: f64, v_brake_exit: f64) -> f64 {
    // Map: well below exit → 0, above enter → 1, linear between exit and enter.
    let lo = v_brake_exit.min(v_brake_enter);
    let hi = v_brake_enter.max(v_brake_exit);
    ramp(delta_v, lo, hi)
}

// --- Long-range full-throttle altitude-hold cruise ---------------------------

/// Horizontal range (m) where soft long-range membership is 0.5.
pub const LONG_RANGE_M: f64 = 5000.0;
/// Soft shoulder half-width (m) around [`LONG_RANGE_M`] (weight 0→1 on
/// `[LONG_RANGE_M − shoulder, LONG_RANGE_M + shoulder]` ≈ 3–7 km).
pub const LONG_RANGE_SHOULDER_M: f64 = 2000.0;
/// Target CoM altitude (m) while in long-range airplane cruise.
pub const LONG_CRUISE_ALT_M: f64 = 800.0;
/// Body-up·world-up floor for nose-down dive at full throttle.
/// Must stay above the caller's airplane flip-recover gate.
pub const LONG_CRUISE_COS_DIVE: f64 = 0.12;
/// Extra cos above `hover` allowed when climbing toward the hold.
const LONG_CRUISE_COS_CLIMB_EXTRA: f64 = 0.18;

/// Soft membership for long-range altitude target blend (0 below ~3 km, 1 above ~7 km).
#[inline]
pub fn long_range_weight(range_m: f64) -> f64 {
    ramp(
        range_m,
        LONG_RANGE_M - LONG_RANGE_SHOULDER_M,
        LONG_RANGE_M + LONG_RANGE_SHOULDER_M,
    )
}

/// Airplane pitch elevator at **full throttle**: desired body-up · world-up (`cos`).
///
/// Go-toward-target is priority; altitude is pitch only:
/// `a_y = (T/m)·cos − g = g·(cos/hover − 1)`. Hold when `cos ≈ hover`.
/// Nose-up climbs; **nose-down / deeper lean sinks**. Full-T climb overshoots
/// easily, so the schedule is asymmetric (soft climb, hard dive + apogee look-ahead).
#[inline]
pub fn long_range_hold_cos(alt: f64, alt_tgt: f64, vy: f64, hover: f64) -> f64 {
    let g = crate::sim::GRAVITY;
    let e = alt_tgt - alt; // + ⇒ below target
    // Coast apogee (underestimates peak under thrust) — dive if it clears hold.
    let apogee_pred = alt + vy.max(0.0) * vy.max(0.0) / (2.0 * g);
    let e_apo = alt_tgt - apogee_pred; // − ⇒ will overshoot even if thr cut
    let mu_very_low = ramp(e, 120.0, 320.0);
    let mu_low = trap(e, 25.0, 55.0, 110.0, 200.0);
    let mu_near = trap(e, -30.0, -10.0, 10.0, 30.0);
    let mu_high = trap(e, -240.0, -120.0, -40.0, -12.0);
    let mu_very_high = ramp_down(e, -480.0, -150.0);
    let mu_apo_high = ramp_down(e_apo, -200.0, -20.0);
    let v_des = defuzz_weighted(
        &[
            (mu_very_low, 7.0),
            (mu_low, 3.5),
            (mu_near, -0.5),
            (mu_high, -16.0),
            (mu_very_high, -32.0),
            (mu_apo_high, -22.0),
        ],
        (0.06 * e).clamp(-34.0, 8.0),
    );
    let loft_kill = if e_apo < 40.0 {
        0.55 * vy.max(0.0)
    } else {
        0.0
    };
    let mut a_cmd = (0.75 * (v_des - vy) - loft_kill).clamp(-22.0, 5.5);
    // Below hold and not going to overshoot: keep climbing (rate damp alone
    // would freeze altitude hundreds of metres low).
    if e > 150.0 && e_apo > 80.0 {
        a_cmd = a_cmd.max(3.0);
    } else if e > 80.0 && e_apo > 50.0 {
        a_cmd = a_cmd.max(1.5);
    }
    let cos_eq = hover * (g + a_cmd) / g;
    let cos_climb_max = (hover + LONG_CRUISE_COS_CLIMB_EXTRA).clamp(0.45, 0.55);
    cos_eq.clamp(LONG_CRUISE_COS_DIVE, cos_climb_max)
}

/// World-frame unit-ish aim for long-range go: lean `acos(cos_up)` toward `(ux,uz)`.
#[inline]
pub fn long_range_go_aim(ux: f64, uz: f64, cos_up: f64) -> [f64; 3] {
    let c = cos_up.clamp(0.05, 0.999);
    let s = (1.0 - c * c).max(0.0).sqrt();
    [ux * s, c, uz * s]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ramp_edges() {
        assert!((ramp(0.0, 0.0, 1.0) - 0.0).abs() < 1e-12);
        assert!((ramp(1.0, 0.0, 1.0) - 1.0).abs() < 1e-12);
        assert!((ramp(0.5, 0.0, 1.0) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn defuzz_midpoint() {
        let y = defuzz_weighted(&[(1.0, 0.0), (1.0, 1.0)], 0.5);
        assert!((y - 0.5).abs() < 1e-12);
    }

    #[test]
    fn late_fast_fall_hard_brakes() {
        let thr = LandingThrottleFuzzy {
            h: 25.0,
            h_env: 25.0,
            h_need: 40.0,
            v_down: 25.0,
            up_y: 0.95,
            contacting: false,
            t_soft: 0.3,
            t_support: 0.0,
            t_brake_cmd: 0.95,
            t_auth: 0.1,
            t_drift: 0.0,
            h_terminal: 4.5,
            h_coast_enable: 12.0,
            upy_soft: 0.6,
            upy_brake: 0.25,
            v_brake_min: 1.5,
        }
        .arbitrate();
        assert!(thr > 0.85, "expected hard brake, thr={thr}");
    }

    #[test]
    fn high_coast_near_zero() {
        let thr = LandingThrottleFuzzy {
            h: 80.0,
            h_env: 80.0,
            h_need: 5.0,
            v_down: 2.0,
            up_y: 1.0,
            contacting: false,
            t_soft: 0.4,
            t_support: 0.0,
            t_brake_cmd: 0.95,
            t_auth: 0.0,
            t_drift: 0.0,
            h_terminal: 4.5,
            h_coast_enable: 12.0,
            upy_soft: 0.6,
            upy_brake: 0.25,
            v_brake_min: 1.5,
        }
        .arbitrate();
        assert!(thr < 0.1, "expected coast, thr={thr}");
    }

    #[test]
    fn soft_near_pad_uses_soft_channel() {
        let thr = LandingThrottleFuzzy {
            h: 3.0,
            h_env: 3.0,
            h_need: 1.0,
            v_down: 1.0,
            up_y: 0.98,
            contacting: false,
            t_soft: 0.42,
            t_support: 0.0,
            t_brake_cmd: 0.95,
            t_auth: 0.05,
            t_drift: 0.0,
            h_terminal: 4.5,
            h_coast_enable: 12.0,
            upy_soft: 0.6,
            upy_brake: 0.25,
            v_brake_min: 1.5,
        }
        .arbitrate();
        assert!(
            (thr - 0.42).abs() < 0.08,
            "near pad should track soft channel, thr={thr}"
        );
    }

    #[test]
    fn physics_pad_coast_brake_shoulder_is_continuous() {
        let base = PhysicsPadThrottleFuzzy {
            h_env: 50.0,
            h_need: 30.0,
            v_down: 6.0,
            vy: -6.0,
            vh: 0.0,
            contacting: false,
            vh_touch: 2.0,
            up_y: 1.0,
            t_coast: 0.0,
            t_brake: 0.85,
            t_bang: 0.95,
            t_ground: 0.05,
            t_skid: 0.4,
            t_anti_loft: 0.1,
            t_drift_hold: 0.05,
            h_terminal: 4.5,
            coast_margin: 12.0,
            envelope_late: 0.75,
            v_brake_min: 1.5,
            upy_brake: 0.25,
        };
        let t_coast = base.arbitrate();
        let mut b = base;
        b.h_env = 44.0;
        let t_mid = b.arbitrate();
        b.h_env = 38.0;
        let t_brake = b.arbitrate();
        assert!(t_coast < 0.15, "high above envelope should coast, t={t_coast}");
        assert!(t_brake > 0.5, "on envelope should brake, t={t_brake}");
        assert!(
            t_mid > t_coast && t_mid < t_brake + 0.05,
            "should ramp between coast and brake, mid={t_mid}"
        );
    }

    #[test]
    fn slew_throttle_respects_asymmetric_rates() {
        let up = slew_throttle(0.0, 1.0, 0.1, 1.0, 3.0);
        assert!((up - 0.1).abs() < 1e-9);
        let down = slew_throttle(1.0, 0.0, 0.1, 1.0, 3.0);
        assert!((down - 0.7).abs() < 1e-9);
    }

    #[test]
    fn gain_settle_higher_than_free() {
        let free = attitude_gain_scales(false, false, 50.0);
        let settle = attitude_gain_scales(true, true, 0.5);
        assert!(settle.kp > free.kp);
        assert!(settle.alpha > free.alpha);
        let near = attitude_gain_scales(false, true, 12.0);
        assert!(near.kp > free.kp && near.kp < settle.kp);
    }

    #[test]
    fn aim_low_vh_prefers_upright_on_quiet_pad() {
        let f = LeanAimFuzzy {
            h: 60.0,
            vh: 0.5,
            vx: 0.3,
            vz: 0.0,
            vy: 0.0,
            v_down: 0.0,
            cheby: 5.0,
            k_lat: 0.022,
            max_lat_tilt: 0.14,
            has_pad: true,
            seeking_center: false,
            terminal_commit: false,
            vh_touch: 2.0,
            lean_max: 1.0,
            lean_seek_max: 0.28,
            lean_terminal_vh: 0.18,
            lean_pad_extra_max: 0.35,
            lat_tilt_gain: 0.06,
            h_terminal: 4.5,
            k_pos: 0.03,
            k_vel: 0.55,
            target_xz: Some([100.0, 0.0]),
            pos_x: 0.0,
            pos_z: 0.0,
        };
        let d = blend_desired_axis(&f);
        // Mostly upright: horizontal components small vs vertical.
        assert!(
            d[0].abs() + d[2].abs() < 0.15 * d[1].abs().max(0.5),
            "expected upright-ish aim, got {d:?}"
        );
    }

    #[test]
    fn aim_seeking_has_position_bias() {
        let f = LeanAimFuzzy {
            h: 80.0,
            vh: 5.0,
            vx: 0.0,
            vz: 0.0,
            vy: -2.0,
            v_down: 2.0,
            cheby: 40.0,
            k_lat: 0.022,
            max_lat_tilt: 0.14,
            has_pad: true,
            seeking_center: true,
            terminal_commit: false,
            vh_touch: 2.0,
            lean_max: 1.0,
            lean_seek_max: 0.28,
            lean_terminal_vh: 0.18,
            lean_pad_extra_max: 0.35,
            lat_tilt_gain: 0.06,
            h_terminal: 4.5,
            k_pos: 0.03,
            k_vel: 0.55,
            target_xz: Some([100.0, 0.0]),
            pos_x: 0.0,
            pos_z: 0.0,
        };
        let d = blend_desired_axis(&f);
        assert!(d[0] > 0.05, "seek +X pad should lean +X, got {d:?}");
    }

    #[test]
    fn flip_weight_edges() {
        assert!(flip_aim_weight(0.5, 1.05) < 0.01);
        assert!((flip_aim_weight(1.05, 1.05) - 1.0).abs() < 1e-12);
        let mid = flip_aim_weight(0.96, 1.05);
        assert!(mid > 0.0 && mid < 1.0);
    }

    #[test]
    fn cruise_brake_weight_monotone() {
        let a = cruise_brake_weight(-3.0, 1.0, -2.0);
        let b = cruise_brake_weight(0.0, 1.0, -2.0);
        let c = cruise_brake_weight(2.0, 1.0, -2.0);
        assert!(a < b && b < c);
        assert!(a < 0.05 && c > 0.95);
    }

    #[test]
    fn long_range_weight_edges() {
        assert!(long_range_weight(2000.0) < 0.01);
        assert!(long_range_weight(4000.0) > 0.2);
        assert!(long_range_weight(7000.0) > 0.99);
        let mid = long_range_weight(LONG_RANGE_M);
        assert!(mid > 0.4 && mid < 0.6, "mid={mid}");
    }

    #[test]
    fn long_range_hold_cos_pitch_elevator() {
        let hover = 1.0 / 3.0;
        let cos_low = long_range_hold_cos(500.0, LONG_CRUISE_ALT_M, 0.0, hover);
        let cos_at = long_range_hold_cos(LONG_CRUISE_ALT_M, LONG_CRUISE_ALT_M, 0.0, hover);
        let cos_hi = long_range_hold_cos(1100.0, LONG_CRUISE_ALT_M, 5.0, hover);
        let cos_climb_thru =
            long_range_hold_cos(LONG_CRUISE_ALT_M, LONG_CRUISE_ALT_M, 20.0, hover);
        // Below → nose-up (higher cos); above → dive lean (lower cos).
        assert!(cos_low > cos_at, "cos_low={cos_low} cos_at={cos_at}");
        assert!(cos_hi < cos_at, "cos_hi={cos_hi} cos_at={cos_at}");
        // Dive must go below hover so full-T can sink; climb above hover.
        assert!(cos_hi < hover, "dive cos={cos_hi} hover={hover}");
        assert!(cos_low > hover, "climb cos={cos_low} hover={hover}");
        // Climbing through the hold band → nose-down (kill loft), not more climb.
        assert!(
            cos_climb_thru < hover,
            "climb-through must dive, cos={cos_climb_thru}"
        );
        // Dive floor / climb cap stay in the designed band.
        assert!(cos_hi >= LONG_CRUISE_COS_DIVE - 1e-9 && cos_low <= 0.55 + 1e-9);
    }

    #[test]
    fn long_range_go_aim_unit_length() {
        let a = long_range_go_aim(1.0, 0.0, 0.5);
        let len = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
        assert!((len - 1.0).abs() < 1e-9, "len={len}");
        assert!(a[0] > 0.5 && a[1] > 0.4);
    }
}

