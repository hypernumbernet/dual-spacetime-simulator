//! Fuel-aware automatic landing via PGA motor sandwich transports.
//!
//! Guidance geometry (attitude aim, body-frame velocity) is obtained by sandwiching
//! free vectors through the pose motor. Control is split into two channels:
//!
//! - Attitude: shortest-arc axis/angle error → √-profile rate command → gimbal PD.
//!   Works for any initial attitude including inverted (antipodal singularity handled).
//! - Vertical: closed-loop suicide-burn schedule — coast while above the braking
//!   envelope, brake hard on it, soft-touch near the pad. Local laws stay closed-form;
//!   bang-brake **engagement** uses fuzzy membership edges ([`crate::fuzzy`]) with a
//!   **hard brake floor** so the envelope is never gated behind attitude recovery.
//!
//! ## Hard vs soft control boundaries
//!
//! **Hard conditionals (do not fuzzy):** `enabled`/`complete`, grounded recovery cut
//! (`contacting && up_y < 0.5`), pad/free-field complete latches, antipodal attitude
//! fallback, hull-probe height switch (`TILT_PROBE`), Chebyshev pad success, the
//! soft-terminal vs coast/bang **regime gate**, multi-channel `max()` priority, and the
//! suicide-burn **hard floor** when late on the envelope.
//!
//! **Fuzzy (smooth edges):**
//! - bang-brake engagement on `{up_y, v_down, h_need − h_env}`
//! - continuous attitude gain scales near pad / settle
//! - lean cone + desired thrust-axis mix (upright / anti-v / trim / pos-seek)
//! - soft shoulder on flip aim around [`TILT_AIM`]
//!
//! With a pad target ([`LandingAutopilot::update_with_target`]): mild position PD
//! only while high; near the ground the lander commits upright (no last-metre
//! walk-in). Survival success is the painted pad square.

use crate::euclidean_pga::{motor_inverse_rotate_vector, world_up_in_body};
use crate::fuzzy::{
    attitude_gain_scales, blend_desired_axis, blend_vec3, flip_aim_weight, lean_max_nominal,
    LandingThrottleFuzzy, LeanAimFuzzy, PhysicsPadThrottleFuzzy, slew_throttle,
};
use crate::sim::{ControlCommand, GRAVITY, RocketState};

/// Aim world-up above this tilt (flip regime); below it the velocity-kill aim
/// takes over. Must sit above the deepest commanded lean to avoid limit cycles.
const TILT_AIM: f64 = 1.05;
/// Above this tilt the burn envelope tracks the lowest hull probe, not the feet.
/// With the legs 16.4 m below CoM at 5.66 m diagonal radius, the feet stay the
/// lowest structure up to ~1.75 rad, so the 39-probe scan is pure overhead below
/// this and only pays off in the flip regime.
const TILT_PROBE: f64 = 1.2;
/// Hysteresis for the HUD "locked" latch (rad / rad/s / m/s).
const TILT_LOCK: f64 = 0.08;
const TILT_UNLOCK: f64 = 0.14;
const OMEGA_LOCK: f64 = 0.07;
const OMEGA_UNLOCK: f64 = 0.14;
const VH_LOCK: f64 = 1.0;
const VH_UNLOCK: f64 = 1.8;
/// Treat as still maneuvering above this attitude *error* angle or body rate.
const ERR_PHASE: f64 = 0.12;
const OMEGA_PHASE: f64 = 0.12;
/// Lateral-lean allowance grows with horizontal speed up to LEAN_MAX (rad), so
/// a vertical-neutral hard burn can kill flip-induced drift instead of crawling
/// at 8°. cos(LEAN_MAX) must stay above UPY_AUTH so support thrust keeps flowing.
const LAT_TILT_GAIN: f64 = 0.06;
const LEAN_MAX: f64 = 1.0;
/// Extra lean allowed while hovering off a drift just above the pad (rad).
const LEAN_PAD_EXTRA_MAX: f64 = 0.35;
/// Bang brake only fights real descent; slower falls belong to the soft law.
const V_BRAKE_MIN: f64 = 1.5;
/// Free-field (L): do not settle while drifting faster than this (m/s).
const VH_TOUCH: f64 = 3.5;
/// T-pad (T): stricter settle gate — residual horizontal speed is the main
/// post-touchdown tip-over driver on the painted square.
const VH_TOUCH_PAD: f64 = 2.0;
/// Soft touchdown target speed (m/s, positive = descent).
const V_TOUCH: f64 = 0.55;
/// Foot height where we switch from hard brake to soft pad control (m).
const H_TERMINAL: f64 = 4.5;
/// Extra height margin on the suicide-burn envelope (m).
const H_BURN_MARGIN: f64 = 3.0;
/// Reaction-time margin on the burn envelope (s of current descent speed).
const T_REACT: f64 = 0.25;
/// Planning throttle fraction for brake-envelope (leave headroom for attitude).
const BURN_PLAN_FRAC: f64 = 0.95;
/// Above this foot height, prefer coast/suicide-burn over continuous soft descent.
const H_COAST_ENABLE: f64 = 12.0;
/// Planning angular deceleration for the √-profile rate command (rad/s²).
/// Conservative fraction of gimbal authority T·sinδ·|r_y|/Ixx ≈ 1.2 rad/s² at full T.
const ALPHA_PLAN: f64 = 0.5;
/// Thrust up-component thresholds gating the vertical sub-channels.
/// Below UPY_BRAKE the engine cannot brake a fall (thrust mostly lateral/down);
/// below UPY_AUTH throttle exists only for gimbal torque authority.
const UPY_BRAKE: f64 = 0.25;
const UPY_AUTH: f64 = 0.5;
const UPY_SOFT: f64 = 0.6;
/// Target / launch pad half-extent (m) — matches `mesh::TARGET_PAD_HALF_EXTENT`.
pub(crate) const PAD_HALF_M: f64 = 30.0;
/// T-mode landing success half-extent (m) — narrower than the painted pad.
pub const TARGET_SUCCESS_HALF_M: f64 = 12.0;

// --- T-pad guidance (high seek, low commit) ---------------------------------
/// Foot height (m) below which position-seek is off (upright + soft-touch only).
const CENTER_SEEK_MIN_H: f64 = 45.0;
/// Drop position-seek once Chebyshev offset is within this (m) — L-mode pad target.
const CENTER_TOL_M: f64 = 15.0;
/// T-Descend: drop high-altitude seek once inside this Chebyshev offset (m).
pub(crate) const TARGET_CENTER_TOL_M: f64 = 8.0;
/// Max lean while high and seeking the pad (rad).
const LEAN_SEEK_MAX: f64 = 0.35;
/// Max lean near the pad for residual vh kill (rad) — never position walk-in.
/// Enough to null a few m/s while soft-touch holds; not so deep it drifts off-pad.
const LEAN_TERMINAL_VH: f64 = 0.18;
/// T-Descend complete latch — tighter than [`TILT_PAD_DONE`] for a stable rest pose.
const TILT_DESCEND_DONE: f64 = 0.12;
/// Position / velocity mix for high-altitude pad seek.
const K_POS_TARGET: f64 = 0.038;
const K_VEL_TARGET: f64 = 0.58;

/// On-pad complete thresholds (looser than free-field L-mode lock on tilt/rate,
/// but strict on residual horizontal speed so complete does not cut thrust mid-skid).
const TILT_PAD_DONE: f64 = 0.18;
const OMEGA_PAD_DONE: f64 = 0.22;
const VH_PAD_DONE: f64 = 1.5;
/// Altitude margin above the suicide-burn envelope before T-Descend coasts (m).
const COAST_MARGIN_M: f64 = 12.0;
/// Pessimism on closed-loop `a_req` (matches [`brake_safe_lean`] 1.15 factor).
const PHYSICS_A_REQ_SAFETY: f64 = 1.15;
/// Envelope lateness (m) that forces a hard brake floor (matches fuzzy hard_late).
const ENVELOPE_LATE_M: f64 = 0.75;
/// Main-engine throttle spool-up rate (0→1 in ~0.9 s).
const THROTTLE_SPOOL_UP: f64 = 1.1;
/// Faster spool when GNC requests a large step (suicide-burn engagement).
const THROTTLE_SPOOL_UP_EMERGENCY: f64 = 4.0;
/// Main-engine throttle spool-down rate (1→0 in ~0.4 s).
const THROTTLE_SPOOL_DOWN: f64 = 2.5;

/// Automatic landing autopilot toggled with `L`.
#[derive(Clone, Debug)]
pub struct LandingAutopilot {
    pub enabled: bool,
    pub complete: bool,
    pub kp_attitude: f64,
    pub kd_attitude: f64,
    pub kd_roll: f64,
    pub k_lat: f64,
    pub max_lat_tilt: f64,
    /// Soft terminal vertical-rate gain (√h schedule).
    pub k_h: f64,
    pub v_max_descent: f64,
    pub kv_descent: f64,
    /// Hard cap on commanded body rate (rad/s); the √-profile keeps rates far
    /// lower near upright, this only limits large-angle flips.
    pub omega_max: f64,
    /// Near-vertical + low rates (HUD only; actuators stay active).
    pub attitude_locked: bool,
    /// Delivered throttle state (lags the GNC setpoint via [`slew_throttle`]).
    throttle_actuator: f64,
    /// Re-sync actuator from vehicle command on arm / enable.
    throttle_actuator_sync: bool,
}

impl Default for LandingAutopilot {
    fn default() -> Self {
        Self {
            enabled: false,
            complete: false,
            // Cascaded attitude: outer rate from angle error, inner rate tracking.
            // +pitch/+yaw gimbal ⇒ −τ_x/−τ_z (nozzle below CoM), opposite RCS roll sign.
            kp_attitude: 1.8,
            kd_attitude: 2.4,
            kd_roll: 1.6,
            k_lat: 0.022,
            max_lat_tilt: 0.14,
            k_h: 0.35,
            v_max_descent: 1.8,
            kv_descent: 0.28,
            omega_max: 1.5,
            attitude_locked: false,
            throttle_actuator: 0.0,
            throttle_actuator_sync: true,
        }
    }
}

impl LandingAutopilot {
    /// Attitude gains for T-key Descend, derived from [`ALPHA_PLAN`] with transit-class
    /// damping (higher `kd/kp` keeps the final upright snap better damped).
    pub fn for_target_pad() -> Self {
        let alpha = ALPHA_PLAN;
        let theta_ref = std::f64::consts::FRAC_PI_2;
        let wn = (2.0 * alpha).sqrt();
        let omega_max = (2.0 * alpha * theta_ref).sqrt() * 1.3;
        Self {
            kp_attitude: 2.0 * wn,
            kd_attitude: 3.0 * wn,
            kd_roll: 2.0 * wn,
            max_lat_tilt: LEAN_TERMINAL_VH,
            omega_max,
            ..Self::default()
        }
    }

    /// Arm this lander (clear complete/lock). Used by the T-key autopilot hand-off.
    pub fn arm(&mut self) {
        self.enabled = true;
        self.complete = false;
        self.attitude_locked = false;
        // Hand-off from transit: don't inherit cruise thrust into Descend.
        self.throttle_actuator = 0.0;
        self.throttle_actuator_sync = false;
    }

    /// Arm from T transit hand-off — inherit cruise throttle to avoid a
    /// zero-thrust drop that excites lateral sway at high altitude.
    pub fn arm_from_transit(&mut self, state: &RocketState) {
        self.enabled = true;
        self.complete = false;
        self.attitude_locked = false;
        self.throttle_actuator = state.command.throttle.clamp(0.0, 1.0);
        self.throttle_actuator_sync = true;
    }

    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
        if self.enabled {
            self.complete = false;
            self.attitude_locked = false;
            self.throttle_actuator_sync = true;
        }
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.complete = false;
        self.attitude_locked = false;
        self.throttle_actuator_sync = true;
    }

    pub fn status_label(&self) -> &'static str {
        if !self.enabled {
            "off"
        } else if self.complete {
            "complete"
        } else if self.attitude_locked {
            "locked"
        } else {
            "active"
        }
    }

    pub fn update(&mut self, state: &RocketState, dt: f64) -> ControlCommand {
        self.update_with_target(state, None, dt)
    }

    /// Same as [`update`](Self::update), but with optional pad target.
    /// Position bias only while **high** (`h > CENTER_SEEK_MIN_H`); near the
    /// ground the lander is upright + soft-touch (no last-metre walk-in).
    pub fn update_with_target(
        &mut self,
        state: &RocketState,
        target_xz: Option<[f64; 2]>,
        dt: f64,
    ) -> ControlCommand {
        if !self.enabled || self.complete || state.destroyed {
            return ControlCommand::default();
        }

        let mass = state.params.mass;
        let max_thrust = state.params.max_thrust;
        let hover = mass * GRAVITY / max_thrust;
        // World-frame lift acceleration available at planned burn throttle (per up_y).
        let a_lift = BURN_PLAN_FRAC * max_thrust / mass;

        // One PGA sandwich transport: world +Y → body (tilt cos + uprighting cross).
        let up_body = world_up_in_body(&state.motor);
        let up_y = up_body[1].clamp(-1.0, 1.0);
        let tilt = up_y.acos();

        // Sandwich world velocity into the body frame (same motor action as pose).
        let v_body = motor_inverse_rotate_vector(&state.motor, state.velocity);
        let omega = state.omega;
        let omega_sq = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
        let vh_sq = state.velocity[0] * state.velocity[0] + state.velocity[2] * state.velocity[2];
        let vh = vh_sq.sqrt();
        let h = state.lowest_foot_y();
        let vy = state.velocity[1];
        let v_down = (-vy).max(0.0);
        let pos = state.position();
        let on_pad = target_xz.is_some_and(|t| on_pad_square(pos, t));
        let cheby = target_xz.map_or(0.0, |t| chebyshev_xz(pos, t));
        let has_pad = target_xz.is_some();
        // High + off-center → mild position PD. Low → upright commit (no walk-in).
        let seeking_center =
            has_pad && !state.contacting && h > CENTER_SEEK_MIN_H && cheby > CENTER_TOL_M;
        let terminal_commit = has_pad && !state.contacting && h <= CENTER_SEEK_MIN_H;
        // Stricter horizontal settle when landing on a marked pad.
        let vh_touch = if has_pad { VH_TOUCH_PAD } else { VH_TOUCH };

        // While tilted the feet are not the lowest structure; run the burn
        // envelope on the true lowest hull point so an inverted fall brakes early.
        let h_env = if tilt > TILT_PROBE { state.lowest_probe_y() } else { h };
        let pad_settle = state.contacting && on_pad;

        // Desired body +Y: pad settle is hard upright; otherwise continuous lean/aim
        // mix with a soft flip shoulder around TILT_AIM (reduces aim snap / wobble).
        let (axis, angle) = if pad_settle {
            axis_angle_from_cross([up_body[2], 0.0, -up_body[0]], up_y)
        } else {
            let vx = state.velocity[0];
            let vz = state.velocity[2];
            let lean_in = LeanAimFuzzy {
                h,
                vh,
                vx,
                vz,
                vy,
                v_down,
                cheby,
                k_lat: self.k_lat,
                max_lat_tilt: self.max_lat_tilt,
                has_pad,
                seeking_center,
                terminal_commit,
                vh_touch,
                lean_max: LEAN_MAX,
                lean_seek_max: LEAN_SEEK_MAX,
                lean_terminal_vh: LEAN_TERMINAL_VH,
                lean_pad_extra_max: LEAN_PAD_EXTRA_MAX,
                lat_tilt_gain: LAT_TILT_GAIN,
                h_terminal: H_TERMINAL,
                k_pos: K_POS_TARGET,
                k_vel: K_VEL_TARGET,
                target_xz: if seeking_center { target_xz } else { None },
                pos_x: pos[0],
                pos_z: pos[2],
            };
            let lat_tilt = brake_safe_lean(
                lean_max_nominal(&lean_in),
                v_down,
                h_env,
                a_lift,
            );
            let lean_desired = clamp_tilt(blend_desired_axis(&lean_in), lat_tilt);
            // Soft flip: blend lean aim toward pure world-up as tilt → TILT_AIM.
            let w_flip = flip_aim_weight(tilt, TILT_AIM);
            let desired = if w_flip >= 1.0 - 1e-9 {
                [0.0, 1.0, 0.0]
            } else if w_flip <= 1e-9 {
                lean_desired
            } else {
                clamp_tilt(blend_vec3(lean_desired, [0.0, 1.0, 0.0], w_flip), lat_tilt.max(0.05))
            };
            let d = motor_inverse_rotate_vector(&state.motor, desired);
            axis_angle_from_cross([d[2], 0.0, -d[0]], d[1].clamp(-1.0, 1.0))
        };

        // √-profile rate command; continuous pad/settle gain scales (no step jumps).
        let g = attitude_gain_scales(state.contacting, on_pad, h);
        let kp = self.kp_attitude * g.kp;
        let kd = self.kd_attitude * g.kd;
        let kd_roll = self.kd_roll * g.kd_roll;
        let alpha = ALPHA_PLAN * g.alpha;
        let w_cap = self.omega_max * g.omega_cap;
        let w_mag = (kp * angle).min((2.0 * alpha * angle).sqrt()).min(w_cap);
        let rate_err_x = omega[0] - axis[0] * w_mag;
        let rate_err_z = omega[2] - axis[2] * w_mag;
        let pitch = saturate(kd * rate_err_x);
        let yaw = saturate(kd * rate_err_z);
        let roll = saturate(-kd_roll * omega[1]);

        update_lock_latch(&mut self.attitude_locked, tilt, omega_sq, vh_sq);

        // Free-field L-mode only (no pad target): tight upright lock.
        if target_xz.is_none()
            && state.contacting
            && vy.abs() < 0.5
            && tilt < TILT_LOCK
            && omega_sq < 0.15 * 0.15
        {
            self.complete = true;
            return ControlCommand::default();
        }
        // T-pad: inside the painted square is success — accept a looser upright
        // sooner so the post-touchdown settle does not drag on.
        if state.contacting
            && on_pad
            && vy.abs() < 0.8
            && vh < VH_PAD_DONE
            && tilt < TILT_PAD_DONE
            && omega_sq < OMEGA_PAD_DONE * OMEGA_PAD_DONE
        {
            self.complete = true;
            return ControlCommand::default();
        }

        // Lying on the ground past recovery: the engine cannot upright a grounded
        // hull, thrusting only shoves it around. Cut power, keep roll damping.
        if state.contacting && up_y < 0.5 {
            return self.emit_command(state, dt, 0.0, 0.0, 0.0, roll);
        }

        // On-pad upright snap: light hover for gimbal/RCS torque, no lateral thrust.
        if pad_settle && up_y >= 0.5 {
            let hover_cmd = (hover / up_y.max(0.35)).clamp(0.0, 0.9);
            let thr = if tilt > 0.06 || omega_sq > 0.02 {
                // Need torque authority until nearly upright.
                (hover_cmd * 0.45 + 0.15 * (pitch.abs() + yaw.abs() + roll.abs())).min(0.55)
            } else {
                0.0
            };
            return self.emit_command(state, dt, thr, pitch, yaw, roll);
        }

        let attitude_busy = angle > ERR_PHASE || omega_sq > OMEGA_PHASE * OMEGA_PHASE;

        // --- Vertical channel (always live; attitude never gates the brake) ---
        let hover_cmd = (hover / up_y.max(0.25)).clamp(0.0, 0.95);

        // Max net upward accel along world +Y at planned burn throttle with the
        // *current* attitude (lift ≈ T·up_y). Pessimistic while tilted ⇒ brakes early.
        let a_brake = (a_lift * up_y.max(0.0) - GRAVITY).max(0.5);
        // Lateral kinetic energy ≈ needs a little extra altitude while we tilt-brake.
        let h_lat = (vh * vh) / (2.0 * a_brake.max(1.0) + 4.0 * GRAVITY);
        let h_need = burn_height(vy, a_brake, V_TOUCH) + H_BURN_MARGIN + T_REACT * v_down + h_lat;

        // Gimbal torque needs thrust. Severely tilted: throttle tracks the rate
        // error so spin-up/spin-down get full torque, while the coasting middle of
        // a flip idles — every N·s of thrust while inverted is downward Δv.
        let t_auth = if up_y < UPY_AUTH {
            // Deadband: don't chase the last ~0.15 rad/s of tracking lag during
            // the coasting middle of a flip — that thrust is downward Δv.
            let lag = (rate_err_x.abs().max(rate_err_z.abs()) - 0.15).max(0.0);
            (0.08 + 1.2 * lag).min(1.0)
        } else {
            attitude_authority_throttle(pitch, yaw, roll, hover)
        };

        let throttle = if state.contacting && vh > vh_touch {
            // Skidding on the pad: thrust while tilted becomes a rocket sled.
            // Keep weight on the feet and let Coulomb friction stop the slide.
            // (Hard safety — not fuzzy.)
            hover_cmd * 0.55
        } else {
            // Local laws stay closed-form; fuzzy only arbitrates regime mix.
            let t_soft = soft_touch_throttle(
                hover_cmd,
                h,
                vy,
                vh,
                state.contacting,
                self.k_h,
                self.v_max_descent,
                self.kv_descent,
                vh_touch,
            );
            // Hold near hover while gimbaling or leaning close to the envelope so
            // we do not dig a hole; with plenty of altitude in hand, coast instead.
            // During a deliberate lean this is the vertical-neutral kill burn:
            // hover/up_y keeps altitude while the lateral component eats drift.
            let leaning = tilt > ERR_PHASE;
            let t_support = if (attitude_busy || leaning)
                && up_y >= UPY_AUTH
                && h_env <= h_need + 12.0
            {
                let lo = hover_cmd * 0.92;
                let hi = (hover_cmd * 1.25).min(1.0);
                // Mild rate damping along body +Y (sandwich-transported velocity).
                // While leaning, lateral drift shows up in this term and lifts the
                // hover slightly — buying time aloft to finish the drift kill.
                (hover_cmd - 0.08 * v_body[1]).clamp(lo, hi)
            } else {
                0.0
            };
            // Bang magnitude when fully engaged (membership applies in fuzzy layer).
            let t_brake_cmd = BURN_PLAN_FRAC + (v_down - V_TOUCH).max(0.0) * 0.015;
            // Lateral thrust authority for high pad-seek, or near-pad drift kill
            // while soft schedule is not yet dominant.
            let t_drift = if seeking_center && vh > 4.0 && up_y >= UPY_AUTH {
                (0.10 + 0.025 * vh).min(0.35)
            } else if terminal_commit && vh > vh_touch && up_y >= UPY_AUTH {
                (hover_cmd * 0.90).min(0.55)
            } else {
                0.0
            };

            LandingThrottleFuzzy {
                h,
                h_env,
                h_need,
                v_down,
                up_y,
                contacting: state.contacting,
                t_soft,
                t_support,
                t_brake_cmd: t_brake_cmd.min(1.0),
                t_auth,
                t_drift,
                h_terminal: H_TERMINAL,
                h_coast_enable: H_COAST_ENABLE,
                upy_soft: UPY_SOFT,
                upy_brake: UPY_BRAKE,
                v_brake_min: V_BRAKE_MIN,
            }
            .arbitrate()
        };

        ControlCommand {
            throttle: self.finalize_throttle(throttle.clamp(0.0, 1.0), dt, state),
            pitch,
            yaw,
            roll,
        }
        .clamp()
    }

    fn finalize_throttle(&mut self, target: f64, dt: f64, state: &RocketState) -> f64 {
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

    fn emit_command(
        &mut self,
        state: &RocketState,
        dt: f64,
        throttle: f64,
        pitch: f64,
        yaw: f64,
        roll: f64,
    ) -> ControlCommand {
        ControlCommand {
            throttle: self.finalize_throttle(throttle, dt, state),
            pitch,
            yaw,
            roll,
        }
        .clamp()
    }

    /// T-key terminal Descend: same attitude geometry as [`update_with_target`], but
    /// vertical thrust is a closed-loop suicide burn (`a_req → throttle`) instead of
    /// √h gains and fuzzy arbitration.
    pub fn update_target_descend(
        &mut self,
        state: &RocketState,
        target_xz: [f64; 2],
        dt: f64,
    ) -> ControlCommand {
        if !self.enabled || self.complete || state.destroyed {
            return ControlCommand::default();
        }

        let mass = state.params.mass;
        let max_thrust = state.params.max_thrust;
        let hover = mass * GRAVITY / max_thrust;
        let a_lift = BURN_PLAN_FRAC * max_thrust / mass;

        let up_body = world_up_in_body(&state.motor);
        let up_y = up_body[1].clamp(-1.0, 1.0);
        let tilt = up_y.acos();

        let omega = state.omega;
        let omega_sq = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
        let vh_sq = state.velocity[0] * state.velocity[0] + state.velocity[2] * state.velocity[2];
        let vh = vh_sq.sqrt();
        let h = state.lowest_foot_y();
        let vy = state.velocity[1];
        let v_down = (-vy).max(0.0);
        let pos = state.position();
        let on_pad = on_target_success_square(pos, target_xz);
        let cheby = chebyshev_xz(pos, target_xz);
        let seeking_center = !state.contacting
            && h > CENTER_SEEK_MIN_H
            && cheby > TARGET_CENTER_TOL_M;
        let terminal_commit = !state.contacting && h <= CENTER_SEEK_MIN_H;
        let vh_touch = VH_TOUCH_PAD;

        let h_env = if tilt > TILT_PROBE {
            state.lowest_probe_y()
        } else {
            h
        };
        let pad_settle = state.contacting && on_pad;
        // Low terminal: commit to pure upright once horizontal motion is modest.
        let force_upright = terminal_commit
            && !seeking_center
            && (h <= H_TERMINAL + 10.0 || vh <= vh_touch * 1.25);

        let (axis, angle) = if pad_settle || force_upright {
            axis_angle_from_cross([up_body[2], 0.0, -up_body[0]], up_y)
        } else {
            let vx = state.velocity[0];
            let vz = state.velocity[2];
            let lean_in = LeanAimFuzzy {
                h,
                vh,
                vx,
                vz,
                vy,
                v_down,
                cheby,
                k_lat: self.k_lat,
                max_lat_tilt: self.max_lat_tilt,
                has_pad: true,
                seeking_center,
                terminal_commit,
                vh_touch,
                lean_max: LEAN_MAX,
                lean_seek_max: LEAN_SEEK_MAX,
                lean_terminal_vh: LEAN_TERMINAL_VH,
                lean_pad_extra_max: LEAN_PAD_EXTRA_MAX,
                lat_tilt_gain: LAT_TILT_GAIN,
                h_terminal: H_TERMINAL,
                k_pos: K_POS_TARGET,
                k_vel: K_VEL_TARGET,
                target_xz: if seeking_center { Some(target_xz) } else { None },
                pos_x: pos[0],
                pos_z: pos[2],
            };
            let lat_tilt = brake_safe_lean(lean_max_nominal(&lean_in), v_down, h_env, a_lift);
            let lean_desired = clamp_tilt(blend_desired_axis(&lean_in), lat_tilt);
            let w_flip = flip_aim_weight(tilt, TILT_AIM);
            let desired = if w_flip >= 1.0 - 1e-9 {
                [0.0, 1.0, 0.0]
            } else if w_flip <= 1e-9 {
                lean_desired
            } else {
                clamp_tilt(blend_vec3(lean_desired, [0.0, 1.0, 0.0], w_flip), lat_tilt.max(0.05))
            };
            let d = motor_inverse_rotate_vector(&state.motor, desired);
            axis_angle_from_cross([d[2], 0.0, -d[0]], d[1].clamp(-1.0, 1.0))
        };

        let g = attitude_gain_scales(state.contacting, on_pad, h);
        let upright_boost = if terminal_commit && h < 30.0 { 1.2 } else { 1.0 };
        let kp = self.kp_attitude * g.kp * upright_boost;
        let kd = self.kd_attitude * g.kd * upright_boost;
        let kd_roll = self.kd_roll * g.kd_roll * upright_boost;
        let alpha = ALPHA_PLAN * g.alpha;
        let w_cap = self.omega_max * g.omega_cap;
        let w_mag = (kp * angle).min((2.0 * alpha * angle).sqrt()).min(w_cap);
        let rate_err_x = omega[0] - axis[0] * w_mag;
        let rate_err_z = omega[2] - axis[2] * w_mag;
        let pitch = saturate(kd * rate_err_x);
        let yaw = saturate(kd * rate_err_z);
        let roll = saturate(-kd_roll * omega[1]);

        update_lock_latch(&mut self.attitude_locked, tilt, omega_sq, vh_sq);

        // T-pad: latch complete once feet are on the painted square, nearly upright,
        // and rates have decayed — do not cut thrust while still visibly tilted.
        if on_pad
            && state.contacting
            && h < 2.5
            && vh < VH_PAD_DONE
            && vy.abs() < 1.0
            && tilt < TILT_DESCEND_DONE
            && omega_sq < OMEGA_PAD_DONE * OMEGA_PAD_DONE
        {
            self.complete = true;
            return ControlCommand::default();
        }

        if state.contacting
            && on_pad
            && vy.abs() < 0.8
            && vh < VH_PAD_DONE
            && tilt < TILT_PAD_DONE
            && omega_sq < OMEGA_PAD_DONE * OMEGA_PAD_DONE
        {
            self.complete = true;
            return ControlCommand::default();
        }

        if state.contacting && up_y < 0.5 {
            return self.emit_command(state, dt, 0.0, 0.0, 0.0, roll);
        }

        if pad_settle && up_y >= 0.5 {
            let hover_cmd = (hover / up_y.max(0.35)).clamp(0.0, 0.9);
            // Ground reaction supports weight — cut thrust once horizontal motion is
            // low so a light bounce does not re-launch into a hover loop.
            if vh <= vh_touch && vy.abs() < 1.0 {
                let effort = pitch.abs() + yaw.abs() + roll.abs();
                let thr = if (tilt > 0.04 || omega_sq > 0.015) && tilt < 0.30 {
                    let t_floor = hover_cmd * (0.18 * (tilt / TILT_DESCEND_DONE).min(1.0) + 0.06);
                    attitude_authority_throttle(pitch, yaw, roll, hover)
                        .max(t_floor)
                        .min(hover * 0.28)
                        .max(0.06 * effort)
                } else {
                    0.0
                };
                return self.emit_command(state, dt, thr, pitch, yaw, roll);
            }
            let thr = if tilt > 0.06 || omega_sq > 0.02 {
                (hover_cmd * 0.45 + 0.15 * (pitch.abs() + yaw.abs() + roll.abs())).min(0.55)
            } else {
                0.0
            };
            return self.emit_command(state, dt, thr, pitch, yaw, roll);
        }

        let hover_cmd = (hover / up_y.max(0.25)).clamp(0.0, 0.95);
        let a_brake = (a_lift * up_y.max(0.0) - GRAVITY).max(0.5);
        let h_lat = (vh * vh) / (2.0 * a_brake.max(1.0) + 4.0 * GRAVITY);
        let h_need = burn_height(vy, a_brake, V_TOUCH) + H_BURN_MARGIN + T_REACT * v_down + h_lat;

        let t_auth = if up_y < UPY_AUTH {
            let lag = (rate_err_x.abs().max(rate_err_z.abs()) - 0.15).max(0.0);
            (0.08 + 1.2 * lag).min(1.0)
        } else {
            attitude_authority_throttle(pitch, yaw, roll, hover)
        };

        let throttle = physics_pad_vertical_throttle(
            mass,
            max_thrust,
            hover_cmd,
            up_y,
            a_brake,
            h,
            h_env,
            h_need,
            v_down,
            vy,
            vh,
            state.contacting,
            vh_touch,
            t_auth,
        );

        self.emit_command(state, dt, throttle, pitch, yaw, roll)
    }
}

fn update_lock_latch(locked: &mut bool, tilt: f64, omega_sq: f64, vh_sq: f64) {
    if *locked {
        if tilt > TILT_UNLOCK || omega_sq > OMEGA_UNLOCK * OMEGA_UNLOCK || vh_sq > VH_UNLOCK * VH_UNLOCK
        {
            *locked = false;
        }
    } else if tilt < TILT_LOCK
        && omega_sq < OMEGA_LOCK * OMEGA_LOCK
        && vh_sq < VH_LOCK * VH_LOCK
    {
        *locked = true;
    }
}

#[inline]
pub(crate) fn on_pad_square(pos: [f64; 3], target_xz: [f64; 2]) -> bool {
    chebyshev_xz(pos, target_xz) <= PAD_HALF_M
}

/// T-mode success: Chebyshev offset inside the inner landing box (painted pad is wider).
#[inline]
pub(crate) fn on_target_success_square(pos: [f64; 3], target_xz: [f64; 2]) -> bool {
    chebyshev_xz(pos, target_xz) <= TARGET_SUCCESS_HALF_M
}

/// Chebyshev (∞-norm) horizontal offset from the pad center (m).
#[inline]
pub(crate) fn chebyshev_xz(pos: [f64; 3], target_xz: [f64; 2]) -> f64 {
    (pos[0] - target_xz[0])
        .abs()
        .max((pos[2] - target_xz[1]).abs())
}

/// Rotation axis (unit, zero-Y cross form) and angle from a `cross(body_up, d)`
/// vector plus `cos(angle)`. Handles the antipodal case (inverted vehicle) where
/// the cross vanishes but a π rotation about any horizontal axis is needed.
pub(crate) fn axis_angle_from_cross(cross: [f64; 3], cos_angle: f64) -> ([f64; 3], f64) {
    let s = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
    if s < 1e-9 {
        if cos_angle > 0.0 {
            ([0.0, 0.0, 0.0], 0.0)
        } else {
            ([1.0, 0.0, 0.0], std::f64::consts::PI)
        }
    } else {
        let inv = 1.0 / s;
        (
            [cross[0] * inv, cross[1] * inv, cross[2] * inv],
            s.atan2(cos_angle),
        )
    }
}

/// Height needed to brake descent speed `−vy` down to `v_touch` at constant `a_brake`.
fn burn_height(vy: f64, a_brake: f64, v_touch: f64) -> f64 {
    let v_down = (-vy).max(0.0);
    if v_down <= v_touch || a_brake <= 1e-9 {
        return 0.0;
    }
    (v_down * v_down - v_touch * v_touch) / (2.0 * a_brake)
}

/// Gimbal torque needs thrust; keep a small floor only while attitude is busy.
fn attitude_authority_throttle(pitch: f64, yaw: f64, roll: f64, hover: f64) -> f64 {
    let effort = pitch.abs() + yaw.abs() + 0.35 * roll.abs();
    if effort < 0.04 {
        0.0
    } else {
        (0.08 + 0.35 * effort).min(hover * 0.55)
    }
}

/// Soft √h descent-rate schedule for the final metres and pad contact.
fn soft_touch_throttle(
    hover_cmd: f64,
    h: f64,
    vy: f64,
    vh: f64,
    contacting: bool,
    k_h: f64,
    v_max: f64,
    kv: f64,
    vh_touch: f64,
) -> f64 {
    let v_tgt = if vh > vh_touch && !contacting {
        // Still drifting: hold off the pad and bleed residual lateral speed.
        if h < 1.2 { 0.35 } else { 0.0 }
    } else if h < 1.0 {
        -0.4
    } else {
        -v_max.min(k_h * h.sqrt())
    };
    let mut t = hover_cmd + kv * (v_tgt - vy);
    if h < 2.0 && !contacting && vh <= vh_touch {
        t -= 0.04;
    }
    // Never loft back up once committed to the soft approach.
    if vy > 0.15 && vh <= vh_touch {
        t = t.min(hover_cmd * 0.85);
    }
    t
}

/// Cap lean so the vertical thrust share can still brake the current fall.
#[inline]
fn brake_safe_lean(nominal: f64, v_down: f64, h_env: f64, a_lift: f64) -> f64 {
    let a_req = if v_down > V_TOUCH {
        1.15 * (v_down * v_down - V_TOUCH * V_TOUCH)
            / (2.0 * (h_env - H_BURN_MARGIN - T_REACT * v_down).max(1.0))
    } else {
        0.0
    };
    let upy_req = ((a_req + GRAVITY) / a_lift).clamp(0.0, 0.98);
    nominal.min(upy_req.acos())
}

/// Closed-loop suicide burn for T-Descend: local laws → [`PhysicsPadThrottleFuzzy`] blend.
fn physics_pad_vertical_throttle(
    mass: f64,
    max_thrust: f64,
    hover_cmd: f64,
    up_y: f64,
    a_brake: f64,
    _h: f64,
    h_env: f64,
    h_need: f64,
    v_down: f64,
    vy: f64,
    vh: f64,
    contacting: bool,
    vh_touch: f64,
    t_auth: f64,
) -> f64 {
    let dh = h_env - h_need;
    let t_coast = if dh > COAST_MARGIN_M {
        t_auth.max(0.0).min(hover_cmd * 0.08)
    } else {
        t_auth.max(0.0)
    };
    let t_skid = hover_cmd * 0.55;
    let t_ground = t_auth.min(0.12);
    let t_drift_hold = t_auth.min(0.08);
    let t_anti_loft = t_auth.min(hover_cmd * 0.35);

    let h_eff = h_env.max(0.35);
    let v_touch_eff = if h_env < H_TERMINAL {
        V_TOUCH * (h_env / H_TERMINAL).sqrt().max(0.18)
    } else {
        V_TOUCH
    };
    let v_sq = v_down * v_down - v_touch_eff * v_touch_eff;
    let a_req = if v_sq > 0.0 {
        (PHYSICS_A_REQ_SAFETY * v_sq / (2.0 * h_eff)).clamp(0.0, a_brake)
    } else {
        0.0
    };

    let up_y_eff = up_y.max(UPY_BRAKE);
    // High above the burn envelope with no real descent yet (climbing hand-off
    // OR hovering at vy≈0): coast so the drop starts now. Holding hover here
    // walks toward the pad center and the walk pumps a pendulum that the
    // later drop then free-falls with (tilted, ~20 m of drift).
    let handoff_climb = vy > -0.35 && v_down < V_BRAKE_MIN && dh > COAST_MARGIN_M;
    let mut t_brake = mass * (a_req + GRAVITY) / (max_thrust * up_y_eff);
    if handoff_climb {
        t_brake = t_coast;
    }
    t_brake = t_brake.clamp(0.0, BURN_PLAN_FRAC).max(t_auth);

    if h_env < H_TERMINAL && v_down < v_touch_eff + 0.25 {
        t_brake = t_brake.min(hover_cmd * 0.92);
    }

    let t_bang = (BURN_PLAN_FRAC + (v_down - V_TOUCH).max(0.0) * 0.015).min(1.0);

    PhysicsPadThrottleFuzzy {
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
        h_terminal: H_TERMINAL,
        coast_margin: COAST_MARGIN_M,
        envelope_late: ENVELOPE_LATE_M,
        v_brake_min: V_BRAKE_MIN,
        upy_brake: UPY_BRAKE,
    }
    .arbitrate()
}

/// Normalize `v` and limit its angle from +Y to `max_tilt` (uses cos test, no acos).
pub(crate) fn clamp_tilt(v: [f64; 3], max_tilt: f64) -> [f64; 3] {
    let len_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if len_sq < 1e-24 {
        return [0.0, 1.0, 0.0];
    }
    let inv = 1.0 / len_sq.sqrt();
    let n = [v[0] * inv, v[1] * inv, v[2] * inv];
    let (sin_t, cos_t) = max_tilt.sin_cos();
    if n[1] >= cos_t {
        return n;
    }
    let horiz = (n[0] * n[0] + n[2] * n[2]).sqrt();
    if horiz < 1e-12 {
        return [0.0, 1.0, 0.0];
    }
    let s = sin_t / horiz;
    [n[0] * s, cos_t, n[2] * s]
}

#[inline]
pub(crate) fn saturate(x: f64) -> f64 {
    x.clamp(-1.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::euclidean_pga::{motor_from_pose, motor_body_up_world, motor_inverse_rotate_vector, attitude_error_body};

    const DT: f64 = 1.0 / 120.0;

    /// Run the lander for ~1 s so the throttle actuator can spool to the setpoint.
    fn lander_cmd_after_spool(ap: &mut LandingAutopilot, state: &RocketState) -> ControlCommand {
        let mut cmd = ControlCommand::default();
        for _ in 0..120 {
            cmd = ap.update(state, DT);
        }
        cmd
    }

    #[test]
    fn toggle_enables_and_disables() {
        let mut ap = LandingAutopilot::default();
        ap.toggle();
        assert!(ap.enabled);
        ap.disable();
        assert!(!ap.enabled);
    }

    #[test]
    fn burn_height_scales_with_speed_squared() {
        let a = 2.0 * GRAVITY;
        let h1 = burn_height(-10.0, a, 0.5);
        let h2 = burn_height(-20.0, a, 0.5);
        assert!(h2 > 3.5 * h1, "h1={h1} h2={h2}");
        assert!(burn_height(-0.2, a, 0.5) < 1e-9);
    }

    #[test]
    fn axis_angle_recovers_full_angle_range() {
        // 45°: cross magnitude sin(θ), cos component cos(θ).
        let th = std::f64::consts::FRAC_PI_4;
        let (axis, angle) = axis_angle_from_cross([th.sin(), 0.0, 0.0], th.cos());
        assert!((angle - th).abs() < 1e-12);
        assert!((axis[0] - 1.0).abs() < 1e-12);

        // 135°: sin is the same as 45° but the angle must not fold back.
        let th = 3.0 * std::f64::consts::FRAC_PI_4;
        let (_, angle) = axis_angle_from_cross([th.sin(), 0.0, 0.0], th.cos());
        assert!((angle - th).abs() < 1e-12, "angle={angle}");

        // Inverted: zero cross, cos = −1 ⇒ π about a fallback horizontal axis.
        let (axis, angle) = axis_angle_from_cross([0.0, 0.0, 0.0], -1.0);
        assert!((angle - std::f64::consts::PI).abs() < 1e-12);
        assert!(axis[0].abs() + axis[2].abs() > 0.5);

        // Aligned: no rotation.
        let (_, angle) = axis_angle_from_cross([0.0, 0.0, 0.0], 1.0);
        assert!(angle.abs() < 1e-12);
    }

    #[test]
    fn coast_above_envelope_uses_near_zero_throttle() {
        // Upright, slow descent, high above the burn envelope ⇒ coast.
        let mut state = RocketState::at_altitude(80.0);
        state.velocity = [0.0, -2.0, 0.0];
        state.contacting = false;
        let mut ap = LandingAutopilot::default();
        ap.enabled = true;
        let cmd = ap.update(&state, DT);
        assert!(cmd.throttle < 0.05, "expected coast, throttle={}", cmd.throttle);
    }

    #[test]
    fn behind_curve_commands_hard_brake() {
        // Upright but falling fast near the ground ⇒ full brake.
        let mut state = RocketState::at_altitude(25.0);
        state.velocity = [0.0, -25.0, 0.0];
        state.contacting = false;
        let mut ap = LandingAutopilot::default();
        ap.enabled = true;
        let cmd = lander_cmd_after_spool(&mut ap, &state);
        assert!(cmd.throttle > 0.85, "expected suicide burn, throttle={}", cmd.throttle);
    }

    #[test]
    fn fast_fall_brakes_even_while_tilted() {
        // Inside the burn envelope, the brake must not be gated behind attitude
        // recovery: a tilted fast fall still commands a hard burn.
        let mut state = RocketState::at_altitude(30.0);
        state.motor = motor_from_pose(0.0, 30.0, 0.0, 0.3, 0.0, 0.0);
        state.velocity = [0.0, -18.0, 0.0];
        state.contacting = false;
        let mut ap = LandingAutopilot::default();
        ap.enabled = true;
        let cmd = lander_cmd_after_spool(&mut ap, &state);
        assert!(
            cmd.throttle > 0.85,
            "tilted fast fall must brake, throttle={}",
            cmd.throttle
        );
    }

    #[test]
    fn inverted_rocket_gets_flip_command() {
        let mut state = RocketState::at_altitude(150.0);
        state.motor = motor_from_pose(0.0, 150.0, 0.0, std::f64::consts::PI - 0.05, 0.0, 0.0);
        let mut ap = LandingAutopilot::default();
        ap.enabled = true;
        let cmd = lander_cmd_after_spool(&mut ap, &state);
        assert!(
            cmd.pitch.abs() > 0.5,
            "inverted vehicle needs strong gimbal, pitch={}",
            cmd.pitch
        );
        assert!(cmd.throttle > 0.2, "flip needs torque authority, throttle={}", cmd.throttle);
    }

    #[test]
    fn tilted_rocket_gets_nonzero_gimbal() {
        let mut state = RocketState::at_altitude(80.0);
        state.motor = motor_from_pose(0.0, 80.0, 0.0, 0.35, 0.0, 0.0);
        let mut ap = LandingAutopilot::default();
        ap.enabled = true;
        let cmd = lander_cmd_after_spool(&mut ap, &state);
        assert!(cmd.throttle > 0.1);
        assert!(cmd.pitch.abs() > 0.05);
    }

    #[test]
    fn rolled_tilt_gets_yaw_gimbal() {
        let mut state = RocketState::at_altitude(50.0);
        state.motor = motor_from_pose(0.0, 50.0, 0.0, 0.0, 0.0, 0.55);
        let mut ap = LandingAutopilot::default();
        ap.enabled = true;
        let cmd = ap.update(&state, DT);
        assert!(
            cmd.yaw.abs() > 0.05,
            "expected yaw recovery from roll tilt, yaw={}",
            cmd.yaw
        );
    }

    #[test]
    fn world_up_in_body_matches_tilt_and_cross_error() {
        let m = motor_from_pose(0.0, 40.0, 0.0, 0.4, -0.2, 0.3);
        let d = world_up_in_body(&m);
        let up = motor_body_up_world(&m);
        assert!((d[1] - up[1]).abs() < 1e-9, "cos(tilt) mismatch");

        let err = attitude_error_body(&m, [0.0, 1.0, 0.0]);
        assert!((err[0] - d[2]).abs() < 1e-9);
        assert!(err[1].abs() < 1e-9);
        assert!((err[2] + d[0]).abs() < 1e-9);

        // Cross-in-world then inverse-rotate (legacy form) must agree.
        let err_w = [
            up[1] * 0.0 - up[2] * 1.0,
            up[2] * 0.0 - up[0] * 0.0,
            up[0] * 1.0 - up[1] * 0.0,
        ];
        let err_legacy = motor_inverse_rotate_vector(&m, err_w);
        assert!((err[0] - err_legacy[0]).abs() < 1e-8);
        assert!((err[2] - err_legacy[2]).abs() < 1e-8);
    }

    #[test]
    fn physics_vertical_handoff_kills_climb() {
        let mass = 1000.0;
        let max_thrust = 3.0 * mass * GRAVITY;
        let hover_cmd = mass * GRAVITY / max_thrust;
        let a_lift = BURN_PLAN_FRAC * max_thrust / mass;
        let a_brake = a_lift - GRAVITY;
        let h_env = 640.0;
        let h_need = 25.0;
        let t = physics_pad_vertical_throttle(
            mass,
            max_thrust,
            hover_cmd,
            0.98,
            a_brake,
            h_env,
            h_env,
            h_need,
            0.0,
            2.3,
            0.0,
            false,
            VH_TOUCH_PAD,
            0.0,
        );
        assert!(
            t < hover_cmd * 0.25,
            "T-Descend hand-off with residual climb must coast, t={t} hover={hover_cmd}"
        );
    }

    #[test]
    fn physics_vertical_coasts_above_envelope() {
        let mass = 1000.0;
        let max_thrust = 3.0 * mass * GRAVITY;
        let hover_cmd = mass * GRAVITY / max_thrust;
        let a_lift = BURN_PLAN_FRAC * max_thrust / mass;
        let a_brake = a_lift - GRAVITY;
        let h_env = 80.0;
        let h_need = burn_height(-8.0, a_brake, V_TOUCH) + H_BURN_MARGIN + T_REACT * 8.0;
        let t = physics_pad_vertical_throttle(
            mass,
            max_thrust,
            hover_cmd,
            1.0,
            a_brake,
            h_env,
            h_env,
            h_need,
            8.0,
            -8.0,
            0.0,
            false,
            VH_TOUCH_PAD,
            0.0,
        );
        assert!(t < 0.15, "expected coast above envelope, t={t}");
    }

    #[test]
    fn physics_vertical_brakes_on_envelope() {
        let mass = 1000.0;
        let max_thrust = 3.0 * mass * GRAVITY;
        let hover_cmd = mass * GRAVITY / max_thrust;
        let a_lift = BURN_PLAN_FRAC * max_thrust / mass;
        let a_brake = a_lift - GRAVITY;
        let v_down = 20.0;
        let h_need = burn_height(-v_down, a_brake, V_TOUCH) + H_BURN_MARGIN + T_REACT * v_down;
        let h_env = h_need * 0.4;
        let t = physics_pad_vertical_throttle(
            mass,
            max_thrust,
            hover_cmd,
            1.0,
            a_brake,
            h_env,
            h_env,
            h_need,
            v_down,
            -v_down,
            0.0,
            false,
            VH_TOUCH_PAD,
            0.0,
        );
        assert!(t > 0.85, "expected hard brake when late on envelope, t={t}");
    }

    #[test]
    fn physics_vertical_a_req_scales_with_speed_squared() {
        let mass = 1000.0;
        let max_thrust = 3.0 * mass * GRAVITY;
        let hover_cmd = mass * GRAVITY / max_thrust;
        let a_lift = BURN_PLAN_FRAC * max_thrust / mass;
        let a_brake = a_lift - GRAVITY;
        let h = 55.0;
        let h_need = 50.0;
        let t_slow = physics_pad_vertical_throttle(
            mass,
            max_thrust,
            hover_cmd,
            1.0,
            a_brake,
            h,
            h,
            h_need,
            4.0,
            -4.0,
            0.0,
            false,
            VH_TOUCH_PAD,
            0.0,
        );
        let t_fast = physics_pad_vertical_throttle(
            mass,
            max_thrust,
            hover_cmd,
            1.0,
            a_brake,
            h,
            h,
            h_need,
            8.0,
            -8.0,
            0.0,
            false,
            VH_TOUCH_PAD,
            0.0,
        );
        assert!(
            t_fast > t_slow,
            "faster fall must need more thrust, slow={t_slow} fast={t_fast}"
        );
    }

    #[test]
    fn physics_vertical_cuts_thrust_on_pad_contact() {
        let mass = 1000.0;
        let max_thrust = 3.0 * mass * GRAVITY;
        let hover_cmd = mass * GRAVITY / max_thrust;
        let a_lift = BURN_PLAN_FRAC * max_thrust / mass;
        let a_brake = a_lift - GRAVITY;
        let t = physics_pad_vertical_throttle(
            mass,
            max_thrust,
            hover_cmd,
            1.0,
            a_brake,
            0.5,
            0.5,
            5.0,
            0.0,
            0.2,
            0.5,
            true,
            VH_TOUCH_PAD,
            0.0,
        );
        assert!(t < 0.15, "pad contact must not hover, t={t}");
    }

    #[test]
    fn physics_vertical_does_not_loft_after_bounce() {
        let mass = 1000.0;
        let max_thrust = 3.0 * mass * GRAVITY;
        let hover_cmd = mass * GRAVITY / max_thrust;
        let a_lift = BURN_PLAN_FRAC * max_thrust / mass;
        let a_brake = a_lift - GRAVITY;
        let t = physics_pad_vertical_throttle(
            mass,
            max_thrust,
            hover_cmd,
            1.0,
            a_brake,
            2.0,
            2.0,
            5.0,
            0.0,
            1.5,
            0.0,
            false,
            VH_TOUCH_PAD,
            0.0,
        );
        assert!(
            t < hover_cmd * 0.5,
            "rising near pad must not command hover, t={t} hover={hover_cmd}"
        );
    }

    #[test]
    fn actuator_slew_limits_step_command() {
        let mut ap = LandingAutopilot::default();
        ap.enabled = true;
        ap.throttle_actuator = 0.0;
        ap.throttle_actuator_sync = false;
        let state = RocketState::at_altitude(50.0);
        let dt = 1.0 / 120.0;
        let t0 = ap.finalize_throttle(0.95, dt, &state);
        assert!(t0 < 0.95, "single frame must not jump to full thrust, t={t0}");
        let mut t = t0;
        for _ in 0..200 {
            t = ap.finalize_throttle(0.95, dt, &state);
        }
        assert!((t - 0.95).abs() < 0.02, "should reach target after spool, t={t}");
    }

    #[test]
    fn target_descend_uses_physics_vertical() {
        let mut state = RocketState::at_altitude(80.0);
        state.velocity = [0.0, -2.0, 0.0];
        state.contacting = false;
        let mut ap = LandingAutopilot::for_target_pad();
        ap.enabled = true;
        let cmd = ap.update_target_descend(&state, [0.0, 0.0], DT);
        assert!(cmd.throttle < 0.05, "expected coast high above envelope, t={}", cmd.throttle);
    }
}
