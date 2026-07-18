//! Fuel-aware automatic landing via PGA motor sandwich transports.
//!
//! Guidance geometry (attitude aim, body-frame velocity) is obtained by sandwiching
//! free vectors through the pose motor. Control is split into two channels that are
//! combined with `max()`:
//!
//! - Attitude: shortest-arc axis/angle error → √-profile rate command → gimbal PD.
//!   Works for any initial attitude including inverted (antipodal singularity handled).
//! - Vertical: closed-loop suicide-burn schedule — coast while above the braking
//!   envelope, brake hard on it, soft-touch near the pad. The envelope brake is
//!   always live (never gated behind attitude recovery), so a fast fall brakes even
//!   while still tilted, as long as thrust has a useful upward component.

use crate::euclidean_pga::{motor_inverse_rotate_vector, world_up_in_body};
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
/// Do not settle onto the pad while drifting faster than this (m/s); hover and
/// bleed the drift first. Touching down fast sideways bounces into a tip-over;
/// slower skids are safely eaten by foot friction.
const VH_TOUCH: f64 = 3.5;
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
/// On-pad complete thresholds (looser than free-field L-mode lock).
const TILT_PAD_DONE: f64 = 0.18;
const OMEGA_PAD_DONE: f64 = 0.22;
const VH_PAD_DONE: f64 = 2.5;

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
        }
    }
}

impl LandingAutopilot {
    /// Snappier gains for T-key pad landings (faster upright settle after touchdown).
    pub fn for_target_pad() -> Self {
        Self {
            kp_attitude: 2.6,
            kd_attitude: 3.4,
            kd_roll: 2.4,
            k_lat: 0.028,
            max_lat_tilt: 0.11,
            k_h: 0.42,
            v_max_descent: 2.2,
            kv_descent: 0.34,
            omega_max: 2.0,
            ..Self::default()
        }
    }

    /// Arm this lander (clear complete/lock). Used by the T-key autopilot hand-off.
    pub fn arm(&mut self) {
        self.enabled = true;
        self.complete = false;
        self.attitude_locked = false;
    }

    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
        if self.enabled {
            self.complete = false;
            self.attitude_locked = false;
        }
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.complete = false;
        self.attitude_locked = false;
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

    /// Same as [`update`](Self::update), but bias lateral guidance toward a world XZ pad.
    pub fn update_with_target(
        &mut self,
        state: &RocketState,
        target_xz: Option<[f64; 2]>,
        _dt: f64,
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
        // Inside the painted square (Chebyshev) — square success, not bullseye.
        let on_pad = target_xz.is_some_and(|t| on_pad_square(pos, t));

        // While tilted the feet are not the lowest structure; run the burn
        // envelope on the true lowest hull point so an inverted fall brakes early.
        let h_env = if tilt > TILT_PROBE { state.lowest_probe_y() } else { h };

        // Once feet are down on the T-pad square, only upright + rate kill matter.
        let pad_settle = state.contacting && on_pad;

        // Desired body +Y as a shortest-arc axis/angle in the body frame.
        // High tilt: aim world-up (flip first). Near upright: velocity-kill / target aim.
        let (axis, angle) = if tilt > TILT_AIM || pad_settle {
            // Flip recovery, or on-pad settle: always aim world-up (no lean chase).
            axis_angle_from_cross([up_body[2], 0.0, -up_body[0]], up_y)
        } else {
            // Allowed lean cone about world-up (all lean policy lives here):
            // - altitude in hand: deeper anti-velocity lean kills drift fast, but
            //   never past the attitude whose vertical thrust share still brakes
            //   the current fall inside the remaining altitude;
            // - hovering off a drift near the pad: moderately deeper lean;
            // - over the pad square: shrink lean — square success, not bullseye;
            // - otherwise: the small touchdown trim cone.
            let lat_tilt = if on_pad && h < H_TERMINAL + 8.0 {
                // Inside the square near the ground: almost upright; kill residual vh only.
                (self.max_lat_tilt * 0.45 + 0.02 * vh).min(0.12)
            } else if h > H_TERMINAL + 2.0 {
                let a_req = if v_down > V_TOUCH {
                    1.15 * (v_down * v_down - V_TOUCH * V_TOUCH)
                        / (2.0 * (h_env - H_BURN_MARGIN - T_REACT * v_down).max(1.0))
                } else {
                    0.0
                };
                let upy_req = ((a_req + GRAVITY) / a_lift).clamp(0.0, 0.98);
                (self.max_lat_tilt + LAT_TILT_GAIN * vh)
                    .min(LEAN_MAX)
                    .min(upy_req.acos())
            } else if h > 1.0 && vy > -1.5 && vh > VH_TOUCH {
                self.max_lat_tilt + (LAT_TILT_GAIN * vh).min(LEAN_PAD_EXTRA_MAX)
            } else {
                self.max_lat_tilt
            };
            // Over the pad: drop position PD (square is success); only kill velocity.
            let aim_target = if on_pad { None } else { target_xz };
            let desired = desired_thrust_axis_world(
                state.velocity[0],
                state.velocity[1],
                state.velocity[2],
                h,
                self.k_lat,
                lat_tilt,
                pos[0],
                pos[2],
                aim_target,
            );
            let d = motor_inverse_rotate_vector(&state.motor, desired);
            axis_angle_from_cross([d[2], 0.0, -d[0]], d[1].clamp(-1.0, 1.0))
        };

        // √-profile rate command. On-pad settle: higher α and gains for a fast snap.
        let (kp, kd, kd_roll, alpha, w_cap) = if pad_settle {
            (
                self.kp_attitude * 1.5,
                self.kd_attitude * 1.55,
                self.kd_roll * 1.45,
                ALPHA_PLAN * 1.8,
                self.omega_max * 1.15,
            )
        } else if on_pad && h < 20.0 {
            (
                self.kp_attitude * 1.2,
                self.kd_attitude * 1.25,
                self.kd_roll * 1.15,
                ALPHA_PLAN * 1.25,
                self.omega_max,
            )
        } else {
            (
                self.kp_attitude,
                self.kd_attitude,
                self.kd_roll,
                ALPHA_PLAN,
                self.omega_max,
            )
        };
        let w_mag = (kp * angle).min((2.0 * alpha * angle).sqrt()).min(w_cap);
        let rate_err_x = omega[0] - axis[0] * w_mag;
        let rate_err_z = omega[2] - axis[2] * w_mag;
        let pitch = saturate(kd * rate_err_x);
        let yaw = saturate(kd * rate_err_z);
        let roll = saturate(-kd_roll * omega[1]);

        update_lock_latch(&mut self.attitude_locked, tilt, omega_sq, vh_sq);

        // Free-field L-mode: tight upright lock.
        if state.contacting
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
            return ControlCommand {
                throttle: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                roll,
            }
            .clamp();
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
            return ControlCommand {
                throttle: thr,
                pitch,
                yaw,
                roll,
            }
            .clamp();
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

        let use_coast_burn = h_env >= H_COAST_ENABLE || h_need + 1.0 >= h_env;
        let soft_regime =
            up_y >= UPY_SOFT && (state.contacting || h < H_TERMINAL || !use_coast_burn);

        let throttle = if state.contacting && vh > VH_TOUCH {
            // Skidding on the pad: thrust while tilted becomes a rocket sled.
            // Keep weight on the feet and let Coulomb friction stop the slide.
            hover_cmd * 0.55
        } else if soft_regime {
            soft_touch_throttle(
                hover_cmd,
                h,
                vy,
                vh,
                state.contacting,
                self.k_h,
                self.v_max_descent,
                self.kv_descent,
            )
            .max(t_auth)
        } else {
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
            // Bang-coast-bang: above the curve → free-fall; on/under → hard brake.
            // Only fights genuine descent; margins alone must never cause a loft.
            let t_brake = if up_y >= UPY_BRAKE && v_down > V_BRAKE_MIN && h_env <= h_need + 0.75 {
                BURN_PLAN_FRAC + (v_down - V_TOUCH).max(0.0) * 0.015
            } else {
                0.0
            };
            t_support.max(t_brake).max(t_auth)
        };

        ControlCommand {
            throttle: throttle.clamp(0.0, 1.0),
            pitch,
            yaw,
            roll,
        }
        .clamp()
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
    (pos[0] - target_xz[0]).abs() <= PAD_HALF_M && (pos[2] - target_xz[1]).abs() <= PAD_HALF_M
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
) -> f64 {
    let v_tgt = if vh > VH_TOUCH && !contacting {
        // Still drifting: hold off the pad (climb gently if skimming it) and
        // let the lateral-kill lean bleed the drift before settling.
        if h < 1.2 { 0.35 } else { 0.0 }
    } else if h < 1.0 {
        -0.4
    } else {
        -v_max.min(k_h * h.sqrt())
    };
    let mut t = hover_cmd + kv * (v_tgt - vy);
    if h < 2.0 && !contacting && vh <= VH_TOUCH {
        t -= 0.04;
    }
    // Never loft back up once committed to the soft approach.
    if vy > 0.15 && vh <= VH_TOUCH {
        t = t.min(hover_cmd * 0.85);
    }
    t
}

/// Position-hold gain when a pad target is set (1/s² scale after mixing with vel).
const K_POS_TARGET: f64 = 0.035;
/// Extra velocity damping toward a pad target (dimensionless mix with pos error).
const K_VEL_TARGET: f64 = 0.50;

/// Desired thrust axis inside the allowed lean cone `lat_tilt` (policy for the
/// cone size lives in `update`). Near the pad or nearly at rest: upright with a
/// small drift-kill trim (and optional pad position bias). Otherwise: anti-velocity
/// for the braking burn, or position-PD lean when `target_xz` is set.
fn desired_thrust_axis_world(
    vx: f64,
    vy: f64,
    vz: f64,
    h: f64,
    k_lat: f64,
    lat_tilt: f64,
    x: f64,
    z: f64,
    target_xz: Option<[f64; 2]>,
) -> [f64; 3] {
    let v_down = (-vy).max(0.0);
    let speed_sq = vx * vx + v_down * v_down + vz * vz;

    if let Some([tx, tz]) = target_xz {
        let ex = tx - x;
        let ez = tz - z;
        // Fade pad-seek while braking a fast fall so vertical thrust stays available.
        let pos_w = if v_down > 8.0 {
            (1.0 - ((v_down - 8.0) / 20.0).clamp(0.0, 0.85)).max(0.15)
        } else {
            1.0
        };
        let ax = pos_w * (K_POS_TARGET * ex - K_VEL_TARGET * vx);
        let az = pos_w * (K_POS_TARGET * ez - K_VEL_TARGET * vz);
        if h < H_TERMINAL + 2.0 || speed_sq < 0.16 {
            return clamp_tilt([ax - k_lat * vx, 1.0, az - k_lat * vz], lat_tilt);
        }
        // Prefer anti-velocity (brake) with a mild pad-seek bias.
        return clamp_tilt([ax - vx, v_down.max(0.5), az - vz], lat_tilt);
    }

    if h < H_TERMINAL + 2.0 || speed_sq < 0.16 {
        return clamp_tilt([-k_lat * vx, 1.0, -k_lat * vz], lat_tilt);
    }
    clamp_tilt([-vx, v_down, -vz], lat_tilt)
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
        let cmd = ap.update(&state, DT);
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
        let cmd = ap.update(&state, DT);
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
        let cmd = ap.update(&state, DT);
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
        let cmd = ap.update(&state, DT);
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
}
