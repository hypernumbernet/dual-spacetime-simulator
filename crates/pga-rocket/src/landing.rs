//! Fuel-aware automatic landing via PGA motor sandwich transports.
//!
//! Guidance geometry (attitude aim, body-frame velocity) is obtained by sandwiching
//! free vectors through the pose motor. Vertical thrust follows a closed-loop
//! suicide-burn schedule: coast while above the braking envelope, then brake hard,
//! then soft-touch near the pad — minimizing ∫throttle dt versus hover-descent.

use crate::euclidean_pga::{attitude_error_body, motor_inverse_rotate_vector, world_up_in_body};
use crate::sim::{ControlCommand, GRAVITY, RocketState};

/// Aim world-up until body +Y is within this of vertical (rad).
const TILT_AIM: f64 = 0.12;
/// Hysteresis for the HUD "locked" latch (rad / rad/s / m/s).
const TILT_LOCK: f64 = 0.08;
const TILT_UNLOCK: f64 = 0.14;
const OMEGA_LOCK: f64 = 0.07;
const OMEGA_UNLOCK: f64 = 0.14;
const VH_LOCK: f64 = 1.0;
const VH_UNLOCK: f64 = 1.8;
/// Treat as still uprighting above this tilt or rate.
const TILT_PHASE: f64 = 0.12;
const OMEGA_PHASE: f64 = 0.12;
/// Soft touchdown target speed (m/s, positive = descent).
const V_TOUCH: f64 = 0.55;
/// Foot height where we switch from hard brake to soft pad control (m).
const H_TERMINAL: f64 = 4.5;
/// Extra height margin on the suicide-burn envelope (m).
const H_BURN_MARGIN: f64 = 3.0;
/// Planning throttle fraction for brake-envelope (leave headroom for attitude).
const BURN_PLAN_FRAC: f64 = 0.95;
/// Above this foot height, prefer coast/suicide-burn over continuous soft descent.
const H_COAST_ENABLE: f64 = 12.0;

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
    /// Max body rate commanded while uprighting (rad/s).
    pub omega_max: f64,
    /// Near-vertical + low rates (HUD only; actuators stay active).
    pub attitude_locked: bool,
}

impl Default for LandingAutopilot {
    fn default() -> Self {
        Self {
            enabled: false,
            complete: false,
            // Cascaded attitude: outer rate from tilt error, inner rate tracking.
            // +pitch/+yaw gimbal ⇒ −τ_x/−τ_z (nozzle below CoM), opposite RCS roll sign.
            kp_attitude: 1.8,
            kd_attitude: 2.4,
            kd_roll: 1.6,
            k_lat: 0.022,
            max_lat_tilt: 0.14,
            k_h: 0.35,
            v_max_descent: 1.8,
            kv_descent: 0.28,
            omega_max: 0.55,
            attitude_locked: false,
        }
    }
}

impl LandingAutopilot {
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

    pub fn update(&mut self, state: &RocketState, _dt: f64) -> ControlCommand {
        if !self.enabled || self.complete {
            return ControlCommand::default();
        }

        let mass = state.params.mass;
        let max_thrust = state.params.max_thrust;
        let hover = mass * GRAVITY / max_thrust;

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

        // Desired body +Y in world, then one sandwich for the body-frame attitude error.
        // High: thrust anti-aligned with velocity (max Δv per fuel). Low: upright + lateral kill.
        let err = if tilt > TILT_AIM {
            [up_body[2], 0.0, -up_body[0]]
        } else {
            let desired = desired_thrust_axis_world(
                state.velocity[0],
                state.velocity[1],
                state.velocity[2],
                h,
                self.k_lat,
                self.max_lat_tilt,
            );
            attitude_error_body(&state.motor, desired)
        };

        let kd = self.kd_attitude;
        let pitch = saturate(kd * (omega[0] - clamp_rate(self.kp_attitude * err[0], self.omega_max)));
        let yaw = saturate(kd * (omega[2] - clamp_rate(self.kp_attitude * err[2], self.omega_max)));
        let roll = saturate(-self.kd_roll * omega[1]);

        update_lock_latch(&mut self.attitude_locked, tilt, omega_sq, vh_sq);

        let vy = state.velocity[1];
        let attitude_phase = tilt > TILT_PHASE || omega_sq > OMEGA_PHASE * OMEGA_PHASE;

        // Max net upward accel along world +Y at planned burn throttle (lift ≈ T·up_y).
        let a_brake = ((BURN_PLAN_FRAC * max_thrust / mass) * up_y.max(0.35) - GRAVITY).max(0.5);
        // Lateral kinetic energy ≈ needs a little extra altitude while we tilt-brake.
        let h_lat = (vh * vh) / (2.0 * a_brake.max(1.0) + 4.0 * GRAVITY);
        let h_burn = burn_height(vy, a_brake, V_TOUCH) + H_BURN_MARGIN + h_lat;

        let att_throttle = attitude_authority_throttle(pitch, yaw, roll, hover);
        let throttle = fuel_optimal_throttle(
            hover,
            up_y,
            h,
            vy,
            v_body[1],
            attitude_phase,
            state.contacting,
            h_burn,
            a_brake,
            att_throttle,
            self.k_h,
            self.v_max_descent,
            self.kv_descent,
        );

        if state.contacting
            && vy.abs() < 0.5
            && tilt < TILT_LOCK
            && omega_sq < 0.15 * 0.15
        {
            self.complete = true;
            return ControlCommand::default();
        }

        ControlCommand {
            throttle,
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

/// Coast above the suicide-burn envelope; bang brake on it; soft √h near the pad.
///
/// Bang-coast-bang (with a soft terminal) avoids the hover equilibrium that a
/// proportional "slack" throttle creates when starting slow and already low.
fn fuel_optimal_throttle(
    hover: f64,
    up_y: f64,
    h: f64,
    vy: f64,
    v_along_body_up: f64,
    attitude_phase: bool,
    contacting: bool,
    h_burn: f64,
    a_brake: f64,
    att_throttle: f64,
    k_h: f64,
    v_max: f64,
    kv: f64,
) -> f64 {
    let lift_factor = if up_y > 0.25 { 1.0 / up_y } else { 1.2 };
    let hover_cmd = (hover * lift_factor).clamp(0.0, 0.95);

    // Recover upright: hold near hover so we do not dig a hole while gimbaling.
    // Slightly under-hover when high enough that the burn envelope still fits.
    if attitude_phase {
        let lo = if h > h_burn + 12.0 {
            hover_cmd * 0.72
        } else {
            hover_cmd * 0.92
        };
        let hi = (hover_cmd * 1.25).min(1.0);
        // Mild rate damping along body +Y (sandwich-transported velocity).
        let mut t = hover_cmd - 0.08 * v_along_body_up;
        t = t.clamp(lo, hi).max(att_throttle);
        return t.clamp(0.0, 1.0);
    }

    let h_need = burn_height(vy, a_brake, V_TOUCH) + H_BURN_MARGIN;
    let use_coast_burn = h >= H_COAST_ENABLE || h_need + 1.0 >= h;

    // Soft pad, or short final when we never entered a high-altitude coast.
    if contacting || h < H_TERMINAL || !use_coast_burn {
        let v_tgt = if h < 1.0 {
            -0.4
        } else {
            -v_max.min(k_h * h.sqrt())
        };
        let mut t = hover_cmd + kv * (v_tgt - vy);
        if h < 2.0 && !contacting {
            t -= 0.04;
        }
        // Never loft back up once committed to the soft approach.
        if vy > 0.15 {
            t = t.min(hover_cmd * 0.85);
        }
        return t.max(att_throttle).clamp(0.0, 1.0);
    }

    // Bang-coast-bang: above the curve → free-fall; on/under → hard brake.
    if h > h_need + 0.75 {
        return att_throttle.clamp(0.0, 1.0);
    }

    let catch_up = (-vy - V_TOUCH).max(0.0) * 0.015;
    (BURN_PLAN_FRAC + catch_up).max(att_throttle).clamp(0.0, 1.0)
}

fn desired_thrust_axis_world(
    vx: f64,
    vy: f64,
    vz: f64,
    h: f64,
    k_lat: f64,
    max_lat_tilt: f64,
) -> [f64; 3] {
    // Near the pad: stay upright and bleed lateral speed with a small tilt.
    if h < H_TERMINAL + 2.0 {
        return clamp_tilt([-k_lat * vx, 1.0, -k_lat * vz], max_lat_tilt);
    }
    // Higher up: aim body +Y against the velocity vector (PGA sandwich maps this
    // into attitude error). Keeps a strong upward bias so we never tip over.
    let v_down = (-vy).max(0.0);
    let speed = (vx * vx + v_down * v_down + vz * vz).sqrt();
    if speed < 0.4 {
        return clamp_tilt([-k_lat * vx, 1.0, -k_lat * vz], max_lat_tilt);
    }
    // anti-velocity with floor on +Y so the rocket stays mostly upright.
    let anti = [-vx / speed, (v_down / speed).max(0.75), -vz / speed];
    clamp_tilt(anti, max_lat_tilt)
}

/// Normalize `v` and limit its angle from +Y to `max_tilt` (uses cos test, no acos).
fn clamp_tilt(v: [f64; 3], max_tilt: f64) -> [f64; 3] {
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
fn clamp_rate(x: f64, max: f64) -> f64 {
    x.clamp(-max, max)
}

#[inline]
fn saturate(x: f64) -> f64 {
    x.clamp(-1.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::euclidean_pga::{motor_from_pose, motor_body_up_world, motor_inverse_rotate_vector};

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
    fn coast_above_envelope_uses_near_zero_throttle() {
        let t = fuel_optimal_throttle(
            1.0 / 3.0,
            1.0,
            80.0,
            -2.0,
            -2.0,
            false,
            false,
            12.0,
            2.0 * GRAVITY,
            0.0,
            0.35,
            1.8,
            0.28,
        );
        assert!(t < 0.05, "expected coast, throttle={t}");
    }

    #[test]
    fn behind_curve_commands_hard_brake() {
        let t = fuel_optimal_throttle(
            1.0 / 3.0,
            1.0,
            8.0,
            -25.0,
            -25.0,
            false,
            false,
            20.0,
            2.0 * GRAVITY,
            0.0,
            0.35,
            1.8,
            0.28,
        );
        assert!(t > 0.85, "expected suicide burn, throttle={t}");
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
