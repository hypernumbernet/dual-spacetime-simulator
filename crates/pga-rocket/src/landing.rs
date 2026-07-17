//! Closed-loop automatic landing: upright attitude, zero spin, gentle descent.

use crate::euclidean_pga::{attitude_error_body, world_up_in_body};
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
    pub k_h: f64,
    pub v_max_descent: f64,
    pub kv_descent: f64,
    pub descent_schedule_h: f64,
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
            k_lat: 0.018,
            max_lat_tilt: 0.12,
            k_h: 0.24,
            v_max_descent: 2.2,
            kv_descent: 0.22,
            descent_schedule_h: 1.5,
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

        let hover = state.params.mass * GRAVITY / state.params.max_thrust;
        // One PGA transport: world +Y in body ⇒ cos(tilt) and uprighting error.
        let up_body = world_up_in_body(&state.motor);
        let up_y = up_body[1].clamp(-1.0, 1.0);
        let tilt = up_y.acos();

        let omega = state.omega;
        let omega_sq = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
        let vh_sq = state.velocity[0] * state.velocity[0] + state.velocity[2] * state.velocity[2];

        // Default error aims at world up. Near vertical, retarget to kill lateral velocity.
        let mut err = [up_body[2], 0.0, -up_body[0]];
        if tilt <= TILT_AIM && vh_sq > 1e-6 {
            let desired = desired_up_world(
                state.velocity[0],
                state.velocity[2],
                self.k_lat,
                self.max_lat_tilt,
            );
            err = attitude_error_body(&state.motor, desired);
        }

        // cmd = kd·(ω − ω_tgt), ω_tgt = clamp(kp·err); gimbal sign flips vs RCS.
        let kd = self.kd_attitude;
        let pitch = saturate(kd * (omega[0] - clamp_rate(self.kp_attitude * err[0], self.omega_max)));
        let yaw = saturate(kd * (omega[2] - clamp_rate(self.kp_attitude * err[2], self.omega_max)));
        let roll = saturate(-self.kd_roll * omega[1]);

        update_lock_latch(
            &mut self.attitude_locked,
            tilt,
            omega_sq,
            vh_sq,
        );

        let h = state.lowest_foot_y();
        let vy = state.velocity[1];
        let attitude_phase = tilt > TILT_PHASE || omega_sq > OMEGA_PHASE * OMEGA_PHASE;
        let v_tgt = target_vertical_rate(
            h,
            vy,
            attitude_phase,
            self.descent_schedule_h,
            self.k_h,
            self.v_max_descent,
        );

        let throttle = landing_throttle(
            hover,
            up_y,
            v_tgt,
            vy,
            h,
            attitude_phase,
            state.contacting,
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

fn target_vertical_rate(
    h: f64,
    vy: f64,
    attitude_phase: bool,
    h_pad: f64,
    k_h: f64,
    v_max: f64,
) -> f64 {
    if h < 1.2 {
        -0.45
    } else if attitude_phase {
        if vy < -1.5 {
            0.4
        } else {
            0.0
        }
    } else {
        let dh = h - h_pad;
        if dh <= 0.0 {
            0.0
        } else {
            -v_max.min(k_h * dh.sqrt())
        }
    }
}

fn landing_throttle(
    hover: f64,
    up_y: f64,
    v_tgt: f64,
    vy: f64,
    h: f64,
    attitude_phase: bool,
    contacting: bool,
    kv: f64,
) -> f64 {
    // Vertical lift ≈ T·up_y. Soft 1/up_y when upright; capped when heavily tilted.
    let lift_factor = if up_y > 0.25 { 1.0 / up_y } else { 1.15 };
    let hover_cmd = (hover * lift_factor).clamp(0.0, 0.95);
    let mut throttle = hover_cmd + kv * (v_tgt - vy);

    if attitude_phase {
        let lo = hover * 0.85;
        let hi = (hover * 1.35).max(hover_cmd);
        throttle = throttle.clamp(lo, hi);
    } else if h > 0.5 {
        throttle = throttle.max(hover_cmd * 0.48);
    }
    if h < 2.0 && !contacting && !attitude_phase {
        throttle = (throttle - 0.06).max(0.0);
    }
    throttle.clamp(0.0, 1.0)
}

fn desired_up_world(vx: f64, vz: f64, k_lat: f64, max_lat_tilt: f64) -> [f64; 3] {
    clamp_tilt([-k_lat * vx, 1.0, -k_lat * vz], max_lat_tilt)
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
