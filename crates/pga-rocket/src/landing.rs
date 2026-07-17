//! Closed-loop automatic landing: upright attitude, zero spin, gentle descent.

use crate::euclidean_pga::{attitude_error_body, motor_body_up_world};
use crate::sim::{ControlCommand, GRAVITY, RocketState};

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
    /// True once near-vertical with low rates (status only; actuators stay active).
    pub attitude_locked: bool,
}

impl Default for LandingAutopilot {
    fn default() -> Self {
        Self {
            enabled: false,
            complete: false,
            // Cascaded attitude: outer rate target from tilt error, inner rate tracking.
            // Gimbal pitch/yaw > 0 produce negative body τ_x / τ_z (nozzle below CoM),
            // so rate damping enters with opposite sign from RCS roll.
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
        let up_world = motor_body_up_world(&state.motor);
        // True uprightness (do not clamp — large tilts must be visible to the controller).
        let up_y = up_world[1].clamp(-1.0, 1.0);
        let tilt = up_y.acos();
        let omega = state.omega;
        let omega_mag =
            (omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]).sqrt();

        // While heavily tilted, aim straight up. Once close, bleed lateral velocity.
        let desired_world = if tilt > 0.12 {
            [0.0, 1.0, 0.0]
        } else {
            desired_up_world(state, self.k_lat, self.max_lat_tilt)
        };

        let err = attitude_error_body(&state.motor, desired_world);
        let v_horiz = (state.velocity[0] * state.velocity[0] + state.velocity[2] * state.velocity[2])
            .sqrt();
        let h = state.lowest_foot_y();

        // Cascaded TVC: ω_tgt ∝ attitude error (same sign as err — rotate along err axis),
        // then track rates. Actuator sign: +pitch/+yaw ⇒ −τ_x/−τ_z ⇒
        // cmd = −kd·(ω_tgt − ω) = −kd·ω_tgt + kd·ω.
        let wx_tgt = (self.kp_attitude * err[0]).clamp(-self.omega_max, self.omega_max);
        let wz_tgt = (self.kp_attitude * err[2]).clamp(-self.omega_max, self.omega_max);
        let pitch = saturate(-self.kd_attitude * wx_tgt + self.kd_attitude * omega[0]);
        let yaw = saturate(-self.kd_attitude * wz_tgt + self.kd_attitude * omega[2]);
        // RCS roll: +cmd ⇒ +τ_y, so conventional −kd·ω damping.
        let roll = saturate(-self.kd_roll * omega[1]);

        // Status latch only (does not freeze actuators — residual ω must stay damped).
        if self.attitude_locked {
            if tilt > 0.14 || omega_mag > 0.14 || v_horiz > 1.8 {
                self.attitude_locked = false;
            }
        } else if tilt < 0.08 && omega_mag < 0.07 && v_horiz < 1.0 {
            self.attitude_locked = true;
        }

        let attitude_phase = tilt > 0.12 || omega_mag > 0.12;
        let vy = state.velocity[1];
        let v_tgt = if h < 1.2 {
            -0.45
        } else if attitude_phase {
            // Hold altitude while uprighting (slight climb if sinking hard).
            if vy < -1.5 {
                0.4
            } else {
                0.0
            }
        } else {
            target_descent_rate(h, self.descent_schedule_h, self.k_h, self.v_max_descent)
        };

        // Vertical lift ≈ throttle * max_thrust * up_y. Use a soft 1/up_y only when
        // mostly upright; when tilted, stay near hover so lateral accel stays bounded.
        let lift_factor = if up_y > 0.25 {
            1.0 / up_y
        } else {
            // Near-horizontal / inverted: keep moderate thrust for TVC authority only.
            1.15
        };
        let hover_cmd = (hover * lift_factor).clamp(0.0, 0.95);

        let mut throttle = hover_cmd + self.kv_descent * (v_tgt - vy);
        if attitude_phase {
            // Enough thrust for gimbal torque, not so much that we fly sideways.
            let lo = hover * 0.85;
            let hi = hover * 1.35;
            throttle = throttle.clamp(lo, hi.max(hover_cmd));
        } else if h > 0.5 {
            throttle = throttle.max(hover_cmd * 0.48);
        }
        if h < 2.0 && !state.contacting && !attitude_phase {
            throttle = (throttle - 0.06).max(0.0);
        }
        throttle = throttle.clamp(0.0, 1.0);

        if state.contacting && vy.abs() < 0.5 && tilt < 0.08 && omega_mag < 0.15 {
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

fn desired_up_world(state: &RocketState, k_lat: f64, max_lat_tilt: f64) -> [f64; 3] {
    let vx = state.velocity[0];
    let vz = state.velocity[2];
    let mut v = [0.0, 1.0, 0.0];
    v[0] -= k_lat * vx;
    v[2] -= k_lat * vz;
    clamp_tilt(normalize(v), max_lat_tilt)
}

fn target_descent_rate(h: f64, h_pad: f64, k_h: f64, v_max: f64) -> f64 {
    let dh = (h - h_pad).max(0.0);
    if dh <= 0.0 {
        return 0.0;
    }
    -v_max.min(k_h * dh.sqrt())
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 {
        return [0.0, 1.0, 0.0];
    }
    [v[0] / len, v[1] / len, v[2] / len]
}

fn clamp_tilt(v: [f64; 3], max_tilt: f64) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 {
        return [0.0, 1.0, 0.0];
    }
    let n = [v[0] / len, v[1] / len, v[2] / len];
    let angle = n[1].clamp(-1.0, 1.0).acos();
    if angle <= max_tilt {
        return n;
    }
    let horiz = (n[0] * n[0] + n[2] * n[2]).sqrt();
    if horiz < 1e-12 {
        return [0.0, 1.0, 0.0];
    }
    let sin_t = max_tilt.sin();
    let cos_t = max_tilt.cos();
    [n[0] / horiz * sin_t, cos_t, n[2] / horiz * sin_t]
}

fn saturate(x: f64) -> f64 {
    x.clamp(-1.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::euclidean_pga::motor_from_pose;

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
        // Pure roll tilts +Y toward −X; recovery axis is body Z (yaw gimbal).
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
}
