//! Rigid-body rocket simulation with PGA pose, gravity, gimbaled main engine,
//! center-body roll thrusters, and ground contact.

use crate::euclidean_pga::{
    Multivector, compose_motors, extract_point, ground_plane, motor_from_pose, motor_rotate_vector,
    motor_translation, point, rotor, translator,
};

/// Standard gravity (m/s²).
pub const GRAVITY: f64 = 9.81;

/// Rocket physical parameters (typical small VTOL stack proportions, SI).
#[derive(Clone, Debug)]
pub struct RocketParams {
    pub mass: f64,
    /// Max engine thrust (N). Must exceed weight for liftoff.
    pub max_thrust: f64,
    /// Body half-height from CoM to nose / engine attach (m).
    pub body_half_height: f64,
    pub body_radius: f64,
    /// Engine bell length below body bottom (m). Feet must sit below this.
    pub nozzle_length: f64,
    /// Clearance from nozzle exit plane down to foot pads (m).
    pub leg_clearance: f64,
    /// Landing leg foot positions in body frame (relative to CoM).
    pub leg_feet: [[f64; 3]; 4],
    /// Rotational inertia about principal axes (body frame), diagonal Ixx,Iyy,Izz.
    pub inertia: [f64; 3],
    /// Max nozzle gimbal half-angle (rad). pitch/yaw commands map to ±this.
    pub max_gimbal_angle: f64,
    /// Body-frame Y of main-engine thrust application point (m, typically nozzle exit).
    pub thrust_application_y: f64,
    /// Max force per roll thruster (N).
    pub rcs_thrust: f64,
    /// Radial distance of roll thrusters from body Y axis (m).
    pub rcs_radius: f64,
    /// Body-frame Y of the roll thruster ring (m). Center of stack ≈ CoM plane.
    pub rcs_y: f64,
    /// Linear ground contact stiffness / damping (penalty method).
    pub contact_stiffness: f64,
    pub contact_damping: f64,
}

impl Default for RocketParams {
    fn default() -> Self {
        // ~1000 kg, T/W ≈ 3.0 at max thrust
        let mass = 1000.0;
        let hh = 12.0;
        let nozzle_length = 1.6;
        // Feet well below the bell exit so the nozzle never sits in the ground.
        let leg_clearance = 2.8;
        let leg_y = -hh - nozzle_length - leg_clearance;
        let leg_r = 4.0;
        let body_radius = 1.8;
        let thrust_application_y = -hh - nozzle_length;
        // 4 thrusters × F × R ≈ roll authority; Iyy is small so a few kN is plenty.
        let rcs_radius = body_radius * 0.9;
        let rcs_thrust = 3500.0;
        Self {
            mass,
            max_thrust: mass * GRAVITY * 3.0,
            body_half_height: hh,
            body_radius,
            nozzle_length,
            leg_clearance,
            leg_feet: [
                [leg_r, leg_y, leg_r],
                [-leg_r, leg_y, leg_r],
                [-leg_r, leg_y, -leg_r],
                [leg_r, leg_y, -leg_r],
            ],
            inertia: [mass * 40.0, mass * 8.0, mass * 40.0],
            // ~7° max gimbal; at full T and |r_y|≈13.6 m → tens of kN·m pitch/yaw.
            max_gimbal_angle: 0.12,
            thrust_application_y,
            rcs_thrust,
            rcs_radius,
            rcs_y: 0.0,
            contact_stiffness: 5.0e5,
            contact_damping: 2.0e4,
        }
    }
}

impl RocketParams {
    /// Body-frame Y of the engine-bell exit plane.
    pub fn nozzle_exit_y(&self) -> f64 {
        -self.body_half_height - self.nozzle_length
    }
}

/// Manual control commands in [0,1] or [-1,1] ranges.
#[derive(Clone, Copy, Debug, Default)]
pub struct ControlCommand {
    /// Throttle fraction in [0, 1].
    pub throttle: f64,
    /// Pitch gimbal command in [-1, 1] (tilts nozzle about body +X).
    pub pitch: f64,
    /// Yaw gimbal command in [-1, 1] (tilts nozzle about body +Z).
    pub yaw: f64,
    /// Roll thruster command in [-1, 1] (fires center RCS couple about body +Y).
    pub roll: f64,
}

impl ControlCommand {
    pub fn clamp(mut self) -> Self {
        self.throttle = self.throttle.clamp(0.0, 1.0);
        self.pitch = self.pitch.clamp(-1.0, 1.0);
        self.yaw = self.yaw.clamp(-1.0, 1.0);
        self.roll = self.roll.clamp(-1.0, 1.0);
        self
    }
}

/// Body-frame force and torque from propulsive actuators.
#[derive(Clone, Copy, Debug, Default)]
pub struct BodyWrench {
    pub force: [f64; 3],
    pub torque: [f64; 3],
}

/// One roll thruster site: body position and body-frame force.
#[derive(Clone, Copy, Debug)]
pub struct ThrusterSample {
    pub position_body: [f64; 3],
    pub force_body: [f64; 3],
}

/// Full rocket rigid-body state. Pose is a PGA motor; velocities are rates.
#[derive(Clone, Debug)]
pub struct RocketState {
    /// SE(3) motor mapping body frame → world.
    pub motor: Multivector,
    /// World-frame linear velocity of CoM (m/s).
    pub velocity: [f64; 3],
    /// Body-frame angular velocity (rad/s).
    pub omega: [f64; 3],
    pub command: ControlCommand,
    /// Infinite ground as a PGA plane (y = 0).
    pub ground: Multivector,
    pub params: RocketParams,
    /// True when any foot is in contact this step.
    pub contacting: bool,
}

impl Default for RocketState {
    fn default() -> Self {
        Self::resting_on_pad()
    }
}

impl RocketState {
    /// Rocket standing on legs on the ground plane (engine down, +Y up).
    pub fn resting_on_pad() -> Self {
        let params = RocketParams::default();
        let foot_y = params.leg_feet[0][1];
        let com_y = -foot_y;
        Self {
            motor: motor_from_pose(0.0, com_y, 0.0, 0.0, 0.0, 0.0),
            velocity: [0.0, 0.0, 0.0],
            omega: [0.0, 0.0, 0.0],
            command: ControlCommand::default(),
            ground: ground_plane(),
            params,
            contacting: true,
        }
    }

    /// Free-flight state at a given CoM altitude (no forced contact flag).
    pub fn at_altitude(altitude_com: f64) -> Self {
        let mut s = Self::resting_on_pad();
        s.motor = motor_from_pose(0.0, altitude_com, 0.0, 0.0, 0.0, 0.0);
        s.contacting = false;
        s
    }

    /// World-space CoM position from the PGA motor.
    pub fn position(&self) -> [f64; 3] {
        motor_translation(&self.motor)
    }

    /// World altitude of CoM (Y).
    pub fn altitude(&self) -> f64 {
        self.position()[1]
    }

    /// Current commanded thrust (N).
    pub fn thrust_newtons(&self) -> f64 {
        self.command.throttle.clamp(0.0, 1.0) * self.params.max_thrust
    }

    /// Gimbal angles (pitch about X, yaw about Z) in radians from current command.
    pub fn gimbal_angles(&self) -> (f64, f64) {
        let cmd = self.command.clamp();
        let a = self.params.max_gimbal_angle;
        (cmd.pitch * a, cmd.yaw * a)
    }

    /// PGA rotor that tilts the nozzle: yaw-about-Z ∘ pitch-about-X.
    pub fn gimbal_rotor(&self) -> Multivector {
        let (pitch, yaw) = self.gimbal_angles();
        gimbal_rotor(pitch, yaw)
    }

    /// Body-frame unit thrust direction after gimbal (default +Y when neutral).
    pub fn thrust_direction_body(&self) -> [f64; 3] {
        rotate_vector_by_rotor(&self.gimbal_rotor(), [0.0, 1.0, 0.0])
    }

    /// World-frame unit direction of vehicle thrust (gimbaled).
    pub fn thrust_direction_world(&self) -> [f64; 3] {
        motor_rotate_vector(&self.motor, self.thrust_direction_body())
    }

    /// Body-frame wrench from main engine alone (force at nozzle + r×F).
    pub fn engine_wrench_body(&self) -> BodyWrench {
        engine_wrench(&self.params, self.command.clamp())
    }

    /// Four center-body roll thrusters (positions + forces in body frame).
    pub fn roll_thrusters(&self) -> [ThrusterSample; 4] {
        roll_thrusters(&self.params, self.command.clamp().roll)
    }

    /// Body-frame wrench from the four roll thrusters.
    pub fn rcs_wrench_body(&self) -> BodyWrench {
        rcs_wrench(&self.params, self.command.clamp().roll)
    }

    /// Combined propulsive wrench in body frame (engine + RCS).
    pub fn propulsive_wrench_body(&self) -> BodyWrench {
        let e = self.engine_wrench_body();
        let r = self.rcs_wrench_body();
        BodyWrench {
            force: [
                e.force[0] + r.force[0],
                e.force[1] + r.force[1],
                e.force[2] + r.force[2],
            ],
            torque: [
                e.torque[0] + r.torque[0],
                e.torque[1] + r.torque[1],
                e.torque[2] + r.torque[2],
            ],
        }
    }

    /// World positions of the four landing feet (PGA sandwich of body points).
    pub fn foot_world_positions(&self) -> [[f64; 3]; 4] {
        let mut out = [[0.0; 3]; 4];
        for (i, foot) in self.params.leg_feet.iter().enumerate() {
            let p = point(foot[0], foot[1], foot[2]);
            out[i] = extract_point(&self.motor.sandwich(&p));
        }
        out
    }

    /// Lowest foot altitude (min world Y).
    pub fn lowest_foot_y(&self) -> f64 {
        self.foot_world_positions()
            .iter()
            .map(|p| p[1])
            .fold(f64::INFINITY, f64::min)
    }

    /// Apply a control command (clamped).
    pub fn set_command(&mut self, cmd: ControlCommand) {
        self.command = cmd.clamp();
    }

    /// Advance physics by `dt` seconds (semi-implicit Euler + PGA motor update).
    pub fn step(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let mass = self.params.mass;
        let cmd = self.command.clamp();
        self.command = cmd;

        // --- Propulsive wrenches (body frame, PGA geometry for directions/arms) ---
        let prop = self.propulsive_wrench_body();
        let f_prop_w = motor_rotate_vector(&self.motor, prop.force);

        // --- Forces (world) ---
        let mut force = [
            f_prop_w[0],
            -mass * GRAVITY + f_prop_w[1],
            f_prop_w[2],
        ];
        let mut body_torque = prop.torque;

        // --- Leg / ground contact (infinite plane y=0, PGA ground element stored) ---
        let _ground = self.ground; // plane kept for PGA geometry consumers / rendering
        self.contacting = false;
        let pos = self.position();
        let feet = self.foot_world_positions();
        for foot in &feet {
            let penetration = -foot[1];
            if penetration <= 0.0 {
                continue;
            }
            self.contacting = true;
            let r = [foot[0] - pos[0], foot[1] - pos[1], foot[2] - pos[2]];
            let omega_w = motor_rotate_vector(&self.motor, self.omega);
            let v_foot_y = self.velocity[1] + omega_w[0] * r[2] - omega_w[2] * r[0];
            let n_force = (self.params.contact_stiffness * penetration
                - self.params.contact_damping * v_foot_y.min(0.0))
            .max(0.0);
            force[1] += n_force;

            // τ_world = r × F with F=(0,n,0) → (−rz n, 0, rx n)
            let tau_w = [-r[2] * n_force, 0.0, r[0] * n_force];
            let tau_b = world_vec_to_body(&self.motor, tau_w);
            body_torque[0] += tau_b[0];
            body_torque[1] += tau_b[1];
            body_torque[2] += tau_b[2];

            // Coulomb-ish horizontal friction.
            let mu = 0.4;
            let speed_h = (self.velocity[0] * self.velocity[0]
                + self.velocity[2] * self.velocity[2])
                .sqrt();
            if speed_h > 1e-4 {
                let f_fr = mu * n_force;
                force[0] -= f_fr * self.velocity[0] / speed_h;
                force[2] -= f_fr * self.velocity[2] / speed_h;
            }
        }

        // Linear integration (semi-implicit Euler).
        let acc = [force[0] / mass, force[1] / mass, force[2] / mass];
        self.velocity[0] += acc[0] * dt;
        self.velocity[1] += acc[1] * dt;
        self.velocity[2] += acc[2] * dt;

        // Angular: I α = τ − ω × (I ω) on principal axes.
        let i = self.params.inertia;
        let w = self.omega;
        let iw = [i[0] * w[0], i[1] * w[1], i[2] * w[2]];
        let w_cross_iw = [
            w[1] * iw[2] - w[2] * iw[1],
            w[2] * iw[0] - w[0] * iw[2],
            w[0] * iw[1] - w[1] * iw[0],
        ];
        let alpha = [
            (body_torque[0] - w_cross_iw[0]) / i[0],
            (body_torque[1] - w_cross_iw[1]) / i[1],
            (body_torque[2] - w_cross_iw[2]) / i[2],
        ];
        self.omega[0] += alpha[0] * dt;
        self.omega[1] += alpha[1] * dt;
        self.omega[2] += alpha[2] * dt;

        // PGA motor update: world translation of CoM + body-frame rotation.
        let t_inc = translator(
            self.velocity[0] * dt,
            self.velocity[1] * dt,
            self.velocity[2] * dt,
        );
        let wx = self.omega[0] * dt;
        let wy = self.omega[1] * dt;
        let wz = self.omega[2] * dt;
        let angle = (wx * wx + wy * wy + wz * wz).sqrt();
        let r_inc = if angle > 1e-12 {
            rotor(wx / angle, wy / angle, wz / angle, angle)
        } else {
            Multivector::one()
        };
        let m_rot = self.motor.geo(&r_inc);
        self.motor = compose_motors(&t_inc, &m_rot);

        // Hard non-penetration: feet must not remain below the ground plane.
        let min_foot = self.lowest_foot_y();
        if min_foot < -1e-4 {
            let fix = translator(0.0, -min_foot, 0.0);
            self.motor = compose_motors(&fix, &self.motor);
            if self.velocity[1] < 0.0 {
                self.velocity[1] = 0.0;
            }
            self.contacting = true;
        }
    }
}

/// Public step API used by UI and tests.
pub fn step_rocket(state: &mut RocketState, dt: f64) {
    state.step(dt);
}

// --- Propulsion / PGA helpers ---

/// Cross product a × b.
pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Body-frame force at body-frame point → (force, torque = r × F).
pub fn body_wrench_at(r_body: [f64; 3], f_body: [f64; 3]) -> BodyWrench {
    BodyWrench {
        force: f_body,
        torque: cross(r_body, f_body),
    }
}

/// PGA rotor for nozzle gimbal: yaw-about-Z ∘ pitch-about-X.
pub fn gimbal_rotor(pitch: f64, yaw: f64) -> Multivector {
    let r_pitch = rotor(1.0, 0.0, 0.0, pitch);
    let r_yaw = rotor(0.0, 0.0, 1.0, yaw);
    r_yaw.geo(&r_pitch)
}

/// Rotate a free vector by a PGA rotor (sandwich of offset point minus origin).
pub fn rotate_vector_by_rotor(r: &Multivector, v: [f64; 3]) -> [f64; 3] {
    let p = point(v[0], v[1], v[2]);
    let o = point(0.0, 0.0, 0.0);
    let pw = extract_point(&r.sandwich(&p));
    let ow = extract_point(&r.sandwich(&o));
    [pw[0] - ow[0], pw[1] - ow[1], pw[2] - ow[2]]
}

/// Main engine wrench in body frame from throttle + gimbal commands.
pub fn engine_wrench(params: &RocketParams, cmd: ControlCommand) -> BodyWrench {
    let thrust = cmd.throttle.clamp(0.0, 1.0) * params.max_thrust;
    if thrust <= 0.0 {
        return BodyWrench::default();
    }
    let pitch = cmd.pitch.clamp(-1.0, 1.0) * params.max_gimbal_angle;
    let yaw = cmd.yaw.clamp(-1.0, 1.0) * params.max_gimbal_angle;
    let dir = rotate_vector_by_rotor(&gimbal_rotor(pitch, yaw), [0.0, 1.0, 0.0]);
    let f_body = [dir[0] * thrust, dir[1] * thrust, dir[2] * thrust];
    let r = [0.0, params.thrust_application_y, 0.0];
    body_wrench_at(r, f_body)
}

/// Four center thrusters for roll about body +Y.
///
/// Tangential couple layout (positive roll → positive τ_y = 4 R F):
/// - (+R, y, 0) force −Z, (−R, y, 0) force +Z
/// - (0, y, +R) force +X, (0, y, −R) force −X
///
/// (Exhaust is opposite the reaction force; mesh visual uses −force.)
pub fn roll_thrusters(params: &RocketParams, roll_cmd: f64) -> [ThrusterSample; 4] {
    let roll = roll_cmd.clamp(-1.0, 1.0);
    let f = roll.abs() * params.rcs_thrust;
    let s = if roll >= 0.0 { 1.0 } else { -1.0 };
    let r = params.rcs_radius;
    let y = params.rcs_y;
    let sf = s * f;
    [
        ThrusterSample {
            position_body: [r, y, 0.0],
            force_body: [0.0, 0.0, -sf],
        },
        ThrusterSample {
            position_body: [-r, y, 0.0],
            force_body: [0.0, 0.0, sf],
        },
        ThrusterSample {
            position_body: [0.0, y, r],
            force_body: [sf, 0.0, 0.0],
        },
        ThrusterSample {
            position_body: [0.0, y, -r],
            force_body: [-sf, 0.0, 0.0],
        },
    ]
}

/// Summed body-frame wrench from roll thrusters.
pub fn rcs_wrench(params: &RocketParams, roll_cmd: f64) -> BodyWrench {
    let mut out = BodyWrench::default();
    for t in roll_thrusters(params, roll_cmd) {
        let w = body_wrench_at(t.position_body, t.force_body);
        out.force[0] += w.force[0];
        out.force[1] += w.force[1];
        out.force[2] += w.force[2];
        out.torque[0] += w.torque[0];
        out.torque[1] += w.torque[1];
        out.torque[2] += w.torque[2];
    }
    out
}

/// Rotate a world-frame vector into the body frame using the motor inverse.
fn world_vec_to_body(motor: &Multivector, v: [f64; 3]) -> [f64; 3] {
    let inv = motor.reverse().normalize_motor();
    motor_rotate_vector(&inv, v)
}

#[cfg(test)]
mod unit_tests {
    use super::*;

    #[test]
    fn engine_lever_arm_matches_t_r_sin_delta() {
        let p = RocketParams::default();
        let cmd = ControlCommand {
            throttle: 1.0,
            pitch: 1.0,
            yaw: 0.0,
            roll: 0.0,
        };
        let w = engine_wrench(&p, cmd);
        let t = p.max_thrust;
        let ry = p.thrust_application_y;
        let delta = p.max_gimbal_angle;
        // Pitch about +X: thrust gains +Z component ≈ T sin δ; τ_x = r_y * F_z.
        let expected_tau_x = ry * t * delta.sin();
        assert!(
            (w.torque[0] - expected_tau_x).abs() < 1.0,
            "τ_x={} expected≈{}",
            w.torque[0],
            expected_tau_x
        );
        assert!(w.torque[1].abs() < 1e-6);
        assert!(w.torque[2].abs() < 1.0);
    }

    #[test]
    fn rcs_pure_couple_about_y() {
        let p = RocketParams::default();
        let w = rcs_wrench(&p, 1.0);
        let expected_ty = 4.0 * p.rcs_radius * p.rcs_thrust;
        assert!((w.torque[1] - expected_ty).abs() < 1e-6);
        assert!(w.torque[0].abs() < 1e-9);
        assert!(w.torque[2].abs() < 1e-9);
        assert!(w.force[0].abs() < 1e-9);
        assert!(w.force[1].abs() < 1e-9);
        assert!(w.force[2].abs() < 1e-9);
    }

    #[test]
    fn zero_throttle_no_engine_wrench() {
        let p = RocketParams::default();
        let w = engine_wrench(
            &p,
            ControlCommand {
                throttle: 0.0,
                pitch: 1.0,
                yaw: 1.0,
                roll: 0.0,
            },
        );
        assert!(w.force[0].abs() < 1e-12);
        assert!(w.force[1].abs() < 1e-12);
        assert!(w.force[2].abs() < 1e-12);
        assert!(w.torque[0].abs() < 1e-12);
        assert!(w.torque[1].abs() < 1e-12);
        assert!(w.torque[2].abs() < 1e-12);
    }
}
