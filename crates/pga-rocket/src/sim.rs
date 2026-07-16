//! Rigid-body rocket simulation with PGA pose, gravity, engine thrust, and ground contact.

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
    /// Body half-height from CoM to nose / engine (m).
    pub body_half_height: f64,
    pub body_radius: f64,
    /// Landing leg foot positions in body frame (relative to CoM).
    pub leg_feet: [[f64; 3]; 4],
    /// Rotational inertia about principal axes (body frame), diagonal Ixx,Iyy,Izz.
    pub inertia: [f64; 3],
    /// Attitude torque authority (N·m) at full command.
    pub max_torque: f64,
    /// Linear ground contact stiffness / damping (penalty method).
    pub contact_stiffness: f64,
    pub contact_damping: f64,
}

impl Default for RocketParams {
    fn default() -> Self {
        // ~1000 kg, T/W ≈ 1.5 at max thrust
        let mass = 1000.0;
        let hh = 12.0;
        let leg_y = -hh - 0.5;
        let leg_r = 3.5;
        Self {
            mass,
            max_thrust: mass * GRAVITY * 1.5,
            body_half_height: hh,
            body_radius: 1.8,
            leg_feet: [
                [leg_r, leg_y, leg_r],
                [-leg_r, leg_y, leg_r],
                [-leg_r, leg_y, -leg_r],
                [leg_r, leg_y, -leg_r],
            ],
            inertia: [mass * 40.0, mass * 8.0, mass * 40.0],
            max_torque: 80_000.0,
            contact_stiffness: 5.0e5,
            contact_damping: 2.0e4,
        }
    }
}

/// Manual control commands in [0,1] or [-1,1] ranges.
#[derive(Clone, Copy, Debug, Default)]
pub struct ControlCommand {
    /// Throttle fraction in [0, 1].
    pub throttle: f64,
    /// Pitch torque command in [-1, 1] (body +X).
    pub pitch: f64,
    /// Yaw torque command in [-1, 1] (body +Y).
    pub yaw: f64,
    /// Roll torque command in [-1, 1] (body +Z).
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

    /// World-frame unit direction of vehicle thrust (body +Y).
    pub fn thrust_direction_world(&self) -> [f64; 3] {
        motor_rotate_vector(&self.motor, [0.0, 1.0, 0.0])
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

        // --- Forces (world) ---
        let mut force = [0.0, -mass * GRAVITY, 0.0];
        let dir = self.thrust_direction_world();
        let thrust = cmd.throttle * self.params.max_thrust;
        force[0] += dir[0] * thrust;
        force[1] += dir[1] * thrust;
        force[2] += dir[2] * thrust;

        // --- Torques (body) ---
        let mut body_torque = [
            cmd.pitch * self.params.max_torque,
            cmd.yaw * self.params.max_torque,
            cmd.roll * self.params.max_torque,
        ];

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

/// Rotate a world-frame vector into the body frame using the motor inverse.
fn world_vec_to_body(motor: &Multivector, v: [f64; 3]) -> [f64; 3] {
    let inv = motor.reverse().normalize_motor();
    motor_rotate_vector(&inv, v)
}

/// Public step API used by UI and tests.
pub fn step_rocket(state: &mut RocketState, dt: f64) {
    state.step(dt);
}
