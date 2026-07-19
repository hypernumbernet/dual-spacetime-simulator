//! Rigid-body rocket simulation with PGA pose, gravity, gimbaled main engine,
//! center-body roll thrusters, and ground contact.

use crate::euclidean_pga::{
    Multivector, compose_motors, extract_point, ground_plane, motor_from_pose, motor_rotate_vector,
    motor_translation, point, rotor, translator,
};
use std::ops::Add;

/// Standard gravity (m/s²).
pub const GRAVITY: f64 = 9.81;

/// Sea-level air density (kg/m³), ISA.
pub const AIR_DENSITY: f64 = 1.225;

/// Effective ballistic coefficient β = m / (C_d A) (kg/m²).
///
/// Real launch-vehicle stages are typically a few ×10³ kg/m². Using
/// `½ ρ C_d A` with this stack's full geometric cross-section would understate
/// β (the visual body is far lighter than a metal vehicle of that size) and
/// make drag dominate. We pin β to a launch-vehicle-like value so
/// drag/weight at subsonic speeds matches the real world:
/// `k = ρ m / (2 β)` ⇒ `F = −k |v| v`.
pub const AIR_BALLISTIC_COEFF: f64 = 2.5e3;

/// Quadratic air-drag coefficient `k` for `F = −k |v| v` from mass and β.
#[inline]
pub fn air_drag_k_from_mass(mass: f64) -> f64 {
    let m = mass.max(1e-6);
    AIR_DENSITY * m / (2.0 * AIR_BALLISTIC_COEFF)
}

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
    /// Max force per roll thruster (N).
    pub rcs_thrust: f64,
    /// Radial distance of roll thrusters from body Y axis (m).
    pub rcs_radius: f64,
    /// Body-frame Y of the roll thruster ring (m). Center of stack ≈ CoM plane.
    pub rcs_y: f64,
    /// Linear ground contact stiffness / damping (penalty method).
    pub contact_stiffness: f64,
    pub contact_damping: f64,
    /// Coulomb friction coefficient for landing feet (μ).
    pub friction_mu: f64,
    /// Coulomb friction coefficient for body / nozzle / nose hull samples (μ).
    pub body_friction_mu: f64,
    /// Regularization speed (m/s) for Coulomb friction; avoids a hard stick/slip jump.
    pub friction_slip_eps: f64,
    /// Height band (m) above the ground plane still treated as planted for friction.
    /// Needed because hard non-penetration often leaves samples exactly at y=0 (zero spring N).
    pub contact_band: f64,
    /// Normal restitution [0, 1] applied when hard-resolving deep ground penetration
    /// (0 = stick/no bounce, 1 = perfectly elastic bounce of CoM vertical velocity).
    pub restitution: f64,
    /// Normal impact speed (m/s) above which the vehicle is destroyed on ground contact.
    pub crash_impact_speed: f64,
    /// Quadratic air-drag coefficient: world force `F = −k |v| v` (N when v in m/s).
    /// Default is [`air_drag_k_from_mass`] (realistic β, not full geometric C_d A).
    pub air_drag_k: f64,
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
            rcs_thrust,
            rcs_radius,
            rcs_y: 0.0,
            contact_stiffness: 5.0e5,
            contact_damping: 2.0e4,
            // High enough that pad friction can dump residual spin; RCS can still
            // overcome it at full thrust if desired.
            friction_mu: 0.6,
            // Hull metal is a bit more slippery than feet pads.
            body_friction_mu: 0.45,
            friction_slip_eps: 0.05,
            contact_band: 0.05,
            restitution: 0.35,
            crash_impact_speed: 10.0,
            // β≈2500 kg/m² → k≈0.245; freefall terminal ≈ √(mg/k) ≈ 200 m/s.
            air_drag_k: air_drag_k_from_mass(mass),
        }
    }
}

impl RocketParams {
    /// Body-frame Y of the engine-bell exit plane (thrust application point).
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

impl BodyWrench {
    #[inline]
    pub fn accum(&mut self, other: BodyWrench) {
        self.force[0] += other.force[0];
        self.force[1] += other.force[1];
        self.force[2] += other.force[2];
        self.torque[0] += other.torque[0];
        self.torque[1] += other.torque[1];
        self.torque[2] += other.torque[2];
    }
}

impl Add for BodyWrench {
    type Output = Self;
    #[inline]
    fn add(mut self, rhs: Self) -> Self {
        self.accum(rhs);
        self
    }
}

/// One roll thruster site: body position and body-frame force.
#[derive(Clone, Copy, Debug)]
pub struct ThrusterSample {
    pub position_body: [f64; 3],
    pub force_body: [f64; 3],
}

/// Kind of ground-contact probe (feet vs structural hull).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactKind {
    /// Landing-pad foot (foot friction μ + rest-share / spring normal).
    Foot,
    /// Body cylinder, nozzle bell, or nose (hull friction μ + rest-share / spring normal).
    Hull,
}

/// Body-frame sample used for ground collision (feet + hull).
#[derive(Clone, Copy, Debug)]
pub struct ContactProbe {
    pub body: [f64; 3],
    pub kind: ContactKind,
}

/// Angular resolution of cylindrical hull rings (matches visual mesh segments).
const HULL_RING_SEGS: usize = 8;

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
    /// True when any ground probe (foot or hull) is in contact this step.
    pub contacting: bool,
    /// True when a body/nozzle/nose hull sample was in contact this step.
    pub body_contacting: bool,
    /// True after a hard ground impact destroys the vehicle.
    pub destroyed: bool,
    /// Seconds since the crash explosion started.
    pub explosion_age: f64,
    /// World CoM position at the moment of destruction.
    pub explosion_origin: [f64; 3],
    /// Peak normal impact speed (m/s) recorded at destruction.
    pub last_impact_speed: f64,
    /// When true, apply quadratic air drag from ground-relative CoM velocity.
    pub air_drag_enabled: bool,
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
            body_contacting: false,
            destroyed: false,
            explosion_age: 0.0,
            explosion_origin: [0.0, com_y, 0.0],
            last_impact_speed: 0.0,
            air_drag_enabled: true,
        }
    }

    /// Free-flight state at a given CoM altitude (no forced contact flag).
    pub fn at_altitude(altitude_com: f64) -> Self {
        let mut s = Self::resting_on_pad();
        s.motor = motor_from_pose(0.0, altitude_com, 0.0, 0.0, 0.0, 0.0);
        s.contacting = false;
        s.body_contacting = false;
        s.destroyed = false;
        s.explosion_age = 0.0;
        s.explosion_origin = [0.0, altitude_com, 0.0];
        s.last_impact_speed = 0.0;
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
    #[inline]
    pub fn gimbal_angles(&self) -> (f64, f64) {
        let a = self.params.max_gimbal_angle;
        (
            self.command.pitch.clamp(-1.0, 1.0) * a,
            self.command.yaw.clamp(-1.0, 1.0) * a,
        )
    }

    /// PGA rotor that tilts the nozzle: yaw-about-Z ∘ pitch-about-X.
    pub fn gimbal_rotor(&self) -> Multivector {
        let (pitch, yaw) = self.gimbal_angles();
        gimbal_rotor(pitch, yaw)
    }

    /// Body-frame unit thrust direction after gimbal (default +Y when neutral).
    pub fn thrust_direction_body(&self) -> [f64; 3] {
        let (pitch, yaw) = self.gimbal_angles();
        thrust_dir_body(pitch, yaw)
    }

    /// World-frame unit direction of vehicle thrust (gimbaled).
    pub fn thrust_direction_world(&self) -> [f64; 3] {
        motor_rotate_vector(&self.motor, self.thrust_direction_body())
    }

    /// Body-frame wrench from main engine alone (force at nozzle + r×F).
    pub fn engine_wrench_body(&self) -> BodyWrench {
        engine_wrench(&self.params, self.command)
    }

    /// Four center-body roll thrusters (positions + forces in body frame).
    pub fn roll_thrusters(&self) -> [ThrusterSample; 4] {
        roll_thrusters(&self.params, self.command.roll)
    }

    /// Body-frame wrench from the four roll thrusters.
    pub fn rcs_wrench_body(&self) -> BodyWrench {
        rcs_wrench(&self.params, self.command.roll)
    }

    /// Combined propulsive wrench in body frame (engine + RCS).
    pub fn propulsive_wrench_body(&self) -> BodyWrench {
        propulsive_wrench(&self.params, self.command)
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
        let mut min_y = f64::INFINITY;
        for foot in &self.params.leg_feet {
            let p = point(foot[0], foot[1], foot[2]);
            let y = extract_point(&self.motor.sandwich(&p))[1];
            if y < min_y {
                min_y = y;
            }
        }
        min_y
    }

    /// Lowest hull/foot probe altitude (min world Y over all ground samples).
    pub fn lowest_probe_y(&self) -> f64 {
        let mut min_y = f64::INFINITY;
        for probe in ground_contact_probes(&self.params) {
            let p = point(probe.body[0], probe.body[1], probe.body[2]);
            let y = extract_point(&self.motor.sandwich(&p))[1];
            if y < min_y {
                min_y = y;
            }
        }
        min_y
    }

    /// Apply a control command (clamped). Ignored while destroyed.
    pub fn set_command(&mut self, cmd: ControlCommand) {
        if self.destroyed {
            self.command = ControlCommand::default();
            return;
        }
        self.command = cmd.clamp();
    }

    /// Advance physics by `dt` seconds (semi-implicit Euler + PGA motor update).
    pub fn step(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        if self.destroyed {
            self.explosion_age += dt;
            self.contacting = true;
            self.velocity = [0.0, 0.0, 0.0];
            self.omega = [0.0, 0.0, 0.0];
            self.command = ControlCommand::default();
            return;
        }
        let mass = self.params.mass;
        let cmd = self.command.clamp();
        self.command = cmd;

        // --- Propulsive wrenches (body frame; closed-form gimbal + RCS couple) ---
        let prop = propulsive_wrench(&self.params, cmd);
        let f_prop_w = motor_rotate_vector(&self.motor, prop.force);

        // --- Forces (world) ---
        let mut force = [
            f_prop_w[0],
            -mass * GRAVITY + f_prop_w[1],
            f_prop_w[2],
        ];
        let mut body_torque = prop.torque;

        // --- Air drag from ground-relative CoM velocity: F = −k |v| v ---
        if self.air_drag_enabled {
            let k = self.params.air_drag_k.max(0.0);
            if k > 0.0 {
                let vx = self.velocity[0];
                let vy = self.velocity[1];
                let vz = self.velocity[2];
                let speed = (vx * vx + vy * vy + vz * vz).sqrt();
                if speed > 1e-9 {
                    let scale = -k * speed;
                    force[0] += scale * vx;
                    force[1] += scale * vy;
                    force[2] += scale * vz;
                }
            }
        }

        // --- Unified ground contact: feet + body/nozzle/nose hull ---
        self.contacting = false;
        self.body_contacting = false;
        let pos = self.position();
        let probes = ground_contact_probes(&self.params);
        let band = self.params.contact_band.max(0.0);

        // Transform probes once (PGA sandwich).
        let mut world_pts = Vec::with_capacity(probes.len());
        let mut planted_flags = Vec::with_capacity(probes.len());
        let mut n_foot_planted = 0usize;
        let mut n_hull_planted = 0usize;
        for probe in &probes {
            let p = point(probe.body[0], probe.body[1], probe.body[2]);
            let w = extract_point(&self.motor.sandwich(&p));
            let planted = w[1] <= band;
            if planted {
                match probe.kind {
                    ContactKind::Foot => n_foot_planted += 1,
                    ContactKind::Hull => n_hull_planted += 1,
                }
            }
            world_pts.push(w);
            planted_flags.push(planted);
        }

        // Most negative normal (world +Y) approach speed among active contacts.
        let mut impact_vn = 0.0_f64;

        if n_foot_planted + n_hull_planted > 0 {
            let omega_w = motor_rotate_vector(&self.motor, self.omega);
            let motor_inv = self.motor.reverse().normalize_motor();
            let mu_foot = self.params.friction_mu.max(0.0);
            let mu_hull = self.params.body_friction_mu.max(0.0);
            let v_eps = self.params.friction_slip_eps.max(1e-6);
            let v_eps2 = v_eps * v_eps;
            // Hard projection leaves probes at y≈0 ⇒ n_spring→0. Without a
            // quasi-static rest normal there is no restoring torque from offset
            // feet and residual tilt freezes. Support demand is the unmet
            // downward load so hover/liftoff (F_prop_y ≥ mg) stays free.
            // Friction capacity still uses full weight share (legacy pad grip
            // while thrusting off the deck) even when support demand is zero.
            let n_planted = n_foot_planted + n_hull_planted;
            let support_demand = (-force[1]).max(0.0);
            let n_rest_support = support_demand / n_planted as f64;
            let n_rest_friction = (mass * GRAVITY) / n_planted as f64;

            for i in 0..probes.len() {
                if !planted_flags[i] {
                    continue;
                }
                let probe = probes[i];
                let world = world_pts[i];
                self.contacting = true;
                if probe.kind == ContactKind::Hull {
                    self.body_contacting = true;
                }

                let r = [world[0] - pos[0], world[1] - pos[1], world[2] - pos[2]];
                let v_pt = [
                    self.velocity[0] + omega_w[1] * r[2] - omega_w[2] * r[1],
                    self.velocity[1] + omega_w[2] * r[0] - omega_w[0] * r[2],
                    self.velocity[2] + omega_w[0] * r[1] - omega_w[1] * r[0],
                ];
                if v_pt[1] < impact_vn {
                    impact_vn = v_pt[1];
                }
                let penetration = (-world[1]).max(0.0);
                let n_spring = (self.params.contact_stiffness * penetration
                    - self.params.contact_damping * v_pt[1].min(0.0))
                .max(0.0);
                let plant = if world[1] <= 0.0 {
                    1.0
                } else if band > 0.0 {
                    (1.0 - world[1] / band).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                // Normal force/torque: spring (impact) or rest-share (settled).
                let n_normal = n_spring.max(n_rest_support * plant);
                force[1] += n_normal;

                if n_normal > 0.0 {
                    let tau_n = [-r[2] * n_normal, 0.0, r[0] * n_normal];
                    let tau_nb = motor_rotate_vector(&motor_inv, tau_n);
                    body_torque[0] += tau_nb[0];
                    body_torque[1] += tau_nb[1];
                    body_torque[2] += tau_nb[2];
                }

                let mu = match probe.kind {
                    ContactKind::Foot => mu_foot,
                    ContactKind::Hull => mu_hull,
                };
                // Friction μN uses weight rest-share so pad grip while
                // thrusting matches the pre-rest-normal model.
                let n_fric = n_spring.max(n_rest_friction * plant);

                // Regularized Coulomb friction: F = −μ N v_h / sqrt(|v_h|² + v_ε²)
                if mu > 0.0 && n_fric > 0.0 {
                    let vh_x = v_pt[0];
                    let vh_z = v_pt[2];
                    let speed2 = vh_x * vh_x + vh_z * vh_z;
                    let denom = (speed2 + v_eps2).sqrt();
                    let scale = -mu * n_fric / denom;
                    let f_fx = scale * vh_x;
                    let f_fz = scale * vh_z;
                    force[0] += f_fx;
                    force[2] += f_fz;
                    let tau_f = [r[1] * f_fz, r[2] * f_fx - r[0] * f_fz, -r[1] * f_fx];
                    let tau_fb = motor_rotate_vector(&motor_inv, tau_f);
                    body_torque[0] += tau_fb[0];
                    body_torque[1] += tau_fb[1];
                    body_torque[2] += tau_fb[2];
                }
            }
        }

        // Hard-impact destruction before bounce/restitution can mask the approach speed.
        let impact_speed = (-impact_vn).max(0.0);
        if self.contacting && impact_speed >= self.params.crash_impact_speed.max(0.0) {
            self.destroyed = true;
            self.explosion_age = 0.0;
            self.explosion_origin = pos;
            self.last_impact_speed = impact_speed;
            self.velocity = [0.0, 0.0, 0.0];
            self.omega = [0.0, 0.0, 0.0];
            self.command = ControlCommand::default();
            return;
        }

        // Linear integration (semi-implicit Euler).
        let inv_mass = 1.0 / mass;
        self.velocity[0] += force[0] * inv_mass * dt;
        self.velocity[1] += force[1] * inv_mass * dt;
        self.velocity[2] += force[2] * inv_mass * dt;

        // Bounce: if we hit the ground approaching fast, lift CoM vertical rate toward
        // v_n' ≈ −e · v_n_impact (penalty forces alone are too damped to rebound cleanly).
        let e = self.params.restitution.clamp(0.0, 1.0);
        if e > 0.0 && impact_vn < -0.5 {
            let target_up = -e * impact_vn;
            if self.velocity[1] < target_up {
                self.velocity[1] = target_up;
            }
        }

        // Angular: I α = τ − ω × (I ω) on principal axes.
        let i = self.params.inertia;
        let w = self.omega;
        let iw = [i[0] * w[0], i[1] * w[1], i[2] * w[2]];
        let w_cross_iw = [
            w[1] * iw[2] - w[2] * iw[1],
            w[2] * iw[0] - w[0] * iw[2],
            w[0] * iw[1] - w[1] * iw[0],
        ];
        self.omega[0] += (body_torque[0] - w_cross_iw[0]) / i[0] * dt;
        self.omega[1] += (body_torque[1] - w_cross_iw[1]) / i[1] * dt;
        self.omega[2] += (body_torque[2] - w_cross_iw[2]) / i[2] * dt;

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

        // Hard non-penetration for the full hull (feet + body + nozzle + nose).
        let min_y = self.lowest_probe_y();
        if min_y < -1e-4 {
            let fix = translator(0.0, -min_y, 0.0);
            self.motor = compose_motors(&fix, &self.motor);
            if self.velocity[1] < 0.0 {
                // Tunneling guard: a fast fall can cross the whole contact band in
                // one step, so the planted-probe destruction check above never sees
                // the approach speed. Destroy here from the pre-projection speed.
                let impact_speed = -self.velocity[1];
                if impact_speed >= self.params.crash_impact_speed.max(0.0) {
                    self.destroyed = true;
                    self.explosion_age = 0.0;
                    self.explosion_origin = self.position();
                    self.last_impact_speed = impact_speed;
                    self.velocity = [0.0, 0.0, 0.0];
                    self.omega = [0.0, 0.0, 0.0];
                    self.command = ControlCommand::default();
                    self.contacting = true;
                    return;
                }
                // Bounce CoM vertical speed: v' = −e v (e=0 → stop, e=1 → reverse).
                let e = self.params.restitution.clamp(0.0, 1.0);
                self.velocity[1] = -e * self.velocity[1];
            }
            self.contacting = true;
        }
    }
}

/// All ground-collision probes in the body frame: 4 feet + structural hull samples.
pub fn ground_contact_probes(params: &RocketParams) -> Vec<ContactProbe> {
    let mut probes = Vec::with_capacity(4 + HULL_RING_SEGS * 4 + 3);
    for foot in &params.leg_feet {
        probes.push(ContactProbe {
            body: *foot,
            kind: ContactKind::Foot,
        });
    }

    let r = params.body_radius;
    let hh = params.body_half_height;
    let exit_y = params.nozzle_exit_y();
    let exit_r = r * 0.95;
    let upper_y = hh * 0.7;
    // Rings: body bottom, mid, upper cylinder, nozzle exit.
    let rings = [(-hh, r), (0.0, r), (upper_y, r), (exit_y, exit_r)];
    for &(y, rad) in &rings {
        for i in 0..HULL_RING_SEGS {
            let a = (i as f64) * std::f64::consts::TAU / HULL_RING_SEGS as f64;
            let (s, c) = a.sin_cos();
            probes.push(ContactProbe {
                body: [c * rad, y, s * rad],
                kind: ContactKind::Hull,
            });
        }
    }
    // Axis samples: nose tip, body bottom center, nozzle center.
    probes.push(ContactProbe {
        body: [0.0, hh, 0.0],
        kind: ContactKind::Hull,
    });
    probes.push(ContactProbe {
        body: [0.0, -hh, 0.0],
        kind: ContactKind::Hull,
    });
    probes.push(ContactProbe {
        body: [0.0, exit_y, 0.0],
        kind: ContactKind::Hull,
    });
    probes
}

/// Public step API used by UI and tests.
pub fn step_rocket(state: &mut RocketState, dt: f64) {
    state.step(dt);
}

// --- Propulsion / PGA helpers ---

/// Cross product a × b.
#[inline]
pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Body-frame force at body-frame point → (force, torque = r × F).
#[inline]
pub fn body_wrench_at(r_body: [f64; 3], f_body: [f64; 3]) -> BodyWrench {
    BodyWrench {
        force: f_body,
        torque: cross(r_body, f_body),
    }
}

/// PGA rotor for nozzle gimbal: yaw-about-Z ∘ pitch-about-X.
///
/// Used for mesh / visualization. Physics uses the closed form [`thrust_dir_body`].
pub fn gimbal_rotor(pitch: f64, yaw: f64) -> Multivector {
    if pitch.abs() < 1e-18 && yaw.abs() < 1e-18 {
        return Multivector::one();
    }
    if yaw.abs() < 1e-18 {
        return rotor(1.0, 0.0, 0.0, pitch);
    }
    if pitch.abs() < 1e-18 {
        return rotor(0.0, 0.0, 1.0, yaw);
    }
    let r_pitch = rotor(1.0, 0.0, 0.0, pitch);
    let r_yaw = rotor(0.0, 0.0, 1.0, yaw);
    r_yaw.geo(&r_pitch)
}

/// Closed form of sandwiching ŷ through [`gimbal_rotor`]: pitch-about-X then yaw-about-Z.
///
/// Result: `(-cos(p)·sin(y), cos(p)·cos(y), sin(p))`.
#[inline]
pub fn thrust_dir_body(pitch: f64, yaw: f64) -> [f64; 3] {
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [-cp * sy, cp * cy, sp]
}

/// Rotate a free vector by a pure PGA rotor about the origin (one sandwich).
pub fn rotate_vector_by_rotor(r: &Multivector, v: [f64; 3]) -> [f64; 3] {
    // Pure rotors fix the origin, so sandwiching the point at `v` yields R(v) directly.
    extract_point(&r.sandwich(&point(v[0], v[1], v[2])))
}

/// Main engine wrench in body frame from throttle + gimbal commands.
pub fn engine_wrench(params: &RocketParams, cmd: ControlCommand) -> BodyWrench {
    let thrust = cmd.throttle.clamp(0.0, 1.0) * params.max_thrust;
    if thrust <= 0.0 {
        return BodyWrench::default();
    }
    let pitch = cmd.pitch.clamp(-1.0, 1.0) * params.max_gimbal_angle;
    let yaw = cmd.yaw.clamp(-1.0, 1.0) * params.max_gimbal_angle;
    let dir = thrust_dir_body(pitch, yaw);
    let f_body = [dir[0] * thrust, dir[1] * thrust, dir[2] * thrust];
    // Application at nozzle exit: r = (0, r_y, 0) ⇒ τ = (r_y Fz, 0, −r_y Fx).
    let ry = params.nozzle_exit_y();
    BodyWrench {
        force: f_body,
        torque: [ry * f_body[2], 0.0, -ry * f_body[0]],
    }
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
    if roll.abs() < 1e-18 {
        let r = params.rcs_radius;
        let y = params.rcs_y;
        return [
            ThrusterSample {
                position_body: [r, y, 0.0],
                force_body: [0.0, 0.0, 0.0],
            },
            ThrusterSample {
                position_body: [-r, y, 0.0],
                force_body: [0.0, 0.0, 0.0],
            },
            ThrusterSample {
                position_body: [0.0, y, r],
                force_body: [0.0, 0.0, 0.0],
            },
            ThrusterSample {
                position_body: [0.0, y, -r],
                force_body: [0.0, 0.0, 0.0],
            },
        ];
    }
    let f = roll.abs() * params.rcs_thrust;
    let s = roll.signum();
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

/// Summed body-frame wrench from roll thrusters (closed form: pure couple about +Y).
///
/// For the layout in [`roll_thrusters`], ΣF = 0 and τ = (0, 4 R s F, 0).
pub fn rcs_wrench(params: &RocketParams, roll_cmd: f64) -> BodyWrench {
    let roll = roll_cmd.clamp(-1.0, 1.0);
    if roll.abs() < 1e-18 {
        return BodyWrench::default();
    }
    let f = roll.abs() * params.rcs_thrust;
    let s = roll.signum();
    BodyWrench {
        force: [0.0, 0.0, 0.0],
        torque: [0.0, 4.0 * params.rcs_radius * s * f, 0.0],
    }
}

/// Combined engine + RCS wrench for a command.
#[inline]
pub fn propulsive_wrench(params: &RocketParams, cmd: ControlCommand) -> BodyWrench {
    engine_wrench(params, cmd) + rcs_wrench(params, cmd.roll)
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
        let ry = p.nozzle_exit_y();
        let delta = p.max_gimbal_angle;
        // Pitch about +X: thrust gains +Z component = T sin δ; τ_x = r_y * F_z.
        let expected_tau_x = ry * t * delta.sin();
        assert!(
            (w.torque[0] - expected_tau_x).abs() < 1e-9,
            "τ_x={} expected≈{}",
            w.torque[0],
            expected_tau_x
        );
        assert!(w.torque[1].abs() < 1e-12);
        assert!(w.torque[2].abs() < 1e-9);
    }

    #[test]
    fn thrust_dir_matches_pga_gimbal_rotor() {
        let cases = [(0.0, 0.0), (0.12, 0.0), (0.0, -0.08), (0.1, 0.07), (-0.05, 0.11)];
        for (p, y) in cases {
            let closed = thrust_dir_body(p, y);
            let pga = rotate_vector_by_rotor(&gimbal_rotor(p, y), [0.0, 1.0, 0.0]);
            for i in 0..3 {
                assert!(
                    (closed[i] - pga[i]).abs() < 1e-12,
                    "dir mismatch at ({p},{y}): closed={closed:?} pga={pga:?}"
                );
            }
        }
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

        // Closed form matches explicit sum of thruster wrenches.
        let mut summed = BodyWrench::default();
        for t in roll_thrusters(&p, 1.0) {
            summed.accum(body_wrench_at(t.position_body, t.force_body));
        }
        assert!((summed.torque[1] - w.torque[1]).abs() < 1e-9);
        assert!(summed.force.iter().all(|c| c.abs() < 1e-9));
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
        assert!(w.force.iter().all(|c| c.abs() < 1e-12));
        assert!(w.torque.iter().all(|c| c.abs() < 1e-12));
    }
}
