//! First-person player: position/velocity/look angles, physics, AABB collision, fly toggle.

use crate::chunk::WORLD_SCALE;
use crate::input::InputState;
use crate::world::World;
use crate::worldgen::{surface_height, SEA_LEVEL};
use glam::{IVec3, Mat4, Vec3};
use winit::keyboard::KeyCode;

pub const HALF_W: f32 = 0.3;
pub const HEIGHT: f32 = 1.8;
pub const EYE: f32 = 1.62;
pub const WALK_SPEED: f32 = 4.3;
pub const FLY_SPEED: f32 = 12.0;
pub const FLY_BOOST: f32 = 3.0;
pub const GRAVITY: f32 = -28.0;
pub const JUMP_VEL: f32 = 8.5;
pub const MAX_FALL: f32 = 30.0;
const MOUSE_SENS: f32 = 0.0022;
const PITCH_LIMIT: f32 = 1.553; // ~89 degrees

pub struct Player {
    pub pos: Vec3, // feet center
    pub vel: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub on_ground: bool,
    pub flying: bool,
}

impl Player {
    /// Spawns the player flying high above dry land for a scenic vista, gazing down.
    /// At large WORLD_SCALE the origin may sit mid-ocean, so scan east for a column
    /// above the beach band (surface_height is pure — no chunks needed).
    pub fn spawn() -> Self {
        let beach = SEA_LEVEL + (2.0 * WORLD_SCALE) as i32;
        let mut wx = 8;
        for k in 0..4096 {
            let x = 8 + k * 24;
            if surface_height(x, 8) > beach {
                wx = x;
                break;
            }
        }
        let h = surface_height(wx, 8) as f32;
        Self {
            pos: Vec3::new(wx as f32 + 0.5, h + 28.0, 8.5),
            vel: Vec3::ZERO,
            yaw: 0.6,
            pitch: -0.35,
            on_ground: false,
            flying: true,
        }
    }

    /// Right and up basis vectors (for sky ray reconstruction).
    pub fn right(&self) -> Vec3 {
        self.forward().cross(Vec3::Y).normalize_or_zero()
    }

    pub fn up_basis(&self) -> Vec3 {
        self.right().cross(self.forward()).normalize_or_zero()
    }

    /// Eye position used as the camera origin.
    pub fn eye(&self) -> Vec3 {
        self.pos + Vec3::new(0.0, EYE, 0.0)
    }

    /// Forward look direction from yaw/pitch.
    pub fn forward(&self) -> Vec3 {
        Vec3::new(
            self.pitch.cos() * self.yaw.sin(),
            self.pitch.sin(),
            -self.pitch.cos() * self.yaw.cos(),
        )
        .normalize()
    }

    /// Applies accumulated mouse delta to yaw/pitch.
    pub fn apply_mouse(&mut self, dx: f64, dy: f64) {
        self.yaw += dx as f32 * MOUSE_SENS;
        self.pitch = (self.pitch - dy as f32 * MOUSE_SENS).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    }

    /// View-projection matrix (right-handed, Vulkan [0,1] depth, Y flipped for framebuffer).
    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        let eye = self.eye();
        let view = Mat4::look_to_rh(eye, self.forward(), Vec3::Y);
        // Far plane covers looking down from tall mountains (terrain spans ~1500 blocks
        // vertically at WORLD_SCALE 10); D32 depth keeps precision fine at this range.
        let mut proj = Mat4::perspective_rh(70.0_f32.to_radians(), aspect, 0.1, 4096.0);
        proj.y_axis.y *= -1.0;
        proj * view
    }

    /// Advances physics by `dt` seconds (clamp dt before calling to avoid tunneling).
    pub fn update(&mut self, input: &InputState, world: &World, dt: f32) {
        // Horizontal wish direction from WASD, on the XZ plane relative to yaw.
        let fwd = Vec3::new(self.yaw.sin(), 0.0, -self.yaw.cos());
        let right = Vec3::new(self.yaw.cos(), 0.0, self.yaw.sin());
        let mut wish = Vec3::ZERO;
        if input.held(KeyCode::KeyW) {
            wish += fwd;
        }
        if input.held(KeyCode::KeyS) {
            wish -= fwd;
        }
        if input.held(KeyCode::KeyD) {
            wish += right;
        }
        if input.held(KeyCode::KeyA) {
            wish -= right;
        }
        if wish.length_squared() > 0.0 {
            wish = wish.normalize();
        }

        if input.just_pressed(KeyCode::KeyF) {
            self.flying = !self.flying;
            self.vel.y = 0.0;
        }

        if self.flying {
            let speed = if input.held(KeyCode::ControlLeft) {
                FLY_SPEED * FLY_BOOST
            } else {
                FLY_SPEED
            };
            self.vel.x = wish.x * speed;
            self.vel.z = wish.z * speed;
            let mut vy = 0.0;
            if input.held(KeyCode::Space) {
                vy += speed;
            }
            if input.held(KeyCode::ShiftLeft) {
                vy -= speed;
            }
            self.vel.y = vy;
        } else {
            // Swimming: buoyant when the torso is submerged — weak gravity, slow sink,
            // Space paddles upward, and horizontal drag.
            let torso = self.pos + Vec3::new(0.0, 0.9, 0.0);
            let in_water = world
                .block(IVec3::new(
                    torso.x.floor() as i32,
                    torso.y.floor() as i32,
                    torso.z.floor() as i32,
                ))
                .is_water();
            if in_water {
                self.vel.x = wish.x * WALK_SPEED * 0.6;
                self.vel.z = wish.z * WALK_SPEED * 0.6;
                self.vel.y += GRAVITY * 0.25 * dt;
                self.vel.y = self.vel.y.max(-4.0);
                if input.held(KeyCode::Space) {
                    self.vel.y = 4.5;
                }
            } else {
                self.vel.x = wish.x * WALK_SPEED;
                self.vel.z = wish.z * WALK_SPEED;
                self.vel.y += GRAVITY * dt;
                self.vel.y = self.vel.y.clamp(-MAX_FALL, MAX_FALL);
                if self.on_ground && input.held(KeyCode::Space) {
                    self.vel.y = JUMP_VEL;
                }
            }
        }

        // Axis-separated collision resolution: X, then Z, then Y (Y sets on_ground).
        self.move_axis(world, 0, self.vel.x * dt);
        self.move_axis(world, 2, self.vel.z * dt);
        self.on_ground = false;
        self.move_axis(world, 1, self.vel.y * dt);
    }

    /// Moves along one axis and resolves penetration against solid voxels.
    fn move_axis(&mut self, world: &World, axis: usize, delta: f32) {
        if delta == 0.0 {
            return;
        }
        self.pos[axis] += delta;

        let min = self.pos - Vec3::new(HALF_W, 0.0, HALF_W);
        let max = self.pos + Vec3::new(HALF_W, HEIGHT, HALF_W);

        let bx0 = min.x.floor() as i32;
        let bx1 = (max.x - 1e-4).floor() as i32;
        let by0 = min.y.floor() as i32;
        let by1 = (max.y - 1e-4).floor() as i32;
        let bz0 = min.z.floor() as i32;
        let bz1 = (max.z - 1e-4).floor() as i32;

        for bx in bx0..=bx1 {
            for by in by0..=by1 {
                for bz in bz0..=bz1 {
                    if !world.solid_for_collision(IVec3::new(bx, by, bz)) {
                        continue;
                    }
                    match axis {
                        0 => {
                            if delta > 0.0 {
                                self.pos.x = bx as f32 - HALF_W - 1e-4;
                            } else {
                                self.pos.x = bx as f32 + 1.0 + HALF_W + 1e-4;
                            }
                            self.vel.x = 0.0;
                        }
                        2 => {
                            if delta > 0.0 {
                                self.pos.z = bz as f32 - HALF_W - 1e-4;
                            } else {
                                self.pos.z = bz as f32 + 1.0 + HALF_W + 1e-4;
                            }
                            self.vel.z = 0.0;
                        }
                        _ => {
                            if delta > 0.0 {
                                self.pos.y = by as f32 - HEIGHT - 1e-4;
                            } else {
                                self.pos.y = by as f32 + 1.0 + 1e-4;
                                self.on_ground = true;
                            }
                            self.vel.y = 0.0;
                        }
                    }
                    return; // one clamp per axis is sufficient (shared face plane)
                }
            }
        }
    }
}
