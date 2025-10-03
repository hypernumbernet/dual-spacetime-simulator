use glam::{Quat, Vec3};
use std::f32::EPSILON;
use std::time::Instant;

const ANIMATION_DURATION: f32 = 0.008;

pub struct OrbitCamera {
    pub position: Vec3,
    pub target: Vec3,
    pub up: Vec3,
    animating_y_top: u32,
    animating_to_origin: u32,
    start_time: Option<Instant>,
}

impl OrbitCamera {
    pub fn new(position: Vec3, target: Vec3) -> Self {
        let up = get_closest_perp_unit_to_y(position, target);
        Self {
            position,
            target,
            up,
            animating_y_top: 0,
            animating_to_origin: 0,
            start_time: None,
        }
    }

    pub fn revolve(&mut self, delta_yaw: f32, delta_pitch: f32) {
        let relative = self.target - self.position;
        if relative.length_squared() <= EPSILON {
            return;
        }
        let axis = self.up.cross(relative).normalize();
        let rotation = Quat::from_axis_angle(axis, -delta_pitch);
        self.up = rotation.mul_vec3(self.up);
        let relative = rotation.mul_vec3(relative);
        self.position = self.target - relative;

        let rotation = Quat::from_axis_angle(self.up, -delta_yaw);
        let relative = rotation.mul_vec3(relative);
        self.position = self.target - relative;
    }

    pub fn look_around(&mut self, dx: f32, dy: f32) {
        let relative = self.target - self.position;
        if relative.length_squared() <= EPSILON {
            return;
        }
        let rotation = Quat::from_axis_angle(self.up, dx);
        let relative = rotation.mul_vec3(relative);
        self.target = self.position + relative;

        let axis = self.up.cross(relative).normalize();
        let rotation = Quat::from_axis_angle(axis, dy);
        let relative = rotation.mul_vec3(relative);
        self.target = self.position + relative;
        self.up = rotation.mul_vec3(self.up);
    }

    pub fn zoom(&mut self, zoom_factor: f32) {
        let direction = (self.target - self.position).normalize_or_zero();
        if direction == Vec3::ZERO {
            return;
        }
        let distance = (self.target - self.position).length();
        let new_distance = (distance - zoom_factor).max(0.1);
        self.position = self.target - direction * new_distance;
    }

    pub fn rotate(&mut self, delta_roll: f32) {
        let relative = self.target - self.position;
        if relative.length_squared() <= EPSILON {
            return;
        }
        let rotation = Quat::from_axis_angle(relative.normalize(), delta_roll);
        self.up = rotation.mul_vec3(self.up);
    }

    pub fn y_top(&mut self) {
        self.animating_y_top = 100;
        self.start_time = Some(Instant::now());
    }

    pub fn center_target_on_origin(&mut self) {
        self.animating_to_origin = 100;
        self.start_time = Some(Instant::now());
    }

    pub fn update_animation(&mut self) {
        if let Some(start) = self.start_time {
            let dt = start.elapsed().as_secs_f32();
            if dt >= ANIMATION_DURATION {
                if self.animating_to_origin > 0 {
                    if let Some(end) = get_up_center_origin(self.position, self.target, self.up) {
                        self.up = self.up.slerp(end.0, 0.15).normalize();
                        self.target = self.target * 0.85 + Vec3::ZERO * 0.15;
                        self.animating_to_origin -= 1;
                        self.start_time = Some(Instant::now());
                    } else {
                        self.animating_to_origin = 0;
                    }
                } else if self.animating_y_top > 0 {
                    let end = get_closest_perp_unit_to_y(self.position, self.target);
                    if self.up.abs_diff_eq(end, 0.01) {
                        self.animating_y_top = 0;
                        self.up = self.up.slerp(end, 1.0).normalize();
                        return;
                    }
                    self.up = self.up.slerp(end, 0.15).normalize();
                    self.animating_y_top -= 1;
                    self.start_time = Some(Instant::now());
                } else {
                    self.start_time = None;
                }
            }
        }
    }
}

fn get_closest_perp_unit_to_y(position: Vec3, target: Vec3) -> Vec3 {
    let dir = (target - position).normalize_or_zero();
    if dir == Vec3::ZERO {
        return Vec3::Y;
    }
    let y = Vec3::Y;
    let proj = dir * dir.dot(y);
    let perp = y - proj;
    let perp_len = perp.length();
    if perp_len > EPSILON {
        perp / perp_len
    } else {
        let mut v = Vec3::X.cross(dir);
        if v.length_squared() < EPSILON {
            v = Vec3::Z.cross(dir);
        }
        v.normalize()
    }
}

fn get_up_center_origin(position: Vec3, target: Vec3, up: Vec3) -> Option<(Vec3, Vec3)> {
    let relative = target - position;
    if relative.length_squared() <= EPSILON {
        return None;
    }
    let new_relative = Vec3::ZERO - position;
    if new_relative.length_squared() <= EPSILON {
        return None;
    }
    let rel_n = relative.normalize();
    let new_n = new_relative.normalize();
    let dot = (rel_n.dot(new_n)).clamp(-1.0, 1.0);
    if dot > 1.0 - EPSILON {
        return None;
    }
    let mut axis = rel_n.cross(new_n);
    if axis.length_squared() < EPSILON {
        if dot < 0.0 {
            axis = rel_n.cross(Vec3::X);
            if axis.length_squared() < EPSILON {
                axis = rel_n.cross(Vec3::Z);
            }
        } else {
            return None;
        }
    }
    let axis = axis.normalize();
    let angle = dot.acos();
    let rotation = Quat::from_axis_angle(axis, angle);
    Some((rotation.mul_vec3(up), new_relative))
}
