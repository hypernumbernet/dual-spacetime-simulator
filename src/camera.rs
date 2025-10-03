use glam::{Quat, Vec3};

const EPSILON: f32 = 1e-6;

pub struct OrbitCamera {
    pub position: Vec3,
    pub target: Vec3,
    pub up: Vec3,
}

impl OrbitCamera {
    pub fn new(position: Vec3, target: Vec3) -> Self {
        let up = get_closest_perp_unit_to_y(position, target);
        Self {
            position,
            target,
            up,
        }
    }

    pub fn revolve(&mut self, delta_yaw: f32, delta_pitch: f32) {
        let relative = self.target - self.position;
        if relative.length_squared() <= std::f32::EPSILON {
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
        if relative.length_squared() <= std::f32::EPSILON {
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
        if relative.length_squared() <= std::f32::EPSILON {
            return;
        }
        let rotation = Quat::from_axis_angle(relative.normalize(), delta_roll);
        self.up = rotation.mul_vec3(self.up);
    }

    pub fn y_top(&mut self) {
        self.up = get_closest_perp_unit_to_y(self.position, self.target);
    }

    pub fn center_target_on_origin(&mut self) {
        let relative = self.target - self.position;
        if relative.length_squared() <= std::f32::EPSILON {
            return;
        }
        let new_relative = Vec3::ZERO - self.position;
        if new_relative.length_squared() <= std::f32::EPSILON {
            return;
        }
        let rel_n = relative.normalize();
        let new_n = new_relative.normalize();
        let dot = (rel_n.dot(new_n)).clamp(-1.0, 1.0);
        if dot > 1.0 - EPSILON {
            return;
        }
        let mut axis = rel_n.cross(new_n);
        if axis.length_squared() < EPSILON {
            if dot < 0.0 {
                axis = rel_n.cross(Vec3::X);
                if axis.length_squared() < EPSILON {
                    axis = rel_n.cross(Vec3::Z);
                }
            } else {
                return;
            }
        }
        let axis = axis.normalize();
        let angle = dot.acos();
        let rotation = Quat::from_axis_angle(axis, angle);
        self.up = rotation.mul_vec3(self.up);
        self.target = Vec3::ZERO;
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
