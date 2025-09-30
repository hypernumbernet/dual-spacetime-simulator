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
        let distance = relative.length();
        if distance <= std::f32::EPSILON {
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
        let distance = relative.length();
        if distance <= std::f32::EPSILON {
            return;
        }
        let rotation = Quat::from_axis_angle(self.up, dx);
        let relative = rotation.mul_vec3(relative);
        self.target = self.position + relative;

        let axis = self.up.cross(relative).normalize();
        let rotation = Quat::from_axis_angle(axis, dy);
        let relative = rotation.mul_vec3(relative);
        self.target = self.position + relative;
    }

    pub fn zoom(&mut self, zoom_factor: f32) {
        let direction = (self.target - self.position).normalize_or_zero();
        if direction.length_squared() == 0.0 {
            return;
        }
        let distance = (self.target - self.position).length();
        let new_distance = (distance - zoom_factor).max(0.1);
        self.position = self.target - direction * new_distance;
    }

    pub fn rotate(&mut self, delta_roll: f32) {
        let relative = self.target - self.position;
        let distance = relative.length();
        if distance <= std::f32::EPSILON {
            return;
        }
        let rotation = Quat::from_axis_angle(relative.normalize(), delta_roll);
        self.up = rotation.mul_vec3(self.up);
    }
}

fn get_closest_perp_unit_to_y(position: Vec3, target: Vec3) -> Vec3 {
    let dir = (target - position).normalize_or_zero();
    if dir.length_squared() == 0.0 {
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
