use glam::{Quat, Vec3};

pub struct OrbitCamera {
    pub position: Vec3,
    pub target: Vec3,
    pub up: Vec3,
    right: Vec3,
    rotation: Quat,
    distance: f32,
}

impl OrbitCamera {
    pub fn new(position: Vec3, target: Vec3) -> Self {
        let relative = target - position;
        let distance = relative.length();
        let forward = relative.normalize();
        let initial_rotation = Quat::from_rotation_arc(Vec3::Z, forward);

        Self {
            position,
            target,
            up: Vec3::Y,
            right: Vec3::X,
            rotation: initial_rotation,
            distance,
        }
    }

    pub fn rotate(&mut self, delta_pitch: f32, delta_yaw: f32) {
        if self.distance <= std::f32::EPSILON {
            return;
        }

        // Yaw: グローバルY軸周りの回転（全体回転）
        let yaw_quat = Quat::from_axis_angle(Vec3::Y, delta_yaw);

        // Pitch: ローカルright軸周りの回転
        let forward = self.rotation.mul_vec3(Vec3::Z);  // 現在のforward (サンドイッチ積で計算)
        let right = self.up.cross(forward).normalize();
        let pitch_quat = Quat::from_axis_angle(right, delta_pitch);

        // 回転を合成（yawを左、pitchを右に）
        self.rotation = yaw_quat * self.rotation * pitch_quat;

        // position更新: forwardをサンドイッチ積で回転させたベクターから計算
        let initial_forward = Vec3::Z;  // 基準forward
        let rotated_forward = self.rotation.mul_vec3(initial_forward);  // q * v * q^{-1}
        self.position = self.target + rotated_forward * self.distance;

        // up更新: 同様にサンドイッチ積
        let initial_up = Vec3::Y;
        self.up = self.rotation.mul_vec3(initial_up);
    }
}