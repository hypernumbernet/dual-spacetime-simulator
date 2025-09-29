use nalgebra::{Matrix4, Point3, Quaternion, RealField, UnitQuaternion, Vector3, base::Scalar};

const EPSILON: f64 = 1e-6;

pub struct OrbitCamera {
    pub eye: Point3<f64>,
    pub target: Point3<f64>,
    pub up: Vector3<f64>,
    //right: Vector3<f64>,
    //rotation: UnitQuaternion<f64>,
    distance: f64,
}

impl OrbitCamera {
    pub fn new(eye: Point3<f64>, target: Point3<f64>) -> Self {
        let up = find_closest_perp_unit(eye, target);
        //dbg!(up, up.norm());
        let relative = target - eye;
        let distance = relative.norm();
        //let forward = relative.normalize();
        //let initial_rotation = UnitQuaternion::from_rotation_arc(Vec3::Z, forward);

        Self {
            eye,
            target,
            up,
            //right: Vec3::X,
            //rotation: initial_rotation,
            distance,
        }
    }

    pub fn view_matrix(&self) -> Matrix4<f64> {
        Matrix4::look_at_rh(&self.eye, &self.target, &self.up)
    }

    pub fn projection_matrix(&self, aspect: f64, fovy: f64, near: f64, far: f64) -> Matrix4<f64> {
        Matrix4::new_perspective(aspect, fovy, near, far)
    }

    pub fn mvp_matrix(&self, aspect: f64, fovy: f64, near: f64, far: f64) -> Matrix4<f64> {
        let view = self.view_matrix();
        let proj = self.projection_matrix(aspect, fovy, near, far);
        proj * view
    }

    pub fn rotate(&mut self, delta_yaw: f64, delta_pitch: f64) {
        if self.distance <= std::f64::EPSILON {
            return;
        }
        // if delta_pitch > std::f64::EPSILON {
        //     let rotation = Quat::from_rotation_z(delta_pitch);
        //     self.up = rotation * self.up;
        // }
        // self.up = self.rotation.mul_vec3(Vec3::Y);
        // self.right = self.up.cross(self.target - self.position).normalize();
        // let yaw_quat = Quat::from_axis_angle(Vec3::Y, delta_yaw);
        // let pitch_quat = Quat::from_axis_angle(self.right, delta_pitch);
        // self.rotation = yaw_quat * self.rotation * pitch_quat;
        // let initial_forward = Vec3::Z;
        // let rotated_forward = self.rotation.mul_vec3(initial_forward);
        // self.position = self.target + rotated_forward * self.distance;
        // self.up = self.rotation.mul_vec3(Vec3::Y);

        // // Yaw: グローバルY軸周りの回転（全体回転）
        // let yaw_quat = Quat::from_axis_angle(Vec3::Y, delta_yaw);

        // // Pitch: ローカルright軸周りの回転
        // let forward = self.rotation.mul_vec3(Vec3::Z);  // 現在のforward (サンドイッチ積で計算)
        // let right = self.up.cross(forward).normalize();
        // let pitch_quat = Quat::from_axis_angle(right, delta_pitch);

        // // 回転を合成（yawを左、pitchを右に）
        // self.rotation = yaw_quat * self.rotation * pitch_quat;

        // // position更新: forwardをサンドイッチ積で回転させたベクターから計算
        // let initial_forward = Vec3::Z;  // 基準forward
        // let rotated_forward = self.rotation.mul_vec3(initial_forward);  // q * v * q^{-1}
        // self.position = self.target + rotated_forward * self.distance;

        // // up更新: 同様にサンドイッチ積
        // let initial_up = Vec3::Y;
        // self.up = self.rotation.mul_vec3(initial_up);
    }
}

fn find_closest_perp_unit(eye: Point3<f64>, target: Point3<f64>) -> Vector3<f64> {
    let direction = target - eye;
    let dir_norm_sq = direction.norm_squared();
    let y_axis = Vector3::new(0.0, 1.0, 0.0);
    if dir_norm_sq == 0.0 {
        return y_axis;
    }
    let dot = y_axis.dot(&direction);
    let proj = y_axis - direction * (dot / dir_norm_sq);
    let proj_norm_sq = proj.norm_squared();
    if proj_norm_sq == 0.0 {
        return Vector3::new(1.0, 0.0, 0.0);
    }
    proj.normalize()
}
