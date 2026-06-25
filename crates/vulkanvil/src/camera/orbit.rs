use glam::{Quat, Vec3};
use std::f32::EPSILON;
use std::time::Instant;

const ANIMATION_DURATION: f32 = 0.004;
const ORIGIN_CENTER_STEP: f32 = 0.30;
const ORIGIN_CENTER_MAX_STEPS: u32 = 40;
/// Snap and stop when target is within this distance of the origin.
const ORIGIN_CENTER_TARGET_EPS: f32 = 0.05;
/// Snap and stop when view direction dot goal direction reaches this value.
const ORIGIN_CENTER_VIEW_DOT_MIN: f32 = 0.992;
const MAX_PITCH_RAD: f32 = 87.0_f32 * std::f32::consts::PI / 180.0_f32;

pub struct OrbitCamera {
    pub position: Vec3,
    pub target: Vec3,
    pub up: Vec3,
    lock_up: bool,
    animating_y_top: u32,
    animating_to_origin: u32,
    start_time: Option<Instant>,
}

impl OrbitCamera {
    /// Creates an orbit camera with an up vector consistent with the current view direction.
    pub fn new(position: Vec3, target: Vec3) -> Self {
        let up = get_closest_perp_unit_to_y(position, target);
        Self {
            position,
            target,
            up,
            lock_up: false,
            animating_y_top: 0,
            animating_to_origin: 0,
            start_time: None,
        }
    }

    /// Orbits the camera around the target using yaw and pitch deltas.
    pub fn revolve(&mut self, delta_yaw: f32, delta_pitch: f32) {
        let mut relative = self.target - self.position;
        if relative.length_squared() <= EPSILON {
            return;
        }
        let axis = self.up.cross(relative).normalize();
        let rotation = Quat::from_axis_angle(axis, -delta_pitch);
        self.up = rotation.mul_vec3(self.up);
        relative = rotation.mul_vec3(relative);

        let rotation = Quat::from_axis_angle(self.up, -delta_yaw);
        relative = rotation.mul_vec3(relative);
        if self.lock_up {
            relative = clamp_pitch(relative);
        }
        self.position = self.target - relative;
        if self.lock_up {
            self.up = get_closest_perp_unit_to_y(self.position, self.target);
        }
    }

    /// Rotates the viewing direction in place while keeping the camera position fixed.
    pub fn look_around(&mut self, dx: f32, dy: f32) {
        let mut relative = self.target - self.position;
        if relative.length_squared() <= EPSILON {
            return;
        }
        let rotation = Quat::from_axis_angle(self.up, dx);
        relative = rotation.mul_vec3(relative);
        self.target = self.position + relative;

        let axis = self.up.cross(relative).normalize();
        let rotation = Quat::from_axis_angle(axis, dy);
        relative = rotation.mul_vec3(relative);
        if self.lock_up {
            relative = clamp_pitch(relative);
        }
        self.target = self.position + relative;
        self.up = rotation.mul_vec3(self.up);
        if self.lock_up {
            self.up = get_closest_perp_unit_to_y(self.position, self.target);
        }
    }

    /// Translates target and position together on the XZ plane, preserving orbit distance.
    pub fn pan_xz(&mut self, delta: Vec3) {
        let offset = Vec3::new(delta.x, 0.0, delta.z);
        if offset.length_squared() <= EPSILON {
            return;
        }
        self.target += offset;
        self.position += offset;
    }

    /// Yaws the camera around the target while keeping the target fixed.
    pub fn orbit_yaw(&mut self, delta_yaw: f32) {
        self.revolve(delta_yaw, 0.0);
    }

    /// Moves the viewpoint along the world Y axis while keeping the target fixed.
    pub fn move_position_y(&mut self, delta_y: f32) {
        if delta_y.abs() <= EPSILON {
            return;
        }
        self.position.y += delta_y;
        if self.lock_up {
            self.up = get_closest_perp_unit_to_y(self.position, self.target);
        }
    }

    /// Moves the rotation center along the world Y axis while keeping the position fixed.
    pub fn move_target_y(&mut self, delta_y: f32) {
        if delta_y.abs() <= EPSILON {
            return;
        }
        self.target.y += delta_y;
        if self.lock_up {
            self.up = get_closest_perp_unit_to_y(self.position, self.target);
        }
    }

    /// Moves the rotation center around the vertical axis through the viewpoint.
    pub fn move_target_around_position_y(&mut self, delta_yaw: f32) {
        if delta_yaw.abs() <= EPSILON {
            return;
        }
        let mut relative = self.target - self.position;
        if relative.length_squared() <= EPSILON {
            return;
        }
        let rotation = Quat::from_axis_angle(Vec3::Y, delta_yaw);
        relative = rotation.mul_vec3(relative);
        self.target = self.position + relative;
        if self.lock_up {
            self.up = get_closest_perp_unit_to_y(self.position, self.target);
        }
    }

    /// Translates position and target together along the view direction.
    pub fn move_forward(&mut self, delta: f32) {
        let direction = (self.target - self.position).normalize_or_zero();
        if direction == Vec3::ZERO || delta.abs() <= EPSILON {
            return;
        }
        let offset = direction * delta;
        self.position += offset;
        self.target += offset;
    }

    /// Moves the camera toward or away from the target while preserving view direction.
    pub fn zoom(&mut self, zoom_factor: f32) {
        let direction = (self.target - self.position).normalize_or_zero();
        if direction == Vec3::ZERO {
            return;
        }
        let distance = (self.target - self.position).length();
        let new_distance = (distance - zoom_factor).max(0.1);
        self.position = self.target - direction * new_distance;
    }

    /// Rolls the camera around the forward axis when up-lock is disabled.
    pub fn rotate(&mut self, delta_roll: f32) {
        if self.lock_up {
            return;
        }
        let relative = self.target - self.position;
        if relative.length_squared() <= EPSILON {
            return;
        }
        let rotation = Quat::from_axis_angle(relative.normalize(), delta_roll);
        self.up = rotation.mul_vec3(self.up);
    }

    /// Starts a short animation that aligns the camera up vector toward world-up.
    pub fn y_top(&mut self) {
        self.animating_y_top = 100;
        self.start_time = Some(Instant::now());
    }

    /// Starts a short animation that shifts the camera target toward the world origin.
    pub fn center_target_on_origin(&mut self) {
        self.animating_to_origin = ORIGIN_CENTER_MAX_STEPS;
        self.start_time = Some(Instant::now());
    }

    /// Returns whether a camera alignment or recentering animation is still running.
    pub fn is_animating(&self) -> bool {
        self.animating_y_top > 0 || self.animating_to_origin > 0
    }

    /// Advances camera alignment and recentering animations based on elapsed time.
    pub fn update_animation(&mut self) {
        if !self.is_animating() {
            self.start_time = None;
            return;
        }
        if let Some(start) = self.start_time {
            let dt = start.elapsed().as_secs_f32();
            if dt >= ANIMATION_DURATION {
                if self.animating_to_origin > 0 {
                    if self.lock_up {
                        if lock_up_origin_center_complete(self.position, self.target) {
                            if let Some((new_position, new_target, new_up)) =
                                snap_lock_up_center_on_origin(self.position, self.target)
                            {
                                self.position = new_position;
                                self.target = new_target;
                                self.up = new_up;
                            }
                            self.animating_to_origin = 0;
                        } else if let Some((new_position, new_target, new_up)) =
                            step_lock_up_center_on_origin(self.position, self.target)
                        {
                            self.position = new_position;
                            self.target = new_target;
                            self.up = new_up;
                            if lock_up_origin_center_complete(self.position, self.target) {
                                if let Some((new_position, new_target, new_up)) =
                                    snap_lock_up_center_on_origin(self.position, self.target)
                                {
                                    self.position = new_position;
                                    self.target = new_target;
                                    self.up = new_up;
                                }
                                self.animating_to_origin = 0;
                            } else {
                                self.animating_to_origin -= 1;
                                self.start_time = Some(Instant::now());
                            }
                        } else {
                            self.animating_to_origin = 0;
                        }
                    } else if origin_center_target_reached(self.target) {
                        self.target = Vec3::ZERO;
                        self.animating_to_origin = 0;
                    } else if let Some(end) =
                        get_up_center_origin(self.position, self.target, self.up)
                    {
                        self.up = self.up.slerp(end.0, ORIGIN_CENTER_STEP).normalize();
                        self.target =
                            self.target * (1.0 - ORIGIN_CENTER_STEP) + Vec3::ZERO * ORIGIN_CENTER_STEP;
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

    /// Enables or disables up-lock and reprojects the up vector when locking.
    pub fn set_lock_up(&mut self, lock: bool) {
        self.lock_up = lock;
        if self.lock_up {
            self.up = get_closest_perp_unit_to_y(self.position, self.target);
        }
    }
}

/// Clamps view pitch to avoid near-vertical singularities during camera motion.
fn clamp_pitch(relative: Vec3) -> Vec3 {
    if relative.length_squared() <= EPSILON {
        return relative;
    }
    let len = relative.length();
    let mut dir = relative / len;
    let horiz_len = (dir.x * dir.x + dir.z * dir.z).sqrt();

    if horiz_len <= EPSILON {
        let pitch = if dir.y > 0.0 {
            MAX_PITCH_RAD
        } else {
            -MAX_PITCH_RAD
        };
        let new_y = pitch.sin();
        let new_h = pitch.cos();
        dir = Vec3::new(new_h, new_y, 0.0);
        return dir * len;
    }

    let pitch = dir.y.atan2(horiz_len);
    if pitch.abs() <= MAX_PITCH_RAD {
        return relative;
    }

    let clamped_pitch = pitch.clamp(-MAX_PITCH_RAD, MAX_PITCH_RAD);
    let new_y = clamped_pitch.sin();
    let new_h = clamped_pitch.cos();
    let scale = new_h / horiz_len;

    dir.x *= scale;
    dir.z *= scale;
    dir.y = new_y;

    dir * len
}

/// Returns a unit up vector perpendicular to view direction and closest to world-up.
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

/// Returns the axis and full angle rotating `from` toward `to` (both unit vectors).
fn rotation_between_units(from: Vec3, to: Vec3) -> Option<(Vec3, f32)> {
    let dot = from.dot(to).clamp(-1.0, 1.0);
    if dot > 1.0 - EPSILON {
        return None;
    }
    let mut axis = from.cross(to);
    if axis.length_squared() < EPSILON {
        if dot < 0.0 {
            axis = from.cross(Vec3::X);
            if axis.length_squared() < EPSILON {
                axis = from.cross(Vec3::Z);
            }
        } else {
            return None;
        }
    }
    Some((axis.normalize(), dot.acos()))
}

/// Partially rotates unit vector `from` toward unit vector `to` by fraction `t`.
fn slerp_unit_toward(from: Vec3, to: Vec3, t: f32) -> Vec3 {
    if let Some((axis, angle)) = rotation_between_units(from, to) {
        Quat::from_axis_angle(axis, angle * t).mul_vec3(from)
    } else {
        from
    }
}

/// Returns true when the target is close enough to the origin for non-lock-up recentering.
fn origin_center_target_reached(target: Vec3) -> bool {
    target.length() <= ORIGIN_CENTER_TARGET_EPS
}

/// Returns true when lock-up origin recentering is within snap/stop tolerances.
fn lock_up_origin_center_complete(position: Vec3, target: Vec3) -> bool {
    let relative = target - position;
    let distance = relative.length();
    if distance <= EPSILON {
        return true;
    }
    let to_origin = Vec3::ZERO - position;
    if to_origin.length_squared() <= EPSILON {
        return true;
    }
    let view_dir = relative / distance;
    let goal_dir = to_origin.normalize();
    target.length() <= ORIGIN_CENTER_TARGET_EPS
        && view_dir.dot(goal_dir) >= ORIGIN_CENTER_VIEW_DOT_MIN
}

/// Snaps lock-up origin recentering to the final goal pose.
fn snap_lock_up_center_on_origin(position: Vec3, target: Vec3) -> Option<(Vec3, Vec3, Vec3)> {
    let relative = target - position;
    let distance = relative.length();
    if distance <= EPSILON {
        return None;
    }
    let to_origin = Vec3::ZERO - position;
    if to_origin.length_squared() <= EPSILON {
        return None;
    }
    let goal_dir = to_origin.normalize();
    let new_relative = clamp_pitch(goal_dir * distance);
    let new_target = Vec3::ZERO;
    let new_position = new_target - new_relative;
    let new_up = get_closest_perp_unit_to_y(new_position, new_target);
    Some((new_position, new_target, new_up))
}

/// One lock-up origin-centering step preserving orbit distance.
fn step_lock_up_center_on_origin(position: Vec3, target: Vec3) -> Option<(Vec3, Vec3, Vec3)> {
    let relative = target - position;
    let distance = relative.length();
    if distance <= EPSILON {
        return None;
    }
    let to_origin = Vec3::ZERO - position;
    if to_origin.length_squared() <= EPSILON {
        return None;
    }
    if lock_up_origin_center_complete(position, target) {
        return None;
    }

    let view_dir = relative / distance;
    let goal_dir = to_origin.normalize();
    let new_target =
        target * (1.0 - ORIGIN_CENTER_STEP) + Vec3::ZERO * ORIGIN_CENTER_STEP;
    let new_view_dir = slerp_unit_toward(view_dir, goal_dir, ORIGIN_CENTER_STEP);
    let new_relative = clamp_pitch(new_view_dir * distance);
    let new_position = new_target - new_relative;
    let new_up = get_closest_perp_unit_to_y(new_position, new_target);
    Some((new_position, new_target, new_up))
}

/// Computes a rotated up vector and centered target direction for origin-centering animation.
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
    let (axis, angle) = rotation_between_units(rel_n, new_n)?;
    let rotation = Quat::from_axis_angle(axis, angle);
    Some((rotation.mul_vec3(up), new_relative))
}
