use glam::DVec3;
use std::f64;

const EPSILON: f64 = 1e-10;

pub fn fuzzy_compare(a: f64, b: f64) -> bool {
    (a - b).abs() < EPSILON
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Spacetime {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Spacetime {
    pub fn new(t: f64, x: f64, y: f64, z: f64) -> Self {
        Self { t, x, y, z }
    }

    pub fn from_t(t: f64) -> Self {
        Self::new(t, 0.0, 0.0, 0.0)
    }

    pub fn from_vector3(v: DVec3) -> Self {
        Self::new(0.0, v.x, v.y, v.z)
    }

    pub fn from_array(a: &[f64]) -> Self {
        assert!(a.len() >= 4);
        Self::new(a[0], a[1], a[2], a[3])
    }

    pub fn from_array_index(a: &[f64], index: usize) -> Self {
        assert!(a.len() >= index + 4);
        Self::new(a[index], a[index + 1], a[index + 2], a[index + 3])
    }

    pub fn set_values(&mut self, t: f64, x: f64, y: f64, z: f64) {
        self.t = t;
        self.x = x;
        self.y = y;
        self.z = z;
    }

    pub fn set_t_only(&mut self, t: f64) {
        self.t = t;
        self.x = 0.0;
        self.y = 0.0;
        self.z = 0.0;
    }

    pub fn set_from_array(&mut self, a: &[f64]) {
        assert!(a.len() >= 4);
        self.t = a[0];
        self.x = a[1];
        self.y = a[2];
        self.z = a[3];
    }

    pub fn set_from_array_index(&mut self, a: &[f64], index: usize) {
        assert!(a.len() >= index + 4);
        self.t = a[index];
        self.x = a[index + 1];
        self.y = a[index + 2];
        self.z = a[index + 3];
    }

    pub const fn zero() -> Self {
        Self {
            t: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub const fn identity() -> Self {
        Self {
            t: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn get(&self, i: usize) -> f64 {
        match i {
            0 => self.t,
            1 => self.x,
            2 => self.y,
            3 => self.z,
            _ => panic!("Index out of bounds"),
        }
    }

    pub fn get_mut(&mut self, i: usize) -> &mut f64 {
        match i {
            0 => &mut self.t,
            1 => &mut self.x,
            2 => &mut self.y,
            3 => &mut self.z,
            _ => panic!("Index out of bounds"),
        }
    }

    /// signature (-+++)
    pub fn norm(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z - self.t * self.t
    }

    pub fn conjugated(&self) -> Self {
        Self::new(-self.t, self.x, self.y, self.z)
    }

    pub fn abs(&self) -> f64 {
        self.norm().abs().sqrt()
    }

    pub fn arg(&self) -> f64 {
        let n = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        (n / self.t).atanh()
    }

    pub fn exp_versor(x: f64, y: f64, z: f64, a: f64) -> Self {
        let s = a.sinh();
        Self::new(a.cosh(), x * s, y * s, z * s)
    }

    pub fn exp(a: f64, v: DVec3) -> Self {
        let s = a.sinh();
        Self::new(a.cosh(), v.x * s, v.y * s, v.z * s)
    }

    pub fn velocities(versor_angle: DVec3, speed_of_light: f64) -> DVec3 {
        let a = versor_angle.length_squared();
        if a == 0.0 {
            return DVec3::ZERO;
        }
        let beta = a.tanh();
        let v = beta * speed_of_light / a;
        DVec3::new(v * versor_angle.x, v * versor_angle.y, v * versor_angle.z)
    }

    #[inline(always)]
    pub fn lorentz_transformation(&mut self, g: Spacetime) {
        let p = g.t;
        let q = g.x;
        let r = g.y;
        let s = g.z;

        let w = self.t;
        let x = self.x;
        let y = self.y;
        let z = self.z;

        let pp = p * p;
        let qq = q * q;
        let rr = r * r;
        let ss = s * s;

        let p_w = p * w;
        let q_x = q * x;
        let r_y = r * y;
        let s_z = s * z;

        self.t = (pp + qq + rr + ss) * w + 2.0 * p * (q_x + r_y + s_z);
        self.x = (pp + qq - rr - ss) * x + 2.0 * q * (p_w - r_y - s_z);
        self.y = (pp - qq + rr - ss) * y + 2.0 * r * (p_w - q_x - s_z);
        self.z = (pp - qq - rr + ss) * z + 2.0 * s * (p_w - q_x - r_y);
    }

    pub fn lorentz_transformation_v(&mut self, v: DVec3, speed_of_light_inv: f64) {
        let l = v.length_squared();
        if l == 0.0 {
            return;
        }
        let a = (l * speed_of_light_inv).atanh();
        let dir = v / l;
        let g = Self::exp(0.5 * a, dir);
        self.lorentz_transformation(g);
    }

    pub fn lorentz_transformation_rapidity(&mut self, rapidity: DVec3) {
        let a = rapidity.length_squared();
        if a == 0.0 {
            return;
        }
        let dir = rapidity / a;
        let g = Self::exp(0.5 * a, dir);
        self.lorentz_transformation(g);
    }

    pub fn fuzzy_compare(&self, a: Spacetime) -> bool {
        fuzzy_compare(self.t, a.t)
            && fuzzy_compare(self.x, a.x)
            && fuzzy_compare(self.y, a.y)
            && fuzzy_compare(self.z, a.z)
    }
}

impl std::fmt::Display for Spacetime {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}, {}, {}, {}", self.t, self.x, self.y, self.z)
    }
}

/// Converting Velocity ​​to Rapidity Vector.
pub fn rapidity_vector(v: DVec3, speed_of_light_inv: f64) -> DVec3 {
    let speed = v.length_squared();
    if speed == 0.0 {
        return DVec3::ZERO;
    }
    let a = (speed * speed_of_light_inv).atanh();
    let b = a / speed;
    DVec3::new(b * v.x, b * v.y, b * v.z)
}

/// Converting Momentum to Rapidity Vector.
///
/// p = mvγ
/// {γ : Lorentz factor : 1 / sqrt(1 - v^2 / c^2)}
///
/// p = mv / sqrt(1 - v^2 / c^2)
/// p^2 (1 -  v^2 / c^2) = m^2 v^2
/// m^2 v^2 + p^2 v^2 / c^2 = p^2
/// (m^2 + p^2 / c^2) v^2 = p^2
///
/// v = p / sqrt(m^2 + p^2 / c^2 )
/// v -> c (p -> ∞, m -> ∞)
/// v = c (m = 0)
///
/// v / c = p / sqrt(m^2 c^2 + p^2) = tanh(a) = pc / E
/// tanh(a) < 1 (p -> ∞)
pub fn rapidity_from_momentum(p: DVec3, m: f64, speed_of_light: f64) -> DVec3 {
    let pn = p.length_squared();
    if pn == 0.0 {
        return DVec3::ZERO;
    }
    let pr = pn.sqrt();
    let l = pr / (m * m * speed_of_light * speed_of_light + pn).sqrt();
    let a = l.atanh();
    let b = a / pr;
    DVec3::new(b * p.x, b * p.y, b * p.z)
}

#[cfg(test)]
mod tests {
    use super::rapidity_vector;
    use super::{DVec3, Spacetime, fuzzy_compare};

    #[test]
    fn test_zero_and_identity() {
        let zero = Spacetime::zero();
        assert_eq!(zero, Spacetime::new(0.0, 0.0, 0.0, 0.0));
        assert_eq!(zero.norm(), 0.0);

        let identity = Spacetime::identity();
        assert_eq!(identity, Spacetime::new(1.0, 0.0, 0.0, 0.0));
        assert_eq!(identity.norm(), -1.0);
    }

    #[test]
    fn test_conjugated_and_norm() {
        let st = Spacetime::new(1.0, 2.0, 3.0, 4.0);
        let conj = st.conjugated();
        assert_eq!(conj, Spacetime::new(-1.0, 2.0, 3.0, 4.0));
        assert_eq!(st.norm(), -1.0 + 4.0 + 9.0 + 16.0);
    }

    #[test]
    fn test_arg() {
        let st = Spacetime::new(1.0, 0.0, 0.0, 0.0);
        assert_eq!(st.arg(), 0.0); // atanh(0)

        let st_with_vec = Spacetime::new(2.0, 1.0, 0.0, 0.0);
        let n: f64 = 1.0;
        assert!(fuzzy_compare(st_with_vec.arg(), (n / 2.0).atanh()));
    }

    #[test]
    fn test_versor_angle() {
        let v_zero = DVec3::ZERO;
        assert_eq!(rapidity_vector(v_zero, 1.0), DVec3::ZERO);

        let v = DVec3::new(1.0, 0.0, 0.0);
        let c_inv: f64 = 1.0 / 3.0e8; // Example
        let versor = rapidity_vector(v, c_inv);
        let a = (v.length_squared() * c_inv).atanh();
        let b = a / v.length_squared();
        assert!(fuzzy_compare(versor.x, b * v.x));
    }

    #[test]
    fn test_lorentz_transformation() {
        let mut st = Spacetime::new(1.0, 0.0, 0.0, 0.0);
        let g = Spacetime::identity();
        st.lorentz_transformation(g);
        assert_eq!(st, Spacetime::new(1.0, 0.0, 0.0, 0.0)); // No change

        // Add more physics-based tests as needed, e.g., boost along x-axis.
    }

    // Expand with more tests for full coverage, following TDD.
}
