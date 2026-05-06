#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BivectorBoost {
    pub i: f64,
    pub j: f64,
    pub k: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BivectorRotation {
    pub i: f64,
    pub j: f64,
    pub k: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExpBoost {
    pub scalar: f64,
    pub i: f64,
    pub j: f64,
    pub k: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExpRotation {
    pub scalar: f64,
    pub i: f64,
    pub j: f64,
    pub k: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VersorBoost {
    pub phi: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VersorRotation {
    pub theta: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}

impl BivectorBoost {
    pub fn new(i: f64, j: f64, k: f64) -> Self {
        Self { i, j, k }
    }

    pub fn norm(&self) -> f64 {
        self.i
            .mul_add(self.i, self.j.mul_add(self.j, self.k * self.k))
            .sqrt()
    }

    pub fn norm_squared(&self) -> f64 {
        self.i
            .mul_add(self.i, self.j.mul_add(self.j, self.k * self.k))
    }

    pub fn exp(&self) -> ExpBoost {
        let phi = self.norm();
        if phi == 0.0 {
            ExpBoost::new(1.0, 0.0, 0.0, 0.0)
        } else {
            let scalar = phi.cosh();
            let ratio = phi.sinh() / phi;
            let i = self.i * ratio;
            let j = self.j * ratio;
            let k = self.k * ratio;
            ExpBoost::new(scalar, i, j, k)
        }
    }

    pub fn from_velocity(vx: f64, vy: f64, vz: f64) -> Self {
        let speed_sq = vx.mul_add(vx, vy.mul_add(vy, vz * vz));
        if speed_sq < 1e-30 {
            return Self::new(0.0, 0.0, 0.0);
        }
        let speed = speed_sq.sqrt();
        let phi = speed.atanh();
        let scale = phi / speed;
        Self {
            i: scale * vx,
            j: scale * vy,
            k: scale * vz,
        }
    }
}

impl BivectorRotation {
    pub fn new(i: f64, j: f64, k: f64) -> Self {
        Self { i, j, k }
    }
}

impl ExpBoost {
    pub fn new(scalar: f64, i: f64, j: f64, k: f64) -> Self {
        Self { scalar, i, j, k }
    }
}

impl ExpRotation {
    pub fn new(scalar: f64, i: f64, j: f64, k: f64) -> Self {
        Self { scalar, i, j, k }
    }
}

impl VersorBoost {
    pub fn new(phi: f64, vx: f64, vy: f64, vz: f64) -> Self {
        Self { phi, vx, vy, vz }
    }
}

impl VersorRotation {
    pub fn new(theta: f64, vx: f64, vy: f64, vz: f64) -> Self {
        Self { theta, vx, vy, vz }
    }
}
