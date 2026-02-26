#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BivectorBoost {
    pub iI: f64,
    pub iJ: f64,
    pub iK: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BivectorRotation {
    pub I: f64,
    pub J: f64,
    pub K: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExpBoost {
    pub scalar: f64,
    pub iI: f64,
    pub iJ: f64,
    pub iK: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExpRotation {
    pub scalar: f64,
    pub I: f64,
    pub J: f64,
    pub K: f64,
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
    pub fn new(iI: f64, iJ: f64, iK: f64) -> Self {
        Self { iI, iJ, iK }
    }

    pub fn norm(&self) -> f64 {
        self.iI
            .mul_add(self.iI, self.iJ.mul_add(self.iJ, self.iK * self.iK))
            .sqrt()
    }

    pub fn norm_squared(&self) -> f64 {
        self.iI
            .mul_add(self.iI, self.iJ.mul_add(self.iJ, self.iK * self.iK))
    }

    pub fn exp(&self) -> ExpBoost {
        let phi = self.norm();
        if phi == 0.0 {
            ExpBoost::new(1.0, 0.0, 0.0, 0.0)
        } else {
            let scalar = phi.cosh();
            let ratio = phi.sinh() / phi;
            let iI = self.iI * ratio;
            let iJ = self.iJ * ratio;
            let iK = self.iK * ratio;
            ExpBoost::new(scalar, iI, iJ, iK)
        }
    }

    pub fn from_velocity(vx: f64, vy: f64, vz: f64) -> Self {
        let phi = (vx * vx + vy * vy + vz * vz).sqrt().atanh();
        let iI = phi * vx / (vx * vx + vy * vy + vz * vz).sqrt();
        let iJ = phi * vy / (vx * vx + vy * vy + vz * vz).sqrt();
        let iK = phi * vz / (vx * vx + vy * vy + vz * vz).sqrt();
        Self { iI, iJ, iK }
    }
}

impl BivectorRotation {
    pub fn new(I: f64, J: f64, K: f64) -> Self {
        Self { I, J, K }
    }
}

impl ExpBoost {
    pub fn new(scalar: f64, iI: f64, iJ: f64, iK: f64) -> Self {
        Self { scalar, iI, iJ, iK }
    }
}

impl ExpRotation {
    pub fn new(scalar: f64, I: f64, J: f64, K: f64) -> Self {
        Self { scalar, I, J, K }
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
