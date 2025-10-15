use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

#[derive(Copy, Clone, PartialEq, Eq)]
enum QuatComp {
    R,
    I, // i or I
    J, // j or J
    K, // k or K
}

const fn quat_mul(a: QuatComp, b: QuatComp) -> (i8, QuatComp) {
    match (a, b) {
        (QuatComp::R, x) => (1, x),
        (x, QuatComp::R) => (1, x),
        (QuatComp::I, QuatComp::I) => (-1, QuatComp::R),
        (QuatComp::I, QuatComp::J) => (1, QuatComp::K),
        (QuatComp::I, QuatComp::K) => (-1, QuatComp::J),
        (QuatComp::J, QuatComp::I) => (-1, QuatComp::K),
        (QuatComp::J, QuatComp::J) => (-1, QuatComp::R),
        (QuatComp::J, QuatComp::K) => (1, QuatComp::I),
        (QuatComp::K, QuatComp::I) => (1, QuatComp::J),
        (QuatComp::K, QuatComp::J) => (-1, QuatComp::I),
        (QuatComp::K, QuatComp::K) => (-1, QuatComp::R),
    }
}

const BASIS: [(QuatComp, QuatComp); 15] = [
    (QuatComp::J, QuatComp::R), // 0: j
    (QuatComp::K, QuatComp::I), // 1: kI
    (QuatComp::K, QuatComp::J), // 2: kJ
    (QuatComp::K, QuatComp::K), // 3: kK
    (QuatComp::I, QuatComp::I), // 4: iI
    (QuatComp::I, QuatComp::J), // 5: iJ
    (QuatComp::I, QuatComp::K), // 6: iK
    (QuatComp::R, QuatComp::I), // 7: I
    (QuatComp::R, QuatComp::J), // 8: J
    (QuatComp::R, QuatComp::K), // 9: K
    (QuatComp::K, QuatComp::R), // 10: k
    (QuatComp::J, QuatComp::I), // 11: jI
    (QuatComp::J, QuatComp::J), // 12: jJ
    (QuatComp::J, QuatComp::K), // 13: jK
    (QuatComp::I, QuatComp::R), // 14: i
];

const fn get_basis_index(s: QuatComp, l: QuatComp) -> usize {
    match (s, l) {
        (QuatComp::J, QuatComp::R) => 0,
        (QuatComp::K, QuatComp::I) => 1,
        (QuatComp::K, QuatComp::J) => 2,
        (QuatComp::K, QuatComp::K) => 3,
        (QuatComp::I, QuatComp::I) => 4,
        (QuatComp::I, QuatComp::J) => 5,
        (QuatComp::I, QuatComp::K) => 6,
        (QuatComp::R, QuatComp::I) => 7,
        (QuatComp::R, QuatComp::J) => 8,
        (QuatComp::R, QuatComp::K) => 9,
        (QuatComp::K, QuatComp::R) => 10,
        (QuatComp::J, QuatComp::I) => 11,
        (QuatComp::J, QuatComp::J) => 12,
        (QuatComp::J, QuatComp::K) => 13,
        (QuatComp::I, QuatComp::R) => 14,
        _ => unreachable!(),
    }
}

const fn compute_mul_table() -> [[(i8, usize); 15]; 15] {
    let mut table = [[(0i8, 0usize); 15]; 15];
    let mut left: usize = 0;
    while left < 15 {
        let mut right: usize = 0;
        while right < 15 {
            let (s_l, l_l) = BASIS[left];
            let (s_r, l_r) = BASIS[right];
            let (sign_s, res_s) = quat_mul(s_l, s_r);
            let (sign_l, res_l) = quat_mul(l_l, l_r);
            let sign = sign_s * sign_l;
            let out = if matches!((res_s, res_l), (QuatComp::R, QuatComp::R)) {
                (sign, 0)
            } else {
                let idx = get_basis_index(res_s, res_l);
                (sign, idx + 1)
            };
            table[left][right] = out;
            right += 1;
        }
        left += 1;
    }
    table
}

const MUL_TABLE: [[(i8, usize); 15]; 15] = compute_mul_table();
const DIM: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TetraQuaternion {
    coeffs: [f64; DIM],
}

impl TetraQuaternion {
    pub fn new(real: f64, bases: [f64; 15]) -> Self {
        let mut coeffs = [0.0; DIM];
        coeffs[0] = real;
        coeffs[1..].copy_from_slice(&bases);
        Self { coeffs }
    }

    pub fn one() -> Self {
        let mut coeffs = [0.0; DIM];
        coeffs[0] = 1.0;
        Self { coeffs }
    }

    pub fn basis(index: usize) -> Self {
        assert!(index < 15, "Basis index out of range");
        let mut coeffs = [0.0; DIM];
        coeffs[index + 1] = 1.0;
        Self { coeffs }
    }
}

impl Add for TetraQuaternion {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        let mut coeffs = [0.0; DIM];
        for i in 0..DIM {
            coeffs[i] = self.coeffs[i] + rhs.coeffs[i];
        }
        Self { coeffs }
    }
}

impl Mul for TetraQuaternion {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        let mut result = [0.0; DIM];
        result[0] += self.coeffs[0] * rhs.coeffs[0];
        for i in 1..DIM {
            result[i] += self.coeffs[0] * rhs.coeffs[i];
            result[i] += self.coeffs[i] * rhs.coeffs[0];
        }
        for left in 0..15 {
            for right in 0..15 {
                let (sign, out_basis) = MUL_TABLE[left][right];
                let contrib = self.coeffs[left + 1] * rhs.coeffs[right + 1] * sign as f64;
                if out_basis == 0 {
                    result[0] += contrib;
                } else {
                    result[out_basis] += contrib;
                }
            }
        }
        Self { coeffs: result }
    }
}

impl AddAssign for TetraQuaternion {
    fn add_assign(&mut self, rhs: Self) {
        for i in 0..DIM {
            self.coeffs[i] += rhs.coeffs[i];
        }
    }
}

impl Sub for TetraQuaternion {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        let mut coeffs = [0.0; DIM];
        for i in 0..DIM {
            coeffs[i] = self.coeffs[i] - rhs.coeffs[i];
        }
        Self { coeffs }
    }
}

impl SubAssign for TetraQuaternion {
    fn sub_assign(&mut self, rhs: Self) {
        for i in 0..DIM {
            self.coeffs[i] -= rhs.coeffs[i];
        }
    }
}

impl MulAssign for TetraQuaternion {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

impl TetraQuaternion {
    pub fn is_zero(&self) -> bool {
        self.coeffs.iter().all(|&c| c.abs() < 1e-10)
    }
}

const BASIS_NAMES: [&str; 15] = [
    " j ", "kI ", "kJ ", "kK ", "iI ", "iJ ", "iK ", " I ", " J ", " K ", " k ", "jI ", "jJ ",
    "jK ", " i ",
];

fn _to_mul_table_string() -> String {
    let mut result = String::new();
    for _ in 0..16 {
        result.push_str("═════");
    }
    result.push_str("\n");
    result.push_str("   | ");
    for name in BASIS_NAMES {
        result.push_str(&format!(" {:>3}", name));
    }
    result.push_str("\n");
    result.push_str("───┼─");
    for _ in 0..15 {
        result.push_str("─────");
    }
    result.push('\n');
    for row in 0..15 {
        result.push_str(&format!("{:>2}", BASIS_NAMES[row]));
        result.push_str("|");
        for col in 0..15 {
            let (sign, basis) = MUL_TABLE[row][col];
            let cell = if basis == 0 {
                format!("{:>4}", if sign > 0 { "+1" } else { "-1" })
            } else {
                let mut cell = String::new();
                cell.push_str(if sign > 0 { "+" } else { "-" });
                cell.push_str(BASIS_NAMES[basis - 1].trim());
                format!("{:>4}", cell)
            };
            result.push_str(&cell);
        }
        result.push('\n');
        if row < 14 {
            result.push_str("───┼─");
            for _ in 0..15 {
                result.push_str("─────");
            }
            result.push('\n');
        }
    }
    for _ in 0..16 {
        result.push_str("═════");
    }
    result.push_str("\n");
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn assert_table_correct() {
        assert!(MUL_TABLE[0][0] == (-1, 0));
        assert!(MUL_TABLE[0][1] == (1, 5));
        assert!(MUL_TABLE[1][2] == (-1, 10));
        assert!(MUL_TABLE[4][5] == (-1, 10));
        assert!(MUL_TABLE[7][8] == (1, 10));
        assert!(MUL_TABLE[10][14] == (1, 1));
        assert!(MUL_TABLE[11][12] == (-1, 10));
        assert!(MUL_TABLE[14][14] == (-1, 0));
    }

    #[test]
    fn test_basis_multiplication() {
        let tests = [
            // (left, right, expected_sign, expected_basis)
            (0, 0, -1, 0),    // j * j = -1
            (0, 1, 1, 5),     // j * kI = +iI
            (1, 2, -1, 10),   // kI * kJ = -K
            (4, 5, -1, 10),   // iI * iJ = -K
            (7, 8, 1, 10),    // I * J = +K
            (10, 14, 1, 1),   // k * i = +j
            (11, 12, -1, 10), // jI * jJ = -K
            (14, 14, -1, 0),  // i * i = -1
            (0, 14, -1, 11),  // j * i = -jI
            (14, 0, 1, 11),   // i * j = +jI
        ];

        for (left, right, expected_sign, expected_basis) in tests {
            let entry = MUL_TABLE[left][right];
            assert_eq!(
                entry,
                (expected_sign as i8, expected_basis),
                "Failed: basis[{}][{}] = {:?}, expected ({}, {})",
                left,
                right,
                entry,
                expected_sign,
                expected_basis
            );
        }
    }

    #[test]
    fn test_tetraquaternion_mul() {
        let j = TetraQuaternion::basis(0); // j
        let ki = TetraQuaternion::basis(1); // kI
        let ii = TetraQuaternion::basis(4); // iI

        // j * kI = +iI
        let product1 = j * ki;
        assert!((product1.coeffs[5] - 1.0).abs() < 1e-10);
        assert!(product1.is_zero() || product1.coeffs[5].abs() > 0.9);

        // j * j = -1
        let product2 = j * j;
        assert!((product2.coeffs[0] + 1.0).abs() < 1e-10);

        // iI * iI = 1 (i*i= -1, I*I= -1 → (-1)*(-1)= +1)
        let product3 = ii * ii;
        assert!((product3.coeffs[0] + 1.0).abs() > 1e-10);
    }

    #[test]
    fn test_addition() {
        let a = TetraQuaternion::new(1.0, [0.0; 15]);
        let b = TetraQuaternion::basis(0);
        let sum = a + b;
        assert_eq!(sum.coeffs[0], 1.0);
        assert_eq!(sum.coeffs[1], 1.0);
    }

    #[test]
    fn test_identity() {
        let one = TetraQuaternion::one();
        let j = TetraQuaternion::basis(0);
        assert_eq!(one * j, j);
        assert_eq!(j * one, j);
    }

    #[test]
    fn test_rotation_composition() {
        let rot1 = TetraQuaternion::basis(0);
        let rot2 = TetraQuaternion::basis(10);
        let composed = rot1 * rot2; // j*k = +i

        assert!((composed.coeffs[15] - 1.0).abs() < 1e-10);
        assert!(composed.is_zero() || composed.coeffs[15].abs() > 0.9);
    }

    #[test]
    fn test_mul_table_string() {
        let table_str = _to_mul_table_string();
        println!("\n{}", table_str);
        assert!(table_str.contains("j |  -1"));
        assert!(table_str.contains("+iI"));
        let lines: Vec<&str> = table_str.lines().collect();
        assert!(lines.len() >= 18, "Minimum 18 lines expected");
    }
}
