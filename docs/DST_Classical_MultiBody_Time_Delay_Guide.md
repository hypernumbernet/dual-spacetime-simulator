# DST Classical Multi-Body Time Delay Guide
## Oscillating Trigonometric Formula with λ_eff

**Version**: 2026-07-01  
**Target**: Rust programmers implementing classical N-body simulations

---

## Purpose

In Double Spacetime Theory (DST), the time delay effect arises from torsional mismatch between usual and dual spacetime.  
For classical approximation, we use a **trigonometric oscillating formula** that allows the effective time factor to swing between positive and negative values. This captures the elliptic (oscillatory) nature of the dual sector.

We propose using an **effective mismatch parameter λ_eff** to drive the oscillation.

---

## Core Formula: Oscillating Time Delay

The effective time progression rate for particle $i$ is given by the trigonometric function:

$$
\frac{d\tau_i}{dt} = \cos(\lambda_{\rm eff},i)
$$

- This formula oscillates between $-1$ and $+1$.
- Positive values mean time progresses forward.
- Negative values represent the influence of the dual spacetime (possible "backward" or phase-reversed effect in the approximation).
- $\lambda_{\rm eff},i$ is the **effective torsional mismatch parameter** for particle $i$.

### Definition of λ_eff

We define the effective mismatch parameter as:

$$
\lambda_{\rm eff},i = k \cdot \Phi_i
$$

where

$$
\Phi_i = -\sum_{j \neq i} \frac{G m_j}{|\mathbf{r}_i - \mathbf{r}_j| + \epsilon}
$$

- $k$ : Scaling constant (theoretical value $k = 2/c^2$ to match weak-field GR in the small-angle limit; start with $k = 1.0$ in natural units for testing).
- $\Phi_i$ : Newtonian gravitational potential at particle $i$ (negative and larger in magnitude where gravity is stronger).
- $\epsilon$ : Softening length to avoid singularities.

**Why this works**:
- When particles cluster (stronger gravity), $|\Phi_i|$ increases → $|\lambda_{\rm eff}|$ grows → $\cos(\lambda_{\rm eff})$ oscillates more rapidly.
- The trigonometric form naturally introduces oscillation (plus/minus) coming from the dual spacetime's rotational character.

---

## Rust Implementation

### Constants

```rust
const G: f64 = 6.67430e-11;
const C: f64 = 299792458.0;
const EPSILON: f64 = 1e-3;
const K_SCALE: f64 = 2.0 / (C * C);   // Start with this; adjust for testing
```

### Data Structure

```rust
use nalgebra::Vector3;

#[derive(Clone, Debug)]
pub struct Particle {
    pub pos: Vector3<f64>,
    pub vel: Vector3<f64>,
    pub mass: f64,
    pub proper_time: f64,      // Accumulated proper time (can receive negative contributions)
    pub lambda_eff: f64,       // Current effective mismatch parameter
}
```

### Compute Potential

```rust
fn compute_potential(i: usize, particles: &[Particle]) -> f64 {
    let mut phi = 0.0;
    for (j, p) in particles.iter().enumerate() {
        if j == i { continue; }
        let r = (particles[i].pos - p.pos).norm() + EPSILON;
        phi -= G * p.mass / r;
    }
    phi
}
```

### Update λ_eff and Time Delay (Main Function)

```rust
fn update_time_delay(particles: &mut [Particle], dt: f64) {
    for i in 0..particles.len() {
        let phi = compute_potential(i, particles);
        // Update effective mismatch parameter
        particles[i].lambda_eff = K_SCALE * phi;

        // Oscillating time delay formula
        let dilation = (particles[i].lambda_eff).cos();

        // Accumulate proper time (can go negative in this approximation)
        particles[i].proper_time += dt * dilation;
    }
}
```

### Usage in Main Loop

```rust
// After updating positions and velocities with Newtonian gravity
update_time_delay(&mut particles, dt);
```

---

## Why Trigonometric Oscillating Form?

- The simple linear form $1 + \Phi/c^2$ only gives slowing (always positive, close to 1).
- The $\cos(\lambda_{\rm eff})$ form introduces **oscillation between positive and negative**, reflecting the dual spacetime's elliptic geometry (rotations in the dual sector).
- In the small $\lambda_{\rm eff}$ limit, $\cos(\lambda_{\rm eff}) \approx 1 - \lambda_{\rm eff}^2/2$, which recovers the quadratic weak-field behavior consistent with DST.
- Negative contributions model the "phase lag" or time-reversed influence from the dual spacetime without breaking the overall simulation stability when averaged over many particles.

---

## Parameter Tuning Tips

- Start with natural units ($G=1$, $C=1$) and $K\_SCALE = 2.0$.
- If oscillation is too violent, reduce $K\_SCALE$ or add a small damping term.
- For visualization, you can plot both `proper_time` and `lambda_eff` to see the oscillation.
- If you want strictly non-negative time, replace with `dilation.max(0.0)` or use `cos(lambda_eff).powi(2)`, but the raw oscillating version is recommended for capturing DST's dual-sector character.

---

## Verification

Test case:
1. Place one heavy central mass.
2. Place a light test particle at distance $r$.
3. Check that `lambda_eff` ≈ $k \cdot (-GM/r)$ and `dilation` oscillates around the expected weak-field value.

---

This guide focuses exclusively on the **oscillating trigonometric time delay using λ_eff**.  
Implement the `update_time_delay` function as shown — it is the heart of the simulation.

Happy coding!