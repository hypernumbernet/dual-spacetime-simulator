//! Explosion geometry: debris ballistics and translucent FX mesh invariants.

use pga_rocket::explosion::{
    DEBRIS_COUNT, FX_DISC_VERTS, debris_sample, explosion_fx_mesh, explosion_opaque_mesh,
};
use pga_rocket::sim::{RocketState, step_rocket};

const ORIGIN: [f32; 3] = [0.0, 12.0, 0.0];
const BODY_R: f32 = 1.8;

fn destroyed_state(age: f64) -> RocketState {
    let mut s = RocketState::resting_on_pad();
    s.destroyed = true;
    s.explosion_age = age;
    s.explosion_origin = [ORIGIN[0] as f64, ORIGIN[1] as f64, ORIGIN[2] as f64];
    s.last_impact_speed = 25.0;
    s
}

#[test]
fn debris_never_below_ground() {
    for i in 0..DEBRIS_COUNT {
        let mut age = 0.0f32;
        while age <= 12.0 {
            let s = debris_sample(i, ORIGIN, BODY_R, age);
            for c in s.center {
                assert!(c.is_finite(), "shard {i} at age {age}: non-finite center");
            }
            assert!(
                s.center[1] >= s.half - 1e-3,
                "shard {i} at age {age}: y {} below rest height {}",
                s.center[1],
                s.half
            );
            age += 0.05;
        }
    }
}

#[test]
fn debris_comes_to_rest() {
    for i in 0..DEBRIS_COUNT {
        let a = debris_sample(i, ORIGIN, BODY_R, 12.0);
        let b = debris_sample(i, ORIGIN, BODY_R, 25.0);
        assert!(a.at_rest, "shard {i} still moving at 12 s");
        assert_eq!(a.center, b.center, "shard {i} drifted after coming to rest");
        assert_eq!(a.axis, b.axis, "shard {i} kept tumbling after coming to rest");
    }
}

#[test]
fn debris_deterministic() {
    for i in 0..DEBRIS_COUNT {
        let a = debris_sample(i, ORIGIN, BODY_R, 1.234);
        let b = debris_sample(i, ORIGIN, BODY_R, 1.234);
        assert_eq!(a.center, b.center);
        assert_eq!(a.axis, b.axis);
        assert_eq!(a.half, b.half);
        assert_eq!(a.at_rest, b.at_rest);
    }
}

#[test]
fn meshes_empty_when_alive() {
    let s = RocketState::resting_on_pad();
    let (fx_verts, fx_idx) = explosion_fx_mesh(&s, [0.0, 30.0, 80.0]);
    assert!(fx_verts.is_empty() && fx_idx.is_empty());
    let (verts, idx) = explosion_opaque_mesh(&s);
    assert!(verts.is_empty() && idx.is_empty());
}

#[test]
fn fx_mesh_nonempty_and_bounded() {
    let eye = [40.0, 25.0, 40.0];
    for age in [0.05, 1.0, 5.0, 60.0] {
        let s = destroyed_state(age);
        let (verts, idx) = explosion_fx_mesh(&s, eye);
        assert!(!idx.is_empty(), "no FX at age {age} (scorch should persist)");
        assert!(verts.len() < 1000, "FX vertex count exploded at age {age}");
        assert_eq!(verts.len() % FX_DISC_VERTS as usize, 0);
        for v in &verts {
            for c in v.pos {
                assert!(c.is_finite());
            }
            let [r, g, b, a] = v.color;
            assert!((0.0..=1.0).contains(&a), "alpha {a} out of range at age {age}");
            assert!(r >= 0.0 && g >= 0.0 && b >= 0.0);
            assert!(r.is_finite() && g.is_finite() && b.is_finite());
        }
    }
}

/// Full pipeline: a real crash from the physics sim must yield both meshes.
#[test]
fn crash_then_meshes_render() {
    let mut s = RocketState::at_altitude(80.0);
    const DT: f64 = 1.0 / 120.0;
    for _ in 0..4000 {
        step_rocket(&mut s, DT);
        if s.destroyed {
            break;
        }
    }
    assert!(s.destroyed, "drop from 80 m should destroy the vehicle");
    for _ in 0..120 {
        step_rocket(&mut s, DT); // advance explosion_age ~1 s
    }
    let (verts, idx) = explosion_opaque_mesh(&s);
    assert!(!idx.is_empty() && !verts.is_empty(), "no debris after crash");
    let (fx_verts, fx_idx) = explosion_fx_mesh(&s, [40.0, 25.0, 40.0]);
    assert!(!fx_idx.is_empty() && !fx_verts.is_empty(), "no FX after crash");
}

#[test]
fn fx_sorted_back_to_front() {
    let eye = [30.0f32, 20.0, -50.0];
    let s = destroyed_state(1.0);
    let (verts, _) = explosion_fx_mesh(&s, eye);
    let dist2 = |p: [f32; 3]| {
        let d = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]];
        d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
    };
    let mut prev = f32::INFINITY;
    for disc in verts.chunks(FX_DISC_VERTS as usize) {
        let d = dist2(disc[0].pos);
        assert!(
            d <= prev + 1e-3,
            "FX discs not back-to-front: {d} after {prev}"
        );
        prev = d;
    }
}
