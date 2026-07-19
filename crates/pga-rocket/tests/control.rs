//! Keyboard → control mapping tests against the real ControlMapper.

use pga_rocket::control::{ControlMapper, FULL_THROTTLE_RAMP_S, KeySnapshot, map_keys};
use pga_rocket::sim::{ControlCommand, RocketState, step_rocket};

#[test]
fn thrust_key_increases_throttle_release_holds() {
    let mut mapper = ControlMapper::default();
    assert_eq!(mapper.command.throttle, 0.0);

    let hold_up = KeySnapshot {
        thrust_up: true,
        ..Default::default()
    };
    let cmd = mapper.apply(&hold_up, 0.5);
    assert!(
        cmd.throttle > 0.1,
        "held thrust key should raise throttle, got {}",
        cmd.throttle
    );
    let after = cmd.throttle;

    let released = KeySnapshot::default();
    let cmd2 = mapper.apply(&released, 0.5);
    assert!(
        (cmd2.throttle - after).abs() < 1e-9,
        "release should hold throttle, was {after} now {}",
        cmd2.throttle
    );
}

#[test]
fn thrust_down_decreases_throttle() {
    let mut mapper = ControlMapper {
        command: ControlCommand {
            throttle: 0.8,
            ..Default::default()
        },
        ..Default::default()
    };
    let down = KeySnapshot {
        thrust_down: true,
        ..Default::default()
    };
    let cmd = mapper.apply(&down, 0.5);
    assert!(
        cmd.throttle < 0.8 - 0.05,
        "thrust_down should reduce throttle, got {}",
        cmd.throttle
    );
}

#[test]
fn attitude_keys_set_pitch_yaw_roll() {
    let mut mapper = ControlMapper::default();
    let keys = KeySnapshot {
        pitch_up: true,
        yaw_left: true,
        roll_right: true,
        ..Default::default()
    };
    let cmd = mapper.apply(&keys, 0.016);
    assert!((cmd.pitch - 1.0).abs() < 1e-12);
    assert!((cmd.yaw - (-1.0)).abs() < 1e-12);
    assert!((cmd.roll - 1.0).abs() < 1e-12);

    let zero = mapper.apply(&KeySnapshot::default(), 0.016);
    assert_eq!(zero.pitch, 0.0);
    assert_eq!(zero.yaw, 0.0);
    assert_eq!(zero.roll, 0.0);
}

#[test]
fn map_keys_is_not_noop() {
    // space, thrust_down, f, w, s, a, d, q, e, r, l, t
    // A/D → roll, Q/E → yaw: d=roll_right, q=yaw_left
    let snap = map_keys(
        true, false, false, true, false, false, true, true, false, false, false, false,
    );
    assert!(snap.thrust_up);
    assert!(!snap.thrust_full);
    assert!(snap.pitch_up);
    assert!(snap.roll_right);
    assert!(snap.yaw_left);
    assert!(!snap.reset);

    let mut mapper = ControlMapper::default();
    let cmd = mapper.apply(&snap, 1.0);
    assert!(cmd.throttle > 0.0);
    assert_eq!(cmd.pitch, 1.0);
    assert_eq!(cmd.yaw, -1.0);
    assert_eq!(cmd.roll, 1.0);
}

#[test]
fn f_key_reaches_full_throttle_in_500ms() {
    let mut mapper = ControlMapper::default();
    let keys = KeySnapshot {
        thrust_full: true,
        ..Default::default()
    };
    // Just under the ramp: should not quite hit 1.0 yet.
    let almost = mapper.apply(&keys, FULL_THROTTLE_RAMP_S - 0.02);
    assert!(
        almost.throttle > 0.9 && almost.throttle < 1.0,
        "near end of ramp: {}",
        almost.throttle
    );
    let full = mapper.apply(&keys, 0.02);
    assert!(
        (full.throttle - 1.0).abs() < 1e-9,
        "F held for 500ms from zero should reach full, got {}",
        full.throttle
    );
    // Release holds at full.
    let held = mapper.apply(&KeySnapshot::default(), 0.1);
    assert!((held.throttle - 1.0).abs() < 1e-9);
}

#[test]
fn f_key_from_partial_reaches_full_sooner() {
    let mut mapper = ControlMapper {
        command: ControlCommand {
            throttle: 0.5,
            ..Default::default()
        },
        ..Default::default()
    };
    let keys = KeySnapshot {
        thrust_full: true,
        ..Default::default()
    };
    // Remaining 0.5 at rate 1/0.5 ⇒ 0.25 s to full.
    let cmd = mapper.apply(&keys, 0.25);
    assert!(
        (cmd.throttle - 1.0).abs() < 1e-9,
        "from 0.5, F for 250ms should hit full, got {}",
        cmd.throttle
    );
}

#[test]
fn mapped_throttle_affects_shipped_physics() {
    let mut s = RocketState::resting_on_pad();
    let mut mapper = ControlMapper::default();
    let keys = KeySnapshot {
        thrust_up: true,
        ..Default::default()
    };
    // Raise throttle for 2s, then fly 3s at that command.
    for _ in 0..120 {
        let cmd = mapper.apply(&keys, 1.0 / 60.0);
        s.set_command(cmd);
        step_rocket(&mut s, 1.0 / 120.0);
    }
    assert!(
        s.command.throttle > 0.5,
        "throttle should be high after hold, {}",
        s.command.throttle
    );
    let y_mid = s.altitude();
    for _ in 0..360 {
        step_rocket(&mut s, 1.0 / 120.0);
    }
    // With high throttle (T/W approaches 1.5 when throttle→1), expect climb or at least
    // less fall than pure freefall would produce; at full-ish throttle should rise.
    if s.command.throttle > 0.7 {
        assert!(
            s.altitude() >= y_mid - 1.0,
            "high throttle should not freefall, mid={y_mid} now={}",
            s.altitude()
        );
    }
}

#[test]
fn hud_text_mentions_altitude_and_thrust() {
    let s = RocketState::resting_on_pad();
    let landing = pga_rocket::LandingAutopilot::default();
    let target = pga_rocket::TargetLandingAutopilot::default();
    let text = pga_rocket::mesh::hud_text(&s, &landing, &target, 60.0);
    assert!(text.contains("alt="), "HUD missing altitude: {text}");
    assert!(text.contains("thr="), "HUD missing thrust: {text}");
    assert!(text.contains("Space"), "HUD missing key help: {text}");
    assert!(text.contains("T:"), "HUD missing T key help: {text}");
}

#[test]
fn exhaust_plume_absent_at_zero_and_grows_with_throttle() {
    use pga_rocket::mesh::rocket_mesh;
    use pga_rocket::sim::ControlCommand;

    let mut s = RocketState::resting_on_pad();
    s.set_command(ControlCommand {
        throttle: 0.0,
        ..Default::default()
    });
    let (v0, _) = rocket_mesh(&s);

    s.set_command(ControlCommand {
        throttle: 0.3,
        ..Default::default()
    });
    let (v_lo, _) = rocket_mesh(&s);

    s.set_command(ControlCommand {
        throttle: 1.0,
        ..Default::default()
    });
    let (v_hi, _) = rocket_mesh(&s);

    assert!(
        v_lo.len() > v0.len(),
        "partial throttle should add flame verts (0={}, 0.3={})",
        v0.len(),
        v_lo.len()
    );
    assert!(
        v_hi.len() == v_lo.len(),
        "flame topology is fixed; only size changes (0.3={}, 1.0={})",
        v_lo.len(),
        v_hi.len()
    );

    // At full throttle the plume tip should reach farther below the body than at 30%.
    let min_y = |verts: &[pga_rocket::mesh::Vertex]| {
        verts
            .iter()
            .map(|v| v.pos[1])
            .fold(f32::INFINITY, f32::min)
    };
    assert!(
        min_y(&v_hi) < min_y(&v_lo) - 1.0,
        "full throttle flame should extend farther: lo={} hi={}",
        min_y(&v_lo),
        min_y(&v_hi)
    );
}
