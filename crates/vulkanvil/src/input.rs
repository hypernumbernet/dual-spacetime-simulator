//! Keyboard/mouse state: held keys, edge-triggered presses, accumulated raw mouse delta.

use std::collections::HashSet;
use winit::event::ElementState;
use winit::keyboard::KeyCode;

#[derive(Default)]
pub struct InputState {
    held: HashSet<KeyCode>,
    just_pressed: HashSet<KeyCode>,
    pub mouse_delta: (f64, f64),
}

impl InputState {
    /// Records a key transition. Press edges are added to `just_pressed` for this frame.
    pub fn key_event(&mut self, code: KeyCode, state: ElementState) {
        match state {
            ElementState::Pressed => {
                if self.held.insert(code) {
                    self.just_pressed.insert(code);
                }
            }
            ElementState::Released => {
                self.held.remove(&code);
            }
        }
    }

    #[inline]
    pub fn held(&self, code: KeyCode) -> bool {
        self.held.contains(&code)
    }

    /// True once on the frame a key was first pressed.
    #[inline]
    pub fn just_pressed(&self, code: KeyCode) -> bool {
        self.just_pressed.contains(&code)
    }

    /// Returns `+1`, `0`, or `-1` from a positive/negative key pair (both held => `0`).
    #[inline]
    pub fn axis(&self, positive: KeyCode, negative: KeyCode) -> f32 {
        (self.held(positive) as i32 - self.held(negative) as i32) as f32
    }

    /// Returns `+1` (Space), `-1` (Shift), or `0` for forward/back camera thrust.
    #[inline]
    pub fn space_shift_thrust(&self, suppressed: bool) -> f32 {
        if suppressed {
            return 0.0;
        }
        if self.held(KeyCode::Space) {
            1.0
        } else if self.held(KeyCode::ShiftLeft) || self.held(KeyCode::ShiftRight) {
            -1.0
        } else {
            0.0
        }
    }

    /// Returns `+1` (Shift), `-1` (Space), or `0` for orbit-mode vertical motion.
    #[inline]
    pub fn space_shift_vertical_axis(&self, suppressed: bool) -> f32 {
        if suppressed {
            0.0
        } else {
            (self.held(KeyCode::ShiftLeft) || self.held(KeyCode::ShiftRight)) as i32 as f32
                - self.held(KeyCode::Space) as i32 as f32
        }
    }

    /// Clears per-frame edge state and consumed mouse delta. Call at end of each frame.
    pub fn end_frame(&mut self) {
        self.just_pressed.clear();
        self.mouse_delta = (0.0, 0.0);
    }
}
