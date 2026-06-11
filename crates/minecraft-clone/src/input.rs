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

    /// Clears per-frame edge state and consumed mouse delta. Call at end of each frame.
    pub fn end_frame(&mut self) {
        self.just_pressed.clear();
        self.mouse_delta = (0.0, 0.0);
    }
}
