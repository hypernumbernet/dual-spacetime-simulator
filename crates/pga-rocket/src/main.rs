#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
// Binary recompiles pure sim modules also used via the lib/tests; quiet unused public APIs.
#![allow(dead_code)]

//! PGA rocket launch/landing simulator with keyboard control.

mod app;
mod control;
mod euclidean_pga;
mod integration;
mod mesh;
mod renderer;
mod sim;
mod texture;
mod ui;

use winit::event_loop::EventLoop;

fn main() -> Result<(), winit::error::EventLoopError> {
    let event_loop = EventLoop::new()?;
    let mut app = app::App::default();
    event_loop.run_app(&mut app)
}
