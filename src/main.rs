#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

/// Starts the simulator application event loop.
fn main() -> Result<(), winit::error::EventLoopError> {
    dual_spacetime_simulator::run()
}
