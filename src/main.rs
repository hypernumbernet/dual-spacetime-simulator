#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

fn main() -> Result<(), winit::error::EventLoopError> {
    dual_spacetime_simulator::run()
}
