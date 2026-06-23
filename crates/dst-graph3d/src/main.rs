#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

/// Starts the 3D graph viewer application event loop.
fn main() -> Result<(), winit::error::EventLoopError> {
    dst_graph3d::run()
}
