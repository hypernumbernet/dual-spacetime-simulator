#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

//! A small Minecraft-style voxel world built on the dual-spacetime-simulator crate's
//! Vulkan 3D foundation (copied `vulkan_base` + buffer helpers).

mod app;
mod block;
mod buffer;
mod chunk;
mod input;
mod mesher;
mod player;
mod renderer;
mod texture;
mod vulkan_base;
mod world;
mod worldgen;

use winit::event_loop::EventLoop;

fn main() -> Result<(), winit::error::EventLoopError> {
    let event_loop = EventLoop::new()?;
    let mut app = app::App::default();
    event_loop.run_app(&mut app)
}
