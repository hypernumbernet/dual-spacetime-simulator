#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

//! A small Minecraft-style voxel world built on the `vulkanvil` crate's
//! Vulkan 3D foundation.

mod app;
mod block;
mod chunk;
mod hud;
mod input;
mod mesher;
mod player;
mod renderer;
mod texture;
mod world;
mod worldgen;

use winit::event_loop::EventLoop;

fn main() -> Result<(), winit::error::EventLoopError> {
    let event_loop = EventLoop::new()?;
    let mut app = app::App::default();
    event_loop.run_app(&mut app)
}
