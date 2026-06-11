//! Shader module creation from SPIR-V bytecode.

use ash::vk;

/// Creates a Vulkan shader module from SPIR-V bytecode.
pub fn create_shader_module(device: &ash::Device, spv: &[u8]) -> vk::ShaderModule {
    let code = ash::util::read_spv(&mut std::io::Cursor::new(spv)).unwrap();
    let ci = vk::ShaderModuleCreateInfo::default().code(&code);
    unsafe { device.create_shader_module(&ci, None) }.unwrap()
}
