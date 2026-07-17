use std::env;
use std::fs;
use std::path::Path;
use std::process::Command;

/// Compiles GLSL shaders to SPIR-V via `glslc` (Vulkan SDK) into `OUT_DIR/shaders`.
fn main() {
    let shader_dir = Path::new("src/shaders");
    let out_dir = env::var("OUT_DIR").unwrap();
    let spv_dir = Path::new(&out_dir).join("shaders");
    fs::create_dir_all(&spv_dir).unwrap();

    let shaders = [
        "mesh.vert",
        "mesh.frag",
        "ground.vert",
        "ground.frag",
        "fx.vert",
        "fx.frag",
    ];

    for shader in &shaders {
        let input = shader_dir.join(shader);
        if !input.exists() {
            continue;
        }
        let output = spv_dir.join(format!("{shader}.spv"));

        println!("cargo:rerun-if-changed={}", input.display());

        let status = Command::new("glslc")
            .arg(input.to_str().unwrap())
            .arg("-o")
            .arg(output.to_str().unwrap())
            .status()
            .expect(
                "Failed to execute glslc. Make sure VulkanSDK is installed and glslc is in PATH.",
            );

        if !status.success() {
            panic!("Failed to compile shader: {shader}");
        }
    }
}
