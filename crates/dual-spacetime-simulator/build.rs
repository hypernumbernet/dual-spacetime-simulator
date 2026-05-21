use std::env;
use std::fs;
use std::path::Path;
use std::process::Command;

fn main() {
    let shader_dir = Path::new("src/shaders");
    let out_dir = env::var("OUT_DIR").unwrap();
    let spv_dir = Path::new(&out_dir).join("shaders");
    fs::create_dir_all(&spv_dir).unwrap();

    let shaders = [
        "axes_vertex.vert",
        "axes_fragment.frag",
        "particles_vertex.vert",
        "particles_fragment.frag",
        "tree_vertex.vert",
        "tree_fragment.frag",
        "tree_compute.comp",
        "egui_vertex.vert",
        "egui_fragment.frag",
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
