#version 450

// Local-plane vertex; world XZ comes from local pos * scale + center push constant.
layout(location = 0) in vec3 in_pos;
layout(location = 1) in vec2 in_uv;

// Push layout (128 bytes). See ground.frag for packing of target pad / edge fog.
layout(push_constant) uniform PC {
    mat4 view_proj;
    vec4 camera_pos;  // xyz = eye; w = target pad X (FS only)
    vec4 fog_color;   // rgb sky/horizon tint; a = plane_scale
    vec4 fog_params;  // x = edge_fog_start, y = half_extent_world, z = grass_mpt, w = paved_mpt
    vec4 ground_origin; // x/z = plane recenter; y = 0; w = target pad Z (FS only)
} pc;

layout(location = 0) out vec2 v_uv;
layout(location = 1) out float v_edge;
layout(location = 2) out vec3 v_world;

void main() {
    // Scale local XZ so the disk grows with altitude; .w on fog_color carries scale.
    float scale = max(pc.fog_color.a, 0.001);
    vec3 world = vec3(in_pos.x * scale, in_pos.y, in_pos.z * scale)
        + vec3(pc.ground_origin.x, pc.ground_origin.y, pc.ground_origin.z);
    gl_Position = pc.view_proj * vec4(world, 1.0);
    // World-space tiling so the grass pattern is continuous as the plane recenters.
    float mpt = max(pc.fog_params.z, 0.001);
    v_uv = world.xz / mpt;
    // Horizontal edge factor: 0 at center, 1 at the rim of the effective disk.
    float half_w = max(pc.fog_params.y, 1.0);
    vec2 d = world.xz - pc.ground_origin.xz;
    v_edge = length(d) / half_w;
    v_world = world;
}
