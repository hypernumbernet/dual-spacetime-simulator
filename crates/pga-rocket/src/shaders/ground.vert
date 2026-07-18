#version 450

// Local-plane vertex; world XZ comes from local pos + center push constant.
layout(location = 0) in vec3 in_pos;
layout(location = 1) in vec2 in_uv;

// Push layout (128 bytes). See ground.frag for .w packing of target pad.
layout(push_constant) uniform PC {
    mat4 view_proj;
    vec4 camera_pos;  // xyz = eye; w = target pad X (FS only)
    vec4 fog_color;   // rgb sky/horizon tint
    vec4 fog_params;  // x = fog_start, y = fog_end, z = grass_mpt, w = paved_mpt
    vec4 ground_origin; // x/z = plane recenter; y = 0; w = target pad Z (FS only)
} pc;

layout(location = 0) out vec2 v_uv;
layout(location = 1) out float v_dist;
layout(location = 2) out vec3 v_world;

void main() {
    // Only xyz offset the mesh; .w carries target_z for the fragment stage.
    vec3 world = in_pos + vec3(pc.ground_origin.x, pc.ground_origin.y, pc.ground_origin.z);
    gl_Position = pc.view_proj * vec4(world, 1.0);
    // World-space tiling so the grass pattern is continuous as the plane recenters.
    float mpt = max(pc.fog_params.z, 0.001);
    v_uv = world.xz / mpt;
    v_dist = distance(world, pc.camera_pos.xyz);
    v_world = world;
}
