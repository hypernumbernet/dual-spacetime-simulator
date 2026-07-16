#version 450

// Local-plane vertex; world XZ comes from local pos + center push constant.
layout(location = 0) in vec3 in_pos;
layout(location = 1) in vec2 in_uv;

layout(push_constant) uniform PC {
    mat4 view_proj;
    vec4 camera_pos;  // xyz = eye, w unused
    vec4 fog_color;   // rgb sky/horizon tint
    vec4 fog_params;  // x = fog_start, y = fog_end, z = meters_per_tile
    vec4 ground_origin; // xyz = world origin of this mesh (snapped under rocket)
} pc;

layout(location = 0) out vec2 v_uv;
layout(location = 1) out float v_dist;
layout(location = 2) out vec3 v_world;

void main() {
    vec3 world = in_pos + pc.ground_origin.xyz;
    gl_Position = pc.view_proj * vec4(world, 1.0);
    // World-space tiling so the grass pattern is continuous as the plane recenters.
    float mpt = max(pc.fog_params.z, 0.001);
    v_uv = world.xz / mpt;
    v_dist = distance(world, pc.camera_pos.xyz);
    v_world = world;
}
