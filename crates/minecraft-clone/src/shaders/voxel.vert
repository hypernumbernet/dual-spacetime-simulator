#version 450

// Compact vertex input: integer chunk-local position + light*255 (u16x4), and
// uv * 512 (u16x2). The chunk's world origin arrives via per-draw push constants.

layout(location = 0) in uvec4 in_pos_light; // xyz = local block coords, w = light*255
layout(location = 1) in uvec2 in_uv_q;      // uv * 512

layout(push_constant) uniform PC {
    mat4 view_proj;
    vec4 camera_pos;   // w = 1 when the camera is underwater
    vec4 fog_color;
    vec4 fog_params;   // x = fog_start, y = fog_end, z = time (seconds), w = sea level
    vec4 chunk_origin; // xyz = world position of the chunk's min corner
} pc;

layout(location = 0) out vec2 v_uv;
layout(location = 1) out float v_light;
layout(location = 2) out float v_dist;
layout(location = 3) out vec3 v_world;

void main() {
    vec3 world = pc.chunk_origin.xyz + vec3(in_pos_light.xyz);
    gl_Position = pc.view_proj * vec4(world, 1.0);
    v_uv = vec2(in_uv_q) / 512.0;
    v_light = float(in_pos_light.w) / 255.0;
    v_dist = distance(world, pc.camera_pos.xyz);
    v_world = world;
}
