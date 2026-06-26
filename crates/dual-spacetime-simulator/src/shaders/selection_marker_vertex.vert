#version 450

struct Particle {
    vec4 position;
    vec4 velocity;
    vec4 attrs;
    vec4 color;
};

layout(std430, binding = 0) readonly buffer Particles {
    Particle particles[];
};

layout(location = 0) out vec4 v_color;

layout(push_constant) uniform PushConstants {
    mat4 view_proj;
    vec4 camera_position;
    vec4 camera_right;
    vec4 camera_up;
    vec4 bracket_params;
} push;

const vec4 MARKER_COLOR = vec4(1.0, 1.0, 1.0, 1.0);
const float OFF_SCREEN = -1e10;
const float CORNER_ARM_RATIO = 0.35;

vec2 bracket_local(int vertex_index) {
    int seg = vertex_index / 2;
    int end = vertex_index - seg * 2;
    float r = CORNER_ARM_RATIO;
    float s = 1.0;

    switch (seg) {
        case 0:
            return (end == 0) ? vec2(-s, -s) : vec2(-s + s * r, -s);
        case 1:
            return (end == 0) ? vec2(s - s * r, -s) : vec2(s, -s);
        case 2:
            return (end == 0) ? vec2(s, -s) : vec2(s, -s + s * r);
        case 3:
            return (end == 0) ? vec2(s, s - s * r) : vec2(s, s);
        case 4:
            return (end == 0) ? vec2(s, s) : vec2(s - s * r, s);
        case 5:
            return (end == 0) ? vec2(-s + s * r, s) : vec2(-s, s);
        case 6:
            return (end == 0) ? vec2(-s, s) : vec2(-s, s - s * r);
        case 7:
            return (end == 0) ? vec2(-s, -s + s * r) : vec2(-s, -s);
        default:
            return vec2(0.0);
    }
}

void main() {
    v_color = MARKER_COLOR;

    int selected_index = floatBitsToInt(push.bracket_params.w);
    if (selected_index < 0) {
        gl_Position = vec4(OFF_SCREEN, OFF_SCREEN, 0.0, 1.0);
        return;
    }

    float size_scale = push.camera_right.w;
    float bracket_ratio = push.bracket_params.x;
    float min_half_size_px = push.bracket_params.y;
    float viewport_w = push.bracket_params.z;
    float viewport_h = push.camera_up.w;

    vec3 center_sim = particles[selected_index].position.xyz;
    vec4 clip_center = push.view_proj * vec4(center_sim, 1.0);
    if (clip_center.w <= 0.0) {
        gl_Position = vec4(OFF_SCREEN, OFF_SCREEN, 0.0, 1.0);
        return;
    }

    float view_depth = abs(clip_center.w);
    float particle_diameter_px = size_scale / max(view_depth, 1.0e-6);
    float half_px = max(min_half_size_px, particle_diameter_px * bracket_ratio);

    vec2 local = bracket_local(gl_VertexIndex);
    vec2 clip_offset = vec2(
        local.x * half_px * 2.0 / max(viewport_w, 1.0),
        -local.y * half_px * 2.0 / max(viewport_h, 1.0)
    ) * view_depth;
    gl_Position = clip_center + vec4(clip_offset, 0.0, 0.0);
}
