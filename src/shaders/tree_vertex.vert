#version 450

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec4 color;

layout(location = 0) out vec4 v_color;
layout(location = 1) out vec3 v_normal;
layout(location = 2) out vec3 v_frag_pos;

layout(push_constant) uniform PushConstants {
    mat4 view_proj;
} push;

void main() {
    gl_Position = push.view_proj * vec4(position, 1.0);
    v_color = color;
    v_normal = normal;
    v_frag_pos = position;
}
