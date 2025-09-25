#version 450
layout(location = 0) in vec3 position;
layout(location = 1) in vec4 color;

layout(location = 0) out vec4 v_color;

layout(push_constant) uniform PushConstants {
    mat4 view_proj;
    float size_scale;
} push;

void main() {
    gl_Position = push.view_proj * vec4(position, 1.0);
    v_color = color;
}