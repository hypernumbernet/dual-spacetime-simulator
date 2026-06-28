#version 450

struct Particle {
    vec4 position;
    vec4 velocity;
    vec4 attrs;
    vec4 dual_state;
    vec4 color;
};

layout(std430, binding = 0) readonly buffer Particles {
    Particle particles[];
};

layout(location = 0) out vec4 v_color;

layout(push_constant) uniform PushConstants {
    mat4 view_proj;
    float size_scale;
} push;

void main() {
    Particle p = particles[gl_VertexIndex];
    gl_Position = push.view_proj * vec4(p.position.xyz, 1.0);
    gl_PointSize = push.size_scale / gl_Position.w;
    v_color = p.color;
}
