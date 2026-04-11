#version 450

layout(location = 0) in vec4 v_color;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec3 v_frag_pos;

layout(location = 0) out vec4 f_color;

void main() {
    vec3 normal = normalize(v_normal);
    vec3 light_dir = normalize(vec3(0.5, 1.0, 0.8));
    float diffuse = max(dot(normal, light_dir), 0.3);
    vec3 lighting = v_color.rgb * diffuse;
    f_color = vec4(lighting, v_color.a);
}
