#version 450
layout(location = 0) in vec4 v_color;

layout(location = 0) out vec4 f_color;

const vec3 LIGHT_DIR = normalize(vec3(-0.4, -0.6, 1.0));

void main() {
    vec2 coord = gl_PointCoord - vec2(0.5);
    float dist2 = dot(coord, coord);
    if (dist2 > 0.25) discard;

    float z = sqrt(0.25 - dist2) * 2.0;
    vec3 normal = vec3(coord * 2.0, z);
    float shade = max(dot(normal, LIGHT_DIR), 0.25);

    f_color = vec4(v_color.rgb * shade, 1.0);
}