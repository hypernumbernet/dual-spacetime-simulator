#version 450
layout(location = 0) in vec4 v_color;

layout(location = 0) out vec4 f_color;

void main() {
    vec2 coord = gl_PointCoord - vec2(0.5);
    float dist = length(coord);
    if (dist > 0.9) discard;
    float intensity = 1.0 - pow(dist * 2.0, 2.0);
    float core = exp(-dist * 8.0);
    intensity = intensity + core * 0.5;
    float energyFalloff = exp(-dist * 4.0);
    vec3 color = v_color.rgb * intensity * energyFalloff;
    float alpha = energyFalloff * v_color.a;
    f_color = vec4(color, alpha);
}