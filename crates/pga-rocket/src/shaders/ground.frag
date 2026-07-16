#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 1) in float v_dist;
layout(location = 2) in vec3 v_world;

layout(set = 0, binding = 0) uniform sampler2D grass;

layout(push_constant) uniform PC {
    mat4 view_proj;
    vec4 camera_pos;
    vec4 fog_color;
    vec4 fog_params; // x = fog_start, y = fog_end, z = meters_per_tile
    vec4 ground_origin;
} pc;

layout(location = 0) out vec4 out_color;

// Soft large-scale variation (same hash idea as minecraft water/terrain noise).
float lhash(vec2 cell) {
    uvec2 q = uvec2(ivec2(floor(cell))) * uvec2(1597334673u, 3812015801u);
    return float((q.x ^ q.y) * 1597334673u) * (1.0 / 4294967295.0);
}

float vnoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    float a = lhash(i);
    float b = lhash(i + vec2(1.0, 0.0));
    float c = lhash(i + vec2(0.0, 1.0));
    float d = lhash(i + vec2(1.0, 1.0));
    return a + (b - a) * u.x + (c - a) * u.y + (a - b - c + d) * u.x * u.y;
}

void main() {
    vec3 grass_col = texture(grass, v_uv).rgb;

    // Broad meadow patches so the field is not a flat repeating stamp.
    float meadow = 0.88 + 0.18 * vnoise(v_world.xz * 0.04);
    float shade = 0.94 + 0.10 * vnoise(v_world.xz * 0.17 + 17.0);
    vec3 lit = grass_col * meadow * shade;

    // Distance fog into the sky color hides the finite plane edge (no hard horizon line).
    float fog = clamp(
        (v_dist - pc.fog_params.x) / max(pc.fog_params.y - pc.fog_params.x, 1.0),
        0.0,
        1.0
    );
    // Smoothstep softens the last band so no linear “stripe” appears.
    fog = fog * fog * (3.0 - 2.0 * fog);
    out_color = vec4(mix(lit, pc.fog_color.rgb, fog), 1.0);
}
