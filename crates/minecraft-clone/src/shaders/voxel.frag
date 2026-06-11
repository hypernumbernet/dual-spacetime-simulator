#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 1) in float v_light;
layout(location = 2) in float v_dist;
layout(location = 3) in vec3 v_world;

layout(set = 0, binding = 0) uniform sampler2D atlas;

layout(push_constant) uniform PC {
    mat4 view_proj;
    vec4 camera_pos;   // w = 1 when the camera is underwater
    vec4 fog_color;
    vec4 fog_params;   // x = fog_start, y = fog_end, z = time (seconds), w = sea level
    vec4 chunk_origin; // used by the vertex stage
} pc;

layout(location = 0) out vec4 out_color;

// --- Noise (same construction as water.vert/water.frag — keep in sync) ---

// Integer-lattice hash: robust far from the origin (no fract precision decay).
float lhash(vec2 cell) {
    uvec2 q = uvec2(ivec2(cell)) * uvec2(1597334673u, 3812015801u);
    return float((q.x ^ q.y) * 1597334673u) * (1.0 / 4294967295.0);
}

// Smooth value noise in [0, 1] with quintic fade.
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

// Ridges along the noise mid-level contours: filament-like bright lines.
float ridge(vec2 p) {
    return 1.0 - abs(vnoise(p) * 2.0 - 1.0);
}

// Caustic web: two ridge fields drifting in different directions, multiplied so the
// bright filaments continuously merge, split, and wander — never a repeating pattern.
float caustics(vec2 p, float t) {
    float c1 = ridge(p * 0.55 + vec2( 0.21,  0.16) * t);
    float c2 = ridge(p * 0.85 + vec2(-0.17,  0.23) * t);
    return pow(c1 * c2, 3.0);
}

void main() {
    vec3 lit = texture(atlas, v_uv).rgb * v_light;

    float underwater = pc.camera_pos.w;
    if (underwater > 0.5) {
        // Blue-green absorption tint, then shimmering caustic light.
        lit *= vec3(0.55, 0.78, 0.88);
        float t = pc.fog_params.z;
        float ca = caustics(v_world.xz, t);
        // Large-scale drifting patches: light concentration varies across the seabed.
        // (named "blotch" because `patch` is a reserved word in GLSL)
        float blotch = 0.45 + 0.55 * vnoise(v_world.xz * 0.13 + vec2(0.06, -0.05) * t);
        // Caustics fade out ~32 blocks below the sea surface (light absorption is a
        // physical length, so it does not scale with WORLD_SCALE).
        float sea = pc.fog_params.w;
        float depth_fade = clamp((v_world.y - (sea - 32.0)) / 32.0, 0.0, 1.0);
        // v_light^2 focuses the effect on upward faces, leaving a faint dance on walls.
        lit += vec3(0.50, 0.70, 0.68) * ca * blotch * depth_fade * v_light * v_light;
    }

    float fog = clamp((v_dist - pc.fog_params.x) / (pc.fog_params.y - pc.fog_params.x), 0.0, 1.0);
    out_color = vec4(mix(lit, pc.fog_color.rgb, fog), 1.0);
}
