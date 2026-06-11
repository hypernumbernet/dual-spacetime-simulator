#version 450

// Water vertex shader: random, organic wave displacement on surface-flagged vertices.
// Compact input: integer chunk-local position + light*255 (u16x4); the UV slot carries
// (column depth * 512, surface flag * 512) instead of atlas coordinates.

layout(location = 0) in uvec4 in_pos_light; // xyz = local block coords, w = light*255
layout(location = 1) in uvec2 in_uv_q;      // x = depth*512, y = surface flag (0 or 512)

layout(push_constant) uniform PC {
    mat4 view_proj;
    vec4 camera_pos;   // w = 1 when the camera is underwater
    vec4 fog_color;
    vec4 fog_params;   // x = fog_start, y = fog_end, z = time (seconds), w = sea level
    vec4 chunk_origin; // xyz = world position of the chunk's min corner
} pc;

layout(location = 0) out vec3 v_world;
layout(location = 1) out vec2 v_data;   // x = depth, y = surface flag
layout(location = 2) out float v_light;

// --- Random wave field (keep wave_height in sync with water.frag) ---

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

// Two noise layers drifting in different directions: irregular, non-repeating
// undulation. A pure function of world xz and time, so shared corners of
// neighboring quads displace identically (no cracks).
float wave_height(vec2 p, float t) {
    float h = 0.0;
    h += (vnoise(p * 0.35 + vec2( 0.25,  0.18) * t) - 0.5) * 0.60;
    h += (vnoise(p * 0.85 + vec2(-0.32,  0.24) * t) - 0.5) * 0.40;
    return h * 0.12; // ±0.06 — stays under the 0.1 shoreline inset below
}

void main() {
    vec3 p = pc.chunk_origin.xyz + vec3(in_pos_light.xyz);
    float t = pc.fog_params.z;
    if (in_uv_q.y > 256u) {
        // Water surface sits slightly below the block top so shorelines read clearly,
        // then waves displace it.
        p.y -= 0.1;
        p.y += wave_height(p.xz, t);
    }
    gl_Position = pc.view_proj * vec4(p, 1.0);
    v_world = p;
    v_data = vec2(in_uv_q) / 512.0;
    v_light = float(in_pos_light.w) / 255.0;
}
