#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 1) in float v_edge;
layout(location = 2) in vec3 v_world;

layout(set = 0, binding = 0) uniform sampler2D grass;
layout(set = 0, binding = 1) uniform sampler2D paved;
layout(set = 0, binding = 2) uniform sampler2D moon;

// Push layout (128 bytes, shared with VS). Unused .w slots carry pad target.
//   camera_pos.w  = target pad world X
//   fog_color.a   = plane_scale (VS)
//   fog_params.x  = edge fog start ratio
//   fog_params.y  = half_extent_world
//   fog_params.z  = grass meters-per-tile
//   fog_params.w  = paved meters-per-tile
//   ground_origin.y = moon mode (1 = lunar regolith, 0 = grass)
//   ground_origin.w = target pad world Z
layout(push_constant) uniform PC {
    mat4 view_proj;
    vec4 camera_pos;
    vec4 fog_color;
    vec4 fog_params;
    vec4 ground_origin;
} pc;

layout(location = 0) out vec4 out_color;

// Keep in sync with mesh.rs LAUNCH_PAD_HALF_EXTENT / pad mark geometry.
const float PAD_HALF = 30.0;
const vec3 MARK_COLOR = vec3(0.95, 0.82, 0.12);

// Cheap large-scale field variation (detail lives in the mipmapped albedo).
float lhash(vec2 cell) {
    uvec2 q = uvec2(ivec2(floor(cell))) * uvec2(1597334673u, 3812015801u);
    return float((q.x ^ q.y) * 1597334673u) * (1.0 / 4294967295.0);
}

float vnoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    // Hermite smoothstep (cheaper than full quintic; fine for soft field tint).
    vec2 u = f * f * (3.0 - 2.0 * f);
    float a = lhash(i);
    float b = lhash(i + vec2(1.0, 0.0));
    float c = lhash(i + vec2(0.0, 1.0));
    float d = lhash(i + vec2(1.0, 1.0));
    return a + (b - a) * u.x + (c - a) * u.y + (a - b - c + d) * u.x * u.y;
}

// Two-scale albedo: hardware mips already kill high-frequency moiré; the second
// slower UV scale breaks phase-locked tile repeats at long range without a 3rd sample.
// `detail_uv` is world.xz / grass_mpt (from VS). Falloff uses cheap reciprocal, not exp.
vec3 dual_scale_albedo(sampler2D tex, vec2 detail_uv, float cam_dist) {
    vec3 detail = texture(tex, detail_uv).rgb;
    // 1/16 world frequency + phase offset so scales do not stack.
    vec3 broad = texture(tex, detail_uv * 0.0625 + vec2(0.71, 0.53)).rgb;
    // ~fade detail over a few hundred metres; residual broad keeps far field alive.
    float w = 1.0 / (1.0 + cam_dist * 0.004);
    return mix(broad, detail, w * 0.78 + 0.10);
}

bool in_aabb(vec2 p, vec2 center, vec2 half_ext) {
    vec2 d = abs(p - center);
    return d.x <= half_ext.x && d.y <= half_ext.y;
}

// Yellow "H" at launch origin (home pad).
bool home_h_mark(vec2 p) {
    if (in_aabb(p, vec2(-9.0, 0.0), vec2(3.0, 12.0))) return true;
    if (in_aabb(p, vec2(9.0, 0.0), vec2(3.0, 12.0))) return true;
    if (in_aabb(p, vec2(0.0, 0.0), vec2(12.0, 3.0))) return true;
    return false;
}

// Yellow "T" centered on the target pad.
bool target_t_mark(vec2 p, vec2 target) {
    if (in_aabb(p, target + vec2(0.0, -2.0), vec2(3.0, 11.0))) return true;
    if (in_aabb(p, target + vec2(0.0, 6.0), vec2(12.0, 3.0))) return true;
    return false;
}

void main() {
    // Edge fog first: fully fogged rim skips all texture / noise work.
    float edge_start = clamp(pc.fog_params.x, 0.0, 0.999);
    float fog = smoothstep(edge_start, 1.0, v_edge);
    if (fog >= 0.999) {
        out_color = vec4(pc.fog_color.rgb, 1.0);
        return;
    }

    vec2 xz = v_world.xz;
    vec2 target = vec2(pc.camera_pos.w, pc.ground_origin.w);
    // Horizontal range only (cheaper than full 3D length; matches ground plane).
    float cam_dist = length(xz - pc.camera_pos.xz);

    bool on_home = max(abs(xz.x), abs(xz.y)) <= PAD_HALF;
    bool on_target = max(abs(xz.x - target.x), abs(xz.y - target.y)) <= PAD_HALF;

    vec3 lit;
    if (on_home || on_target) {
        // Pads are ~60 m across: single LINEAR+mip sample is enough (no dual-scale).
        float paved_mpt = max(pc.fog_params.w, 0.001);
        lit = texture(paved, xz / paved_mpt).rgb;
        if ((on_home && home_h_mark(xz)) || (on_target && target_t_mark(xz, target))) {
            lit = MARK_COLOR;
        }
    } else if (pc.ground_origin.y > 0.5) {
        // Moon: dual-scale regolith + one low-frequency dust tint.
        vec3 moon_col = dual_scale_albedo(moon, v_uv, cam_dist);
        float dust = 0.88 + 0.16 * vnoise(xz * 0.03);
        // Sparse dark crater fields (threshold on same cheap noise family).
        float crater = 1.0 - 0.10 * step(0.90, vnoise(xz * 0.01 + 3.0));
        lit = moon_col * dust * crater;
    } else {
        // Earth meadow: dual-scale grass + one meadow patch tint.
        vec3 grass_col = dual_scale_albedo(grass, v_uv, cam_dist);
        float meadow = 0.90 + 0.14 * vnoise(xz * 0.035);
        lit = grass_col * meadow;
    }

    out_color = vec4(mix(lit, pc.fog_color.rgb, fog), 1.0);
}
