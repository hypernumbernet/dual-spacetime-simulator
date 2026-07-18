#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 1) in float v_edge;
layout(location = 2) in vec3 v_world;

layout(set = 0, binding = 0) uniform sampler2D grass;
layout(set = 0, binding = 1) uniform sampler2D paved;

// Push layout (128 bytes, shared with VS). Unused .w slots carry pad target.
//   camera_pos.w  = target pad world X
//   fog_color.a   = plane_scale (VS)
//   fog_params.x  = edge fog start ratio
//   fog_params.y  = half_extent_world
//   fog_params.z  = grass meters-per-tile
//   fog_params.w  = paved meters-per-tile
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

bool in_aabb(vec2 p, vec2 center, vec2 half_ext) {
    vec2 d = abs(p - center);
    return d.x <= half_ext.x && d.y <= half_ext.y;
}

// Yellow "H" at launch origin (home pad). Dimensions match former home_h_mark_mesh.
bool home_h_mark(vec2 p) {
    // Left / right uprights: half (3, 12), centers (±9, 0).
    if (in_aabb(p, vec2(-9.0, 0.0), vec2(3.0, 12.0))) return true;
    if (in_aabb(p, vec2(9.0, 0.0), vec2(3.0, 12.0))) return true;
    // Crossbar: half (12, 3), center (0, 0).
    if (in_aabb(p, vec2(0.0, 0.0), vec2(12.0, 3.0))) return true;
    return false;
}

// Yellow "T" centered on the target pad. Dimensions match former target_t_mark_mesh.
bool target_t_mark(vec2 p, vec2 target) {
    // Stem: half (3, 11), center (tx, tz - 2).
    vec2 stem_c = target + vec2(0.0, -2.0);
    if (in_aabb(p, stem_c, vec2(3.0, 11.0))) return true;
    // Crossbar on +Z end of stem: half (12, 3), center (tx, tz + 6).
    vec2 bar_c = target + vec2(0.0, 6.0);
    if (in_aabb(p, bar_c, vec2(12.0, 3.0))) return true;
    return false;
}

void main() {
    vec2 xz = v_world.xz;
    vec2 target = vec2(pc.camera_pos.w, pc.ground_origin.w);

    bool on_home = max(abs(xz.x), abs(xz.y)) <= PAD_HALF;
    bool on_target = max(abs(xz.x - target.x), abs(xz.y - target.y)) <= PAD_HALF;
    bool on_pad = on_home || on_target;

    vec3 lit;
    if (on_pad) {
        float paved_mpt = max(pc.fog_params.w, 0.001);
        vec2 paved_uv = xz / paved_mpt;
        lit = texture(paved, paved_uv).rgb;
        // Letter marks painted in-plane (no second mesh → no z-fighting).
        if ((on_home && home_h_mark(xz)) || (on_target && target_t_mark(xz, target))) {
            lit = MARK_COLOR;
        }
    } else {
        vec3 grass_col = texture(grass, v_uv).rgb;
        // Broad meadow patches so the field is not a flat repeating stamp.
        float meadow = 0.88 + 0.18 * vnoise(xz * 0.04);
        float shade = 0.94 + 0.10 * vnoise(xz * 0.17 + 17.0);
        lit = grass_col * meadow * shade;
    }

    // Edge fog hides the finite plane rim (circular fade); altitude-independent so
    // the ground under the rocket stays visible from any height.
    float edge_start = clamp(pc.fog_params.x, 0.0, 0.999);
    float fog = smoothstep(edge_start, 1.0, v_edge);
    out_color = vec4(mix(lit, pc.fog_color.rgb, fog), 1.0);
}
