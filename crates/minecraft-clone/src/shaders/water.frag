#version 450

// Procedural water surface: depth-tinted color, animated wave normals, Fresnel sky
// reflection, and a sun glint. No atlas sampling.

layout(location = 0) in vec3 v_world;
layout(location = 1) in vec2 v_data;   // x = column depth 0..1, y = surface flag
layout(location = 2) in float v_light;

layout(push_constant) uniform PC {
    mat4 view_proj;
    vec4 camera_pos;   // w = 1 when the camera is underwater
    vec4 fog_color;
    vec4 fog_params;   // x = fog_start, y = fog_end, z = time (seconds), w = sea level
    vec4 chunk_origin; // used by the vertex stage
} pc;

layout(location = 0) out vec4 out_color;

// Keep in sync with SUN_DIR in mesher.rs.
const vec3 SUN_DIR = normalize(vec3(0.45, 0.85, 0.30));

// --- Random wave field (wave_height kept in sync with water.vert) ---

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

// Geometry waves: identical to the vertex displacement.
float wave_height(vec2 p, float t) {
    float h = 0.0;
    h += (vnoise(p * 0.35 + vec2( 0.25,  0.18) * t) - 0.5) * 0.60;
    h += (vnoise(p * 0.85 + vec2(-0.32,  0.24) * t) - 0.5) * 0.40;
    return h * 0.12;
}

// Shading waves: geometry waves plus fine ripples that live only in the normal
// (too small for the 1-block vertex grid, smooth per-pixel).
float wave_height_detail(vec2 p, float t) {
    return wave_height(p, t)
         + (vnoise(p * 2.60 + vec2(0.55, -0.40) * t) - 0.5) * 0.030
         + (vnoise(p * 5.10 + vec2(-0.62, 0.51) * t) - 0.5) * 0.012;
}

// Normal from central differences, with slope boosted for lively glints.
vec3 wave_normal(vec2 p, float t) {
    const float e = 0.12;
    float hx = wave_height_detail(p + vec2(e, 0.0), t) - wave_height_detail(p - vec2(e, 0.0), t);
    float hz = wave_height_detail(p + vec2(0.0, e), t) - wave_height_detail(p - vec2(0.0, e), t);
    const float slope_boost = 2.2;
    return normalize(vec3(-hx / (2.0 * e) * slope_boost, 1.0, -hz / (2.0 * e) * slope_boost));
}

void main() {
    float t = pc.fog_params.z;
    float depth = v_data.x;
    float surface = v_data.y;
    float underwater = pc.camera_pos.w;

    vec3 n = wave_normal(v_world.xz, t);

    vec3 to_cam = pc.camera_pos.xyz - v_world;
    float dist = length(to_cam);
    vec3 view = to_cam / max(dist, 1e-4);
    if (!gl_FrontFacing) {
        n = -n; // seen from below
    }

    // Depth-based body color: bright teal shallows to dark blue depths.
    vec3 shallow = vec3(0.10, 0.42, 0.50);
    vec3 deep = vec3(0.02, 0.13, 0.25);
    vec3 col = mix(shallow, deep, depth);

    // Fresnel: grazing angles reflect the sky/horizon (only meaningful from above).
    float fres = pow(1.0 - max(dot(view, n), 0.0), 3.0);
    float from_above = surface * (1.0 - underwater);
    col = mix(col, pc.fog_color.rgb, fres * 0.6 * from_above);

    // Sun glint on the wavy surface.
    vec3 h = normalize(SUN_DIR + view);
    float spec = pow(max(dot(n, h), 0.0), 240.0);
    col += vec3(1.0, 0.95, 0.80) * spec * 1.2 * from_above;

    col *= 0.75 + 0.25 * v_light;

    float fog = clamp((dist - pc.fog_params.x) / (pc.fog_params.y - pc.fog_params.x), 0.0, 1.0);
    col = mix(col, pc.fog_color.rgb, fog);

    // Deeper water is more opaque; grazing reflections read nearly solid. From below
    // the surface is a thin bright film.
    float alpha = mix(0.55, 0.88, depth);
    alpha = mix(alpha, 0.95, fres * from_above);
    alpha = mix(alpha, 0.40, underwater);
    out_color = vec4(col, alpha);
}
