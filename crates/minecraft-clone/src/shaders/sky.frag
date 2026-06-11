#version 450

layout(location = 0) in vec3 v_ray;

layout(push_constant) uniform SkyPC {
    vec4 cam_right;
    vec4 cam_up;
    vec4 cam_fwd;
    vec4 sun_dir;
    vec4 params; // x = tan(fov/2), y = aspect, z = underwater flag
} pc;

layout(location = 0) out vec4 out_color;

void main() {
    vec3 dir = normalize(v_ray);

    if (pc.params.z > 0.5) {
        // Underwater backdrop: murky depths below, brighter filtered light above.
        vec3 deep = vec3(0.01, 0.08, 0.14);
        vec3 lit = vec3(0.06, 0.28, 0.38);
        vec3 col = mix(deep, lit, clamp(dir.y * 0.8 + 0.55, 0.0, 1.0));
        float s = max(dot(dir, normalize(pc.sun_dir.xyz)), 0.0);
        col += vec3(0.20, 0.30, 0.30) * pow(s, 10.0); // wavering sunbeam glow
        out_color = vec4(col, 1.0);
        return;
    }

    vec3 zenith = vec3(0.27, 0.47, 0.78);
    vec3 horizon = vec3(0.74, 0.84, 0.94);
    vec3 haze = vec3(0.62, 0.66, 0.68);

    // Vertical gradient: horizon band up to deep zenith; gentle haze below the horizon.
    float up = smoothstep(0.0, 0.5, dir.y);
    vec3 col = mix(horizon, zenith, up);
    col = mix(col, haze, clamp(-dir.y * 2.0, 0.0, 1.0));

    // Sun disk + halo.
    float s = max(dot(dir, normalize(pc.sun_dir.xyz)), 0.0);
    float halo = pow(s, 8.0) * 0.18 + pow(s, 48.0) * 0.6;
    float disk = smoothstep(0.9968, 0.9986, s);
    col += vec3(1.0, 0.96, 0.84) * (halo + disk * 1.3);

    out_color = vec4(col, 1.0);
}
