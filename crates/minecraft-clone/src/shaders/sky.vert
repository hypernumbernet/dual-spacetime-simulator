#version 450

// Fullscreen triangle (no vertex buffer). Reconstructs a per-pixel view ray from the
// camera basis so the fragment shader can shade a gradient sky with a sun.

layout(push_constant) uniform SkyPC {
    vec4 cam_right; // xyz
    vec4 cam_up;    // xyz
    vec4 cam_fwd;   // xyz
    vec4 sun_dir;   // xyz
    vec4 params;    // x = tan(fov/2), y = aspect
} pc;

layout(location = 0) out vec3 v_ray;

void main() {
    // NDC positions covering the whole screen: (-1,-1), (3,-1), (-1,3).
    vec2 ndc = vec2(gl_VertexIndex == 1 ? 3.0 : -1.0,
                    gl_VertexIndex == 2 ? 3.0 : -1.0);
    gl_Position = vec4(ndc, 0.0, 1.0);

    float tan_h = pc.params.x;
    float aspect = pc.params.y;
    // Vulkan NDC y points down, so top of screen (ndc.y = -1) maps to camera up.
    v_ray = pc.cam_fwd.xyz
          + pc.cam_right.xyz * (ndc.x * tan_h * aspect)
          + pc.cam_up.xyz    * (-ndc.y * tan_h);
}
