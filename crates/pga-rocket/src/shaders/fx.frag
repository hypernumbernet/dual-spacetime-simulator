#version 450

layout(location = 0) in vec4 v_color;
layout(location = 0) out vec4 out_color;

void main() {
    // Color is premultiplied on the CPU; blend is (ONE, ONE_MINUS_SRC_ALPHA).
    out_color = v_color;
}
