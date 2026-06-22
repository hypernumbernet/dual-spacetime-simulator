//! Developer HUD: ASCII text rendered with the public-domain `font8x8` bitmap font.
//!
//! The 96 printable glyphs are baked into a small RGBA atlas (alpha = coverage),
//! and each frame's text becomes a vertex list of textured quads in screen pixels
//! (top-left origin). A black drop-shadow copy is emitted first for readability.

use font8x8::legacy::BASIC_LEGACY;

pub const GLYPH_PX: u32 = 8;
const FONT_COLS: u32 = 16;
const FONT_ROWS: u32 = 6;
pub const FONT_W: u32 = FONT_COLS * GLYPH_PX;
pub const FONT_H: u32 = FONT_ROWS * GLYPH_PX;
const FIRST_CHAR: u32 = 32;
const LAST_CHAR: u32 = 127;

/// On-screen glyph scale factor (8px glyphs -> 16px).
pub const TEXT_SCALE: f32 = 2.0;
/// HUD top-left corner in screen pixels.
pub const TEXT_MARGIN: f32 = 8.0;
const LINE_HEIGHT: f32 = 10.0; // in glyph pixels, scaled

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct TextVertex {
    pub pos: [f32; 2],
    pub uv: [f32; 2],
    pub color: [f32; 4],
}

/// Generates the RGBA8 font atlas: white pixels with alpha as glyph coverage.
pub fn generate_font_pixels() -> Vec<u8> {
    let mut px = vec![0u8; (FONT_W * FONT_H * 4) as usize];
    for c in FIRST_CHAR..LAST_CHAR {
        let glyph = BASIC_LEGACY[c as usize];
        let i = c - FIRST_CHAR;
        let ox = (i % FONT_COLS) * GLYPH_PX;
        let oy = (i / FONT_COLS) * GLYPH_PX;
        for (y, row) in glyph.iter().enumerate() {
            for x in 0..GLYPH_PX {
                if row & (1 << x) != 0 {
                    let p = (((oy + y as u32) * FONT_W + ox + x) * 4) as usize;
                    px[p] = 255;
                    px[p + 1] = 255;
                    px[p + 2] = 255;
                    px[p + 3] = 255;
                }
            }
        }
    }
    px
}

/// Builds quads for a multi-line ASCII string: a shadow pass offset by one glyph
/// pixel, then the white main pass. Non-ASCII characters render as '?'.
pub fn build_text_vertices(text: &str) -> Vec<TextVertex> {
    let mut verts = Vec::with_capacity(text.len() * 12);
    let shadow = [0.0, 0.0, 0.0, 0.85];
    let main = [1.0, 1.0, 1.0, 1.0];
    let off = TEXT_SCALE; // one glyph pixel
    emit_pass(
        &mut verts,
        text,
        TEXT_MARGIN + off,
        TEXT_MARGIN + off,
        shadow,
    );
    emit_pass(&mut verts, text, TEXT_MARGIN, TEXT_MARGIN, main);
    verts
}

fn emit_pass(verts: &mut Vec<TextVertex>, text: &str, x0: f32, y0: f32, color: [f32; 4]) {
    let advance = GLYPH_PX as f32 * TEXT_SCALE;
    let mut x = x0;
    let mut y = y0;
    for ch in text.chars() {
        if ch == '\n' {
            x = x0;
            y += LINE_HEIGHT * TEXT_SCALE;
            continue;
        }
        let c = ch as u32;
        let c = if (FIRST_CHAR..LAST_CHAR).contains(&c) {
            c
        } else {
            '?' as u32
        };
        if c != ' ' as u32 {
            emit_glyph(verts, x, y, c - FIRST_CHAR, color);
        }
        x += advance;
    }
}

fn emit_glyph(verts: &mut Vec<TextVertex>, x: f32, y: f32, index: u32, color: [f32; 4]) {
    let size = GLYPH_PX as f32 * TEXT_SCALE;
    let u0 = ((index % FONT_COLS) * GLYPH_PX) as f32 / FONT_W as f32;
    let v0 = ((index / FONT_COLS) * GLYPH_PX) as f32 / FONT_H as f32;
    let du = GLYPH_PX as f32 / FONT_W as f32;
    let dv = GLYPH_PX as f32 / FONT_H as f32;

    let p = |px: f32, py: f32, u: f32, v: f32| TextVertex {
        pos: [px, py],
        uv: [u, v],
        color,
    };
    let (x1, y1) = (x + size, y + size);
    let (u1, v1) = (u0 + du, v0 + dv);
    verts.extend_from_slice(&[
        p(x, y, u0, v0),
        p(x1, y, u1, v0),
        p(x1, y1, u1, v1),
        p(x, y, u0, v0),
        p(x1, y1, u1, v1),
        p(x, y1, u0, v1),
    ]);
}
