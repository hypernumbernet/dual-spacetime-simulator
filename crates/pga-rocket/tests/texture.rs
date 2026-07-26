//! Unit tests for open-world ground albedo generators and mip chain helpers.
//! Drives the real shipped functions in `pga_rocket::texture` (no golden frames).

use pga_rocket::texture::{
    GROUND_TILE_PX, downsample_rgba8, generate_grass_pixels, generate_grass_pixels_size,
    generate_mip_chain, generate_moon_pixels, generate_moon_pixels_size, generate_paved_pixels,
    mean_rgb, mip_level_count,
};

#[test]
fn grass_tile_size_and_rgba_length() {
    let px = generate_grass_pixels();
    let expected = (GROUND_TILE_PX * GROUND_TILE_PX * 4) as usize;
    assert_eq!(px.len(), expected, "grass RGBA length must match GROUND_TILE_PX");
    assert!(
        GROUND_TILE_PX >= 64,
        "open-world grass must not be the old 16×16 stamp, got {}",
        GROUND_TILE_PX
    );
    // Fully opaque.
    for chunk in px.chunks_exact(4) {
        assert_eq!(chunk[3], 255);
    }
}

#[test]
fn moon_tile_size_and_rgba_length() {
    let px = generate_moon_pixels();
    let expected = (GROUND_TILE_PX * GROUND_TILE_PX * 4) as usize;
    assert_eq!(px.len(), expected);
    for chunk in px.chunks_exact(4) {
        assert_eq!(chunk[3], 255);
    }
}

#[test]
fn paved_tile_size_and_rgba_length() {
    let px = generate_paved_pixels();
    assert_eq!(px.len(), (GROUND_TILE_PX * GROUND_TILE_PX * 4) as usize);
}

#[test]
fn grass_mean_is_green_dominant() {
    let m = mean_rgb(&generate_grass_pixels());
    // Meadow: green channel leads; not gray / not dirt-brown overall.
    assert!(
        m[1] > m[0] && m[1] > m[2],
        "grass mean should be green-dominant, got {:?}",
        m
    );
    assert!(m[1] > 0.25 && m[1] < 0.75, "grass green in meadow gamut, {:?}", m);
    assert!(m[0] > 0.10 && m[0] < 0.55, "grass red channel, {:?}", m);
}

#[test]
fn moon_mean_is_gray_regolith() {
    let m = mean_rgb(&generate_moon_pixels());
    // Lunar dust: channels close, mid-gray, not green meadow.
    let spread = (m[0] - m[1]).abs().max((m[1] - m[2]).abs()).max((m[0] - m[2]).abs());
    assert!(
        spread < 0.08,
        "moon regolith should be near-neutral gray, mean={:?} spread={}",
        m,
        spread
    );
    assert!(
        m[0] > 0.25 && m[0] < 0.70,
        "moon luminance in regolith range, {:?}",
        m
    );
    // Distinct from grass: moon is not green-led.
    let grass = mean_rgb(&generate_grass_pixels());
    assert!(
        grass[1] - grass[0] > m[1] - m[0] + 0.05,
        "grass green bias must exceed moon; grass={:?} moon={:?}",
        grass,
        m
    );
}

#[test]
fn grass_and_moon_are_visually_distinct_modes() {
    let g = mean_rgb(&generate_grass_pixels());
    let m = mean_rgb(&generate_moon_pixels());
    let dist = ((g[0] - m[0]).powi(2) + (g[1] - m[1]).powi(2) + (g[2] - m[2]).powi(2)).sqrt();
    assert!(
        dist > 0.12,
        "Earth grass vs moon regolith means too similar: g={:?} m={:?} dist={}",
        g,
        m,
        dist
    );
}

#[test]
fn mip_level_count_for_256() {
    assert_eq!(mip_level_count(256, 256), 9); // 256..1 inclusive
    assert_eq!(mip_level_count(16, 16), 5);
    assert_eq!(mip_level_count(1, 1), 1);
}

#[test]
fn mip_chain_covers_full_pyramid() {
    let base = generate_grass_pixels_size(64);
    let chain = generate_mip_chain(base, 64, 64);
    assert_eq!(chain.len(), mip_level_count(64, 64) as usize);
    assert_eq!(chain[0].0, 64);
    assert_eq!(chain[0].1, 64);
    assert_eq!(chain[0].2.len(), 64 * 64 * 4);
    let (lw, lh, last) = chain.last().unwrap();
    assert_eq!(*lw, 1);
    assert_eq!(*lh, 1);
    assert_eq!(last.len(), 4);
    // Each level halves (until 1).
    for i in 1..chain.len() {
        let (pw, ph, _) = chain[i - 1];
        let (cw, ch, cpx) = &chain[i];
        assert_eq!(*cw, (pw / 2).max(1));
        assert_eq!(*ch, (ph / 2).max(1));
        assert_eq!(cpx.len(), (*cw * *ch * 4) as usize);
    }
}

#[test]
fn downsample_preserves_mean_approximately() {
    // Solid-ish field: downsample mean stays close to source mean.
    let size = 32u32;
    let src = generate_moon_pixels_size(size);
    let src_mean = mean_rgb(&src);
    let (nw, nh, half) = downsample_rgba8(&src, size, size);
    assert_eq!(nw, 16);
    assert_eq!(nh, 16);
    let half_mean = mean_rgb(&half);
    for i in 0..3 {
        assert!(
            (src_mean[i] - half_mean[i]).abs() < 0.03,
            "channel {i}: src={} half={}",
            src_mean[i],
            half_mean[i]
        );
    }
}

#[test]
fn generators_respond_to_size_parameter() {
    let a = generate_grass_pixels_size(32);
    let b = generate_moon_pixels_size(48);
    assert_eq!(a.len(), 32 * 32 * 4);
    assert_eq!(b.len(), 48 * 48 * 4);
}
