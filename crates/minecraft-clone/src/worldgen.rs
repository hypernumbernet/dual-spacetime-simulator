//! Procedural natural terrain: domain-warped layered value noise producing continents,
//! oceans, mountain ranges and rolling hills, with height/slope-based biome materials,
//! sea-level water, snow caps, beaches, and varied trees.
//!
//! All functions of world coordinates are pure (deterministic, no global state), which
//! lets every chunk independently materialize the identical slice of any structure that
//! overlaps it — no cross-chunk communication, no generation-order dependence.

use crate::block::Block;
use crate::chunk::{CX, CY, CZ, ChunkBlocks, WORLD_SCALE};
use glam::IVec2;

/// Shorthand: scales horizontal wavelengths and vertical relief together.
const S: f32 = WORLD_SCALE;

const SEED: u32 = 0xC0FF_EE11;
const HILL_SEED: u32 = 0x51A2_77B3;
const MOUNT_SEED: u32 = 0x7E15_AC09;
const WARP_SEED: u32 = 0x2D9F_1B47;
const SNOW_SEED: u32 = 0x3C6E_F35A;
const TREE_SEED: u32 = 0x1234_5678;
const TRUNK_SEED: u32 = 0x9E37_79B9;

pub const SEA_LEVEL: i32 = (62.0 * S) as i32;
const MIN_HEIGHT: i32 = (4.0 * S) as i32;
const MAX_HEIGHT: i32 = CY as i32 - 6;

// ---------------------------------------------------------------------------
// Noise primitives
// ---------------------------------------------------------------------------

/// Integer hash (xxHash-style mix) of a 2D coordinate.
fn hash2(x: i32, z: i32, seed: u32) -> u32 {
    let mut h = seed;
    h = h.wrapping_add((x as u32).wrapping_mul(0x85EB_CA6B));
    h = h.wrapping_add((z as u32).wrapping_mul(0xC2B2_AE35));
    h ^= h >> 15;
    h = h.wrapping_mul(0x2C1B_3C6D);
    h ^= h >> 12;
    h = h.wrapping_mul(0x297A_2D39);
    h ^= h >> 15;
    h
}

/// Hash -> float in [0, 1).
fn rand01(x: i32, z: i32, seed: u32) -> f32 {
    (hash2(x, z, seed) >> 8) as f32 / (1u32 << 24) as f32
}

/// Smooth quintic fade curve t^3(t(6t-15)+10).
fn fade(t: f32) -> f32 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

/// Hermite smoothstep.
fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Bilinearly interpolated value noise on the integer lattice, output in [-1, 1].
fn value_noise(x: f32, z: f32, seed: u32) -> f32 {
    let x0 = x.floor() as i32;
    let z0 = z.floor() as i32;
    let tx = fade(x - x0 as f32);
    let tz = fade(z - z0 as f32);

    let v00 = rand01(x0, z0, seed);
    let v10 = rand01(x0 + 1, z0, seed);
    let v01 = rand01(x0, z0 + 1, seed);
    let v11 = rand01(x0 + 1, z0 + 1, seed);

    let a = v00 + (v10 - v00) * tx;
    let b = v01 + (v11 - v01) * tx;
    (a + (b - a) * tz) * 2.0 - 1.0
}

/// Fractal Brownian motion (lacunarity 2, gain 0.5), output ~[-1, 1].
fn fbm(x: f32, z: f32, seed: u32, octaves: u32) -> f32 {
    let mut freq = 1.0;
    let mut amp = 1.0;
    let mut sum = 0.0;
    let mut norm = 0.0;
    for _ in 0..octaves {
        sum += value_noise(x * freq, z * freq, seed) * amp;
        norm += amp;
        freq *= 2.0;
        amp *= 0.5;
    }
    sum / norm
}

/// Ridged noise in [0, 1]: sharp crests where the underlying fbm crosses zero.
fn ridged(x: f32, z: f32, seed: u32) -> f32 {
    let v = 1.0 - fbm(x, z, seed, 4).abs();
    v * v
}

// ---------------------------------------------------------------------------
// Terrain shape
// ---------------------------------------------------------------------------

/// Continuous terrain surface height, in blocks, at a world column.
/// Horizontal wavelengths and vertical amplitudes both scale with `S`.
fn terrain_height_f(wx: i32, wz: i32) -> f32 {
    let fx = wx as f32;
    let fz = wz as f32;

    // Domain warp to break up the grid-aligned look of the lattice noise.
    let warp = 18.0 * S;
    let qx = fbm(fx / (80.0 * S), fz / (80.0 * S), WARP_SEED, 3);
    let qz = fbm(
        (fx + 131.0) / (80.0 * S),
        (fz - 71.0) / (80.0 * S),
        WARP_SEED,
        3,
    );
    let wxw = fx + warp * qx;
    let wzw = fz + warp * qz;

    // Continental shelf: large-scale land/ocean shape in [-1, 1].
    let continent = fbm(wxw / (260.0 * S), wzw / (260.0 * S), SEED, 5);
    // Rolling hills detail.
    let hills = fbm(wxw / (55.0 * S), wzw / (55.0 * S), HILL_SEED, 4);
    // Mountain ridges.
    let mnt = ridged(wxw / (130.0 * S), wzw / (130.0 * S), MOUNT_SEED);

    let mut h = SEA_LEVEL as f32 + continent * 28.0 * S;
    // Mountains rise only well inland (where the continent is high).
    let mountain_mask = smoothstep(0.15, 0.6, continent);
    h += mnt.powf(1.7) * 52.0 * S * mountain_mask;
    // Hills are stronger on land, gentle near coasts.
    h += hills * 7.0 * S * (0.4 + 0.6 * smoothstep(-0.2, 0.4, continent));

    h
}

/// Terrain surface height (topmost solid block's Y) at a world column.
pub fn surface_height(wx: i32, wz: i32) -> i32 {
    (terrain_height_f(wx, wz).round() as i32).clamp(MIN_HEIGHT, MAX_HEIGHT)
}

/// Local steepness: half the largest height difference to nearby columns.
fn column_slope(wx: i32, wz: i32) -> i32 {
    let h = surface_height(wx, wz);
    let mut max_d = 0;
    for (dx, dz) in [(-2, 0), (2, 0), (0, -2), (0, 2)] {
        let d = (surface_height(wx + dx, wz + dz) - h).abs();
        max_d = max_d.max(d);
    }
    max_d / 2
}

/// Noisy snow line so mountain caps have an organic edge.
fn snow_line(wx: i32, wz: i32) -> i32 {
    (94.0 * S) as i32
        + (value_noise(wx as f32 / (40.0 * S), wz as f32 / (40.0 * S), SNOW_SEED) * 7.0 * S) as i32
}

/// Surface material for a column, from height and slope. Height bands scale with the
/// world; the slope threshold is scale-invariant (both axes stretch equally).
fn top_block(wx: i32, wz: i32, h: i32, slope: i32) -> Block {
    let sand_bed = (3.0 * S) as i32;
    let beach = (2.0 * S) as i32;
    if h < SEA_LEVEL {
        // Sea/lake bed.
        if h >= SEA_LEVEL - sand_bed {
            Block::Sand
        } else {
            Block::Gravel
        }
    } else if h <= SEA_LEVEL + beach {
        Block::Sand // beach
    } else if h >= snow_line(wx, wz) {
        Block::Snow
    } else if slope > 3 {
        Block::Stone // exposed cliff faces
    } else {
        Block::Grass
    }
}

// ---------------------------------------------------------------------------
// Trees
// ---------------------------------------------------------------------------

/// Single knob for overall tree size (1.0 = the original small trees): trunk heights
/// and canopy radii scale together. Forest density is divided by TREE_SCALE² so the
/// canopy coverage of a forest stays roughly constant as trees grow.
const TREE_SCALE: f32 = 3.0;

/// Tree kinds with different silhouettes.
#[derive(Clone, Copy, PartialEq)]
enum TreeKind {
    Broadleaf,
    Pine,
}

/// Returns (trunk height, kind) if a tree's base sits on this grassy column.
/// Tree sizes follow TREE_SCALE, independent of the terrain's WORLD_SCALE.
fn tree_at(wx: i32, wz: i32) -> Option<(i32, TreeKind)> {
    // Cheap probability gate first — the terrain queries below cost many noise evals,
    // and only ~max_prob of columns pass (matters: chunk gen scans a padded grid).
    let r = rand01(wx, wz, TREE_SEED);
    let density = TREE_SCALE * TREE_SCALE;
    if r >= 0.05 / density {
        return None;
    }
    let h = surface_height(wx, wz);
    let slope = column_slope(wx, wz);
    // Only on gentle grassy land, below the snow line.
    if !(top_block(wx, wz, h, slope) == Block::Grass && h < snow_line(wx, wz) - (4.0 * S) as i32) {
        return None;
    }
    // Pines prefer higher elevation; broadleaf the lowlands.
    let high = h > SEA_LEVEL + (28.0 * S) as i32;
    let kind = if high {
        TreeKind::Pine
    } else {
        TreeKind::Broadleaf
    };
    let prob = if high { 0.05 } else { 0.045 } / density;
    if r < prob {
        let extra = (hash2(wx, wz, TRUNK_SEED) % 4) as i32;
        let base_trunk = match kind {
            TreeKind::Broadleaf => 4 + extra,
            TreeKind::Pine => 6 + extra,
        };
        Some(((base_trunk as f32 * TREE_SCALE).round() as i32, kind))
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Chunk assembly
// ---------------------------------------------------------------------------

/// Full-height scratch buffer used during generation; tracks the highest written Y so
/// the result can be stored height-capped (see ChunkBlocks::from_full).
struct GenChunk {
    data: Vec<Block>,
    max_y: usize,
}

impl GenChunk {
    fn new() -> Self {
        GenChunk {
            data: vec![Block::Air; CX * CY * CZ],
            max_y: 0,
        }
    }

    #[inline]
    fn idx(x: usize, y: usize, z: usize) -> usize {
        (y * CZ + z) * CX + x
    }

    #[inline]
    fn get(&self, x: usize, y: usize, z: usize) -> Block {
        self.data[Self::idx(x, y, z)]
    }

    #[inline]
    fn set(&mut self, x: usize, y: usize, z: usize, b: Block) {
        self.data[Self::idx(x, y, z)] = b;
        if y > self.max_y {
            self.max_y = y;
        }
    }

    fn finish(self) -> ChunkBlocks {
        let max_y = self.max_y;
        ChunkBlocks::from_full(self.data, max_y + 1)
    }
}

/// Generates a full chunk: terrain columns, sea-level water, then overlapping trees.
pub fn generate_chunk(coord: IVec2) -> ChunkBlocks {
    let mut blocks = GenChunk::new();
    let ox = coord.x * CX as i32;
    let oz = coord.y * CZ as i32;

    for lx in 0..CX {
        for lz in 0..CZ {
            let wx = ox + lx as i32;
            let wz = oz + lz as i32;
            let h = surface_height(wx, wz);
            let slope = column_slope(wx, wz);
            let top = top_block(wx, wz, h, slope);

            for y in 0..=h.min(CY as i32 - 1) {
                let depth = h - y;
                let block = column_block(top, depth);
                blocks.set(lx, y as usize, lz, block);
            }

            // Fill water up to sea level over submerged columns.
            if h < SEA_LEVEL {
                for y in (h + 1)..=SEA_LEVEL.min(CY as i32 - 1) {
                    if blocks.get(lx, y as usize, lz) == Block::Air {
                        blocks.set(lx, y as usize, lz, Block::Water);
                    }
                }
            }
        }
    }

    // Trees: scan a padded column range so trees whose base is just outside this chunk
    // still stamp their overlapping canopy/trunk into it (structure-overlap trick).
    // The pad must cover the widest canopy radius (2.0 × TREE_SCALE, broadleaf).
    let pad = (2.0 * TREE_SCALE).ceil() as i32 + 1;
    for dx in -pad..(CX as i32 + pad) {
        for dz in -pad..(CZ as i32 + pad) {
            let wx = ox + dx;
            let wz = oz + dz;
            if let Some((trunk_h, kind)) = tree_at(wx, wz) {
                stamp_tree(&mut blocks, dx, dz, wx, wz, trunk_h, kind);
            }
        }
    }

    blocks.finish()
}

/// Subsurface material for a given block depth below the surface block.
fn column_block(top: Block, depth: i32) -> Block {
    if depth == 0 {
        return top;
    }
    match top {
        Block::Grass => {
            if depth <= 3 {
                Block::Dirt
            } else {
                Block::Stone
            }
        }
        Block::Sand => {
            if depth <= 3 {
                Block::Sand
            } else {
                Block::Stone
            }
        }
        Block::Gravel => {
            if depth <= 2 {
                Block::Gravel
            } else {
                Block::Stone
            }
        }
        // Snow caps and cliffs are stone underneath.
        _ => Block::Stone,
    }
}

/// Stamps the portion of a tree (base at local dx,dz) that lies within this chunk.
fn stamp_tree(
    blocks: &mut GenChunk,
    dx: i32,
    dz: i32,
    wx: i32,
    wz: i32,
    trunk_h: i32,
    kind: TreeKind,
) {
    let h = surface_height(wx, wz);

    // Trunk.
    for i in 1..=trunk_h {
        set_if_inside(blocks, dx, h + i, dz, Block::Wood, true);
    }

    match kind {
        TreeKind::Broadleaf => {
            // Ellipsoidal canopy around the trunk top, radii following TREE_SCALE.
            let rh = (2.0 * TREE_SCALE).round() as i32;
            let rv = (1.6 * TREE_SCALE).round() as i32;
            let cy = h + trunk_h;
            for ly in -rv..=rv {
                for lx in -rh..=rh {
                    for lz in -rh..=rh {
                        let fx = lx as f32 / rh as f32;
                        let fy = ly as f32 / rv as f32;
                        let fz = lz as f32 / rh as f32;
                        if fx * fx + fy * fy + fz * fz <= 1.0 {
                            set_if_inside(blocks, dx + lx, cy + ly, dz + lz, Block::Leaves, false);
                        }
                    }
                }
            }
        }
        TreeKind::Pine => {
            // Conical canopy: radius shrinks toward the top, with a single tip.
            let base = h + (trunk_h as f32 * 0.45) as i32;
            let top = h + trunk_h + TREE_SCALE.round() as i32;
            let max_r = 1.8 * TREE_SCALE;
            for layer in base..=top {
                let t = (top - layer) as f32 / (top - base).max(1) as f32; // 0 at tip
                let radius = (t * max_r).round() as i32;
                for lx in -radius..=radius {
                    for lz in -radius..=radius {
                        if lx.abs() + lz.abs() > radius + 1 {
                            continue; // diamond-ish cross-section
                        }
                        set_if_inside(blocks, dx + lx, layer, dz + lz, Block::Leaves, false);
                    }
                }
            }
            set_if_inside(blocks, dx, top + 1, dz, Block::Leaves, false);
        }
    }
}

/// Writes a block if the local coordinate falls inside this chunk. `overwrite` lets the
/// trunk replace leaves; leaves only fill air.
fn set_if_inside(blocks: &mut GenChunk, lx: i32, y: i32, lz: i32, block: Block, overwrite: bool) {
    if lx < 0 || lx >= CX as i32 || lz < 0 || lz >= CZ as i32 || y < 0 || y >= CY as i32 {
        return;
    }
    let (lx, y, lz) = (lx as usize, y as usize, lz as usize);
    if overwrite || blocks.get(lx, y, lz) == Block::Air {
        blocks.set(lx, y, lz, block);
    }
}
