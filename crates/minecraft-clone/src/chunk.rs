//! Chunk storage: 16 x 128 x 16 columns keyed by `IVec2` (chunk x, z).
//!
//! Terrain is a heightmap (no caves), so vertical chunking buys nothing; a column
//! keeps streaming a simple 2D ring.

use crate::block::Block;
use glam::{IVec2, IVec3, Vec3};

/// Single knob scaling the terrain: multiplies both the horizontal feature sizes
/// (continent/hill/mountain wavelengths) and the vertical relief (sea level, hill and
/// mountain amplitudes, snow line) in `worldgen.rs`. Physical sizes that should not
/// grow with the landscape (trees, soil depth, fog, wave height) stay absolute.
pub const WORLD_SCALE: f32 = 10.0;

pub const CX: usize = 16;
/// Vertical room for the tallest terrain: max theoretical surface is ~149×scale
/// (sea 62 + continent 28 + mountains 52 + hills 7), so 152×scale leaves headroom.
pub const CY: usize = (152.0 * WORLD_SCALE) as usize;
pub const CZ: usize = 16;
pub const CX_I: i32 = CX as i32;
pub const CZ_I: i32 = CZ as i32;

/// Height-capped block storage: only the Y levels up to the chunk's tallest non-air
/// block are allocated (everything above reads as Air). With tall worlds most of the
/// nominal CY range is empty sky, so this cuts chunk memory by half or more.
pub struct ChunkBlocks {
    data: Box<[Block]>,
    height: usize,
}

impl ChunkBlocks {
    /// Takes a full-height (CX*CY*CZ) generation buffer and keeps only the used prefix.
    /// The (y, z, x) layout makes the used part a contiguous prefix.
    pub fn from_full(mut data: Vec<Block>, used_height: usize) -> Self {
        let height = used_height.clamp(1, CY);
        data.truncate(CX * CZ * height);
        data.shrink_to_fit();
        ChunkBlocks {
            data: data.into_boxed_slice(),
            height,
        }
    }

    #[inline]
    fn idx(x: usize, y: usize, z: usize) -> usize {
        (y * CZ + z) * CX + x
    }

    #[inline]
    pub fn get(&self, x: usize, y: usize, z: usize) -> Block {
        if y >= self.height {
            return Block::Air;
        }
        self.data[Self::idx(x, y, z)]
    }

    /// Number of allocated Y levels (no non-air block at or above this).
    #[inline]
    pub fn height(&self) -> usize {
        self.height
    }
}

/// World block position -> (chunk coord, local block coord within that chunk).
pub fn world_to_chunk(p: IVec3) -> (IVec2, IVec3) {
    let cx = p.x.div_euclid(CX_I);
    let cz = p.z.div_euclid(CZ_I);
    let lx = p.x.rem_euclid(CX_I);
    let lz = p.z.rem_euclid(CZ_I);
    (IVec2::new(cx, cz), IVec3::new(lx, p.y, lz))
}

/// Floating-point world position -> chunk coord.
pub fn chunk_of_pos(p: Vec3) -> IVec2 {
    IVec2::new(
        (p.x.floor() as i32).div_euclid(CX_I),
        (p.z.floor() as i32).div_euclid(CZ_I),
    )
}

