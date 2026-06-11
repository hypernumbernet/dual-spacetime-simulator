//! Block types, cube faces, and atlas tile mapping.

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Block {
    Air = 0,
    Water,
    Grass,
    Dirt,
    Stone,
    Sand,
    Gravel,
    Snow,
    Wood,
    Leaves,
}

impl Block {
    /// Opaque blocks occlude adjacent faces and collide with the player. Water is neither.
    #[inline]
    pub fn is_opaque(self) -> bool {
        !matches!(self, Block::Air | Block::Water)
    }

    /// Collidable blocks (currently same set as opaque; water is passable).
    #[inline]
    pub fn is_solid(self) -> bool {
        self.is_opaque()
    }

    #[inline]
    pub fn is_water(self) -> bool {
        self == Block::Water
    }
}

#[derive(Clone, Copy)]
pub enum Face {
    PosX,
    NegX,
    PosY,
    NegY,
    PosZ,
    NegZ,
}

// Atlas tile indices in the 4x4-tile (64px) atlas.
pub const TILE_GRASS_TOP: u32 = 0;
pub const TILE_GRASS_SIDE: u32 = 1;
pub const TILE_DIRT: u32 = 2;
pub const TILE_STONE: u32 = 3;
pub const TILE_SAND: u32 = 4;
pub const TILE_WOOD_BARK: u32 = 5;
pub const TILE_WOOD_RINGS: u32 = 6;
pub const TILE_LEAVES: u32 = 7;
pub const TILE_SNOW: u32 = 8;
pub const TILE_WATER: u32 = 9;
pub const TILE_GRAVEL: u32 = 10;

/// Atlas tile index for a given block face.
pub fn tile_for(block: Block, face: Face) -> u32 {
    match block {
        Block::Air => 0,
        Block::Water => TILE_WATER,
        Block::Grass => match face {
            Face::PosY => TILE_GRASS_TOP,
            Face::NegY => TILE_DIRT,
            _ => TILE_GRASS_SIDE,
        },
        Block::Dirt => TILE_DIRT,
        Block::Stone => TILE_STONE,
        Block::Sand => TILE_SAND,
        Block::Gravel => TILE_GRAVEL,
        Block::Snow => TILE_SNOW,
        Block::Wood => match face {
            Face::PosY | Face::NegY => TILE_WOOD_RINGS,
            _ => TILE_WOOD_BARK,
        },
        Block::Leaves => TILE_LEAVES,
    }
}
