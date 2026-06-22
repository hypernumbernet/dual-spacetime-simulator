//! Chunk meshing with hidden-face culling, neighbor-aware borders, baked directional
//! sunlight, and a separate translucent water surface mesh.
//!
//! Vertices are compact (12 bytes): chunk-local integer positions as u16 (the renderer
//! supplies the chunk origin via push constants), light quantized to u8 range, and UVs
//! quantized ×512. All vertex coordinates are integers — the water surface inset and
//! waves are applied in the water vertex shader.

use crate::block::{Block, Face, tile_for};
use crate::chunk::{CX, CY, CZ, ChunkBlocks};
use glam::{IVec3, Vec3};

/// Quantization factor for UVs (shader divides by this). Atlas UVs are multiples of
/// 0.5/64, so ×512 is exact.
pub const UV_Q: f32 = 512.0;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct VoxelVertex {
    pub pos: [u16; 3], // chunk-local block coords (y is world y; fits u16 for CY < 65536)
    pub light: u16,    // brightness * 255
    pub uv: [u16; 2],  // uv * UV_Q; water reuses as (depth * UV_Q, surface flag * UV_Q)
}

/// Direction toward the sun (world space). Faces are lit by max(0, n·sun); the sky uses
/// the same direction for its sun disk.
pub const SUN_DIR: Vec3 = Vec3::new(0.45, 0.85, 0.30);
const AMBIENT: f32 = 0.40;
const DIFFUSE: f32 = 0.60;

/// 6 faces: outward normal dir and the 4 corner offsets (CCW seen from outside).
struct FaceDef {
    face: Face,
    normal: IVec3,
    corners: [[u16; 3]; 4],
}

const FACES: [FaceDef; 6] = [
    FaceDef {
        face: Face::PosX,
        normal: IVec3::new(1, 0, 0),
        corners: [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
    },
    FaceDef {
        face: Face::NegX,
        normal: IVec3::new(-1, 0, 0),
        corners: [[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 1, 0]],
    },
    FaceDef {
        face: Face::PosY,
        normal: IVec3::new(0, 1, 0),
        corners: [[0, 1, 1], [1, 1, 1], [1, 1, 0], [0, 1, 0]],
    },
    FaceDef {
        face: Face::NegY,
        normal: IVec3::new(0, -1, 0),
        corners: [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]],
    },
    FaceDef {
        face: Face::PosZ,
        normal: IVec3::new(0, 0, 1),
        corners: [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
    },
    FaceDef {
        face: Face::NegZ,
        normal: IVec3::new(0, 0, -1),
        corners: [[1, 0, 0], [0, 0, 0], [0, 1, 0], [1, 1, 0]],
    },
];

/// Lambert brightness for a face normal under the fixed sun, quantized to 0..255.
fn face_light_q(normal: IVec3) -> u16 {
    let n = Vec3::new(normal.x as f32, normal.y as f32, normal.z as f32);
    let d = n.dot(SUN_DIR.normalize()).max(0.0);
    ((AMBIENT + DIFFUSE * d) * 255.0).round() as u16
}

/// Quantized UV corners for a tile, half-texel inset to avoid NEAREST bleeding, ordered
/// to match the corner winding so textures stay upright on side faces.
/// u = (16*col + 0.5)/64 * 512 = 128*col + 4, etc.
fn tile_uvs_q(tile: u32) -> [[u16; 2]; 4] {
    let col = (tile % 4) as u16;
    let row = (tile / 4) as u16;
    let u0 = 128 * col + 4;
    let u1 = 128 * col + 124;
    let v0 = 128 * row + 4; // top of tile
    let v1 = 128 * row + 124; // bottom of tile
    [[u0, v1], [u1, v1], [u1, v0], [u0, v0]]
}

/// A 5-chunk view (center + 4 horizontal neighbors) so border faces query real neighbors
/// without a HashMap lookup per block.
pub struct ChunkNeighborhood<'a> {
    pub center: &'a ChunkBlocks,
    pub neg_x: &'a ChunkBlocks,
    pub pos_x: &'a ChunkBlocks,
    pub neg_z: &'a ChunkBlocks,
    pub pos_z: &'a ChunkBlocks,
}

impl ChunkNeighborhood<'_> {
    /// Reads a block at local coordinates where lx/lz may be -1 or CX/CZ (one step into a
    /// neighbor). Out-of-vertical-range is Air.
    #[inline]
    fn get(&self, lx: i32, ly: i32, lz: i32) -> Block {
        if ly < 0 || ly >= CY as i32 {
            return Block::Air;
        }
        let y = ly as usize;
        if lx < 0 {
            return self.neg_x.get(
                (CX as i32 - 1) as usize,
                y,
                lz.clamp(0, CZ as i32 - 1) as usize,
            );
        }
        if lx >= CX as i32 {
            return self.pos_x.get(0, y, lz.clamp(0, CZ as i32 - 1) as usize);
        }
        if lz < 0 {
            return self.neg_z.get(lx as usize, y, (CZ as i32 - 1) as usize);
        }
        if lz >= CZ as i32 {
            return self.pos_z.get(lx as usize, y, 0);
        }
        self.center.get(lx as usize, y, lz as usize)
    }
}

/// Output of meshing a chunk: opaque geometry plus a translucent water surface, and the
/// vertical bounds of all emitted geometry (for frustum culling).
pub struct ChunkMeshData {
    pub opaque_verts: Vec<VoxelVertex>,
    pub opaque_indices: Vec<u32>,
    pub water_verts: Vec<VoxelVertex>,
    pub water_indices: Vec<u32>,
    pub y_min: f32,
    pub y_max: f32,
}

/// Builds opaque + water meshes for the center chunk with hidden-face culling. Only the
/// center chunk's allocated height is scanned (everything above is sky).
pub fn mesh_chunk(n: &ChunkNeighborhood) -> ChunkMeshData {
    let mut data = ChunkMeshData {
        opaque_verts: Vec::new(),
        opaque_indices: Vec::new(),
        water_verts: Vec::new(),
        water_indices: Vec::new(),
        y_min: 0.0,
        y_max: 0.0,
    };
    let mut y_min: u16 = u16::MAX;
    let mut y_max: u16 = 0;

    let height = n.center.height() as i32;

    for lx in 0..CX as i32 {
        for ly in 0..height {
            for lz in 0..CZ as i32 {
                let block = n.center.get(lx as usize, ly as usize, lz as usize);
                if block == Block::Air {
                    continue;
                }
                let is_water = block.is_water();
                // Water vertices carry (column depth, surface flag) in the UV slot
                // instead of atlas coordinates — the water shader is fully procedural.
                let (water_depth_q, water_surface) = if is_water {
                    let mut d = 1;
                    while d < 8 && n.get(lx, ly - d, lz).is_water() {
                        d += 1;
                    }
                    (
                        (d as f32 / 8.0 * UV_Q) as u16,
                        n.get(lx, ly + 1, lz) == Block::Air,
                    )
                } else {
                    (0, false)
                };
                for fd in &FACES {
                    let nb = n.get(lx + fd.normal.x, ly + fd.normal.y, lz + fd.normal.z);
                    if is_water {
                        // Water surface: only where it meets open air.
                        if nb != Block::Air {
                            continue;
                        }
                    } else {
                        // Opaque: skip faces hidden by another opaque block.
                        if nb.is_opaque() {
                            continue;
                        }
                    }

                    let (verts, indices) = if is_water {
                        (&mut data.water_verts, &mut data.water_indices)
                    } else {
                        (&mut data.opaque_verts, &mut data.opaque_indices)
                    };

                    let uvs = tile_uvs_q(tile_for(block, fd.face));
                    let light = face_light_q(fd.normal);
                    let base = verts.len() as u32;
                    for (ci, corner) in fd.corners.iter().enumerate() {
                        let y = ly as u16 + corner[1];
                        y_min = y_min.min(y);
                        y_max = y_max.max(y);
                        // Surface-flagged corners are inset and waved by the water
                        // vertex shader; flagging only corners that touch open air
                        // keeps adjacent quads crack-free.
                        let uv = if is_water {
                            let flag = if water_surface && corner[1] == 1 {
                                UV_Q as u16
                            } else {
                                0
                            };
                            [water_depth_q, flag]
                        } else {
                            uvs[ci]
                        };
                        verts.push(VoxelVertex {
                            pos: [lx as u16 + corner[0], y, lz as u16 + corner[2]],
                            light,
                            uv,
                        });
                    }
                    indices.extend_from_slice(&[
                        base,
                        base + 1,
                        base + 2,
                        base,
                        base + 2,
                        base + 3,
                    ]);
                }
            }
        }
    }

    data.y_min = y_min.min(y_max) as f32;
    data.y_max = y_max as f32;
    data
}
