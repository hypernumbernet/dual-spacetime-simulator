//! Voxel world: chunk storage, lazy streaming around the player, and block queries.
//!
//! Memory strategy: the world is immutable and worldgen is deterministic, so a chunk's
//! CPU block data is only needed (a) until it and its 4 neighbors are meshed, and
//! (b) near the player for collision/underwater queries. Block data outside those
//! roles is dropped and regenerated on demand — CPU memory stays proportional to the
//! meshing frontier instead of the whole loaded area.

use crate::block::Block;
use crate::chunk::{world_to_chunk, ChunkBlocks, CY};
use crate::mesher::{mesh_chunk, ChunkNeighborhood};
use crate::renderer::Renderer;
use crate::worldgen::generate_chunk;
use glam::{IVec2, IVec3};
use std::collections::HashMap;

/// Single knob: the gen/unload rings (and the renderer's fog) derive from this.
pub const RENDER_DISTANCE: i32 = 50;
const GEN_DISTANCE: i32 = RENDER_DISTANCE + 1;
const UNLOAD_DISTANCE: i32 = RENDER_DISTANCE + 3;
/// Chunks within this radius of the player keep CPU block data for collision and
/// underwater checks; beyond it, meshed chunks drop their blocks.
const BLOCKS_KEEP_DISTANCE: i32 = 4;
const GEN_BUDGET: usize = 10;
const MESH_BUDGET: usize = 4;
const BLOCK_REGEN_BUDGET: usize = 4;

pub struct Chunk {
    pub blocks: Option<ChunkBlocks>,
    pub meshed: bool,
}

pub struct World {
    chunks: HashMap<IVec2, Chunk>,
    pending_gen: Vec<IVec2>,    // sorted by distance DESC; pop() = nearest
    pending_mesh: Vec<IVec2>,   // sorted by distance DESC; scan from end = nearest
    pending_blocks: Vec<IVec2>, // dropped block data to regenerate near the player
    last_player_chunk: Option<IVec2>,
}

impl World {
    pub fn new() -> Self {
        Self {
            chunks: HashMap::new(),
            pending_gen: Vec::new(),
            pending_mesh: Vec::new(),
            pending_blocks: Vec::new(),
            last_player_chunk: None,
        }
    }

    /// Block at a world position. Missing chunk, dropped block data, or
    /// out-of-vertical-range is Air. (Near the player, block data is always kept.)
    pub fn block(&self, p: IVec3) -> Block {
        if p.y < 0 || p.y >= CY as i32 {
            return Block::Air;
        }
        let (coord, local) = world_to_chunk(p);
        match self.chunks.get(&coord).and_then(|c| c.blocks.as_ref()) {
            Some(b) => b.get(local.x as usize, local.y as usize, local.z as usize),
            None => Block::Air,
        }
    }

    /// Collision query: an unloaded chunk (or one whose block data was dropped) is
    /// treated as SOLID so the player can't fall through; below y=0 is solid, above
    /// the world is air.
    pub fn solid_for_collision(&self, p: IVec3) -> bool {
        if p.y < 0 {
            return true;
        }
        if p.y >= CY as i32 {
            return false;
        }
        let (coord, local) = world_to_chunk(p);
        match self.chunks.get(&coord).and_then(|c| c.blocks.as_ref()) {
            Some(b) => b
                .get(local.x as usize, local.y as usize, local.z as usize)
                .is_solid(),
            None => true,
        }
    }

    /// Streams chunks around the player: rebuilds queues on chunk-cross, then spends
    /// per-frame budgets on generation, meshing (nearest-first), block-data regen near
    /// the player, and unloads/drops distant data.
    pub fn update_streaming(&mut self, player_chunk: IVec2, renderer: &mut Renderer) {
        if self.last_player_chunk != Some(player_chunk) {
            self.last_player_chunk = Some(player_chunk);
            self.rebuild_queues(player_chunk);
            self.unload_distant(player_chunk, renderer);
            self.drop_far_blocks(player_chunk);
        }

        // Safety net: the player's immediate ring must always have collision data.
        for dx in -1..=1 {
            for dz in -1..=1 {
                let c = player_chunk + IVec2::new(dx, dz);
                if let Some(chunk) = self.chunks.get_mut(&c) {
                    if chunk.blocks.is_none() {
                        chunk.blocks = Some(generate_chunk(c));
                    }
                }
            }
        }

        // Budgeted regen of nearby dropped block data (worldgen is deterministic).
        let mut regens = 0;
        while regens < BLOCK_REGEN_BUDGET {
            let Some(c) = self.pending_blocks.pop() else {
                break;
            };
            if let Some(chunk) = self.chunks.get_mut(&c) {
                if chunk.blocks.is_none() {
                    chunk.blocks = Some(generate_chunk(c));
                    regens += 1;
                }
            }
        }

        // Generation.
        let mut gens = 0;
        while gens < GEN_BUDGET {
            let Some(coord) = self.pending_gen.pop() else {
                break;
            };
            if self.chunks.contains_key(&coord) {
                continue;
            }
            let blocks = generate_chunk(coord);
            self.chunks.insert(
                coord,
                Chunk {
                    blocks: Some(blocks),
                    meshed: false,
                },
            );
            if cheb(coord, player_chunk) <= RENDER_DISTANCE {
                self.pending_mesh.push(coord);
            }
            gens += 1;
        }
        // Keep nearest at the end for pop-like scanning.
        self.pending_mesh.sort_by_key(|c| -dist2(*c, player_chunk));

        // Meshing: only chunks whose own and 4 neighbors' block data is present.
        let mut meshed = 0;
        let mut i = self.pending_mesh.len();
        while meshed < MESH_BUDGET && i > 0 {
            i -= 1;
            let coord = self.pending_mesh[i];
            let Some(chunk) = self.chunks.get(&coord) else {
                self.pending_mesh.swap_remove(i);
                continue;
            };
            if chunk.meshed {
                self.pending_mesh.swap_remove(i);
                continue;
            }
            let neighbors = [
                coord + IVec2::new(-1, 0),
                coord + IVec2::new(1, 0),
                coord + IVec2::new(0, -1),
                coord + IVec2::new(0, 1),
            ];
            let ready = chunk.blocks.is_some()
                && neighbors
                    .iter()
                    .all(|n| self.chunks.get(n).is_some_and(|c| c.blocks.is_some()));
            if !ready {
                continue; // wait for neighbors; leave in queue
            }

            let m = {
                let n = ChunkNeighborhood {
                    center: self.chunks[&coord].blocks.as_ref().unwrap(),
                    neg_x: self.chunks[&neighbors[0]].blocks.as_ref().unwrap(),
                    pos_x: self.chunks[&neighbors[1]].blocks.as_ref().unwrap(),
                    neg_z: self.chunks[&neighbors[2]].blocks.as_ref().unwrap(),
                    pos_z: self.chunks[&neighbors[3]].blocks.as_ref().unwrap(),
                };
                mesh_chunk(&n)
            };
            renderer.upload_chunk_mesh(coord, &m);
            self.chunks.get_mut(&coord).unwrap().meshed = true;
            self.pending_mesh.swap_remove(i);
            meshed += 1;
        }
    }

    /// Rebuilds gen/mesh/regen queues for the ring around the player.
    fn rebuild_queues(&mut self, player_chunk: IVec2) {
        self.pending_gen.clear();
        for dx in -GEN_DISTANCE..=GEN_DISTANCE {
            for dz in -GEN_DISTANCE..=GEN_DISTANCE {
                let coord = player_chunk + IVec2::new(dx, dz);
                if !self.chunks.contains_key(&coord) {
                    self.pending_gen.push(coord);
                }
            }
        }
        self.pending_gen.sort_by_key(|c| -dist2(*c, player_chunk));

        self.pending_mesh.clear();
        for (coord, chunk) in &self.chunks {
            if !chunk.meshed && cheb(*coord, player_chunk) <= RENDER_DISTANCE {
                self.pending_mesh.push(*coord);
            }
        }
        self.pending_mesh.sort_by_key(|c| -dist2(*c, player_chunk));

        self.pending_blocks.clear();
        for dx in -BLOCKS_KEEP_DISTANCE..=BLOCKS_KEEP_DISTANCE {
            for dz in -BLOCKS_KEEP_DISTANCE..=BLOCKS_KEEP_DISTANCE {
                let coord = player_chunk + IVec2::new(dx, dz);
                if self
                    .chunks
                    .get(&coord)
                    .is_some_and(|c| c.blocks.is_none())
                {
                    self.pending_blocks.push(coord);
                }
            }
        }
        self.pending_blocks.sort_by_key(|c| -dist2(*c, player_chunk));
    }

    /// Removes chunks (and their GPU meshes) beyond the unload radius.
    fn unload_distant(&mut self, player_chunk: IVec2, renderer: &mut Renderer) {
        let to_remove: Vec<IVec2> = self
            .chunks
            .keys()
            .copied()
            .filter(|c| cheb(*c, player_chunk) > UNLOAD_DISTANCE)
            .collect();
        for coord in to_remove {
            self.chunks.remove(&coord);
            renderer.remove_chunk_mesh(coord);
        }
    }

    /// Drops CPU block data of far chunks whose meshing role is finished. A chunk's
    /// blocks are only ever read again to mesh a *neighbor*, so once the chunk and all
    /// 4 neighbors are meshed (and it's outside the collision radius) the data is dead
    /// weight — worldgen regenerates it deterministically if the player returns.
    fn drop_far_blocks(&mut self, player_chunk: IVec2) {
        let candidates: Vec<IVec2> = self
            .chunks
            .iter()
            .filter(|(c, ch)| {
                ch.blocks.is_some()
                    && ch.meshed
                    && cheb(**c, player_chunk) > BLOCKS_KEEP_DISTANCE
            })
            .map(|(c, _)| *c)
            .collect();
        for c in candidates {
            let neighbors_meshed = [(-1, 0), (1, 0), (0, -1), (0, 1)].iter().all(|&(dx, dz)| {
                self.chunks
                    .get(&(c + IVec2::new(dx, dz)))
                    .is_some_and(|n| n.meshed)
            });
            if neighbors_meshed {
                self.chunks.get_mut(&c).unwrap().blocks = None;
            }
        }
    }
}

/// Squared Euclidean distance between chunk coords.
fn dist2(a: IVec2, b: IVec2) -> i32 {
    let d = a - b;
    d.x * d.x + d.y * d.y
}

/// Chebyshev (chessboard) distance between chunk coords.
fn cheb(a: IVec2, b: IVec2) -> i32 {
    let d = a - b;
    d.x.abs().max(d.y.abs())
}
