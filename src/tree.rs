use glam::{FloatExt, Quat, Vec3};
use rand::{Rng, SeedableRng};
use std::f32::consts::TAU;

/// A structure containing TreeParams packed for GPU Compute (compliant with std140 layout)
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuTreeComputeParams {
    pub seed: u32,
    pub max_depth: u32,
    pub branch_factor: u32,
    pub _padding: u32,
    pub trunk_height: f32,
    pub trunk_radius_base: f32,
    pub branch_angle: f32,
    pub tropism: f32,
}

impl From<TreeParams> for GpuTreeComputeParams {
    fn from(params: TreeParams) -> Self {
        Self {
            seed: params.seed,
            max_depth: params.max_depth as u32,
            branch_factor: params.branch_factor,
            _padding: 0,
            trunk_height: params.trunk_height,
            trunk_radius_base: params.trunk_radius_base,
            branch_angle: params.branch_angle,
            tropism: params.tropism,
        }
    }
}

/// Same as the xz coordinate axis auxiliary grid (synchronized with `pipeline::create_axes_buffer`)
pub const AXIS_XZ_GRID_EXTENT: f32 = 2.0;
pub const AXIS_XZ_GRID_LINE_COUNT: usize = 9;

/// Simple tree generation inspired by HPG 2025 / Weber-Penn style
/// Supports both CPU and GPU Compute shader versions (using ComputeMode::GPU for GPU version).
/// The GPU version is implemented in tree_compute.comp.

#[derive(Clone, Copy)]
pub struct TreeParams {
    pub seed: u32,
    pub trunk_height: f32,
    pub trunk_radius_base: f32,
    pub max_depth: u8,
    pub branch_factor: u32,
    pub branch_angle: f32,
    pub tropism: f32, // Strength of bending towards gravity direction
}

impl Default for TreeParams {
    fn default() -> Self {
        Self {
            seed: 42,
            trunk_height: 0.8,
            trunk_radius_base: 0.08,
            max_depth: 5,
            branch_factor: 3,
            branch_angle: 0.6,
            tropism: 0.3,
        }
    }
}

#[derive(Clone)]
pub struct HermiteSpline {
    pub p0: Vec3,
    pub p1: Vec3,
    pub m0: Vec3,
    pub m1: Vec3,
}

impl HermiteSpline {
    pub fn new(p0: Vec3, p1: Vec3, m0: Vec3, m1: Vec3) -> Self {
        Self { p0, p1, m0, m1 }
    }

    pub fn eval(&self, t: f32) -> Vec3 {
        let t2 = t * t;
        let t3 = t2 * t;
        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let h10 = t3 - 2.0 * t2 + t;
        let h01 = -2.0 * t3 + 3.0 * t2;
        let h11 = t3 - t2;
        self.p0 * h00 + self.m0 * h10 + self.p1 * h01 + self.m1 * h11
    }

    pub fn eval_tangent(&self, t: f32) -> Vec3 {
        let t2 = t * t;
        let h00 = 6.0 * t2 - 6.0 * t;
        let h10 = 3.0 * t2 - 4.0 * t + 1.0;
        let h01 = -6.0 * t2 + 6.0 * t;
        let h11 = 3.0 * t2 - 2.0 * t;
        (self.p0 * h00 + self.m0 * h10 + self.p1 * h01 + self.m1 * h11).normalize()
    }
}

#[derive(Clone)]
pub struct Branch {
    pub spline: HermiteSpline,
    pub radius_base: f32,
    pub radius_tip: f32,
    pub depth: u8,
    pub children: Vec<Branch>,
}

pub struct Tree {
    pub root: Branch,
}

impl Tree {
    pub fn generate(params: TreeParams) -> Self {
        let mut rng = rand::rngs::SmallRng::seed_from_u64(params.seed as u64);
        let mut bounds_min = Vec3::splat(f32::MAX);
        let mut bounds_max = Vec3::splat(f32::MIN);

        let root_pos = Vec3::ZERO;
        let root_tangent = Vec3::Y;
        let trunk_height = params.trunk_height;
        let trunk_p1 = root_pos + root_tangent * trunk_height;

        let m0 = root_tangent * trunk_height * 0.6;
        let m1 = root_tangent * trunk_height * 0.4;

        let trunk_spline = HermiteSpline::new(root_pos, trunk_p1, m0, m1);

        let mut root = Branch {
            spline: trunk_spline,
            radius_base: params.trunk_radius_base,
            radius_tip: params.trunk_radius_base * 0.2,
            depth: 0,
            children: vec![],
        };

        Self::grow_branch(
            &mut root,
            &params,
            &mut rng,
            &mut bounds_min,
            &mut bounds_max,
        );

        Self::update_bounds(&root, &mut bounds_min, &mut bounds_max);

        Tree { root }
    }

    /// Place one tree at each xz grid crossing (0.5 interval) and connect the drawing vertices.
    pub fn generate_forest_vertices_on_axis_xz_grid(
        params: TreeParams,
    ) -> Vec<([f32; 3], [f32; 4])> {
        let n = AXIS_XZ_GRID_LINE_COUNT;
        let step = (2.0 * AXIS_XZ_GRID_EXTENT) / ((n - 1) as f32);
        let mut out = Vec::new();
        for i in 0..n {
            for j in 0..n {
                let x = -AXIS_XZ_GRID_EXTENT + i as f32 * step;
                let z = -AXIS_XZ_GRID_EXTENT + j as f32 * step;
                let base = Vec3::new(x, 0.0, z);
                let mut p = params;
                p.seed = params
                    .seed
                    .wrapping_add((i as u32).wrapping_mul(1_000_003))
                    .wrapping_add((j as u32).wrapping_mul(97));
                let tree = Tree::generate(p);
                out.extend(tree.generate_vertices_at(base));
            }
        }
        out
    }

    /// Generate vertices for the specified layout (single tree or grid forest).
    pub fn generate_vertices_for_layout(
        layout: crate::ui_state::GpuTreeLayout,
        params: TreeParams,
    ) -> Vec<([f32; 3], [f32; 4])> {
        match layout {
            crate::ui_state::GpuTreeLayout::Single => {
                let tree = Tree::generate(params);
                tree.generate_vertices_at(Vec3::ZERO)
            }
            crate::ui_state::GpuTreeLayout::ForestOnGrid => {
                Self::generate_forest_vertices_on_axis_xz_grid(params)
            }
        }
    }

    /// Place one tree at each xz grid crossing (0.5 interval) and connect the Tube mesh.
    pub fn generate_forest_tube_vertices_on_axis_xz_grid(
        params: TreeParams,
    ) -> Vec<([f32; 3], [f32; 3], [f32; 4])> {
        let n = AXIS_XZ_GRID_LINE_COUNT;
        let step = (2.0 * AXIS_XZ_GRID_EXTENT) / ((n - 1) as f32);
        let mut out = Vec::new();
        for i in 0..n {
            for j in 0..n {
                let x = -AXIS_XZ_GRID_EXTENT + i as f32 * step;
                let z = -AXIS_XZ_GRID_EXTENT + j as f32 * step;
                let base = Vec3::new(x, 0.0, z);
                let mut p = params;
                p.seed = params
                    .seed
                    .wrapping_add((i as u32).wrapping_mul(1_000_003))
                    .wrapping_add((j as u32).wrapping_mul(97));
                let tree = Tree::generate(p);
                out.extend(tree.generate_tube_vertices_at(base));
            }
        }
        out
    }

    fn grow_branch(
        branch: &mut Branch,
        params: &TreeParams,
        rng: &mut impl Rng,
        bounds_min: &mut Vec3,
        bounds_max: &mut Vec3,
    ) {
        if branch.depth >= params.max_depth {
            return;
        }

        let num_children = if branch.depth == 0 {
            4
        } else {
            params.branch_factor as usize
        };
        let attach_step = 1.0 / (num_children as f32 + 1.0);

        for i in 0..num_children {
            let attach_t = 0.3 + attach_step * (i as f32 + 0.5);

            let attach_pos = branch.spline.eval(attach_t);
            let tangent = branch.spline.eval_tangent(attach_t);

            // Simple branching frame
            let up = Vec3::Y;
            let side = if tangent.cross(up).length() > 0.01 {
                tangent.cross(up).normalize()
            } else {
                Vec3::X
            };
            let binormal = tangent.cross(side).normalize();

            // Random angle
            let angle = (rng.random::<f32>() - 0.5) * params.branch_angle * 2.0;
            let rot = Quat::from_axis_angle(binormal, angle);
            let child_dir = rot * tangent;

            let child_length = branch.spline.p1.distance(branch.spline.p0)
                * 0.6
                * (0.7f32).powi(branch.depth as i32);
            let child_p1 = attach_pos + child_dir * child_length;

            let child_m0 = child_dir * child_length * 0.5;
            let child_m1 = child_dir * child_length * 0.3 * (1.0 - params.tropism); // Simple tropism

            let child_spline = HermiteSpline::new(attach_pos, child_p1, child_m0, child_m1);

            let mut child = Branch {
                spline: child_spline,
                radius_base: branch.radius_tip * 0.8,
                radius_tip: branch.radius_tip * 0.4,
                depth: branch.depth + 1,
                children: vec![],
            };

            Self::grow_branch(&mut child, params, rng, bounds_min, bounds_max);
            branch.children.push(child);
        }
    }

    fn update_bounds(branch: &Branch, min: &mut Vec3, max: &mut Vec3) {
        let steps = 8;
        for i in 0..=steps {
            let t = i as f32 / steps as f32;
            let p = branch.spline.eval(t);
            *min = min.min(p);
            *max = max.max(p);
        }
        for child in &branch.children {
            Self::update_bounds(child, min, max);
        }
    }

    /// Generate Tube mesh at the specified base position (Forest support).
    pub fn generate_tube_vertices_at(&self, base: Vec3) -> Vec<([f32; 3], [f32; 3], [f32; 4])> {
        let mut verts = Vec::new();
        Self::collect_tube_vertices(&self.root, base, &mut verts);
        verts
    }

    fn collect_tube_vertices(
        branch: &Branch,
        base: Vec3,
        verts: &mut Vec<([f32; 3], [f32; 3], [f32; 4])>,
    ) {
        let segments = 8; // Number of segments (balance of accuracy and performance)
        let sides = 8; // Number of sides (affects polygon count)
        let color = if branch.depth == 0 {
            [0.55, 0.35, 0.15, 1.0] // Trunk: dark brown
        } else if branch.depth < 3 {
            [0.45, 0.28, 0.12, 1.0]
        } else {
            [0.1, 0.7, 0.15, 1.0] // Leaf: green (leaves are thin)
        };

        let radius_start = branch.radius_base;
        let radius_end = branch.radius_tip;

        for i in 0..segments {
            let t0 = i as f32 / segments as f32;
            let t1 = (i + 1) as f32 / segments as f32;

            let p0 = branch.spline.eval(t0) + base;
            let p1 = branch.spline.eval(t1) + base;
            let tangent0 = branch.spline.eval_tangent(t0);
            let tangent1 = branch.spline.eval_tangent(t1);

            let r0 = radius_start.lerp(radius_end, t0);
            let r1 = radius_start.lerp(radius_end, t1);

            // Frame calculation (tangent to normal and binormal)
            let up = Vec3::Y;
            let binormal0 = if tangent0.cross(up).length() > 0.01 {
                tangent0.cross(up).normalize()
            } else {
                Vec3::X
            };
            let normal0 = binormal0.cross(tangent0).normalize();

            let binormal1 = if tangent1.cross(up).length() > 0.01 {
                tangent1.cross(up).normalize()
            } else {
                Vec3::X
            };
            let normal1 = binormal1.cross(tangent1).normalize();

            for s in 0..sides {
                let s0 = s as f32 / sides as f32;
                let s1 = (s + 1) as f32 / sides as f32;
                let angle0 = s0 * TAU;
                let angle1 = s1 * TAU;
                let cos0 = angle0.cos();
                let sin0 = angle0.sin();
                let cos1 = angle1.cos();
                let sin1 = angle1.sin();

                let offset0_s0 = normal0 * cos0 + binormal0 * sin0;
                let pos0_s0 = p0 + offset0_s0 * r0;
                let n0_s0 = offset0_s0.normalize();
                let y0_s0 = -pos0_s0.y;

                let offset0_s1 = normal0 * cos1 + binormal0 * sin1;
                let pos0_s1 = p0 + offset0_s1 * r0;
                let n0_s1 = offset0_s1.normalize();
                let y0_s1 = -pos0_s1.y;

                let offset1_s0 = normal1 * cos0 + binormal1 * sin0;
                let pos1_s0 = p1 + offset1_s0 * r1;
                let n1_s0 = offset1_s0.normalize();
                let y1_s0 = -pos1_s0.y;

                let offset1_s1 = normal1 * cos1 + binormal1 * sin1;
                let pos1_s1 = p1 + offset1_s1 * r1;
                let n1_s1 = offset1_s1.normalize();
                let y1_s1 = -pos1_s1.y;

                let col = color;

                verts.push((
                    [pos0_s0.x, y0_s0, pos0_s0.z],
                    [n0_s0.x, -n0_s0.y, n0_s0.z],
                    col,
                ));
                verts.push((
                    [pos0_s1.x, y0_s1, pos0_s1.z],
                    [n0_s1.x, -n0_s1.y, n0_s1.z],
                    col,
                ));
                verts.push((
                    [pos1_s0.x, y1_s0, pos1_s0.z],
                    [n1_s0.x, -n1_s0.y, n1_s0.z],
                    col,
                ));

                verts.push((
                    [pos1_s0.x, y1_s0, pos1_s0.z],
                    [n1_s0.x, -n1_s0.y, n1_s0.z],
                    col,
                ));
                verts.push((
                    [pos0_s1.x, y0_s1, pos0_s1.z],
                    [n0_s1.x, -n0_s1.y, n0_s1.z],
                    col,
                ));
                verts.push((
                    [pos1_s1.x, y1_s1, pos1_s1.z],
                    [n1_s1.x, -n1_s1.y, n1_s1.z],
                    col,
                ));
            }
        }

        for child in &branch.children {
            Self::collect_tube_vertices(child, base, verts);
        }

        if branch.children.is_empty() && branch.depth >= 3 {
            let tip = branch.spline.eval(1.0) + base;
            let leaf_r = 0.03f32;
            let leaf_color = [0.1, 0.8, 0.2, 1.0];
            let leaf_tangent = branch.spline.eval_tangent(1.0);
            let leaf_binormal = if leaf_tangent.cross(Vec3::Y).length() > 0.01 {
                leaf_tangent.cross(Vec3::Y).normalize()
            } else {
                Vec3::X
            };
            let leaf_normal = leaf_binormal.cross(leaf_tangent).normalize();

            for s in 0..sides {
                let s0 = s as f32 / sides as f32;
                let s1 = (s + 1) as f32 / sides as f32;
                let angle0 = s0 * TAU;
                let angle1 = s1 * TAU;
                let cos0 = angle0.cos();
                let sin0 = angle0.sin();
                let cos1 = angle1.cos();
                let sin1 = angle1.sin();

                let offset0 = leaf_normal * cos0 + leaf_binormal * sin0;
                let pos0 = tip + offset0 * leaf_r;
                let n0 = offset0.normalize();
                let y0 = -pos0.y;

                let offset1 = leaf_normal * cos1 + leaf_binormal * sin1;
                let pos1 = tip + offset1 * leaf_r;
                let n1 = offset1.normalize();
                let y1 = -pos1.y;

                let tip_y = -tip.y;
                let tip_n = [0.0f32, 1.0, 0.0];

                let col = leaf_color;

                verts.push(([pos0.x, y0, pos0.z], [n0.x, -n0.y, n0.z], col));
                verts.push(([pos1.x, y1, pos1.z], [n1.x, -n1.y, n1.z], col));
                verts.push(([tip.x, tip_y, tip.z], tip_n, col));

                verts.push(([pos1.x, y1, pos1.z], [n1.x, -n1.y, n1.z], col));
                verts.push(([pos0.x, y0, pos0.z], [n0.x, -n0.y, n0.z], col));
                verts.push(([tip.x, tip_y, tip.z], tip_n, col));
            }
        }
    }

    pub fn generate_vertices_at(&self, base: Vec3) -> Vec<([f32; 3], [f32; 4])> {
        let mut verts = Vec::new();
        Self::collect_branch_vertices(&self.root, base, &mut verts, 0.0);
        verts
    }

    fn collect_branch_vertices(
        branch: &Branch,
        base: Vec3,
        verts: &mut Vec<([f32; 3], [f32; 4])>,
        t_offset: f32,
    ) {
        let steps = 12;
        let color = if branch.depth == 0 {
            [0.4, 0.25, 0.1, 1.0] // 幹: 茶
        } else if branch.depth < 3 {
            [0.35, 0.22, 0.08, 1.0]
        } else {
            [0.1, 0.6, 0.1, 1.0] // 葉: 緑
        };

        for i in 0..steps {
            let t0 = i as f32 / steps as f32;
            let t1 = (i + 1) as f32 / steps as f32;
            let p0 = branch.spline.eval(t0) + base;
            let p1 = branch.spline.eval(t1) + base;

            verts.push(([p0.x, -p0.y, p0.z], color));
            verts.push(([p1.x, -p1.y, p1.z], color));
        }

        for child in &branch.children {
            Self::collect_branch_vertices(child, base, verts, t_offset);
        }

        if branch.children.is_empty() && branch.depth >= 3 {
            let tip = branch.spline.eval(1.0) + base;
            let leaf_color = [0.0, 0.8, 0.2, 1.0];
            for _ in 0..6 {
                let offset = Vec3::new(
                    (rand::random::<f32>() - 0.5) * 0.15,
                    (rand::random::<f32>() - 0.5) * 0.08 + 0.05,
                    (rand::random::<f32>() - 0.5) * 0.15,
                );
                let leaf_p = tip + offset;
                verts.push(([tip.x, -tip.y, tip.z], leaf_color));
                verts.push(([leaf_p.x, -leaf_p.y, leaf_p.z], leaf_color));
            }
        }
    }
}
