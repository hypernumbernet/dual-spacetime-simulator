use glam::{Quat, Vec3};
use rand::{Rng, SeedableRng};

/// xz 平面の座標軸補助グリッドと同じ設定（`pipeline::create_axes_buffer` と同期）
pub const AXIS_XZ_GRID_EXTENT: f32 = 2.0;
pub const AXIS_XZ_GRID_LINE_COUNT: usize = 9;

/// HPG 2025 / Weber-Penn 風の簡易木生成 (CPU版)
/// 1本のOak-like木を再帰的に生成。まずは中心線 (LineListで描画)

#[derive(Clone, Copy)]
pub struct TreeParams {
    pub seed: u32,
    pub trunk_height: f32,
    pub trunk_radius_base: f32,
    pub max_depth: u8,
    pub branch_factor: u32,
    pub branch_angle: f32,
    pub tropism: f32, // 重力方向への曲げ強度
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
    pub m0: Vec3, // 開始接線 (scaled)
    pub m1: Vec3, // 終了接線
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
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
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

        // 簡単なHermite (開始/終了接線をスケール)
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

        Self::grow_branch(&mut root, &params, &mut rng, &mut bounds_min, &mut bounds_max);

        // バウンズ更新 (rootも)
        Self::update_bounds(&root, &mut bounds_min, &mut bounds_max);

        Tree {
            root,
            bounds_min,
            bounds_max,
        }
    }

    /// 軸グリッド線（0.5 間隔）の xz 交点すべてに 1 本ずつ木を置き、描画頂点を連結する。
    pub fn generate_forest_vertices_on_axis_xz_grid(params: TreeParams) -> Vec<([f32; 3], [f32; 4])> {
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

    /// 指定されたレイアウトに応じて頂点を生成 (シングル木 or グリッド森)。
    /// UI からの呼び出し用ヘルパー。
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

    /// TreeParams の fingerprint (ui_state と同期)
    pub fn params_fingerprint(params: TreeParams) -> u64 {
        let mut hash = 0u64;
        hash = hash.wrapping_add(params.seed as u64);
        hash = hash.wrapping_mul(31).wrapping_add((params.trunk_height * 100.0) as u64);
        hash = hash.wrapping_mul(31).wrapping_add(params.max_depth as u64);
        hash = hash.wrapping_mul(31).wrapping_add(params.branch_factor as u64);
        hash = hash.wrapping_mul(31).wrapping_add((params.branch_angle * 100.0) as u64);
        hash = hash.wrapping_mul(31).wrapping_add((params.tropism * 100.0) as u64);
        hash
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

        let num_children = if branch.depth == 0 { 4 } else { params.branch_factor as usize };
        let attach_step = 1.0 / (num_children as f32 + 1.0);

        for i in 0..num_children {
            let attach_t = 0.3 + attach_step * (i as f32 + 0.5); // 幹の下部から

            let attach_pos = branch.spline.eval(attach_t);
            let tangent = branch.spline.eval_tangent(attach_t);

            // シンプルな分岐フレーム
            let up = Vec3::Y;
            let side = if tangent.cross(up).length() > 0.01 {
                tangent.cross(up).normalize()
            } else {
                Vec3::X
            };
            let binormal = tangent.cross(side).normalize();

            // ランダム角度
            let angle = (rng.random::<f32>() - 0.5) * params.branch_angle * 2.0;
            let rot = Quat::from_axis_angle(binormal, angle);
            let child_dir = rot * tangent;

            let child_length = branch.spline.p1.distance(branch.spline.p0) * 0.6 * (0.7f32).powi(branch.depth as i32);
            let child_p1 = attach_pos + child_dir * child_length;

            let child_m0 = child_dir * child_length * 0.5;
            let child_m1 = child_dir * child_length * 0.3 * (1.0 - params.tropism); // トロピズム簡易

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

    /// 描画用頂点生成 (AxesVertex互換で中心線 + 色)
    #[allow(dead_code)]
    pub fn generate_vertices(&self) -> Vec<([f32; 3], [f32; 4])> {
        self.generate_vertices_at(Vec3::ZERO)
    }

    /// `base` を幹の根元（xz 上の植栽位置）として頂点を生成する。
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

            // ワールド座標と表示の Y 向きを揃える（描画時のみ Y を反転）
            verts.push(([p0.x, -p0.y, p0.z], color));
            verts.push(([p1.x, -p1.y, p1.z], color));
        }

        // 子枝
        for child in &branch.children {
            Self::collect_branch_vertices(child, base, verts, t_offset);
        }

        // 葉の簡易表現 (末端に星型 or 点群)
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
