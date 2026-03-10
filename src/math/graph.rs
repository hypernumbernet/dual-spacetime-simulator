use crate::ui_state::MathGraphSurfaceFunction;

/// 3D サーフェス用の頂点群（位置と色）を生成する。
/// ここでは頂点をポイントクラウドとして扱い、既存の粒子レンダラに流し込むことを想定する。
pub fn generate_surface_points(
    surface_function: MathGraphSurfaceFunction,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    resolution: u32,
) -> (Vec<[f32; 3]>, Vec<[f32; 4]>) {
    let n = resolution.max(2) as usize;
    let dx = if n > 1 {
        (x_max - x_min) / (n - 1) as f64
    } else {
        1.0
    };
    let dy = if n > 1 {
        (y_max - y_min) / (n - 1) as f64
    } else {
        1.0
    };

    let mut positions = Vec::with_capacity(n * n);
    let mut zs = Vec::with_capacity(n * n);

    for iy in 0..n {
        let y = y_min + iy as f64 * dy;
        for ix in 0..n {
            let x = x_min + ix as f64 * dx;
            let z = match surface_function {
                MathGraphSurfaceFunction::SinCos => x.sin() * y.cos(),
                MathGraphSurfaceFunction::Paraboloid => x * x + y * y,
            };
            positions.push([x as f32, z as f32, y as f32]);
            zs.push(z);
        }
    }

    // z の範囲に応じて簡易的なカラーマップを適用する。
    let (min_z, max_z) = zs
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min_z, max_z), &z| {
            (min_z.min(z), max_z.max(z))
        });
    let span = (max_z - min_z).abs().max(std::f64::EPSILON);

    let colors: Vec<[f32; 4]> = zs
        .into_iter()
        .map(|z| {
            let t = ((z - min_z) / span).clamp(0.0, 1.0);
            // 青→シアン→黄→赤 のグラデーションっぽいカラーマップ
            let r = if t < 0.5 { 0.0 } else { (t - 0.5) * 2.0 };
            let g = (1.0 - (t - 0.5).abs() * 2.0).max(0.0);
            let b = if t > 0.5 { 0.0 } else { (0.5 - t) * 2.0 };
            [r as f32, g as f32, b as f32, 1.0]
        })
        .collect();

    (positions, colors)
}

