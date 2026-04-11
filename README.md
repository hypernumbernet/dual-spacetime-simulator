# dual-spacetime-simulator

四次元時空や特殊時空まわりの挙動を、N体重力などのシミュレーションと 3D 可視化で試すデスクトップアプリです。描画は **Vulkan（Vulkano）**、UI は **egui** です。

## ビルド・実行

### 必要な環境

- **OS**: Windows、Linux など Vulkan が使える環境を想定
- **Rust**: `edition = "2024"` を使用しているため、**2024 Edition に対応した十分新しい stable**の `rustc` / `cargo`
- **Vulkan 1.x 対応 GPU** とドライバ（インスタンス・スワップチェーンが作成できること）
- **シェーダコンパイル**: `vulkano-shaders` が **shaderc** に依存するため、ビルド環境に **Vulkan SDK** など、shaderc がリンク／取得できるセットアップがあること

### コマンド

```powershell
cargo run --release
```

リリースプロファイルは `Cargo.toml` で `lto` 等が有効です。`cargo build --release` でも同様に成果物を生成できます。

## 主要クレート

| クレート | バージョン（`Cargo.toml`） | メモ |
|----------|---------------------------|------|
| **vulkano** | `0.35.2` | **features は `Cargo.toml` では未指定**（crates.io パッケージのデフォルト機能のみ） |
| **vulkano-shaders** | `0.35.0` | GLSL のコンパイル（ビルド時） |
| **vulkano-util** | `0.35.0` | `VulkanoContext`、ウィンドウ／スワップチェーン補助 |
| **winit** | `0.30.12` | ウィンドウ・入力 |
| **egui** / **egui-winit** | `0.31.1` | UI（`egui-winit` は `default-features = false`） |
| **glam** | `0.30.8` | 行列・ベクトル（ビュー／投影など） |
| **rayon** | `1.11.0` | シミュレーションの並列更新 |

その他: `ahash`, `rand`, `rand_distr`, `num_cpus`, `serde` / `serde_json`, `satkit` など。

## アーキテクチャ（描画まわり）

- **エントリとループ**（`src/main.rs`）  
  - `winit` の `ApplicationHandler` 実装 `App` がウィンドウ生成、`RedrawRequested` で 1 フレーム処理。  
  - `about_to_wait` で `request_redraw`、シミュレーション結果のバッファ更新、3D グラフモード時は `graph3d` の再計算。  
  - シミュレーション進行は別スレッド（`rayon` スレッドプール）で `SimulationManager::advance` を実行。

- **シーン描画** — `ParticleRenderPipeline`（`src/pipeline.rs`）  
  - `ordered_passes_renderpass!` で **同一カラーアタッチメントを 2 サブパス**（パス0: 3D、パス1: egui）に分割。  
  - **`pipeline_axes`**: `PrimitiveTopology::LineList` — 座標グリッド・軸、および 3D Graph モードの折れ線。頂点型 `AxesVertex`、プッシュ定数で `view_proj`。  
  - **`pipeline_particles`**: `PrimitiveTopology::PointList` — 粒子は点描画。`ParticleVertex` + プッシュ定数（`view_proj`, `size_scale`）。  
  - `ParticleRenderPipeline::render(...)` がプライマリコマンドバッファを組み立て、サブパス0ではセカンダリで軸／線／粒子を描画し、サブパス1で `Gui::draw_on_subpass_image` を実行。

- **UI オーバーレイ** — `Gui`（`src/integration.rs`）と **`Renderer`**（`src/renderer.rs`）  
  - `Renderer` は **egui 用**の `GraphicsPipeline`（頂点は `EguiVertex`、シェーダはファイルではなく **`vulkano_shaders::shader!` でインライン GLSL**）。  
  - `Gui::new_with_subpass` でレンダパスの **第 2 サブパス** に乗せる。

## シェーダ（GLSL ファイル）

次の 4 ファイルは `vulkano_shaders::shader!` の `path` から参照されます（`src/pipeline.rs`）。

| ファイル | 用途 |
|----------|------|
| `src/shaders/axes_vertex.vert` | 軸・グリッド・グラフ線（頂点） |
| `src/shaders/axes_fragment.frag` | 上記フラグメント |
| `src/shaders/particles_vertex.vert` | 粒子（点）頂点 |
| `src/shaders/particles_fragment.frag` | 粒子フラグメント |

※ egui レイヤーの頂点／フラグメントは `src/renderer.rs` 内の `shader!` マクロに埋め込まれています。

## 想定 GPU

- **Vulkan 対応**の一般的なディスクリート GPU または統合 GPU。  
- スワップチェーンは `B8G8R8A8_UNORM` を要求する初期化があります（`main.rs` の `create_window` クロージャ）。  
- レイトレーシング等の拡張機能には依存していません。

## 機能・現在の表示内容

- **Simulation モード**: N 体（ニュートン重力）に近い更新（種類は UI のシミュレーションタイプで切替）で粒子の位置・速度を更新し、**色付きの点**として描画。オプションで **XZ 平面のグリッドと座標軸**（線）を表示。  
- **3D Graph モード**: パラメータに応じた **3D 曲線／グラフを線分**で表示し、同じく点群も表示（`graph3d` モジュール）。  
- **egui**: メニュー（File / Mode / Panel / View など）、シミュレーション設定、初期条件、FPS・フレーム表示など。

## 操作方法

| 操作 | 内容 |
|------|------|
| **左ドラッグ** | カメラを軌道回転（revolve） |
| **右ドラッグ** | 視線の向き変更（look around） |
| **中ドラッグ** | 画面中心まわりのロール回転 |
| **マウスホイール** | ズーム |
| **左ダブルクリック** | カメラを **Y 軸上から** の見下ろし寄りに |
| **右ダブルクリック** | 注視点を原点付近にリセット |
| **メニュー / パネル** | シミュの開始・一時停止、リセット、モード切替、グリッド表示、各種スライダ・設定 |
