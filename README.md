# dual-spacetime-simulator

四次元時空や相対論モチーフの可視化を、粒子シミュレーションと 3D 描画で試すデスクトップアプリです。  
描画は **Vulkan（ash）**、UI は **egui** を使用しています。

## ビルド・実行

### 必要な環境

- **OS**: Windows / Linux など Vulkan が使える環境
- **Rust**: `edition = "2024"` に対応した stable (`rustc`, `cargo`)
- **GPU/Driver**: Vulkan 1.x 対応
- **GLSL コンパイラ**: `build.rs` で `glslc` を呼び出すため、`glslc` が `PATH` にあること（通常は Vulkan SDK 同梱）

### 実行コマンド

```powershell
cargo run --release
```

`cargo build --release` でも同じ設定（`lto = true`, `codegen-units = 1`, `panic = "abort"`）でビルドできます。

## 現在の主要依存

- **Vulkan 基盤**: `ash`, `ash-window`, `gpu-allocator`
- **UI**: `egui`, `egui-winit`, `egui-ash-renderer`
- **ウィンドウ/入力**: `winit`
- **数学/並列**: `glam`, `rayon`
- **その他**: `rand`, `rand_distr`, `serde`, `serde_json`, `satkit` など

## アーキテクチャ概要

### アプリ進行 (`src/main.rs`)

- `winit` の `ApplicationHandler` 実装 `App` がイベントループを管理
- `resumed` で `VulkanBase` と `ParticleRenderPipeline`、`Gui` を初期化
- シミュレーション更新は別スレッドで実行し、描画タイミングと分離
- `UiState` と `SimulationManager` は `Arc<RwLock<...>>` で共有

### Vulkan 初期化 (`src/vulkan_base.rs`)

- `ash` で Vulkan インスタンス / サーフェス / 論理デバイスを構築
- グラフィックス + プレゼント可能なキューファミリを選択
- スワップチェーン、イメージビュー、コマンドプール、同期オブジェクトを管理
- メモリ割り当ては `gpu-allocator` を使用

### 描画パイプライン (`src/pipeline.rs`)

- 1 つのレンダーパス内でシーン描画後に egui を重ねる構成
- グラフィックスパイプラインは以下を用途別に保持:
  - 軸・グリッド・グラフ線 (`LineList`)
  - 粒子 (`PointList`)
  - GPU Tree (`LineList` / `TriangleList` をモードで切替)
- GPU Tree 用のコンピュートパイプラインを持ち、木構造頂点の生成更新を実行

### UI 統合 (`src/integration.rs`, `src/ui.rs`, `src/ui_state.rs`)

- `egui-winit` で入力処理、`egui-ash-renderer` で Vulkan コマンドに描画を記録
- モード切替、各種パネル表示、設定保存 (`AppSettings`) を UI から操作
- パネル表示状態やシミュレーション状態は `UiState` に集約

## 実装済みモード

- **Simulation**: N 体重力系ベースの粒子更新と可視化
- **3D Graph**: 相対論モチーフの各種 3D グラフ（Light Cone など）を線描画
- **GPU Tree**: GPU 側計算を使った木構造描画（Lines / Polygons）

## シェーダ

シェーダは `src/shaders/` 配下の GLSL を `build.rs` で `glslc` コンパイルし、  
実行時には `include_bytes!(concat!(env!("OUT_DIR"), ...))` で `.spv` を読み込みます。

主なファイル:

- `axes_vertex.vert` / `axes_fragment.frag`
- `particles_vertex.vert` / `particles_fragment.frag`
- `tree_vertex.vert` / `tree_fragment.frag`
- `tree_compute.comp`
- `egui_vertex.vert` / `egui_fragment.frag`

## 操作方法

- **左ドラッグ**: カメラを軌道回転
- **右ドラッグ**: 視線方向を回転
- **中ドラッグ**: ロール回転
- **マウスホイール**: ズーム
- **左ダブルクリック**: 俯瞰寄りの視点へ
- **右ダブルクリック**: 注視点を原点付近へリセット
- **メニュー / パネル**: モード切替、シミュレーション開始/停止、各種パラメータ変更
