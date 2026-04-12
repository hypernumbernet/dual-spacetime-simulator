# プロジェクトの全体設計概要

このリポジトリは **Rust** で書かれたデスクトップアプリです。**Vulkan（ash）** で 3D を描画し、**egui** で操作パネルを重ねます。中身は大きく次の 3 つです。

1. **N 体に近い重力シミュレーション**（ニュートン近似に、光速スケールやローレンツ変換に基づく変種を追加）
2. **3D Graph モード**：双四元数・双ベクトルなどの幾何を、サンプル点と補助線として可視化
3. **GPU Tree モード**：計算シェーダで木状メッシュを生成し、ラインまたはポリゴンで描画

設計方針は、**ウィンドウ・Vulkan・UI・シミュレーションの責務をモジュールで分ける**こと、および **シミュレーションを描画ループから切り離してバックグラウンドスレッドで進める**ことです。

---

## 1. 依存関係と外部ライブラリ

`Cargo.toml`（エディション **2024**、パッケージ **0.2.0**）より。

| 用途 | クレート |
|------|-----------|
| ウィンドウ・イベント | `winit` |
| Vulkan ローレベル | `ash`, `ash-window` |
| メモリ割り当て | `gpu-allocator` |
| UI | `egui`, `egui-winit`, `egui-ash-renderer` |
| 数学 | `glam` |
| CPU 並列（粒子の力積分など） | `rayon`, `num_cpus` |
| 乱数・分布 | `rand`, `rand_distr` |
| 設定ファイル | `serde`, `serde_json` |
| 太陽系初期条件（暦） | `satkit` |
| その他 | `bytemuck`, `raw-window-handle`, `ahash` |

**注**：以前の設計メモにあった **Vulkano** は現行実装では使っていません。Vulkan は **ash** 直叩きです。

---

## 2. アプリケーションの中心構造

`winit` の `ApplicationHandler` を実装した **`App`**（`src/main.rs`）がエントリです。

### 2.1 `App` が保持する主な状態

- **`VulkanBase`**：インスタンス、スワップチェーン、コマンドバッファ、フェンス／セマフォ、`gpu-allocator` など
- **`ParticleRenderPipeline`**：レンダパス、グラフィックス／コンピュートパイプライン、頂点バッファ更新、軌道カメラ
- **`Gui`**（`integration.rs`）：`egui` + `egui-ash-renderer` による UI メッシュの Vulkan への載せ込み
- **`Arc<RwLock<UiState>>`**：UI とシミュスレッド双方から読み書き
- **`Arc<RwLock<SimulationManager>>`**：シミュレーション状態（粒子ベクトル）
- **`need_redraw` / `skip_redraw`**：シミュ結果を GPU バッファへ反映するタイミング制御
- **`AppSettings`**：実行ファイル近傍の `setting.config`（JSON）から読み込み

**ドロップ順**：コメントどおり、`gui` と `render_pipeline` を **`vulkan_base` より先に** 破棄する必要があります。

### 2.2 シミュレーションと描画の分離

- メインスレッド：`RedrawRequested` で **egui の即時 UI** → **コマンドバッファ記録** → **Present**
- 別スレッド：`UiState` の `is_running` と `AppMode::Simulation` のときだけ、`rayon` スレッドプール上で `SimulationManager::advance` を周期実行。終了後に `need_redraw` を立て、`about_to_wait` 側で粒子バッファを `pipeline.set_particles` に流し込みます。

これにより、UI の応答性とシミュレーションのスループットを両立しています。

---

## 3. アプリモード（`UiState` / `src/ui_state.rs`）

| モード | 役割 |
|--------|------|
| **`Simulation`** | 粒子シミュレーション。左ペインは Simulation / Initial Condition / Settings |
| **`Graph3D`** | `graph3d.rs` が点列とグラフ用ライン頂点を生成し、粒子パイプライン＋ライン描画で表示。ペインは 3D Graph / Settings |
| **`GpuTree`** | `tree_compute.comp` で頂点を生成（または CPU でライン頂点）。**Lines** / **Polygons**、**Single** / **Forest on XZ Grid**。ペインは GPU Tree / Settings |

モード切替時に `about_to_wait` でフィンガープリントを見て、GPU バッファの更新をスキップまたは差し替えます。

---

## 4. シミュレーション（`src/simulation.rs`）

- **`SimulationManager`**：`reset` で初期条件から `SimulationState` を構築し、`advance` で時間発展（`advance_time` のあと `update_velocities`）
- **`SimulationState`** のバリアント：
  - **Normal**：古典的 N 体風（ペア和の重力、並列）
  - **SpeedOfLightLimit**：前進を \(\gamma^{-1}\) でスケール
  - **LorentzTransformation**：速度更新にラピディティ／`Spacetime` を用いた変種、位置更新も相対論的な形

物理定数（`G`, `c`, `AU` 等）は `simulation.rs` に集約されています。

---

## 5. 初期条件（`src/initial_condition.rs`）

乱数球・立方体、二球、渦巻き円盤、**太陽系（`satkit` + JPL 系データ）** など、列挙子 `InitialCondition` として定義され、`generate_particles` で `Particle` ベクトルを返します。

---

## 6. 数学モジュール（`src/math/`）

- **`spacetime.rs`**：時空・ラピディティ周り（ユニットテストあり）
- **`biquaternion.rs`**, **`bivector.rs`**：Graph3D の可視化に使用（四元数関連にテストあり）

---

## 7. 木生成（`src/tree.rs` + `src/shaders/tree_compute.comp`）

Weber–Penn 系に着想を得た手続き木。**CPU** でスプライン頂点を出す経路と、**GPU コンピュート**で三角形メッシュを書き出す経路があり、`GpuTreeRenderMode` と `ParticleRenderPipeline::compute_tree_vertices` で切り替わります。

---

## 8. レンダリングパイプライン（`src/pipeline.rs`）

- **軌道カメラ**（`camera.rs`）：ドラッグで回転、ホイールでズーム、ダブルクリックで視点プリセット（`main.rs` のマウス処理から呼び出し）
- **座標軸・グリッド**、**粒子（点スプライト風のサイズ定数）**、**補助ライン**、**木メッシュ**、最後に **egui** を同一レンダパス／フレームバッファへ合成
- シェーダは `build.rs` が `glslc` で **SPIR-V** にコンパイル（`OUT_DIR/shaders/*.spv`）

シェーダ一覧（ソースは `src/shaders/`）：

- `axes_vertex.vert` / `axes_fragment.frag`
- `particles_vertex.vert` / `particles_fragment.frag`
- `tree_vertex.vert` / `tree_fragment.frag`
- `tree_compute.comp`
- `egui_vertex.vert` / `egui_fragment.frag`

---

## 9. UI（`src/ui.rs`, `src/ui_styles.rs`）

`draw_ui` が `UiState` と `AppSettings` を編集します。実行パラメータ（粒子数、時間刻み、スケール、シミュ種別、Graph／GpuTree のパラメータなど）と、設定の保存（`AppSettings::save`）を担当します。

---

## 10. ビルド要件

- **Rust**：`edition = "2024"` に対応したツールチェーン（安定版の新しめ推奨）
- **Vulkan SDK**：`glslc` が `PATH` に通っていること（`build.rs` がコンパイル失敗時に明示的にエラーを出します）

---

## 11. テスト

- **統合テスト用の `tests/` ディレクトリは現状なし**
- **`src/math/spacetime.rs`** と **`src/math/biquaternion.rs`** にモジュール内 `#[test]` が存在します

---

## 12. 開発環境のメモ

- **OS**：Windows 10/11 での動作確認が主想定。Vulkan 対応 GPU と最新ドライバを推奨
- **エディタ**：Visual Studio Code + `rust-analyzer` など
- **リポジトリ**：例として `git clone https://github.com/hypernumbernet/dual-spacetime-simulator.git`
- **CI**：リポジトリ直下に `.github` ワークフローが無い場合もあるため、自動ビルドはプロジェクト設定に依存します

### セットアップの流れ（要約）

1. `rustup` で Rust を入れ、`cargo build` が通る状態にする  
2. Vulkan SDK を入れ、`glslc` が使えることを確認する  
3. 必要ならリポジトリをクローンし、ルートで `cargo run`

---

以上が、現行ソースツリー（`main.rs` が宣言するモジュール構成と `Cargo.toml`）に整合した設計概要です。
