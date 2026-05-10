# プロジェクトの全体設計概要

このリポジトリは **Rust** で書かれたデスクトップアプリです。**Vulkan（ash）** で 3D を描画し、**egui** で UI を重ねます。主要機能は次の 3 つです。

1. **N 体に近い重力シミュレーション**（ニュートン近似に、光速スケールやローレンツ変換に基づく変種を追加）
2. **3D Graph モード**：双四元数・行列によるラピディティ場や球上フィボナッチ格子などを、サンプル点と補助線として可視化
3. **GPU Tree モード**：計算シェーダで木状メッシュを生成し、ラインまたはポリゴンで描画

設計方針は、**ウィンドウ / Vulkan / UI / シミュレーションの責務を分離すること**、および **シミュレーション更新を描画ループから切り離すこと**です。

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

---

## 2. アプリケーションの中心構造

- **`src/main.rs`**：バイナリのエントリ。`dual_spacetime_simulator::run()` のみ呼び出します。
- **`src/lib.rs`**：`winit` の `ApplicationHandler` を実装した **`App`**、`run()`、`spawn_simulation_worker` を含みます。統合テスト用にモジュールを公開します。

### 2.1 `App` が保持する主な状態

- **`VulkanBase`**：インスタンス、スワップチェーン、コマンドバッファ、フェンス／セマフォ、`gpu-allocator` など
- **`ParticleRenderPipeline`**：レンダパス、グラフィックス／コンピュートパイプライン、頂点バッファ更新、軌道カメラ
- **`Gui`**（`integration.rs`）：`egui` + `egui-ash-renderer` による UI メッシュの Vulkan への載せ込み
- **`Arc<RwLock<UiState>>`**：UI とシミュスレッド双方から読み書き
- **`Arc<RwLock<SimulationManager>>`**：シミュレーション状態（粒子ベクトル）
- **`need_redraw` / `skip_redraw`**：シミュ結果を GPU バッファへ反映するタイミング制御
- **`AppSettings`**：`setting.config`（実行ファイルと同じディレクトリの JSON）へのロード／セーブ。起動時に `UiState::apply_settings` でランタイム状態へ反映
- **`graph3d_pending_rx`**：`AppMode::Graph3D` のとき、パラメータ変更時にバックグラウンドスレッドで点列・ライン頂点を生成し、`mpsc` でメインループへ返す非同期ビルド用の受信側
- **`last_graph3d_fingerprint` / `last_gpu_tree_fingerprint`**：モード別に GPU バッファ更新が必要か判定
- **`prev_app_mode`**：モード遷移時（Simulation 復帰、GpuTree から Graph3D へ等）の再構築・リセット制御
- **`drag_owner`**（`DragOwner`）：egui がポインタを掴んでいるときはシーンのカメラ操作と衝突しないよう、左／右／中ドラッグの担当を区別

**ドロップ順**：`gui` と `render_pipeline` を **`vulkan_base` より先に** 破棄する必要があります。

### 2.2 シミュレーションと描画の分離

- メインスレッド：`RedrawRequested` で **UI 更新** → **描画コマンド記録** → **Present**。スワップチェーンは `UiState::mailbox_present_mode` と一致するよう、必要に応じて再作成します。
- 別スレッド：`UiState` の `is_running` と `AppMode::Simulation` のときだけ、`rayon` スレッドプール上で `SimulationManager::advance` を周期実行。終了後に `need_redraw` を立て、`about_to_wait` 側で粒子バッファを `pipeline.set_particles` に流し込みます。
- **`AppMode::Graph3D`**：`about_to_wait` でフィンガープリントが変わったときのみ、専用スレッドで `graph3d::build_points` / `build_graph_line_vertices` を実行し、完了次第パイプラインへ反映します（UI スレッドをブロックしません）。
- **`AppMode::GpuTree`**：フィンガープリントに応じて CPU ライン頂点生成または `compute_tree_vertices`（コンピュートシェーダ）を選択します。

これにより、UI の応答性とシミュレーション／重い可視化のスループットを両立しています。

---

## 3. アプリモード（`UiState` / `src/ui_state.rs`）

| モード | 役割 |
|--------|------|
| **`Simulation`** | 粒子シミュレーション。左ペインは Simulation / Initial Condition / Settings |
| **`Graph3D`** | `graph3d.rs` が点列とグラフ用ライン頂点を生成し、粒子パイプライン＋ライン描画で表示。ペインは 3D Graph / Settings。**グラフ種別**は `GraphType`：`SphericalFibonacciLattice`、`RapidityFieldMatrix`、`RapidityFieldBiquaternion` |
| **`GpuTree`** | `tree_compute.comp` で頂点を生成（または CPU でライン頂点）。**Lines** / **Polygons**、**Single** / **Forest on XZ Grid**。ペインは GPU Tree / Settings |

モード切替時は `about_to_wait` でフィンガープリントや前モードを参照し、必要な GPU バッファのみ更新します。

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

乱数球・立方体、二球、渦巻き円盤、太陽系、衛星軌道、楕円軌道などを `InitialCondition` として定義し、`generate_particles` で `Particle` ベクトルを返します。

---

## 6. 数学モジュール（`src/math/`）

- **`spacetime.rs`**：時空・ラピディティ周り（ユニットテストあり）。Graph3D のラピディティ場モードでも参照されます。
- **`biquaternion.rs`**, **`bivector.rs`**：Graph3D（双四元数ベースのラピディティ可視化など）に使用（四元数関連にテストあり）

---

## 7. 木生成（`src/tree.rs` + `src/shaders/tree_compute.comp`）

Weber–Penn 系に着想を得た手続き木。**CPU** でスプライン頂点を出す経路と、**GPU コンピュート**で三角形メッシュを書き出す経路があり、`GpuTreeRenderMode` と `ParticleRenderPipeline::compute_tree_vertices` で切り替わります。

---

## 8. レンダリングパイプライン（`src/pipeline.rs`）

- **軌道カメラ**（`camera.rs` とパイプライン経由）：**左ドラッグ**で軌道回転（revolve）、**右ドラッグ**で視線の向き変更（look around）、**中ドラッグ**で画面中心周りの回転、**ホイール**でズーム。**左ダブルクリック**で上方向を整える（`y_top`）、**右ダブルクリック**でターゲットを原点付近へ（`center_target_on_origin`）。`UiState::lock_camera_up` により挙動を制御できます。
- **座標軸・グリッド**、**粒子（点スプライト風のサイズ定数）**、**補助ライン**、**木メッシュ**、最後に **egui** を同一レンダパス／フレームバッファへ合成
- **`shader_blobs`**：ビルド済み SPIR-V の一部を `include_bytes!` で公開し、Vulkan 系統合テストがバイナリ存在を検証できるようにしています。
- シェーダは `build.rs` が `glslc` で **SPIR-V** にコンパイル（`OUT_DIR/shaders/*.spv`）

シェーダ一覧（ソースは `src/shaders/`）：

- `axes_vertex.vert` / `axes_fragment.frag`
- `particles_vertex.vert` / `particles_fragment.frag`
- `tree_vertex.vert` / `tree_fragment.frag`
- `tree_compute.comp`
- `egui_vertex.vert` / `egui_fragment.frag`

---

## 9. UI（`src/ui.rs`, `src/ui_styles.rs`）

`draw_ui` が `UiState` と `AppSettings` を編集します。粒子数、時間刻み、スケール、シミュレーション種別、Graph / GPU Tree の各パラメータ、ウィンドウ・プレゼントモード・カメラ関連の固定設定、および設定保存（`AppSettings::save`）を担当します。

---

## 10. ビルド要件

- **Rust**：`edition = "2024"` に対応したツールチェーン（安定版の新しめ推奨）
- **Vulkan SDK**：`glslc` が `PATH` に通っていること（`build.rs` がコンパイル失敗時に明示的にエラーを出します）

---

## 11. テスト

- **`src/lib.rs`** でモジュールを公開し、**`tests/`** に統合テストを配置（`cargo test`）。例：`simulation`、`initial_condition`、`tree`、`camera`、`settings`、`ui_state`、`graph3d`、`math_spacetime`、`math_biquaternion`、`math_bivector` など。
- **Vulkan ヘッドレス系**（`tests/vulkan_base_headless.rs`, `tests/vulkan_compute.rs`）は `#[ignore]` 既定。GPU 環境で `cargo test -- --ignored` を実行。
- **`src/math/spacetime.rs`** と **`src/math/biquaternion.rs`** にモジュール内 `#[test]` が引き続き存在します。

---

## 12. 補足

- バイナリの `main` は `src/main.rs` のみ。アプリ本体の **`App`** とイベントループ **`run()`** は **`src/lib.rs`** にあります。
- その他のモジュールは `camera`, `graph3d`, `initial_condition`, `integration`, `math`, `pipeline`, `settings`, `simulation`, `tree`, `ui`, `ui_state`, `ui_styles`, `vulkan_base` を参照します。
- シェーダは `src/shaders/*.vert|*.frag|*.comp` を `build.rs` で `glslc` コンパイルし、`OUT_DIR/shaders/*.spv` を `include_bytes!` で読み込む構成です。
