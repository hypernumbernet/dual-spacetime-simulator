# dual-spacetime-simulator

四次元時空や相対論モチーフの可視化を、粒子シミュレーションと 3D 描画で試すデスクトップアプリです。  
描画は **Vulkan（ash）**、UI は **egui** を使用しています。ワークスペースには粒子シミュレータ本体
（`dual-spacetime-simulator`）のほか、PGA ロケット打ち上げ・着陸シミュレータ（`pga-rocket`）なども
含まれます。

共有の代数学ライブラリは **`crates/dst-math`** にあり、Cargo workspace の path 依存で各クレートから利用します。

## リポジトリ構成

Cargo workspace 構成です。ルート `Cargo.toml` は workspace 定義（メンバー・共有依存・`[profile.release]`）のみを持ち、実装は `crates/` 配下の複数クレートに分かれます。クレート間は path 依存で連携します。

- **`dst-math`**（`crates/dst-math`・lib）— 数学ライブラリ
  - 双四元数・ビベクトル・PGA・`Spacetime` / ローレンツ変換を提供
  - 依存は `glam` のみで、Vulkan や UI に依存しない純粋な計算用クレート
  - 主なモジュール: `biquaternion` / `bivector` / `pga` / `spacetime`
  - 利用元: `dst-expand`、`dual-spacetime-simulator`、`pga-rocket`
- **`dst-expand`**（`crates/dst-expand`・lib + bin）— 代数の記号展開ツール
  - 基底積・サンドイッチ積・PGA を、数値評価せず記号のまま展開する
  - `dst-math` の乗法表をもとに計算する
  - CLI として `mul`（基底積）/ `table`（乗法表）/ `expr`（式の展開）などを提供
  - 主なモジュール: `biquaternion` / `expr` / `pga` / `pga_expr` / `format` / `algebra`
- **`dual-spacetime-simulator`**（`crates/dual-spacetime-simulator`・lib + bin）— 本アプリ
  - Vulkan（ash）+ egui で動くデスクトップ・シミュレータ本体
  - N 体重力シミュレーション（CPU / GPU）を可視化
  - `dst-math`（数学）と `vulkanvil`（Vulkan 基盤）に path 依存
  - 主なモジュール:
    - `simulation` / `gpu_simulation`: 粒子の物理更新（CPU は rayon 並列 / GPU はコンピュートシェーダ）
    - `pipeline` / `camera`: 描画パイプラインと軌道カメラ
    - `ui` / `ui_state` / `ui_styles` / `integration`: egui の UI 描画と状態管理
    - `object_input` / `solar_system_data`: 粒子の初期配置生成と JPL 暦データ取得
    - `settings` / `particle_snapshot`: 設定の永続化とスナップショットの保存・読み込み
  - `build.rs` が `glslc` で `src/shaders/` の GLSL を SPIR-V にコンパイルする
- **`pga-rocket`**（`crates/pga-rocket`・lib + bin）— PGA 脚付きロケット打ち上げ・着陸シミュレータ
  - G(3,0,1) 射影幾何代数（PGA）のモーター 1 個で剛体姿勢・位置・誘導を統一
  - 剛体物理（ジンバル推力・RCS・接地・破壊）と L/T 自動着陸オートパイロット
  - **L**: その場着陸 / **T**: 100–8000 m 先の T マーク目標への航法着陸（遠距離は ~520 m airplane 巡航）
  - `vulkanvil`（Vulkan）+ egui、`dst-math`（PGA 乗法表）に path 依存
  - 主なモジュール: `euclidean_pga` / `sim` / `landing` / `target_landing` / `fuzzy` / `mesh`
  - 詳細・操作・誘導アルゴリズム: [crates/pga-rocket/README.md](crates/pga-rocket/README.md)
- **`vulkanvil`**（`crates/vulkanvil`・lib）— 共有 Vulkan 基盤ライブラリ
  - ワークスペース内のレンダラが共有する Vulkan の土台
  - インスタンス / デバイス / スワップチェーン構築、バッファ・イメージ確保、シェーダモジュール生成を担う
  - 主なモジュール: `base` / `buffer` / `shader`
  - 利用元: `dual-spacetime-simulator`、`minecraft-clone`、`pga-rocket`
- **`minecraft-clone`**（`crates/minecraft-clone`・bin）— ボクセルデモアプリ
  - `vulkanvil` を土台にした Minecraft 風ボクセルワールドのデモ
  - 共有 Vulkan 基盤 `vulkanvil` の実利用例も兼ねる

ドキュメントは `docs/` にまとまっています。

- `design_overview.md`: プロジェクト全体の設計概要
- `double_spacetime_theory_overview.md`: 二重時空理論の数学的原理・力学要約
- `dst-expand-symbolic-computation-spec.md`: `dst-expand` を代数研究ツールへ発展させる設計仕様

## ビルド・実行

### 必要な環境

- **OS**: Windows / Linux など Vulkan が使える環境
- **Rust**: `edition = "2024"` に対応した stable (`rustc`, `cargo`)
- **GPU/Driver**: Vulkan 1.x 対応
- **GLSL コンパイラ**: `build.rs` で `glslc` を呼び出すため、`glslc` が `PATH` にあること（通常は Vulkan SDK 同梱）

### 実行コマンド

**粒子シミュレータ本体**（`dual-spacetime-simulator`）:

```powershell
cargo run -p dual-spacetime-simulator --release
```

**PGA ロケットシミュレータ**（`pga-rocket`）:

```powershell
cargo run -p pga-rocket --release
```

`cargo build -p dual-spacetime-simulator --release` や `cargo build -p pga-rocket --release` でも同じ設定（ルート `Cargo.toml` の `[profile.release]`）でビルドできます。

### バリデーションレイヤ付き実行（開発時のみ）

Vulkan の使い方に誤りがないかを開発中に検証したい場合は、**Vulkan バリデーションレイヤ**を有効にして起動します。

```powershell
$env:VK_INSTANCE_LAYERS = "VK_LAYER_KHRONOS_validation"
cargo run -p dual-spacetime-simulator
```

バリデーションレイヤは、API の呼び出し順序・同期・リソースのライフタイムなどの違反を検出し、標準エラー出力に警告（`VUID-...`）として報告してくれる開発支援機能です。実機ドライバでは見逃されがちな不正な使い方を、ハングやクラッシュとして表面化する前に発見できます。

有効化は上記の環境変数（または Vulkan SDK 付属の `vkconfig`）で行う**外部の仕組み**で、バイナリ自体はレイヤを組み込みません。そのため通常実行や `--release` ビルドでは何も指定しなければ自動的にオフになり、検証によるオーバーヘッドや出力は本番に持ち込まれません。検証中は警告ゼロを目安にしてください。

数学・展開クレート、または個別アプリのテスト:

```powershell
cargo test -p dst-math
cargo test -p dst-expand
cargo test -p pga-rocket
```

記号展開 CLI（例: 基底 `i` と `j` の積、インデックス 14 と 0）:

```powershell
cargo run -p dst-expand -- mul 14 0
cargo run -p dst-expand -- table
cargo run -p dst-expand -- expr "(ai+bkI)(cj+dkK)"
```

ワークスペース全体:

```powershell
cargo test --workspace
```

### テストと外部ネットワーク

**`cargo test` は外部ネットワーク（HTTP 等）に接続しません。** CI やオフライン環境でもそのまま実行できます。

太陽系配置モード（Solar System）の暦データ取得は **アプリ実行時のみ** 行われます。`satkit` 用の JPL 暦ファイル等を Google Cloud Storage 上のミラー（`astrokit-astro-data`）から `ureq` でダウンロードします。ダウンロードに失敗した場合は、組み込みのフォールバック粒子配置に切り替わります。

## 現在の主要依存

- **Vulkan 基盤**: `ash`, `ash-window`, `gpu-allocator`
- **UI**: `egui`, `egui-winit`, `egui-ash-renderer`
- **ウィンドウ/入力**: `winit`
- **数学/並列**: `glam`, `rayon`
- **その他**: `rand`, `rand_distr`, `serde`, `serde_json`, `satkit` など

## アーキテクチャ概要

設計の柱は 2 つです。**「ウィンドウ / Vulkan / UI / シミュレーションの責務を分離する」** こと、そして **「シミュレーション更新を描画ループから切り離す」** こと。その結果、各レイヤがほぼ独立した部品になっており、他アプリへ "つまみ食い" で持ち出しやすい構成になっています。以下、再利用しやすい順に紹介します。

### 共有 Vulkan 基盤 `vulkanvil`（まるごと流用できます）

`crates/vulkanvil` は、毎回書くのが面倒な Vulkan の定型処理を 1 クレートに閉じ込めた土台です。**同じワークスペースの `minecraft-clone` がこの基盤だけで動いている**ことが、汎用性の何よりの証拠です。

- `VulkanBase::new(window, mailbox, app_name, app_version)` の一呼び出しで、インスタンス / サーフェス / 物理・論理デバイス / スワップチェーン / コマンドプール / 同期オブジェクト / `gpu-allocator` までまとめて初期化
- フレームループ用ヘルパーが一通り揃う: `wait_for_fence` → `acquire_next_image` → `current_command_buffer` → `reset_fence` → `submit_and_present` → `advance_frame`
- リサイズは `recreate_swapchain(window)` を呼ぶだけ
- 実務で踏みがちな落とし穴に対処済み: **render-finished セマフォをスワップチェーンイメージごとに保持**（`VUID-vkQueueSubmit-pSignalSemaphores-00067` を回避）、`Drop` での逆順破棄、`device_wait_idle` の徹底
- `MAILBOX` / `FIFO`（Vsync）の切り替えと、2 枚の in-flight フレームによるダブルバッファリング

バッファ / イメージ / シェーダのヘルパーも単体で使えます。

- `create_buffer_with_data::<T: Pod>(...)`: `bytemuck` で型付きデータを CPU→GPU へアップロード
- `AllocatedBuffer` / `AllocatedImage`、デプス用の `create_depth_image` / `select_depth_format`
- `create_shader_module(device, spv)`: SPIR-V バイト列から `vk::ShaderModule` を生成

### 描画と計算を分離したアプリループ（`src/lib.rs`、エントリは `src/main.rs`）

`winit` の `ApplicationHandler` を実装した `App` がイベントループの中心です。重い更新処理を持つ可視化アプリにそのまま応用できる「UI スレッドと計算スレッドを分け、共有状態 + アトミックなフラグで橋渡しする」型を実装しています。

- **シミュレーションは専用ワーカースレッドで進行**し、描画フレームレートに縛られない。状態は `Arc<RwLock<UiState>>` と `Arc<RwLock<SimulationManager>>` で共有
- CPU→GPU の受け渡しはロックフリーの `GpuParticleSync`（`AtomicBool` / `AtomicU32`）で調停し、「全件アップロード」「既存を保ったまま追加」「進めるステップ数」をスレッド間で安全に伝える
- 最大 FPS 制限・描画スキップ・ダブルクリック判定など、対話アプリに必要な制御を内蔵
- 終了時は `App` の `Drop` で先に `device_wait_idle` してから GPU リソースを解放し、in-flight なコマンドバッファによる `ERROR_DEVICE_LOST` を防止

### 描画パイプライン `ParticleRenderPipeline`（`src/pipeline.rs`）

シーンと UI を **1 つのレンダーパス**で描き切る、軽量な前方描画パイプラインです。

- 深度テスト付きで 3D シーンを描いたあと、同じレンダーパス内で egui を重ねる（追加パス不要）
- グラフィックスパイプラインを用途別に保持: 軸・グリッド・中心マーカーは `LineList`、粒子は `PointList`（点表示と球表示のフラグメント違いを `ParticleDisplayMode` ごとに用意）
- **粒子は頂点バッファを使わず SSBO から直接読む**（`particles_vertex_ssbo.vert` が `gl_VertexIndex` で storage buffer を参照）。コンピュートが書き込むバッファをそのまま描画に使う **ゼロコピー構成**
- カメラ行列とポイントサイズは push constants で渡すだけ。リサイズは `recreate_framebuffers`、バッファ差し替えは retired バッファキューで安全に処理

### GPU コンピュート・シミュレーション（`src/gpu_simulation.rs`）

N 体計算をコンピュートシェーダ（`particles_compute.comp`）で回す部分です。CPU 版（`simulation.rs`、`rayon` 並列）と UI から切り替えられる二刀流になっています。

- 粒子は 64 バイト（`vec4` ×4: 位置 / 速度 / 属性 / 色）の SSBO に格納
- `record_gpu_advance` / `dispatch` で更新ステップをコマンドバッファに記録し、`upload_from_cpu` / `readback_to_cpu` で CPU と往復
- **同じ SSBO をコンピュートの書き込み先と頂点シェーダの入力に共用**するため、計算結果を描画へ転送するコストがない
- `add_particles_preserving_simulated`: 走行中の GPU 状態を読み戻してから追加・再アップロードし、既存粒子の位置を保ったまま粒子を足せる
- `remove_particle_preserving_simulated`: GPU キューを待機したうえで mapped SSBO を in-place に詰め、残り粒子の位置を保ったまま粒子を減らせる

### egui 統合 `Gui`（`src/integration.rs`）

`egui-winit`（入力）と `egui-ash-renderer`（Vulkan 描画）を薄くまとめたラッパです。

- `immediate_ui(window, |gui| { ... })` で即時 UI を記述し、`prepare_frame` → `draw(cb, extent)` → `finish_frame` の定型でフレームに載せる
- `update(event)` がイベントを egui が消費したかを返し、`pointer_wants_input()` と合わせて **「UI とシーンのどちらがマウスを取るか」** の取り合いを綺麗に裁ける
- スワップチェーンの sRGB 判定やテクスチャの set / free ライフサイクルも内側で面倒を見る

### 軌道カメラ `OrbitCamera`（`src/camera.rs`）

`glam` だけに依存し、**Vulkan 非依存**でどんな 3D アプリにも移植できるカメラです。

- 軌道回転 `revolve` / 視線回転 `look_around` / ズーム `zoom` / ロール `rotate`
- `center_target_on_origin` は slerp による短いアニメーション付き
- up ロック時はピッチをクランプしてジンバルロック（特異点）を回避

### そのほか流用しやすい部品

- `AppSettings`（`src/settings.rs`）: `serde` で設定を JSON 永続化するシンプルな実装
- `ParticleSnapshot`（`src/particle_snapshot.rs`）: 粒子状態を zip で保存・読み込み
- `solar_system_data.rs`: `ureq` でリモートデータを取得し、失敗時はフォールバックに切り替える堅牢な取得処理
- `dst-math` / `dst-expand`: `glam` だけで完結する双四元数・PGA・ローレンツ変換と、その記号展開（前述）

## シミュレーターの操作方法（dual-spacetime-simulator）

### カメラモード

**Lock Camera Up/Down**（設定パネルのチェックボックス）で **軌道カメラ**（ON・デフォルト）と **宇宙船カメラ**（OFF）を切り替えます。シーン上の **中ボタンクリック** または **End** キーでも切り替えられます。

### 軌道カメラ（Lock Camera Up/Down = ON）

**マウス**

- **左ドラッグ**: 注視点を中心に軌道回転
- **右ドラッグ**: 視線方向を回転
- **中ドラッグ**: 画面中心周りにロール
- **ホイール**: 視線方向へ前後移動
- **右ダブルクリック** / **Home**: 注視点を原点付近へリセット
- **左クリック**（ドラッグなし）: 画面上の最近傍粒子を選択

**キーボード**

- **W / S**, **A / D**: 水平パン
- **Q / E**: 注視点周りのヨー
- **Space / Shift**: カメラの上下移動
- **↑↓←→**: 注視点の上下・水平移動

### 宇宙船カメラ（Lock Camera Up/Down = OFF）

**マウス**

- **左クリック**: 操舵アンカー（⊕ マーカー）の ON/OFF。アンカー位置からのカーソルオフセットでロール／ピッチ
- **右ボタン押下中**: ヨー操舵アンカー（⇔ マーカー）。水平オフセットでヨー（離すと解除）
- **ホイール**: 視線方向への前後スラスト
- ドラッグによる軌道操作は行いません

**キーボード**

- **W / S**: ピッチ、**A / D**: ロール、**Q / E**: ヨー
- **Space / Shift**: 前後スラスト

### 粒子追跡（Trace）

- 軌道カメラで粒子を選択し、粒子情報パネルの **Trace** ボタンで有効化
- カメラが選択粒子の後方から追従
- Trace 有効中: **ホイール** および **W / S** で追従距離を調整。**Space / Shift** は無効

### シミュレーション全般

- **Pause**: シミュレーションの一時停止 / 再開（トグル）
- **Escape**: シミュレーション停止（再開不可）。Trace モード中は Trace オフ、⊕ マーク表示中は消去
- **メニュー / パネル**: シミュレーション開始/停止、各種パラメータ変更
- UI スライダーの **ダブルクリック** でデフォルト値にリセット
