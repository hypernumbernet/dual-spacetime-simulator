# pga-rocket

3D 射影幾何代数（PGA: Projective Geometric Algebra）を土台にした、脚付きロケットの
打ち上げ・着陸シミュレータです。剛体の姿勢・位置を PGA のモーター（motor）1 個で表現し、
物理・接地判定・自動着陸誘導のすべてを PGA のサンドイッチ積で計算します。
描画は Vulkan（vulkanvil）、UI は egui で、半辺 20 km のオープンワールド地面と
手続き生成テクスチャの上を飛びます（**Moon mode** で真空・月面に切り替え）。

```
cargo run --release -p pga-rocket        # シミュレータ本体を起動
cargo test -p pga-rocket                 # 物理・制御・着陸の全テスト
```

## 操作方法

| キー | 動作 |
|---|---|
| Space | スロットル増加（押している間） |
| 左/右 Ctrl | スロットル減少（押している間） |
| F | フルスロットルへ 200 ms でランプ（離しても継続） |
| C | スロットル 0 へ 200 ms でランプ（離しても継続） |
| W / S | ピッチ（メインエンジンのジンバル、体軸 +X 回り。推力がないと効かない） |
| Q / E | ヨー（ジンバル、体軸 +Z 回り） |
| A / D | ロール（胴体中央の RCS スラスタ 4 基、体軸 +Y 回り。地上でも効く） |
| L | **その場への自動着陸オートパイロットのトグル** |
| T | **T マーク目標地点への自動着陸のトグル**（Climb → Cruise → Descend の 3 フェーズ。詳細は「[T モードの自動ターゲット着陸](#t-モードの自動ターゲット着陸)」） |
| M | **Moon mode** のトグル（空気抵抗ゼロ＋月面ビジュアル） |
| R | リセット（機体を発射パッドへ戻し、T マークを再抽選） |
| Y | **負荷実験**: ランダムな位置・高度・速度・姿勢・角速度で飛行開始し、**T モードを強制 ON**（失敗もサンプリングから除外しない） |
| ←→↑↓ | カメラ回転（マウスドラッグでも可） |
| PageUp / PageDown | カメラ距離（マウスホイールでも可。20–400 m） |
| Esc | 終了 |

L と T は相互排他で、一方を ON にすると他方は OFF になります。オートパイロット動作中に
**手動飛行キー（Space / Ctrl / F / C / W / S / A / D / Q / E）を押すと両方とも解除**され、
即座に手動操縦へ戻ります。

接地時の法線方向衝突速度が `crash_impact_speed`（既定 10 m/s）を超えると機体は爆発します。
自動着陸（L）はどんな姿勢からでも起動でき、横倒し・高速落下・倒立からの回復に対応しています
（倒立は CoM 高度 ~150 m 以上が物理的な回復下限。詳細は後述）。
目標着陸（T）は **100–8000 m** の環状領域に置かれる黄色 T マーク地点へ航法します。
誘導は PGA の逆サンドイッチ輸送で得た幾何量だけを使い、上昇と水平移動を同時に行う
フルスロットル Climb → 停止距離 `d_stop` と短ホライズン MPC で組み立てる Cruise →
閉ループ自殺バーンの Descend、という 3 段構成です。**着陸成功（complete）** は描画パッド
（半辺 **30 m**、`TARGET_PAD_HALF_M`）上への静かな接地で、内側 Chebyshev 箱（半辺 **12 m**、
`TARGET_SUCCESS_HALF_M`）は誘導目標であって complete の条件ではありません。

### Y キー負荷実験（ランダム IC → T モード）

**Y** を押すと、[`RocketState::random_load_test`](src/sim.rs) が次の範囲で
位置・速度・姿勢・角速度を一様乱数で抽選し、**T オートパイロットを強制 ON** します
（既存の T マーク位置はそのまま。目標の再抽選は **R** のみ）。

| 量 | 範囲 |
|---|---|
| 横位置 `x`, `z` | 各 ±30 000 m |
| CoM 高度 | パッド静止相当 〜 100 000 m |
| 速度 | 0 〜 3000 km/h（方向は球面一様） |
| 姿勢 pitch / yaw / roll | 各 ±π rad |
| 角速度 | 各 ±π rad/s |

倒立・高速落下・低高度など回復不能に近い IC も意図的に含めます。
**失敗（爆発・オフパッド等）をサンプリングや評価から除外しません** — 負荷実験として
そのまま T 誘導の限界を観察する用途です。

## モジュール構成

| モジュール | 内容 |
|---|---|
| [euclidean_pga.rs](src/euclidean_pga.rs) | G(3,0,1) の 16 成分マルチベクタと幾何プリミティブ |
| [sim.rs](src/sim.rs) | 剛体物理（重力・ジンバル推力・RCS・接地・破壊判定） |
| [fuzzy.rs](src/fuzzy.rs) | メンバシップ・TS ブレンド・L/T 誘導の連続仲裁（安全ラッチは置換しない） |
| [landing.rs](src/landing.rs) | その場への自動着陸オートパイロット（L キー） |
| [target_landing.rs](src/target_landing.rs) | T マーク目標への航法付き自動着陸（T キー） |
| [control.rs](src/control.rs) | キー入力 → 制御コマンドの純粋写像（F/C ラッチは 0.2 s ランプ） |
| [mesh.rs](src/mesh.rs) / [explosion.rs](src/explosion.rs) | 機体・地面（半辺 20 km）・発射／目標パッド・爆発のジオメトリ生成、目標 XZ の抽選 |
| [texture.rs](src/texture.rs) | 地面アルベドの手続き生成（草地 fBm / 月レゴリス＋クレータ / 舗装、256 px タイル＋ミップ） |
| [app.rs](src/app.rs) / [renderer.rs](src/renderer.rs) / [ui.rs](src/ui.rs) | ウィンドウ・Vulkan 描画・左ドックパネル（原点パッド＋**100 m 〜 8000 m** のランダム距離先の T 目標。パッドマークは `ground.frag` が描画） |

`lib.rs` が公開するのは物理・制御・生成系（`control` / `euclidean_pga` / `explosion` /
`fuzzy` / `landing` / `mesh` / `sim` / `target_landing` / `texture`）で、
`app` / `renderer` / `ui` / `integration` はバイナリ専用です。

左パネルの速度表示は **km/h**（`MS_TO_KMH`）、上部 HUD の `vel_y` は m/s です。
`Land (L)` / `Target (T)` 行にはオートパイロットの現在フェーズラベルが出ます。

物理・制御・誘導のモジュールはウィンドウ/GPU に依存しない純粋な計算なので、
実際の物理そのものをユニットテストで検証できます。

## PGA 計算について

### 代数の定義: G(3,0,1)

生成元は 4 つで、計量は縮退（degenerate）しています:

- `e0` — 零基底（e0² = 0）: 射影方向。「無限遠」と平行移動を担う
- `e1, e2, e3` — 通常のユークリッド基底（e² = +1）

これらの外積で 2⁴ = 16 個の基底ブレードができ、
[`Multivector`](src/euclidean_pga.rs) は 16 成分の係数配列としてこれを保持します。
幾何積の符号・縮約規則は `dst_math::pga::basis_mul_with_metric` によるビットマスク演算で、
テーブルの手書きはしていません。

### 幾何要素の表現

| 要素 | グレード | 式 |
|---|---|---|
| 平面 ax+by+cz+d=0 | 1（ベクタ） | `d·e0 + a·e1 + b·e2 + c·e3` |
| 点 (x,y,z) | 3（トライベクタ） | `e123 − x·e023 + y·e013 − z·e012` |
| 地面 y=0 | 1 | `e2`（`ground_plane()`） |

PGA では「平面が最も基本の要素」で、点はその双対（トライベクタ）です。
点の e123 成分は同次座標の重みで、`extract_point` はこれで割って (x,y,z) を取り出します。

### 剛体変換 = モーター

回転と並進は、どちらも**偶数グレードの元（モーター）**として統一的に表現されます:

- 回転子（rotor）: 原点を通る軸 `n` 回りの角度 θ →
  `cos(θ/2) − sin(θ/2)·(n を双対にした e23/e13/e12 成分)`
- 並進子（translator）: 変位 t →
  `1 − ½(tx·e01 + ty·e02 + tz·e03)`（e0 が零基底なので指数展開が 1 次で切れる）
- 合成: 幾何積 `T * R` がそのまま SE(3) の合成（`motor_from_pose`, `compose_motors`）

任意の要素 X（点・平面・方向）への剛体変換は、要素の種類によらず同じ
**サンドイッチ積**ひとつです:

```text
X' = M X M~      (M~ は反転 reverse)
```

### このクレートでの使われ方

**姿勢・位置の状態はモーター 1 個** — `RocketState::motor` が体フレーム→世界の SE(3)
そのものです。クォータニオン+位置ベクトルのペアや 4×4 行列は登場しません。

- **積分**（[sim.rs](src/sim.rs) `step`）: 毎ステップ、速度から並進子
  `translator(v·dt)`、角速度から回転子 `rotor(ω̂, |ω|·dt)` を作り、
  `M' = T_inc * (M * R_inc)` と合成して正規化するだけで姿勢が更新されます。
- **接地判定**: 脚 4 点 + 船体 35 サンプル（計 39 プローブ）の体フレーム点をサンドイッチで世界へ移し、
  地面平面 y=0 との貫入で罰則法の法線力とクーロン摩擦を計算します。
  深い貫入は並進子 `translator(0, −min_y, 0)` を左から合成して押し戻します。
- **ジンバル**: ノズルの首振りは回転子の合成 `R_yaw * R_pitch`（`gimbal_rotor`）。
  物理は閉形式 `thrust_dir_body` を使い、両者の一致をテストで担保しています。
- **着陸誘導**（[landing.rs](src/landing.rs)）: 誘導に必要な幾何量はすべて
  逆向きサンドイッチ輸送（inverse transport）1〜2 回で得ています。
  - `world_up_in_body(M)` — 世界の +Y を体フレームへ。第 2 成分がそのまま cos(傾き)、
    (z, −x) 成分がそのまま起立誤差の外積になる、というのがミソです
  - `motor_inverse_rotate_vector(M, v)` — 世界速度を体フレームへ（垂直支持の減衰項に使用）
  - `attitude_error_body(M, d)` — 目標推力軸 d への最短弧誤差

### 自動着陸アルゴリズムの概要

`LandingAutopilot::update` は姿勢チャンネルと垂直チャンネルの独立な計算を
`max()` で合成します:

- **姿勢**: 誤差を軸+角度（atan2 ベース）で取り、sin(θ) 表現の縮退を回避。
  倒立（外積が消える対蹠点）は水平軸フォールバックで処理。レート指令は
  √プロファイル `ω = min(kp·θ, √(2αθ), ω_max)` で大角度は素早く、直立付近は
  オーバーシュートなく収束します。
- **垂直**: コースト → スーサイドバーン包絡線でのハードブレーキ → √h ソフト接地。
  包絡線判定は姿勢回復中も常時有効で、傾き 1.2 rad 超では足ではなく船体最下点を
  基準にします（倒立時は機首が足より約 28 m 低いため）。
  連続ブレンド（ブレーキ投入肩・姿勢ゲイン・リーン aim 混合など）は
  [fuzzy.rs](src/fuzzy.rs) を経由します（詳細は次節）。
  go↔brake の **方向ベクトルは離散選択**（反対向きの自由ベクトルを平均すると
  水平 aim が打ち消されるため）。包絡線ハードフロア・ソフト領域ゲート・
  complete / 接地カットも離散のままです。
- **横速度**: 高度に余裕があれば反速度方向に最大 1 rad までリーンして
  垂直中立スロットルでドリフトを焼き切ります。リーン量は「残り高度で止まれる
  垂直推力成分」から常時逆算して制限。横速度 3.5 m/s 超では接地せずホバーで除去します
  （高速の横滑り接地はバウンド→転倒爆発につながるため）。

チューニング時に判明した壊れやすい不変条件は各定数のコメントに記載しています。

## T モードの自動ターゲット着陸

T キーで起動する [`TargetLandingAutopilot`](src/target_landing.rs) は、離れた場所にある
黄色 T マークまで飛行し、その描画パッド上へ軟着陸するまでを一貫して受け持ちます。
制御の作りは 4 層です。

| 層 | 役割 | 例 |
|---|---|---|
| **閉形式の物理予測** | 権威。指令の大きさを決める | 停止距離 `d_stop`、許容接近速度 `v_allow`、`HandoffSettlePlan`、自殺バーン `a_req` |
| **短ホライズン MPC** | レジーム（行動）の選択 | `CruiseGo` / `Brake` / `Coast` / `SinkGo` / `AirplaneHold` / `LoftGo` |
| **ファジー仲裁** | レジーム境界の肩付き接続 | [`CruiseThrottleFuzzy`](src/fuzzy.rs)、`cruise_brake_hardness`、`long_range_hold_cos` |
| **離散ラッチ / ゲート** | 安全と非チャタ | ブレーキラッチ、ターミナル包絡ラッチ、ハンドオフ AND ゲート |

さらにその外側に **アクチュエータ層**があり、GNC のセットポイントをそのまま機体へ渡さず、
スロットルは [`slew_throttle`](src/fuzzy.rs) で非対称スプール（上げ 1.1 /s、
大きな段差では 4.0 /s、下げ 2.5 /s）、ジンバルは `GIMBAL_SLEW_RATE = 5.0`（全偏向/秒）、
推力 aim は `AIM_SLEW_SOFT` 1.0 〜 `AIM_SLEW_HARD` 3.0 rad/s でレート制限してから
姿勢 PD に入れます。飽和した rate-PD がノズルをバンバン叩くのを防ぐためです。

### 目標マークの配置

`mesh.rs` の `random_target_xz` が **R リセットのたびに** 目標 XZ を抽選します。
距離は `TARGET_DISTANCE_MIN_M` = 100 m 〜 `TARGET_DISTANCE_MAX_M` = 8000 m、
方位は一様、半径は **面積一様**（`r = √(u·(r_max² − r_min²) + r_min²)`）です。
描画パッドは半辺 30 m（`LAUNCH_PAD_HALF_EXTENT`）の正方形で、専用メッシュではなく
`ground.frag` が地面テクスチャ上に T マークとして描きます。

### フェーズと HUD ラベル

`status_label()` は左パネル `Target (T)` 行と上部 HUD に出る **14 文字以内**の
コンパクトラベルを返します。

| フェーズ | ラベル | 内容 |
|---|---|---|
| Climb | `climb+go` | ロフトゲート未達。フルスロットル上昇＋開ループ pitch program |
| Cruise | `cruise/loft` | ロフト前の MPC 上昇（低高度セーフティネット） |
| Cruise | `cruise/air` | 遠距離 airplane 巡航（フル T ＋ pitch エレベータ） |
| Cruise | `cruise/go` | 中距離の加速接近 |
| Cruise | `cruise/brake` | 逆リーンブレーキ（MPC 選択 or `brake_latched`） |
| Cruise | `cruise/coast` | `v_allow` 超過・弾道時の惰性 |
| Cruise | `cruise/sink` | 巡航上限高度を超えたときの降下つき接近 |
| Cruise | `cruise/s-brake` / `cruise/s-align` | fine settle のサブフェーズ（Brake / Align） |
| Descend | `descend` | パッド上の閉ループ自殺バーン |
| — | `off` / `complete` | 無効 / 着陸完了 |

### 1 フレームの処理順（`update`）

1. **ターミナル包絡ラッチ**を更新（[`careful_terminal_latch`](src/fuzzy.rs)）
2. **fine settle 判定**: `pad_settle_active = terminal_latched && cheby ≤ 80 m`（`RANGE_FAR_M`）
3. **ロフトゲート**[`transit_lofted`](src/target_landing.rs) で Climb / Cruise を決定
4. **ハンドオフ AND ゲート**を評価。`HANDOFF_SETTLE_MIN_S` = 0.25 s 連続成立で
   Descend へ遷移し、`lander.arm_from_transit(state)` が現在スロットルを引き継ぐ
   （ハンドオフ直後の推力ゼロ落下を避けるため）
5. 高度 ≥ `h_freefall_m`（地球 6000 m / 月 10000 m）なら**高高度ダイブ**へ分岐
6. フェーズ別 GNC を計算し、アクチュエータ層を通して `ControlCommand` を出力

Cheby はすべて **Chebyshev 距離**（`max(|dx|, |dz|)`）、`range` は水平ユークリッド距離、
`range_eff = (range − 40 m)`（`CAREFUL_NEAR_M`）です。

### 中核の物理: 停止距離と許容接近速度

T モードの「いつ減速を始めるか」は、ヒューリスティックではなく**閉形式の予測停止距離**が
決めます。

```text
d_stop = d_flip + d_burn
d_flip = v·t_flip + ½·a_coast·t_flip²      a_coast = FLIP_COAST_ACCEL_FRAC(0.5) × go 側 a_lat
t_flip = max( √(2θ/α_plan), θ/ω_max )      α_plan = 0.70 rad/s²、ω_max = 1.35 rad/s
d_burn = (v² − v_end²) / (2a)                          （β = 0）
       = (1/2β)·ln( (a + βv²) / (a + βv_end²) )        （β = k/m > 0）
v_end  = VH_HANDOFF_MAX = 4.0 m/s      Moon（無風・無抗力）は d_stop に ×1.15
```

横加速度 `a_lat` は推力レジームで切り替えます。

```text
VerticalNeutral: a_lat = max( g·tan θ, 0.15 )
FullThrottle   : a_lat = max( THR_FULL·(T/m)·sin θ, 0.15 )   THR_FULL = 0.97
```

`FullThrottle` を選ぶのは **airplane 域 / vh > 20 m/s（`VH_BRAKE_FULL_THR`）/ Moon** のいずれか
（`brake_lateral_mode`）。実際にフル T を焚くのはさらにファジー hardness > 0.55 を要求します
（減速し終わった低速ラッチが full-T を打ち続けないように）。

同じ式を二分法（≤16 反復）で逆に解いたものが `allowed_approach_speed` で、
**「残距離で止まりきれる接近速度 `v_allow`」** を返します。go 側はこれを速度上限として使い、
`v_approach > v_allow` の間は加速をやめて `Coast` に落ちます。

### 0. Climb（ロフトゲート未達）

MPC も速度フィードバック lean も使わない、単純物理の上昇です。

- スロットルは常に `THR_FULL = 0.97`
- 接地中・`CLIMB_CLEAR_ALT_M` = 25 m 未満・残距離 1 m 未満は直立 `[0,1,0]`
- クリア後は**開ループ pitch program**:
  `u = smoothstep01(ramp(alt, 25, 480))`、`lean_cap = 0.30 + u·(0.90 − 0.30)`、
  `lean = u·lean_cap` をパッド方向へ `clamp_tilt`。距離による lean 床は持ちません
  （dive 用の `LEAN_LONG_MAX` は Cruise 専用）
- 姿勢 PD は全区間ソフト（lean が開き切る前にジンバルを蹴らないため）
- **Cruise 移行**（`transit_lofted`）: `alt ≥ 480 m`（`GATE_ALT_MIN`）、
  near-handoff 時は `alt ≥ 260 m`（`HANDOFF_ALT_MIN_M`）、または
  **弾道アポジ** `alt + vy²/2g ≥ 500 m`（`CLIMB_ALT_M`）。アポジ判定があるので
  過剰ロフトしません
- パッド上空（cheby ≤ 30 m、ラッチ中は ≤ 45 m）では Climb へ戻らない
  （settle 中にフル T 再ロフトしないため）

### 1. Cruise: Transit MPC

`transit_mpc_select` が簡易 3DOF ロールアウトで候補を評価します。

| 項目 | 値 |
|---|---|
| 刻み / ホライズン | `MPC_DT` = 0.10 s / 8・10・12 s（range < 80 m、< 1500 m、それ以上） |
| 再計画 | `MPC_REPLAN_EVERY` = 2 フレームごと（receding horizon） |
| 状態 | 位置・速度・**lean の 1 次遅れ**（時定数は `brake_flip_time` 相当、下限 0.35 s） |
| 力 | `(T/m)·thr·û`、二次抗力 `−(k/m)·‖v‖·v`（Moon は k = 0）、重力、地面クランプ |

候補集合は状況で絞り込みます。

| 状況 | 候補 |
|---|---|
| ブレーキ確定 | `Brake` のみ |
| `v_approach > v_allow + 0.25` | `Coast`, `Brake` |
| airplane 域で go 中 | `AirplaneHold`, `Brake` |
| ロフト前 | `LoftGo`, `CruiseGo`, `Brake`, `Coast`, `SinkGo` |
| ロフト後 | `CruiseGo`, `Brake`, `Coast`, `SinkGo`, `AirplaneHold` |

コストは次の重み付き和です（`mpc_rollout_cost`）。

```text
cost = 55.0  · (480 m ゲート未達)²        W_MPC_GATE
     + 0.45  · (巡航上限超過)²            W_MPC_OVERLOFT   上限 520 m / airplane は 540 m
     + 0.07  · 終端残距離                 W_MPC_RANGE
     + 0.015 · ホライズン長               W_MPC_TIME
     + 16.0  · (オーバーシュート)²        W_MPC_OVERSHOOT
     + 18.0  · boost · ハンドオフ可行性   W_MPC_HANDOFF
     + 0.12  · ∫throttle dt               W_MPC_IMPULSE
```

ハンドオフ項の `boost` は残距離 200 m（`MPC_HANDOFF_BOOST_RANGE_M`）から
**×1 → ×2.5** へ連続的に立ち上がり、パッドに近いほど「Descend にアームできる状態か」を
重視します。残距離 ≲140 m では発散 `v_cheby`・ドリフト予算・予測ミスも可行性に加算されます。
`Coast` には非弾道時 +25、残距離 > 80 m で +35 のペナルティ。保持中の候補には
`MPC_COST_HYSTERESIS` = 2.5（`Brake` ラッチ時は満額、それ以外の保持は半額）のコスト優遇が入り、
候補のちらつきを抑えます。

### 2. 遠距離 airplane 巡航（水平 ≳ 1.5 km）

`LONG_AIRPLANE_RANGE_M` = 1500 m を超えると、飛行機のように **推力は前進、高度は pitch** で
取る巡航に入ります。

- 停止距離の外側ではフルスロットルでターゲット方向へ。ただし `v_approach > v_allow` の間は
  `Coast` が優先され、横速度は常に停止距離の包絡内に拘束されます
- 巡航高度は全距離 **`LONG_CRUISE_ALT_M` ≈ 520 m**（短距離の `CRUISE_ALT_CAP` と同じ帯）
- [`long_range_hold_cos(alt, alt_tgt, vy, hover)`](src/fuzzy.rs) が高度誤差・鉛直速度・
  **弾道予測アポジ**のメンバシップから `v_des → a_cmd → cos`（=aim の鉛直成分）を作ります。
  フル T では `a_y = g·(cos/hover − 1)` なので平衡は `cos ≈ hover`（T/W = 3 なら ≈ 1/3）。
  **非対称**で、上昇は控えめ、過高度・通過上昇は `cos` 下限 0.12 まで許す強い機首下げ dive
- `long_range_go_aim(ux, uz, cos_up)` が水平（パッド方向）と鉛直を合成した単位 aim を返す
- 深リーン中は flip 復帰ゲートを `COS_TILT_AIM_AIR` = 0.10 に下げ、正当な dive を
  「倒立」と誤認して直立復帰しないようにしています
- `range_eff ≤ d_stop` に入った瞬間、airplane も**同じ物理ゲート**で逆リーンへ譲ります
  （`is_long_range_cruise` はブレーキ中 false）

### 3. 中距離 go / brake

- **投入**: `range_eff ≤ d_stop + BRAKE_ENGAGE_MARGIN_M`（25 m 早め）。
  **保持**: `range_eff ≤ d_stop + BRAKE_RELEASE_MARGIN_M`（10 m）という幾何ヒステリシスで
  go↔brake のチャタを抑止。オーバーシュート（`v_approach < −1.5`）は即ブレーキ
- **計画 lean** は `LEAN_BRAKE_MAX` = 1.45 rad を [`apply_cruise_alt_lean_cap`](src/target_landing.rs) が
  `long_range_hold_cos` の高度保持 `cos` 下限で cap した値（計画と実行が同じ天井を見る）
- **実行**: 高速時は逆リーン＋フル T。減速後は
  [`cruise_brake_hardness`](src/fuzzy.rs)（vh 6→22 m/s の肩＋オーバーシュート項）が
  lean・full-T・aim・rate-kill を連続減衰させ、低速では直立寄り＋ソフト PD へ移ります
- **aim** は高速で反速度ブレーキ、低速で直立とファジーブレンド。ただし
  **go と brake の「選択」自体は離散**のまま（反対向きの自由ベクトルを平均すると
  水平 aim が打ち消されるため）
- 上昇中の横速度床 `V_CLIMB_H_MAX` ≈ 28 m/s は **未ロフト時のみ**。ロフト後の
  airplane 巡航に掛けると長距離が Coast に張り付きます
- 垂直は `cruise_v_des_y` が高度保持: `CRUISE_ALT_CAP` 超過分を時定数 ~12 s で
  1〜8 m/s の沈下として抜き、ロフト後は決して上昇を指令しません

### 4. ターミナル settle（Cruise → Descend の手前）

パッド周辺で「位置・速度・姿勢を、降下しながら」整える区間です。

**ラッチ**（`careful_terminal_latch`）

| 遷移 | 条件 |
|---|---|
| 進入 | lofted かつ（`range_eff ≤ d_stop + 25 m` **または** `range ≤ 300 m`（`CAREFUL_TERMINAL_ENTER_M`）） |
| 退出 | `range > 400 m`（`CAREFUL_TERMINAL_EXIT_M`）かつ `cheby > 45 m`（`TERMINAL_EXIT_CHEBY_M`） |

**外側ラッチと fine settle の分離**が要点です。ラッチしていても Chebyshev が
**80 m（`RANGE_FAR_M`）を超える間は中距離の go / `d_stop` ブレーキを維持**し
（HUD も `cruise/go|brake`）、高度も落としません。Chebyshev ≤ 80 m で初めて
`pad_settle_active` が立ち、Brake | Align 一本化＋`HANDOFF_ALT_M` = 300 m への沈下
（`cruise_v_des_y(terminal = true)`、−0.08·(alt − 300) を 0.8〜8 m/s にクランプ）が始まります。
早期ラッチだけでは Climb を切らない設計で、フルスロットル上昇はゲート／弾道アポジまで続きます。

**サブフェーズ**は Brake と Align の 2 相のみ。静かな進入は **Align から開始**します。

```text
needs_brake ⇔ vh > vh_hot
            ∨ a_stop_req = vh²/(2·max(cheby − 10, 1)) > g·tan(LEAN_BRAKE_MAX)
            ∨ v_cheby < −1.2                       （明確な発散）
vh_hot = min(2·v_creep + 0.8, VH_HANDOFF_MAX·1.35)
```

- **Brake**: 需要 `demand` で `a_cmd` をシェーピングし、
  [`lean_for_lateral_accel`](src/target_landing.rs) で lean を逆算。天井は物理的な
  `LEAN_BRAKE_MAX` のみ（`careful_brake_lean_cap` の浅い屋根は撤廃済み）。
  位置 PD と反速度 aim を `terminal_brake_blend` の重みで混ぜます
- **Align**: 目標クリープ速度 `v_creep = trim_creep_speed(cheby, aggression)` へ
  横速度を合わせる連続トリム。Chebyshev ≤ 6 m（`ALIGN_DEADZONE_CHEBY_M`）かつ
  `vh ≤ 3.4 m/s` かつ非発散なら直立ホールド（追いかけない）
- **クリープ上限** `cheby_creep_cap` は近距離ほど遅く: パッド近傍は
  `0.45 + 0.12·cheby`（上限 2.60 m/s）、外側は `4.50 + ramp(cheby, 10, 50)·2.00`、
  12–22 m で連続ブレンド。**ハンドオフの vh 制限を下回るように設計**されているので、
  クリープしたまま Descend にアームできます
- `careful_aggression(range)` は中距離のみ（近い 0.70 → 遠い 1.0）。fine settle 内では
  常に 1.0 固定です

**残り時間の物理予測**（[`HandoffSettlePlan`](src/target_landing.rs)）が settle のゲインを決めます。

```text
t_att   : 現 tilt → hand-off tilt（√-profile 反転時間 + レート減速）
t_vh    : 残 vh → 包絡の vh_max（a_lat ≈ g·tan θ、抗力込みの ∫dv/(a+βv²)）
t_pos   : Chebyshev 残差 → 包絡の cheby_max（接近率 v_cheby、発散時は減速＋反転時間を加算）
t_settle = max(t_att, t_vh, t_pos)        cleared() ⇔ t_settle ≤ 1e-3
```

`t_att` が支配的なときは aim を直立寄りにし、**スロットルを hover/cos 付近まで上げて
ジンバルトルクを優先**します（深リーン中に 0.35–0.55 で頭打ちにする旧仕様は撤廃）。
`settle_lean_freedom` / `settle_freedom_effective` / `settle_brake_lean_scale` は
**常に 1.0** で、直立優先は `settle_attitude_constraint`（`t_att` / tilt / レートの最大値）
だけが担います。

### 5. Descend へのハンドオフ AND ゲート

**高度自体は進入条件ではありません。** 整い次第すみやかに渡します。ただし全条件が
`HANDOFF_SETTLE_MIN_S` = 0.25 s 連続で成立する必要があります。

```text
phase == Cruise
∧ cheby ≤ env.cheby_max
∧ vh    ≤ env.vh_max
∧ v_cheby > −0.25                                   （速い発散でない）
∧ (  近傍枝: cheby ≤ 0.60·env.cheby_max ∧ vh ≤ env.drift_near_m / t_drift
   ∨ 接近枝: v_cheby > 0.12 ∧ vh ≤ env.drift_closing_m / t_drift
             ∧ |cheby − v_cheby·t_drift| ≤ env.miss_max_m )
∧ ω_pitch_yaw ≤ env.omega_max
∧ world_up_in_body[1] ≥ env.cos_tilt_min
t_drift = clamp( √(2·alt/g), 8, 16 ) s              （ハンドオフ後の惰性時間）
```

包絡 `env` は **CoM 高度 150 m（厳格）→ 600 m（緩和）で線形補間**されます
（`handoff_envelope`）。高いところで渡すほど、着陸機側に修正の余地が残るためです。

| 閾値 | 150 m 以下 | 600 m 以上 |
|---|---|---|
| Chebyshev | 10 m | 20 m |
| 横速度 `vh` | 4.0 m/s | 7.0 m/s |
| ピッチ/ヨー角速度 | 0.12 rad/s | 0.20 rad/s |
| `up_y`（cos 傾き） | ≥ 0.95 | ≥ 0.90 |
| ドリフト予算（近傍枝 / 接近枝） | 9 m / 12 m | 16 m / 20 m |
| 予測ミス上限 | 6 m | 12 m |

### 6. Descend（パッド上・ハンドオフ後）

`LandingAutopilot::update_target_descend`（[landing.rs](src/landing.rs)）に委譲します。

- **垂直**: 閉ループ自殺バーン。
  `a_req = 1.15·(v_down² − v_touch_eff²)/(2h)`、`t = m(a_req + g)/(T_max·up_y)`。
  コースト／ブレーキ／接地カットは [`PhysicsPadThrottleFuzzy`](src/fuzzy.rs) が
  肩付きでブレンドし、包絡遅刻時の hard floor だけ離散のまま
- **飛行中の最低推力**: 再点火をモデル化しないため、接地前は
  `DESCEND_MIN_THROTTLE` = 0.03 を下回りません。接地 settle / complete のゼロカットは従来どおり
- **姿勢**: 足高度 45 m 超かつ Chebyshev > 8 m（`TARGET_CENTER_TOL_M`）ならパッド seek lean、
  それ以下では位置微調整をやめて**直立＋ソフト接地にコミット**。lean は
  `brake_safe_lean`（`LEAN_TERMINAL_VH` = 0.18）で自殺バーンの必要減速度から制限
- ハンドオフ直後は現在スロットルを引き継ぎ、`v_down < 0.6` かつ十分な高度差があれば
  一度コーストして降下を始めます（ホバーのまま歩み寄らない）

### 7. 成功判定

| 判定 | 領域 |
|---|---|
| **complete（成功）** | 描画パッド上（Chebyshev ≤ **30 m** = `TARGET_PAD_HALF_M`）に接地し、傾き < 0.12–0.18 rad・横速度 < 1.5 m/s・鉛直速度 < 0.8–1.0 m/s・ω < 0.22 rad/s |
| 誘導目標 | 内側 Chebyshev 箱 半辺 **12 m**（`TARGET_SUCCESS_HALF_M`） |
| Descend の seek 打ち切り | Chebyshev ≤ **8 m**（`TARGET_CENTER_TOL_M`） |

内側 12 m 箱への収束は complete の条件ではありません（誘導が狙う場所と、成功と認める場所を
分けています）。

### 8. 高高度ダイブ

CoM 高度が `h_freefall_m`（地球 6000 m / 月 10000 m）以上のとき、フェーズ計算を迂回して
`high_alt_freefall_to_pad` が走ります。

- 速度包絡の下では**機首下げのフル T ダイブ**。`range > alt + HIGH_ALT_OVERHEAD_BIAS_M`（1000 m）
  なら目標方向へ斜めに、そうでなければ純鉛直 `[0, −1, 0]`
- 予測停止距離の内側に入ると横方向の傾きを純鉛直ダイブへフェード
- `freefall_v_cap` を超える過速では直立へブレンドしてブレーキ（**安全降下速度が最優先**）
- flip 復帰ゲートは `COS_TILT_AIM_FF` = −1.01。ダイブ中に「倒立」と誤認して
  姿勢を奪い合わないようにしています

### 9. Moon mode（M キー / 左パネルのチェックボックス）

重力は 9.81 m/s² のままで、変わるのは**空気と見た目、そしてそれに追随する GNC** です。

| 項目 | Earth | Moon |
|---|---|---|
| 空気抵抗 | `k(h) = k_sl·exp(−h/8500)` の二次抗力 | **0**（真空） |
| 停止距離 | 抗力込みの閉形式 | ×1.15 の悲観係数 |
| ブレーキ推力レジーム | 中距離は垂直中立が多い | 常に full-T |
| 自由落下しきい値 / 速度上限 | 6000 m / 80–240 m/s | 10000 m / 60–120 m/s |
| ビジュアル | 空 `[0.45, 0.62, 0.85]`・草地テクスチャ | 黒空・レゴリス＋クレータテクスチャ |

### 10. 定数リファレンス（抜粋）

| 定数 | 値 | 意味 |
|---|---|---|
| `CLIMB_ALT_M` / `GATE_ALT_MIN` | 500 / 480 m | 公称ロフト高度 / ソフト床 |
| `CRUISE_ALT_CAP` / `LONG_CRUISE_ALT_M` | 520 / 520 m | 巡航上限 / airplane 保持高度 |
| `HANDOFF_ALT_M` / `HANDOFF_ALT_MIN_M` | 300 / 260 m | settle 沈下目標 / near-handoff 床 |
| `CLIMB_CLEAR_ALT_M` | 25 m | lean を開き始める高度 |
| `LONG_AIRPLANE_RANGE_M` | 1500 m | airplane 巡航に入る水平距離 |
| `CAREFUL_TERMINAL_ENTER_M` / `_EXIT_M` | 300 / 400 m | ターミナル包絡ラッチ |
| `RANGE_FAR_M` | 80 m | fine settle（Brake\|Align）の箱 |
| `CAREFUL_NEAR_M` | 40 m | `range_eff` のオフセット |
| `BRAKE_ENGAGE_MARGIN_M` / `_RELEASE_` | 25 / 10 m | ブレーキ投入 / 保持ヒステリシス |
| `LEAN_LONG_MAX` = `LEAN_BRAKE_MAX` | 1.45 rad | airplane / 逆ブレーキの lean 天井 |
| `LEAN_BURN_MAX` | 0.30 rad | Climb pitch program の初期天井（最終 0.90） |
| `THR_FULL` | 0.97 | フルスロットル指令 |
| `ALPHA_PLAN` / `OMEGA_MAX` | 0.70 rad/s² / 1.35 rad/s | 姿勢 √-profile の計画値 |
| `VH_BRAKE_SOFT` / `_HARD` / `_FULL_THR` | 6 / 22 / 20 m/s | ブレーキ hardness の肩 / full-T しきい |
| `TARGET_PAD_HALF_M` / `TARGET_SUCCESS_HALF_M` | 30 / 12 m | complete 領域 / 誘導目標箱 |
| `DESCEND_MIN_THROTTLE` | 0.03 | 飛行中の最低スロットル |

数値の権威はソース先頭のコメントとユニットテスト（`fuzzy::tests`、`target_landing` の
MPC / long-range / terminal settle 系）です。

## ファジー制御

実装は [fuzzy.rs](src/fuzzy.rs) です。ここでの「ファジー」は **閉形式の物理スケジューラを
置き換えるものではなく**、局所法則どうしの **連続仲裁（Takagi–Sugeno 型ブレンド）** です。

| 層 | 役割 | 例 |
|---|---|---|
| **閉形式（ハード）** | 幾何・安全・幾何学的拘束 | スーサイドバーン包絡、√h ソフト接地、√-profile 姿勢、パッド内 complete |
| **ファジー（ソフト）** | レジーム境界の滑らかな接続 | ブレーキ投入肩、姿勢ゲイン、lean aim 混合、T の高度エレベータ |

**やらないこと:** 安全ラッチ（包絡線遅刻時のブレーキ床、ソフト領域ゲート、complete 条件、
接地カット）をメンバシップで薄めること。境界を跨いで「半分ブレーキ・半分ホバー」に
すると横滑りやロフトに回帰したため、**安全ゲートは離散のまま**、投入量やゲインだけ
連続にしています。

### プリミティブ

| 関数 | 意味 |
|---|---|
| `ramp` / `ramp_down` | 上昇・下降ランプ（0↔1） |
| `tri` / `trap` | 三角・台形メンバシップ |
| `and` / `or` | 代数積 AND、確率和 OR |
| `defuzz_weighted` | 重み付き平均（TS 非ファジィ化） |
| `weighted_max` | 正コマンドのソフト最大（ブレーキとコーストを平均しない） |

### L モード（その場着陸）

1. **垂直スロットル** — `LandingThrottleFuzzy`
   - 入力: 高度 `h`、包絡 `h_env` / `h_need`、下降速度、`up_y`、各局所指令
     （`t_soft`, `t_support`, `t_brake_cmd`, `t_auth`, `t_drift`）。
   - **ソフト領域ゲートはハード**（直立かつ低高度／包絡に余裕など）。ゲートを
     ファジーで跨ぐとリーンホバーが暴走したため。
   - ファジーは **bang ブレーキの投入肩**のみ:
     `μ_can_brake · μ_falling · μ_on_curve` で `t_brake` を連続化。
   - 遅刻（`h_env ≤ h_need+…` かつ下降中）は **hard floor** で `t_brake_cmd` を強制。

2. **姿勢ゲイン** — `attitude_gain_scales(contacting, on_pad, h)`
   - フリーフィールド / パッド上空 / 接地 settle を TS ブレンド。
   - 接地エッジや 20 m ノッチでゲインが段飛びしないようにする。

3. **リーン錐と目標推力軸** — `LeanAimFuzzy` / `lean_max_nominal` / `blend_desired_axis`
   - 候補軸: 直立、反速度（ドリフト焼き）、ソフト横 trim、高高度時のパッド pos-seek。
   - メンバシップ重みで軸を合成（正規化前）。その後 `clamp_tilt` と
     **ブレーキ安全 lean 上限**（残り高度で垂直成分が足りるか）でハード制限。
   - `flip_aim_weight`: 傾きが `TILT_AIM` 付近で lean aim ↔ 純直立を肩付きで切替。

### T モード（目標着陸）

制御の全体像は「[T モードの自動ターゲット着陸](#t-モードの自動ターゲット着陸)」にまとめてあります。
ここでは **ファジー層がどこで何を連続化しているか**だけを整理します。

| 関数 / 構造体 | 使われる場所 | 何を肩付きにするか |
|---|---|---|
| [`CruiseThrottleFuzzy`](src/fuzzy.rs) | Cruise の垂直スロットル | `t_hold` / `t_full` / `t_auth` / `t_deep` / `t_settle` / `t_contact` の局所則を、full-T ↔ authority、deep、settle、ballistic 床の境界で連続合成 |
| `cruise_brake_hardness` | 中距離ブレーキ実行 | vh 6→22 m/s の肩とオーバーシュート項から「ブレーキの硬さ」を 0–1 で出し、lean・full-T・aim・rate-kill をまとめて減衰 |
| `long_range_hold_cos` / `long_range_go_aim` | airplane 巡航の高度保持 | 高度誤差・鉛直速度・弾道アポジ → aim の鉛直成分 `cos`（上昇は控えめ、過高度は dive） |
| `long_range_weight` | 巡航レジーム | 約 3–7 km の肩で長距離寄りの重み |
| `careful_aggression` | 中距離 settle | 距離に応じた動きの控えめさ（近い 0.70 → 遠い 1.0） |
| `careful_terminal_latch` | ターミナル包絡 | **離散ラッチ**（ここはファジー化しない） |
| `settle_urgency` / `settle_aim_blend` / `settle_motion_scale` / `settle_trim_rate_gate` | fine settle | 残り時間に応じた直立寄せ・トリム量・レートゲート |
| `freefall_v_cap` / `FreefallThrottleFuzzy` | 高高度ダイブ | 降下速度包絡と直立ブレーキの遷移 |
| [`slew_throttle`](src/fuzzy.rs) | 全フェーズのアクチュエータ | スロットルの非対称スプール（L/T 共通） |

**離散のまま残しているもの**: go↔brake の選択とヒステリシス、ターミナル包絡ラッチ、
fine settle の Brake / Align 判定、Descend ハンドオフの AND ゲート、complete 条件。
ここをメンバシップで薄めると「半分ブレーキ・半分ホバー」になり、横滑りやロフトに戻ります。

`settle_lean_freedom` / `settle_freedom_effective` / `settle_brake_lean_scale` は
現在**常に 1.0** を返すスタブで、直立優先は `settle_attitude_constraint` だけが決めます
（`SETTLE_LEAN_V_STRICT` などの速度ベース定数は現状未使用）。
`cruise_brake_weight` / `careful_envelope_membership` も現在は誘導本体から呼ばれておらず、
ユニットテストだけが参照しています。

### 設計上の教訓（要約）

**ルールでの制限を増やすほど挙動は不自然になり、物理計算を精密にするほど制御はうまくいく。**
これが本クレートの GNC 設計の基本方針です。

lean 上限・速度キャップ・aim の人工クランプ・「安全のため」の後付けヒューリスティックを
積み重ねると、意図した安全より先に副作用（通り過ぎ、姿勢の頭打ち、go↔brake チャタ、
巡航→減速の不連続）が出やすくなります。定数やクランプを足すだけでは、aim 幾何と
実行推力が噛み合わず、上限を上げても実際の姿勢が変わらない、といった齟齬も起きます。

一方、停止距離 `d_stop = d_flip + d_burn`、横加速度 `a_lat ≈ g·tan(θ)` または
full-T 時 `(T/m)·thr·sin(θ)`、√-profile 反転時間、二次抗力込みの減速時間など、
**植物モデルに沿った閉形式を権威**にすると、同じ定数でもシミュレーション上のロケットは
減速・整列・ハンドオフが一貫します。ファジー層はこの物理スケジューラを置き換えるのではなく、
レジーム境界の肩付き接続だけに使います（次表）。

| やってよい | やってはいけない |
|---|---|
| 物理から逆算した lean / aim / 停止距離 | 物理と無関係な lean・速度の頭打ちルール |
| 投入量・ゲイン・aim 混合の連続化 | 安全ゲート自体をソフトブレンド |
| go / brake の **選択**を離散＋ヒステリシス | go と brake の自由ベクトルを平均 |
| フル T 巡航で高度を pitch で取る | 高度不足を thr 床で補ってロフト |
| 過高度で機首下げ dive | dive を「倒立」とみなして upright 復帰 |
| 姿勢 settle 中は thr 床でトルク確保 | 深リーン回転中に thr を 0.55 で頭打ち |

定数の数値はソース先頭のコメントとユニットテスト（`fuzzy::tests`、
`target_landing` の long-range 系）が仕様の一部です。

## example: landing_stress

[examples/landing_stress.rs](examples/landing_stress.rs) は自動着陸の回帰確認・チューニング用
ハーネスです。**landing.rs に手を入れたら必ずこれを回してください。**

### 1. 一括ストレステスト

```
cargo run --release -p pga-rocket --example landing_stress
```

無理な初期姿勢 17 シナリオ（傾き 35°〜倒立、-40 m/s の高速落下、回転付きタンブリング、
横速度 15 m/s など）で自動着陸を最後まで走らせ、結果を 1 行ずつ出力します:

```
tilt90_60m       landed    t=  14.8s impulse=  7.24 tilt= 0.06 vy=  0.17 h=   0.05 impact= 0.00
inverted_120m    DESTROYED t=   2.9s impulse=  1.94 tilt= 0.36 vy=  0.00 h=   0.00 impact=39.46   <-- FAIL
```

| 列 | 意味 |
|---|---|
| `landed / DESTROYED / timeout` | 着陸完了ラッチ / 爆発 / 120 秒超過 |
| `t` | 経過時間（秒）— 「素早さ」の指標 |
| `impulse` | ∫throttle dt（throttle·秒）— 燃料消費の指標 |
| `tilt` | 終了時の傾き（rad） |
| `vy` / `h` | 終了時の鉛直速度（m/s）/ 最下脚高度（m） |
| `impact` | 破壊時の衝突速度（m/s、無事なら 0） |

**`inverted_120m` は落ちるのが正常です。** 倒立からのフリップには下向き Δv 約 50 m/s と
高度約 60 m を消費し、その後のブレーキに約 69 m 必要なため、T/W = 3・ジンバル ±7° では
CoM 120 m からの回復軌道が理論上存在しません（実現可能下限は ~150 m。テストは 170 m で検証）。
これ以外のシナリオが 1 つでも落ちたら回帰です。

### 2. 単一シナリオのトレース

```
cargo run --release -p pga-rocket --example landing_stress tilt90_60m
```

シナリオ名を引数に渡すと、そのシナリオだけ 0.25 秒刻みで状態をダンプします:

```
t=  1.26 h=   51.53 vy= -25.16 vx=  0.00 vz= 22.80 tilt= 1.31 w=(-0.99, 0.00, 0.00) thr=1.00 p= 0.36 y= 0.00 contact=0
```

高度・速度・傾き・角速度・スロットル・ジンバル指令・接地フラグが並ぶので、
「どのフェーズで何が起きたか」（コーストが長すぎる、ブレーキが遅い、ロフトしている等）を
時系列で追えます。挙動がおかしいときはまず一括実行で落ちたシナリオを特定し、
次にトレースで原因フェーズを絞り込む、という使い方を想定しています。

### 3. マイクロベンチマーク

```
cargo run --release -p pga-rocket --example landing_stress -- --bench
```

`LandingAutopilot::update` 1 回のコストを姿勢レジームごとに計測します:

```
upright       907 ns/update
lean 0.6     1723 ns/update
flip 2.5     8248 ns/update
```

フリップ中だけ高いのは全 39 接地プローブの最下点スキャン（サンドイッチ 78 回）が
走るためで、意図的にその場面に限定しています（傾き ~1.75 rad までは脚が常に最下点
なのでスキャン不要）。120 Hz 制御でも最悪 0.1 % 程度のフレーム予算です。

### シナリオの追加

`landing_stress.rs` 冒頭の `scenarios` ベクタに 1 行足すだけです:

```rust
Scenario { name: "my_case", alt: 70.0, pitch: 0.8, yaw: 0.0, roll: 0.3,
           vel: [5.0, -12.0, 0.0], omega: [0.2, 0.0, 0.0] },
```

`alt` は CoM 高度（脚はその約 16.4 m 下）、`pitch/yaw/roll` は初期姿勢（rad）、
`vel` は世界フレーム初速、`omega` は体フレーム角速度です。恒久的な保証にしたい
ケースは [tests/landing.rs](tests/landing.rs) の生存テスト群にも追加してください。

## example: target_stress

[examples/target_stress.rs](examples/target_stress.rs) は **T キー目標着陸**の回帰確認用
ハーネスです。**target_landing.rs に手を入れたらこちらも回してください。**

```
cargo run --release -p pga-rocket --example target_stress
```

500 m 〜 8 km の 8 シナリオで `TargetLandingAutopilot` を最後まで走らせ（1 本あたり最大 400 秒）、
飛行時間・最大高度・スロットル積分・Descend 中の姿勢揺れを 1 行ずつ出力します。

| シナリオ | 内容 |
|---|---|
| `pad_500x` / `pad_500diag` / `pad_800x` | 中距離（MPC ＋ `d_stop` ブレーキが主役） |
| `pad_overhead` | 目標の真上から発射（水平移動なしの settle → Descend） |
| `high_600_off400` | 高度 600 m からの開始（Climb をスキップして Cruise 入り） |
| `mid_250_500x` | 途中高度 250 m からの開始（ロフト再開の確認） |
| `pad_6000x` / `pad_8000x` | 長距離 airplane 巡航（~520 m 保持 → 逆リーンブレーキ） |

シナリオ名を引数に渡すと 0.25 秒刻みのトレースが得られます（`landing_stress` と同様）。

```
cargo run --release -p pga-rocket --example target_stress pad_6000x
```

## テスト

```
cargo test -p pga-rocket
```

- [tests/physics.rs](tests/physics.rs) — 剛体物理・接地・反発・ジンバル / RCS・破壊判定、
  Moon mode で抗力が消えること
- [tests/landing.rs](tests/landing.rs) — L モード自動着陸の統合テスト（横倒し・倒立・高速落下・
  タンブリングからの無傷着陸、コースト燃費、ソフト接地、高高度ダイブ）
- [tests/target_landing.rs](tests/target_landing.rs) — T モード目標着陸の統合テスト
  （500 m 級の Climb→Cruise→Descend、高高度スタート、Moon の長距離巡航）
- [tests/control.rs](tests/control.rs) / [tests/explosion.rs](tests/explosion.rs) — 入力写像・爆発演出
- [tests/texture.rs](tests/texture.rs) — 手続き地面テクスチャ（サイズ・色統計・ミップ生成）
- 各モジュール内ユニットテスト — PGA 恒等式（閉形式とサンドイッチの一致など)、
  包絡線・軸角度変換の境界値、**target_landing** の MPC / 長距離 cruise / ターミナル settle /
  ハンドオフ包絡
