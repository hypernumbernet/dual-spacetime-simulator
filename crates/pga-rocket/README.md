# pga-rocket

3D 射影幾何代数（PGA: Projective Geometric Algebra）を土台にした、脚付きロケットの
打ち上げ・着陸シミュレータです。剛体の姿勢・位置を PGA のモーター（motor）1 個で表現し、
物理・接地判定・自動着陸誘導のすべてを PGA のサンドイッチ積で計算します。
描画は Vulkan（vulkanvil）、UI は egui です。

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
| T | **T マーク目標地点への自動着陸**（上昇と水平移動を同時に行い、**約 1.5 km 以遠は飛行機型：フルスロットルでターゲットへ、高度は機首角/lean だけで ~800 m 保持**（機首下げ dive 可）、接近後はヒステリシス付きブレーキ包絡→約 500 m 超えてから降下） |
| R | リセット |
| ←→↑↓ | カメラ回転（マウスドラッグでも可） |
| PageUp / PageDown | カメラ距離（マウスホイールでも可） |
| Esc | 終了 |

接地時の法線方向衝突速度が `crash_impact_speed`（既定 10 m/s）を超えると機体は爆発します。
自動着陸（L）はどんな姿勢からでも起動でき、横倒し・高速落下・倒立からの回復に対応しています
（倒立は CoM 高度 ~150 m 以上が物理的な回復下限。詳細は後述）。
目標着陸（T）は黄色 T マーク地点へ航法します。誘導は PGA の点として「パッド上空の
ロフトウェイポイント」を取り、CoM からの自由ベクトル変位で **上昇と水平移動を同時に**
行います。遠距離はフルスロットル＋ pitch エレベータ高度保持、中距離はブレーキ包絡線
で減速し、姿勢・横速度が静かになってから最終降下にハンドオフします。降下の後半
（低高度）は位置微調整をせず直立＋ソフト接地にコミットします。成功条件はターゲット
正方形パッド内（半辺 30 m）への着陸です。CoM 高度がおおよそ 500 m 未満のままでは
最終降下に入らず、ゲートを超えてからソフト着陸に切り替わります（L とその場着陸と
相互排他）。ファジー層の役割は後述「ファジー制御」を参照。

## モジュール構成

| モジュール | 内容 |
|---|---|
| [euclidean_pga.rs](src/euclidean_pga.rs) | G(3,0,1) の 16 成分マルチベクタと幾何プリミティブ |
| [sim.rs](src/sim.rs) | 剛体物理（重力・ジンバル推力・RCS・接地・破壊判定） |
| [fuzzy.rs](src/fuzzy.rs) | メンバシップ・TS ブレンド・L/T 誘導の連続仲裁（安全ラッチは置換しない） |
| [landing.rs](src/landing.rs) | その場への自動着陸オートパイロット（L キー） |
| [target_landing.rs](src/target_landing.rs) | T マーク目標への航法付き自動着陸（T キー） |
| [control.rs](src/control.rs) | キー入力 → 制御コマンドの純粋写像 |
| [mesh.rs](src/mesh.rs) / [explosion.rs](src/explosion.rs) | 機体・草地・発射／目標パッド・爆発のジオメトリ生成 |
| [app.rs](src/app.rs) / [renderer.rs](src/renderer.rs) / [ui.rs](src/ui.rs) | ウィンドウ・Vulkan 描画・HUD（原点パッド＋500 m 〜 8000 m のランダム距離先の T 目標） |

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

0. **Transit MPC（Climb / Cruise）** — 簡易 3DOF 前方ロールアウト + 候補サンプリング
   - 状態: 位置・速度 + lean 1 次遅れ（`brake_flip_time` 相当）。推力は `(T/m)·thr·û`、
     二次抗力 `F=−k|v|v`、重力。Moon は `k=0`。
   - 候補: `LoftGo` / `AirplaneHold` / `CruiseGo` / `Brake` / `Coast` / `SinkGo`。
     遠距離 go 中は `AirplaneHold` と `Brake` のみ。2 フレームごとに再計画（receding horizon）。
   - コスト: 500 m ゲート未達・過剰ロフト・残距離・オーバーシュート・ハンドオフ余裕・∫throttle dt。
   - 内ループは従来どおり姿勢 PD + throttle 整形。ターミナル settle / Descend 硬 AND は不変。

1. **中距離 go / brake** — 物理予測停止距離 `d_stop = d_flip + d_burn`
   - `a_lat = g·tan(θ)`（垂直中立逆リーン）または **airplane 域** full-T 時 `(T/m)·thr·sin(θ)`。
   - 二次抗力 `β = k/m`（Moon は `β=0`）。`d_burn = (1/2β) ln((a+βv²)/(a+βv_end²))`。
   - 減速時間 `t_decel` も同型の閉形式（`a_eff` 近似は廃止）。
   - 姿勢反転 `d_flip = v·t_flip`（現姿勢→逆リーン aim の角度から √-profile で `t_flip`）。
   - **開始条件:** `range_eff ≤ d_stop`（ターミナル station-keep を除く）。幾何ヒステリシス
     `BRAKE_RELEASE_MARGIN` で go↔brake チャタを抑止。オーバーシュート（`v_approach < 0`）も即 brake。
   - **aim 方向は離散**（go 自由ベクトル or 反速度ブレーキ）。ベクトル平均はしない。
   - go 側の目標接近速度は同じ式の逆算（`allowed_approach_speed`）— ハード速度キャップなし。

2. **遠距離 airplane 巡航**（水平距離 ≳ 1.5 km）
   - **優先順位:** 予測停止距離の外側ではフルスロットルでターゲット方向へ行く。高度は
     **pitch / lean（エレベータ）** のみ。
   - `range_eff ≤ d_stop` に入った瞬間、airplane も **同じ物理ゲート**で逆リーンへ譲る
     （`is_long_range_cruise` は brake 中 false）。
   - `long_range_weight(range)`: 約 3–7 km の肩でロフト目標 520 m ↔ 800 m を連続ブレンド。
   - `long_range_hold_cos(alt, alt_tgt, vy, hover)`:
     - フル T では `a_y = g·(cos/hover − 1)`。平衡は `cos ≈ hover`（T/W≈3 なら ≈1/3）。
     - 高度誤差・鉛直速度・**弾道予測アポジ**のメンバシップから `v_des` → `a_cmd` → `cos`。
     - **非対称:** 上昇は控えめ、過高度・通過上昇は強い dive（`cos` 下限 ≈ 0.12、
       機首下げを許容）。フリップ復帰ゲートも dive を邪魔しないよう低くする。
   - `long_range_go_aim(ux, uz, cos_up)`: 水平はパッド方向、鉛直は上記 `cos` の単位 aim。
   - 距離フロアにより、高速で 3 km まで寄っても ballistic thr カット／直立ストレッチに
     落ちず、airplane 法を維持する。

2.5. **ターミナル settle（Cruise→Descend 手渡し前、~90–140 m 肩）**
   - **アーム条件は離散 AND のまま**（高度 ≥480 m、Chebyshev ≤10 m、`vh` ≤4.0 m/s、
     `ω_py` ≤0.12 rad/s、`up_y` ≥0.95）。安全ゲート自体はソフト化しない。
   - 進入は ~90 m、退出は ~140 m のヒステリシス。包絡内の motion/lean は
     [`careful_aggression(range)`](src/fuzzy.rs) で距離連続スケール（近い→0.40、遠い→1.0）。
   - その手前の settle 制御は [`HandoffSettlePlan`](src/target_landing.rs) で
     **クリアまでの残り時間**を物理予測:
     - `t_att`: 現 tilt → hand-off tilt（√-profile 反転時間 + レート減速）
     - `t_vh`: 残 `vh` → `VH_HANDOFF_MAX`（`a_lat ≈ g·tan(θ_lean)`、抗力込み）
     - `t_pos`: Chebyshev 残差 → `HANDOFF_CHEBY_MAX`（**Chebyshev 接近率**
     `v_cheby` —  diverging 時は減速＋反転時間を加算）
     - `t_settle = max(t_att, t_vh, t_pos)` — 大きいほど lean / aim ゲインを上げる。
   - lean 上限は固定 `LEAN_MID`/`LEAN_CRUISE` ではなく **物理逆算 + `LEAN_BRAKE_MAX` ソフト天井**。
   - `t_att` 支配時は aim を直立寄りにし、**スロットルを hover/cos 付近まで上げて
     ジンバルトルクを優先**（深リーン中の 0.35–0.55 上限は撤廃）。
   - `t_settle ≈ 0`（閾値目前）だけ静かな lean に戻し、go↔brake チャタを抑える。
   - ターミナル域では `vh` 過大・オーバーシュート・Chebyshev 超過時に **逆リーン
     ブレーキ**を再投入。`a_stop = vh²/(2Δcheby)` から lean を逆算して位置収束を加速。

3. **ターミナル Descend（パッド上・hand-off 後）**
   - 垂直: 物理閉ループ自殺バーン — `a_req = clamp((v² − v_touch²)/(2h), 0, a_brake)`、
     `t = m(a_req + g)/(T_max·up_y)`。コースト／ブレーキ／接地カットは
     [`PhysicsPadThrottleFuzzy`](src/fuzzy.rs) で肩付きブレンド（離散 step なし）。
     包絡遅刻の hard floor のみ離散のまま。
   - **エンジンアクチュエータ:** GNC セットポイントを [`slew_throttle`](src/fuzzy.rs)
     で非対称スプール（上 ~0.9 s、下 ~0.4 s の 0↔1）してから sim に渡す（L/T 共通）。
   - 姿勢: lean aim + √-profile PD + **`brake_safe_lean`**（`LEAN_TERMINAL_VH=0.18`）。
     固定 0.10 rad キャップは廃止。mid-range 包絡が残差 `vh` の大半を既に落とす。

4. **成功判定（T モード）**
   - **描画パッド:** 半辺 30 m（`TARGET_PAD_HALF_M`、mesh / shader と同期）。
   - **着陸成功:** 内側の Chebyshev 箱 半辺 **12 m**（`TARGET_SUCCESS_HALF_M`）。
   - Descend の高高度 seek 打ち切り: Chebyshev ≤ **8 m**（`TARGET_CENTER_TOL_M`）。

### 設計上の教訓（要約）

| やってよい | やってはいけない |
|---|---|
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

## テスト

```
cargo test -p pga-rocket
```

- [tests/physics.rs](tests/physics.rs) — 剛体物理・接地・破壊判定
- [tests/landing.rs](tests/landing.rs) — 自動着陸の統合テスト（横倒し・倒立・高速落下・
  タンブリングからの無傷着陸、コースト燃費、ソフト接地）
- [tests/control.rs](tests/control.rs) / [tests/explosion.rs](tests/explosion.rs) — 入力写像・爆発演出
- 各モジュール内ユニットテスト — PGA 恒等式（閉形式とサンドイッチの一致など)、
  包絡線・軸角度変換の境界値
