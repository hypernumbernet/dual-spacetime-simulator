# dst-expand 代数研究統合ツール 設計仕様書

## 1. はじめに

本ドキュメントは、`crates/dst-expand` を **クリフォード代数・PGA・ケイリー・ディクソン代数** を横断的に扱う**物理学・幾何学研究支援ツール**として進化させるための仕様を定義する。

本ツールの位置づけは、**ganja.js の物理学研究用 Rust 版拡張**に相当する。ganja.js が JavaScript で Geometric Algebra の可視化と代数演算を研究者に提供しているように、本ツールは Rust の高性能・型安全環境で以下の代数系を統一的に扱う：

- クリフォード代数（Cl(p,q,r) および PGA である G(3,1,1)）
- ケイリー・ディクソン構成（ℂ, ℍ, 𝕆, sedenion など）
- これらのテンソル積（特に ℍ⊗ℍ、ℂ⊗ℍ など二重時空理論で用いられる構造）

**最重要方針**:
- クリフォード代数 / PGA（特に G(3,1,1)）とケイリー・ディクソン代数およびそのテンソル積を**完全網羅**する
- **微分積分・方程式求解は一切行わない**
- 代数演算、幾何オブジェクト操作、物理研究向けユーティリティ（motor, torsional mismatch, norm, conjugation など）、可視化支援に集中
- 研究者が「紙と鉛筆で計算する」のと同じ感覚で対話的に実験できる REPL を提供

## 2. 現状分析

### 2.0 Phase 0: 現行 CLI 仕様（実装済み）

Phase 0 は **biquaternion（15 非スカラー基底 + スカラー）** の記号展開 CLI を安定化することを目的とする。G(3,1,1) / PGA / REPL など Phase 1 以降の機能は本節の対象外である。

#### 2.0.1 サブコマンド

| コマンド | 引数 | 成功時 stdout | 終了コード |
|---------|------|--------------|-----------|
| （引数なし） | なし | usage 全文 | 0 |
| `help` / `-h` / `--help` | なし | usage 全文 | 0 |
| `table` | なし（余分な引数不可） | 15×15 Markdown 乗法表 | 0 |
| `mul <i> <j>` | 基底インデックス 2 個（`0..14`） | 展開結果 1 行 | 0 |
| `sandwich <l> <m> <r>` | 基底インデックス 3 個（`0..14`） | サンドイッチ積の展開結果 1 行 | 0 |
| `expr <expression>` | 式文字列 1 個 | 式展開結果 1 行 | 0 |

未知のサブコマンド: stderr に `error: unknown command ...`、stdout に usage、終了コード `2`。

#### 2.0.2 エラー方針

- ユーザー入力エラー（引数不足・過剰、範囲外インデックス、式パース失敗）は **stderr** に `error:` で始まるメッセージを出力し、終了コード **2** を返す。
- 成功時は **stdout** のみに結果を出力し、stderr は空とする。
- `mul` の引数不足は従来どおり個別メッセージを維持する:
  - 0 個: `error: missing left basis index`
  - 1 個: `error: missing right basis index`
  - 3 個以上: `error: mul takes exactly 2 argument(s)`
- `sandwich` の引数個数不一致: `error: sandwich requires exactly 3 basis indices`（不足）または `error: sandwich takes exactly 3 argument(s)`（過剰）
- `expr` の引数個数不一致: `error: missing expression` / `error: expr takes exactly 1 argument(s)`
- `table` の余分な引数: `error: table takes exactly 0 argument(s)`
- 基底インデックス範囲外: `error: basis index must be 0..14, got ...`
- 式パース失敗: `error: at offset N: ...`（`ParseError` の表示形式）

#### 2.0.3 `expr` 式言語（Phase 0）

- 15 基底ラベル: `j`, `kI`, `kJ`, `kK`, `iI`, `iJ`, `iK`, `I`, `J`, `K`, `k`, `jI`, `jJ`, `jK`, `i`（長いラベル優先で字句解析）
- 係数: ASCII 識別子・数値リテラル（記号簡約は `coeff_format` が担当）
- 演算: 暗黙積、`+` / `-` 加減、括弧、明示 `*`
- 出力: `[label]` 形式の基底モノミアル和（例: `[j]`, `-1`, `2ab + (-aa+bb)[i]`）

**字句解析の注意**:

- 長いラベルを優先するため、`ki` は基底 `k`（インデックス 10）と `i`（14）の積として解釈される（単一トークン `kI` ではない）。
- 係数と基底の境界は最長一致で切り分ける。例: `ai` → 係数 `a` + 基底 `i`、`2ab` → 係数 `2ab`（識別子全体）。
- 同一モノミアル内に複数基底が連続する場合（例: `iI*j` または `iIj`）、左から順に Cl(3,1) 互換の乗法表で展開する。

**未サポート（Phase 1 以降）**: 外積 `^`、逆元 `~` / `inv()`、`exp` / `log`、サンドイッチ演算子 `>>`、G(3,1,1) 用の `e0`..`e4` 表記（5.4 節参照）。

#### 2.0.4 基底インデックス対応表

`mul` / `sandwich` は 0..14 の整数インデックス、`expr` は右列のラベル文字列を使う。いずれも `dst-math` の biquaternion 15 基底（四元数サイド ⊗ 四元数レーン）に対応する。

| インデックス | ラベル | 構成（side ⊗ lane） |
|-------------|--------|---------------------|
| 0 | `j` | j ⊗ 1 |
| 1 | `kI` | k ⊗ i |
| 2 | `kJ` | k ⊗ j |
| 3 | `kK` | k ⊗ k |
| 4 | `iI` | i ⊗ i |
| 5 | `iJ` | i ⊗ j |
| 6 | `iK` | i ⊗ k |
| 7 | `I` | 1 ⊗ i |
| 8 | `J` | 1 ⊗ j |
| 9 | `K` | 1 ⊗ k |
| 10 | `k` | k ⊗ 1 |
| 11 | `jI` | j ⊗ i |
| 12 | `jJ` | j ⊗ j |
| 13 | `jK` | j ⊗ k |
| 14 | `i` | i ⊗ 1 |

スカラー部（16 次元目）は `[label]` を付けず `-1` や `2ab` のように出力する。

#### 2.0.5 ビルドと起動

リポジトリルートから:

```bash
cargo build -p dst-expand
cargo run -p dst-expand -- <subcommand> [args...]
```

インストール済みバイナリを使う場合は `dst-expand` を直接呼び出す。引数なし、または `help` / `-h` / `--help` で usage が表示される。

#### 2.0.6 使用例と API

コマンドの実行例および Rust ライブラリ API の詳細は **5.5 節「式の例」** および `tests/cli_smoke.rs` の cross-check テストを参照。代表的な挙動は以下の通り。

- `mul 0 0` → `-1`（j × j）
- `expr "(b-ai)(a+bi)"` → `2ab + (-aa+bb)[i]`
- `sandwich 14 0 14` → `[j]`（i · j · i）

エラーはすべて `stderr` に `error:` 接頭辞付きで出力され、終了コード 2 を返す（2.0.2 節参照）。

#### 2.0.7 出力形式とテスト

出力は `[label]` 形式の基底モノミアル和（例: `b[kI] + a[i]`）。詳細は `format.rs` / `coeff_format.rs` を参照。テストは `tests/cli_smoke.rs` の CLI contract test で stdout / stderr / 終了コードを検証（2.0.2 節のエラー方針に準拠）。

#### 2.0.8 テスト方針（Phase 0）

`tests/cli_smoke.rs` に CLI contract test を置き、仕様（2.0.1〜2.0.2）に忠実に stdout / stderr / 終了コードを検証。ライブラリ API との一致も cross-check。Phase 1 以降の機能は対象外。

### 2.1 既存機能（Phase 0 実装済み）

`biquaternion.rs`（記号展開・テーブル）、`expr.rs`（パーサ）、`format.rs` / `coeff_format.rs`（表示）、`main.rs`（CLI）、`algebra/mod.rs`（`Algebra` enum スタブ）、`dst-math`（数値テーブル）。詳細は 6.1 節のモジュール構成を参照。

### 2.2 限界と課題
- 現在の基底は Cl(3,1) の 16 次元止まりで、G(3,1,1) の 32 次元構造（特に null 方向 \(e_4\)）に対応していない
- PGA 特有のオブジェクト（plane, line, motor, null bivector）に対する高レベル API が存在しない
- 四元数・八元数およびそれらのテンソル積に対する体系的なサポートが欠如
- 物理研究で頻出する「torsional mismatch」「Poincaré 代数再現」「null translation」「Cayley-Dickson doubling」などの操作が手動で煩雑
- ganja.js のようなインタラクティブな代数実験環境が不足

## 3. 基本方針

### 3.1 核心原則
1. **代数系の完全網羅（最優先）**
   - クリフォード代数 Cl(p,q) / G(p,q,r)（特に G(3,1,1)）
   - ケイリー・ディクソン代数（ℂ, ℍ, 𝕆 とその doubling 構成）
   - テンソル積代数（ℍ⊗ℍ, ℂ⊗ℍ, 𝕆⊗ℍ など）
   - 各代数系の基底、乗法表、norm、conjugate、reverse を第一級で扱う
2. **物理研究向け設計**
   - Double Spacetime Theory で用いられる motor, extended mismatch bivector, Killing form, duality map を直接構築できる API を提供
   - null vector / bivector の特殊性（自乗ゼロ、相互積ゼロ）を型レベルで保証
3. **ganja.js 風の体験**
   - REPL で代数式を対話的に試行・展開
   - 結果の Markdown / LaTeX / Unicode 出力
   - 将来的に 3D/4D 可視化との連携

### 3.2 やらないこと
- 微分・積分
- 方程式求解（線形・非線形）
- 一般的な CAS 機能（因数分解、多項式 GCD など）
- 数値シミュレーションエンジン（dual-spacetime-simulator で行う）

## 4. 主要ターゲット機能（優先度順）

### 4.1 統一代数基盤の構築（Phase 1 以降・最優先）
- 各代数系に対する `Algebra` trait または enum で文脈を管理
  - `Pga` — G(3,1,1) Projective Geometric Algebra
  - `G { p, q, r }` — 一般幾何代数 Cl(p, q, r)
  - `CayleyDickson { dimension: usize }`（2,4,8,16,...）
  - `TensorProduct { left: Algebra, right: Algebra }`
- 基底要素の名前解決（Clifford では `e0,e1,...`、`i,j,k` もエイリアス、Cayley-Dickson では `e0..e7` など）
- 乗法表の動的生成とキャッシュ

### 4.2 G(3,1,1) PGA の完全サポート
- 32次元 Clifford 代数の基底（スカラー + 5ベクトル + 10 bivector + 10 trivector + 5 quadvector + pseudoscalar）
- null 方向 \(e_4\) と null bivector \(N_\mu = e_4 \wedge e_\mu\) の特殊処理
- 10次元 bivector 生成子（hyperbolic / cyclic / null）の分類と操作

#### 4.2.1 G(3,1,1) 基底表現（Phase 1 実装済み）

**実装場所**: `crates/dst-math/src/pga.rs`

基底は **ビットマスク 0..31** で表現する。生成元 \(e_k\) は bit \(k\) に対応する（\(k = 0, 1, 2, 3, 4\)）。

| インデックス | ラベル | グレード | 意味 |
|-------------|--------|---------|------|
| 0 | `1` | 0 | スカラー |
| 1 | `e0` | 1 | 第 0 生成元 |
| 2 | `e1` | 1 | 第 1 生成元 |
| 4 | `e2` | 1 | 第 2 生成元 |
| 8 | `e3` | 1 | 第 3 生成元 |
| 16 | `e4` | 1 | null 生成元 |
| 3 | `e0e1` | 2 | bivector |
| … | … | … | 上位ビットの XOR 積 |
| 31 | `e0e1e2e3e4` | 5 | pseudoscalar |

**メトリック（Double Spacetime Theory 向け割り当て）**:

| 生成元 | \(e_k^2\) |
|--------|-----------|
| \(e_0\) | \(-1\) |
| \(e_1, e_2, e_3\) | \(+1\) |
| \(e_4\) | \(0\)（null） |

`Algebra::Pga`（[`Self::pga`] ショートカット）の名前は **`PGA`**。Phase 0 の biquaternion 15 基底は **Cl(3,1) 部分代数**（\(e_4\) なし）として位置づけ、後方互換を維持する。

#### 4.2.2 乗法表生成アルゴリズム（Phase 1 実装済み）

`Pga::basis_mul(i, j) -> (i8, usize)` は以下の手順で **幾何積** を計算する。

1. スカラー（インデックス 0）は恒等元として透過
2. 右因子 `b` の最下位ビット `e_k` を順に処理
3. 左因子 `a` 内で `e_k` **より上位**のビット数に応じて反符号（反交換）
4. `a` に `e_k` が既にあれば \(e_k^2 = \text{metric}[k]\) を適用しビットを除去、なければ追加
5. 最終マスクと累積符号を返す

32×32 テーブルは `const fn compute_pga_mul_table()` でコンパイル時生成し、`PGA_MUL_TABLE` としてキャッシュする。

#### 4.2.3 Null 成分ルール（Phase 1 実装済み）

- \(e_4^2 = 0\)
- null bivector \(N_\mu = e_4 e_\mu\)（\(\mu = 0..3\)）は **自乗ゼロ**: \(N_\mu^2 = 0\)
- \(N_\mu N_\nu + N_\nu N_\mu = 0\)（\(\mu \neq \nu\)）— 異なる null bivector は反交換
- ヘルパ: `Pga::null_bivector_index(mu)` → 基底インデックス（例: \(\mu=0\) → `17` = `e0e4`）

#### 4.2.4 Multivector 基本 API（Phase 1 実装済み）

**型**: `Multivector` — `[f64; 32]` 係数配列

| 操作 | メソッド / 演算子 | 説明 |
|------|------------------|------|
| 加算 | `+`, `-` | 成分ごと |
| 幾何積 | `*` | 32×32 テーブル展開 |
| スカラー倍 | `*` / `Mul<f64>` | 全成分に適用 |
| グレード射影 | `grade(k)` | popcount = k の成分のみ残す |
| リバース | `reverse()` | k-blade に \((-1)^{k(k-1)/2}\) |
| 共役 | `conjugate()` | リバース + 奇数グレード符号反転 |
| 単位元 | `Multivector::one()` | スカラー 1 |
| 基底 | `Multivector::basis(i)` | 第 i 基底 |

**未実装（Phase 2 以降）**: 外積 `^`、逆元、exp/log、Motor/Plane 高レベル型。

### 4.3 PGA / 幾何オブジェクトの高レベル API
- `Plane`, `Line`, `Point`, `Motor`, `NullBivector` などの型
- `geometric_product`, `sandwich`, `exp`, `log`, `reverse`, `conjugate`
- Motor の構築: `Motor::from_bivector(Ω)` や `exp(Ω)`
- null 成分を含む exp の有限多項式展開の明示

### 4.4 ケイリー・ディクソン特有機能
- Doubling construction の明示的サポート（任意次元まで）
- Norm, conjugate, inverse の計算
- Zero-divisor 検出（sedenion 以降）
- 乗法表の自動生成と可視化

### 4.5 テンソル積代数の扱い
- ℍ⊗ℍ の基底を `i⊗1, 1⊗i, j⊗1, ...` や `I1, J1, I2, J2` などの表記で表現
- 左右の因子の独立した演算と混合積の展開
- Double Spacetime Theory で用いられる biquaternion（ℂ⊗ℍ または ℍ⊗ℍ の実部）との対応

### 4.6 物理研究向けユーティリティ
- Torsional mismatch bivector の構築と Killing form 計算
- Poincaré 代数 \(\mathfrak{iso}(3,1)\) の再現検証
- Duality map \(X \mapsto X i\) の各代数系での実装
- Plane-based bracket product と dual spacetime の対応

### 4.7 入出力と可視化支援
- LaTeX 出力（amsmath 準拠）
- 研究ノート向け Unicode テキスト出力（上付き・下付き、色分け）
- REPL からの直接可視化リクエスト（将来的に egui 連携）

### 4.8 インタラクティブ環境（Phase 2 以降）
- `dst-expand repl` で対話的計算
- コンテキスト切り替え（`use G(3,1,1)` や `use HxH`）
- 履歴、自動 LaTeX スニペット保存

## 5. 入力形式の詳細設計（REPL / expr）

### 5.1 基本設計方針
- 人間が紙に書く数式に近い直観的な記法を優先
- 基底名は代数系ごとに自然な短縮形を許可（エイリアス機構）
- スカラー係数は Unicode ギリシャ文字や ASCII を混在可能
- 指数・対数・逆元などの演算子を明示
- コンテキスト（使用する代数系）を明示的に指定可能

### 5.2 コンテキスト指定
REPL 起動時またはコマンドで指定：
```
dst-expand repl --algebra G(3,1,1)
dst-expand repl --algebra "H tensor H"
```
または REPL 内で：
```
> use G(3,1,1)
> use HxH
```

### 5.3 基底要素の表記例

**Phase 0**: 2.0.4 節の 15 基底ラベル（`j`, `iI`, `kI` など）を使用。

**Phase 1（ライブラリ API 実装済み・CLI 未対応）**

- **PGA**: `e0`, `e1`, `e2`, `e3`, `e4`（null 方向）— `dst-math::pga::BASIS_LABELS` 参照
  - エイリアス（Phase 2 CLI 向け設計目標）: `j` = `e0`, `kI` = `e1`, `iI` = `e0*e1` など（Cl(3,1) 互換）
- **四元数 (H)**: `i`, `j`, `k` または `e1`, `e2`, `e3`
- **八元数 (O)**: `e0`..`e7` または `e,f,g,h,i,j,k,l`
- **テンソル積 (H⊗H)**: `i1`, `j1`, `k1`（左因子）、 `i2`, `j2`, `k2`（右因子）、または `i⊗1`, `1⊗i`
- **混合**: `e0 + 0.5*e4` や `α*e1 + β*N0`

### 5.4 演算子の優先順位と記法

**Phase 0（実装済み）**

**Phase 0（実装済み）**: 括弧 > 単項 `-` > 積（暗黙 / `*`） > 加減。

**Phase 1 以降（設計目標）**: 外積 `^`、内積 `|`、逆元 `~` / `inv()`、指数 `exp`、対数 `log`、サンドイッチ `>>` など（5.5 節の未実装例参照）。

### 5.5 式の例（Phase 0 / CLI 対応）

**基底積・サンドイッチ（`mul` / `sandwich`）**

| コマンド | 結果 | 意味 |
|---------|------|------|
| `mul 0 0` | `-1` | j × j |
| `mul 14 0` | `[k]` | i × j |
| `mul 4 5` | `-[K]` | iI × iJ |
| `sandwich 0 0 0` | `-[j]` | j · j · j |
| `sandwich 14 0 14` | `[j]` | i · j · i |

**記号式展開（`expr`）**

```
(j)(j)                     →  -1
ai + bkI                   →  b[kI] + a[i]
a*i                        →  a[i]          （明示 * と暗黙積は等価）
(ai+bkI)(cj+dkK)           →  -bc[iI] + bd[J] + ac[k] - ad[jK]
(b-ai)(a+bi)               →  2ab + (-aa+bb)[i]
ij                         →  [k]
ki                         →  [j]           （k × i；ラベル kI とは別）
```

**Windows（PowerShell）での引用**

```powershell
cargo run -p dst-expand -- expr "(ai+bkI)(cj+dkK)"
```

**エラー例**

```
$ dst-expand nosuch
error: unknown command "nosuch"   （終了コード 2、stdout に usage）

$ dst-expand expr "(ai"
error: at offset 3: expected ')'
```

**Phase 1 以降（未実装・設計目標）**

```
> exp( (pi/2)*iI + (0.1)*N0 )
> (e0 + e4) * (e1 + 0.5*e2)
> Motor::from(0.5*iI + 0.3*J + N1 + N2)
> HxH: i1*j2 + k1*1
> norm( exp(Ω) )
```

### 5.6 パーサ実装方針
- `nom` または `pest` を用いた PEG パーサ
- 基底名解決は `AlgebraContext` が担当
- 型推論で文脈を自動補完（可能なら）
- エラー報告は「紙と鉛筆」感覚で親切に

## 6. アーキテクチャ設計

### 6.1 モジュール構成

**現状（Phase 0 + Phase 1 ライブラリ）**

```
crates/dst-math/
├── src/
│   ├── pga.rs                 # G(3,1,1) PGA 32基底・Multivector（Phase 1）
│   ├── biquaternion.rs        # Cl(3,1) 互換 16次元（Phase 0）
│   └── ...
└── tests/
    └── math_pga.rs            # G(3,1,1) PGA 単体テスト（Phase 1）

crates/dst-expand/
├── src/
│   ├── lib.rs                 # 公開 API の re-export
│   ├── main.rs                # CLI（table / mul / sandwich / expr）
│   ├── biquaternion.rs        # 記号展開コア
│   ├── expr.rs                # 式パーサ
│   ├── format.rs              # 展開結果の表示
│   ├── coeff_format.rs        # 係数文字列の簡約
│   └── algebra/
│       └── mod.rs             # Algebra enum + mul_table 連携（Phase 1）
├── tests/
│   └── cli_smoke.rs           # Phase 0 CLI 契約テスト
└── Cargo.toml
```

**予定（Phase 2 以降）**

```
crates/dst-expand/
├── src/
│   ├── cli.rs                 # main.rs から分離
│   ├── algebra/
│   │   ├── clifford.rs        # Cl(p,q,r) / G(3,1,1)
│   │   ├── cayley_dickson.rs  # H, O, sedenion, doubling
│   │   └── tensor.rs          # テンソル積
│   ├── pga/                   # PGA 特化（高レベルオブジェクト）
│   │   ├── multivector.rs
│   │   ├── motor.rs
│   │   ├── plane.rs
│   │   └── mismatch.rs
│   └── repl.rs
├── tests/
│   ├── pga/
│   └── cayley_dickson/
└── ...
```

### 6.2 型システム
- `Algebra` trait + 具体型で文脈を表現
- `MultiVector` または `SparseMultivector` で要素を保持
- 高レベル型（`Motor`, `Plane`）は内部で multivector をラップし、null 特殊性を保証

### 6.3 依存関係
- 既存 `dst-math` との連携
- `nom` / `pest`（パーサ）
- 将来的に可視化用 `egui` / `plotters`

## 7. 開発ロードマップ

### Phase 0（現行・安定）

- biquaternion 記号展開 CLI（`table` / `mul` / `sandwich` / `expr`）の契約明文化
- 引数検証の厳密化（余分な引数の拒否、`sandwich` の 4 引数以上バグ修正）
- CLI contract test の網羅（`tests/cli_smoke.rs`）
- 係数表示の簡約（`coeff_format`）
- `Algebra` enum スタブ（`algebra/mod.rs`）
- 本仕様書 2.0 節・使用例との整合

### Phase 1（短期・ライブラリ実装済み）

- 統一 `Algebra` 基盤と G(3,1,1) PGA 32基底（`dst-math::pga`）
- null 成分の完全サポート（\(e_4^2=0\)、\(N_\mu^2=0\) ヘルパとテスト）
- 基本 multivector 演算（加算、幾何積、グレード射影、reverse、conjugate）
- メトリック: \(e_0^2=-1\), \(e_1^2=e_2^2=e_3^2=+1\), \(e_4^2=0\)
- 単体テスト: `crates/dst-math/tests/math_pga.rs`（20+ ケース）
- **CLI / REPL 拡張は Phase 2**（Phase 0 biquaternion CLI は後方互換のまま維持）

#### Phase 1 成功基準（自動テスト）

| 検証項目 | 期待 |
|---------|------|
| 次元 | `Pga::dimension() == 32` |
| \(e_0^2\) | \(-1\) |
| \(e_4^2\) | \(0\) |
| \(N_\mu^2\) | \(0\)（\(\mu=0..3\)） |
| 結合律 | 代表基底三つ組で成立 |
| 分配律 | \(a(b+c) = ab + ac\) |
| reverse | \((ab)^\sim = \tilde{b}\tilde{a}\) |

### Phase 2（中期）
- ケイリー・ディクソン（H, O）と doubling
- テンソル積（特に H⊗H）
- Motor / Plane API と REPL
- LaTeX / Unicode 出力

### Phase 3（長期）
- Poincaré / Killing form ユーティリティ
- 論文執筆支援（自動スニペット）
- egui 可視化連携

## 8. 成功指標
- G(3,1,1) と H⊗H の主要性質が自動検証可能
- 「ganja.js でやっていたことを Rust で自然に再現できた」と感じられる
- Double Spacetime Theory の motor や mismatch を短いコードで構築可能
- 既存 Cl(3,1) ユーザーがシームレスに移行できる

## 9. 補足・注意事項
- 既存 API の後方互換を最優先（Phase 0 biquaternion CLI / 記号展開は変更しない）
- Phase 1 の PGA 実装は **`dst-math::pga`** に置き、`dst-expand::algebra` は文脈管理とテーブル参照を担当
- null 成分と zero-divisor は専用の型・チェックで扱う（Phase 1 では `null_bivector_index` ヘルパとテストで検証）
- データ並列は `rayon` を検討
- 数学的正確性は ganja.js および Double Spacetime Theory 論文を参照

---

本仕様書は、ganja.js の精神を Rust に移植し、Double Spacetime Theory の物理的文脈と Cayley-Dickson・Clifford・テンソル積の数学的統一を両立させる研究者向けツールとして設計された。微分積分や一般 CAS 機能は意図的に除外し、代数と幾何の深い研究に特化する。