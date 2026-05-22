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

### 2.1 既存機能
- 15基底の Cl(3,1) 乗法表に基づく `expand_basis_product`
- サンドイッチ積 `expand_sandwich`
- 簡易式パーサ `expand_expr`
- 様々な形式の乗法表出力

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

### 4.1 統一代数基盤の構築（最優先）
- 各代数系に対する `Algebra` trait または enum で文脈を管理
  - `Clifford { p, q, r }`
  - `CayleyDickson { dimension: usize }`（2,4,8,16,...）
  - `TensorProduct { left: Algebra, right: Algebra }`
- 基底要素の名前解決（Clifford では `e0,e1,...`、`i,j,k` もエイリアス、Cayley-Dickson では `e0..e7` など）
- 乗法表の動的生成とキャッシュ

### 4.2 G(3,1,1) PGA の完全サポート
- 32次元 Clifford 代数の基底（スカラー + 5ベクトル + 10 bivector + 10 trivector + 5 quadvector + pseudoscalar）
- null 方向 \(e_4\) と null bivector \(N_\mu = e_4 \wedge e_\mu\) の特殊処理
- 10次元 bivector 生成子（hyperbolic / cyclic / null）の分類と操作

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

### 4.8 インタラクティブ環境
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
- **Clifford / PGA**: `e0`, `e1`, `e2`, `e3`, `e4`（null 方向）
  - エイリアス: `j` = `e0`, `kI` = `e1`, `iI` = `e0*e1` など（Cl(3,1) 互換）
- **四元数 (H)**: `i`, `j`, `k` または `e1`, `e2`, `e3`
- **八元数 (O)**: `e0`..`e7` または `e,f,g,h,i,j,k,l`
- **テンソル積 (H⊗H)**: `i1`, `j1`, `k1`（左因子）、 `i2`, `j2`, `k2`（右因子）、または `i⊗1`, `1⊗i`
- **混合**: `e0 + 0.5*e4` や `α*e1 + β*N0`

### 5.4 演算子の優先順位と記法
- 積:  juxtaposition（`e0 e1`）または `*`（`e0*e1`）
- 幾何積 / Clifford 積: デフォルトで `*`（文脈による）
- 外積: `^`（`e0^e1`）
- 内積: `|` または `.`（`e0|e1`）
- 逆元: `~` または `inv()`（`~M`）
- 指数関数: `exp(φ/2 * iI + θ/2 * N0)`
- 対数: `log(M)`
- サンドイッチ: `M >> X` または `sandwich(M, X)`

### 5.5 式の例
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

### 6.1 モジュール構成（予定）
```
crates/dst-expand/
├── src/
│   ├── lib.rs
│   ├── cli.rs
│   ├── algebra/               # 代数系の抽象化
│   │   ├── mod.rs
│   │   ├── clifford.rs        # Cl(p,q,r) / G(3,1,1)
│   │   ├── cayley_dickson.rs  # H, O, sedenion, doubling
│   │   └── tensor.rs          # テンソル積
│   ├── pga/                   # PGA 特化（高レベルオブジェクト）
│   │   ├── multivector.rs
│   │   ├── motor.rs
│   │   ├── plane.rs
│   │   └── mismatch.rs
│   ├── format.rs
│   ├── repl.rs
│   └── biquaternion.rs        # 後方互換レイヤ
├── tests/
│   ├── pga/
│   └── cayley_dickson/
└── Cargo.toml
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

### Phase 0
- 既存機能安定化 + 本仕様書合意

### Phase 1（短期）
- 統一 `Algebra` 基盤と G(3,1,1) 32基底
- null 成分の完全サポート
- 基本 multivector 演算

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
- 既存 API の後方互換を最優先
- null 成分と zero-divisor は専用の型・チェックで扱う
- データ並列は `rayon` を検討
- 数学的正確性は ganja.js および Double Spacetime Theory 論文を参照

---

本仕様書は、ganja.js の精神を Rust に移植し、Double Spacetime Theory の物理的文脈と Cayley-Dickson・Clifford・テンソル積の数学的統一を両立させる研究者向けツールとして設計された。微分積分や一般 CAS 機能は意図的に除外し、代数と幾何の深い研究に特化する。