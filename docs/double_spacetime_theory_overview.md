**二重時空理論（Double Spacetime Theory）  
プログラマ向け数学的原理・基本力学要約資料**  

（シミュレーション実装用・2026年5月版）  

---

### 1. 目的と全体像
二重時空理論は、重力を「連続時空の曲率」ではなく「各粒子が内在する**通常時空**と**双対時空**のねじれ不整合（torsional mismatch）」として再定式化する理論です。  
すべての数学は**16実次元双四元数代数**（biquaternion algebra ≅ Cl(3,1)）の中で完結し、Christoffel記号・Riemann曲率・連続時空仮説は一切不要です。  

シミュレーションで必要な核心は以下の4点のみです：
1. 双四元数の演算
2. 通常／双対ベクトルとその変換
3. ベルソル（rotor）によるローレンツ変換と平行移動
4. ねじれ不整合ローター Ω からスカラー J を計算し、重力・慣性として扱う

これらを正確に実装すれば、GRの弱場・強場・回転曲線・核力・原子軌道まですべて再現可能です。

---

### 2. 双四元数代数（16実次元）の定義

二つの互いに可換な四元数系を定義：
- 一次四元数：$i,j,k$　（$i^2=j^2=k^2=-1$, $ij=k$, $ji=-k$ など）
- 二次四元数：$I,J,K$　（同規則、$IJ=K$ など）

**可換性**：$iI=Ii$, $iJ=Ji$, …（全9通り）

**基底**（16個）：
$$
1,\ i,\ j,\ k,\ I,\ J,\ K,\ iI,\ iJ,\ iK,\ jI,\ jJ,\ jK,\ kI,\ kJ,\ kK
$$

**Cl(3,1) との明示的同相**：
$$
e_0 = j, e_1 = kI, e_2 = kJ, e_3 = kK
$$
これにより $(e_0^2 = -1), (e_i^2 = +1)$ が満たされ、Minkowski計量が自然に現れます。

**乗法表**は添付ファイル（double-spacetime-theory.tex の Appendix）に完全版があります。

---

### 3. 通常時空ベクトルと双対時空ベクトル

各粒子は**2つのコンパクト化されたMinkowski時空**を内在します。

- **通常時空ベクトル**（Usual spacetime）  
  $$
  X = ct\, j + x\, kI + y\, kJ + z\, kK
  $$
  不変量：$X^2 = -(ct)^2 + x^2 + y^2 + z^2$

- **双対時空ベクトル**（Dual spacetime）  
  $$
  X' = ct'\, k + x'\, jI + y'\, jJ + z'\, jK
  $$
  不変量：${X'}^2 = -(ct')^2 + {x'}^2 + {y'}^2 + {z'}^2$

**双対写像**（Dual map）：
$$
X' = X i \quad \Rightarrow \quad (ct\, j)i = -ct\, k
$$
（時間の矢印が反転するが、ノルムは保存）

**実装Tips**：1つの16成分双四元数オブジェクトで両方を保持。右から $i$ を掛けるだけで瞬時に切り替え可能。

---

### 4. ベルソル（Versor）によるローター（Rotor）とローレンツ変換

すべての平行移動（Translator）・ローレンツ変換（Boost）は**サンドイッチ積**で表現されます。  
**unit rotor**（$R R^{-1} = 1$）の場合、一般形は
$$
\tilde{X} = R\, X\, R^{-1}
$$
です。以下で段階的に構築します。

#### 4.1 通常時空での平行移動とローレンツ変換

**（a）純粋平行移動**（双対ローター成分のみ使用）  
生成子 $\hat{q} = q_1 I + q_2 J + q_3 K$（$\hat{q}^2 = -1$）に対し
$$
R_{\rm rot} = \exp\!\left( \frac{\theta}{2} \hat{q} \right) = \cos\frac{\theta}{2} + \hat{q} \sin\frac{\theta}{2}
$$
サンドイッチ積：
$$
\tilde{X} = R_{\rm rot}\, X\, R_{\rm rot}^{-1}
$$
（通常の四元数ライブラリの回転と同じ動作）

**（b）ローレンツブースト**（通常ローター成分のみ使用）  
生成子 $\hat{p} = p_1 iI + p_2 iJ + p_3 iK$（$\hat{p}^2 = +1$）に対し
$$
R_{\rm boost} = \exp\!\left( \frac{\phi}{2} \hat{p} \right) = \cosh\frac{\phi}{2} + \hat{p} \sinh\frac{\phi}{2}
$$
サンドイッチ積：
$$
\tilde{X} = R_{\rm boost}\, X\, R_{\rm boost}^{-1}
$$
（双曲関数で展開され、標準ローレンツ変換になる）

**実装Tips**：  
- `R_usual_boost(φ, p_hat)` と `R_usual_rot(θ, q_hat)` を別々に用意するとデバッグが容易。  
- 逆元は `R.inverse()` = `exp(-B)` で計算（符号反転）。

#### 4.2 双対時空でのローター

双対時空では生成子が $\hat{q}$（回転）のみで、ブーストは通常時空から「逆方向」に作用します。  
双対ローター：
$$
R_{\rm dual} = \exp\!\left( \frac{\theta}{2} \hat{q} \right) = \cos\frac{\theta}{2} + \hat{q} \sin\frac{\theta}{2}
$$
サンドイッチ積（双対ベクトル $X'$ に対して）：
$$
\tilde{X}' = R_{\rm dual}\, X'\, R_{\rm dual}^{-1}
$$

#### 4.3 完全ローターによる一般変換（すべての変換）

通常時空と双対時空のローターを合成：
$$
R_{\rm total} = R_{\rm usual}\, R_{\rm dual}
$$
（生成子が可換なので積の順序は自由）

**逆元**（サンドイッチ積に必須）：
$$
R_{\rm total}^{-1} = R_{\rm dual}^{-1}\, R_{\rm usual}^{-1}
$$

**一般変換式**（すべての変換を統一的に扱う）：
$$
\tilde{X} = R_{\rm total}\, X\, R_{\rm total}^{-1}
$$
$$
\tilde{X}' = R_{\rm total}\, X'\, R_{\rm total}^{-1}
$$

これ1本で**通常時空・双対時空の同時変換**が可能になります。

**実装上のポイント**：
- `R_total = R_usual * R_dual`
- `R_total_inv = R_dual.inverse() * R_usual.inverse()`
- サンドイッチ積関数：`sandwich(R_total, X)` = `R_total * X * R_total_inv`

---

### 5. ねじれ不整合（Torsional Mismatch）と重力スカラー $J$（daggerはここで特別扱い）

**dagger変換（†）** は**Ω計算専用の特別操作**です（通常の逆元 $^{-1}$ とは区別）。

**dagger変換の定義**：
1. まず逆元を取る：$R^{-1} = \exp(-B)$
2. さらに**dual map（右から $i$ をかける）**を適用して双対セクターへ再解釈  
   （特に $\hat{p}\, i = -\hat{q}'$ という変換が生じる）

**ねじれ不整合ローター**（dagger変換を使用）：
$$
\Omega = R_{\rm usual}^\dagger\, R_{\rm dual}
$$

以降の $\Omega_{\rm biv} = \log \Omega$ と $J$ の計算は変更ありません。

---

### 実装チェックリスト（最新版）

1. 双四元数クラス  
   - `inverse()`：純粋逆元（exp(-B)）  
   - `dagger()`：逆元＋dual map（†）※Ω計算専用  
2. 通常ローター（boost / rot 別）・双対ローター生成関数  
3. **完全ローター** `R_total = R_usual * R_dual` と `R_total_inv`  
4. **サンドイッチ積関数**（$R\, X\, R^{-1}$)  
5. Ω計算：`R_usual.dagger() * R_dual` → log → J  

---

### 4. ベルソル（Versor / Rotor）とローレンツ変換

すべての変換は**サンドイッチ積**で表現されます：
$$
\tilde{X} = R_{total}\, X\, R_{total}^{-1}
$$

#### 4.1 完全ローターの定義
$$
R_{total} = R_{usual}\, R_{dual}
$$
$$
R_{total}^{-1} = R_{dual}^{-1} R_{usual}^{-1} \quad \text{(逆順注意)}
$$

- 通常ローター：
  $$
  R_{usual} = \exp\!\left( \frac{\phi}{2}\hat{p} \right) \quad \Rightarrow \quad
  R_{usual}^{-1} = \exp\!\left( -\frac{\phi}{2}\hat{p} \right) = \cosh\frac{\phi}{2} - \hat{p}\sinh\frac{\phi}{2}
  $$
  （$\hat{p}^2 = +1$）

- 双対ローター：
  $$
  R_{dual} = \exp\!\left( \frac{\theta}{2}\hat{q} \right) \quad \Rightarrow \quad
  R_{dual}^{-1} = \exp\!\left( -\frac{\theta}{2}\hat{q} \right) = \cos\frac{\theta}{2} - \hat{q}\sin\frac{\theta}{2}
  $$
  （$\hat{q}^2 = -1$）

#### 4.1 dagger変換（†）の厳密定義

**dagger変換** は以下の2段階の操作です：

1. **Algebraic inverse（逆元）** を取る：  
   任意の unit rotor $R = \exp(B)$ に対して  
   $$
   R^{-1} = \exp(-B)
   $$

2. **Dual map（右から $i$ をかける）** を適用して dual sector へ再解釈する：  
   特に通常ローターの場合、
   $$
   \hat{p}\, i = -(p_1 I + p_2 J + p_3 K) \equiv -\hat{q}'
   $$
   （$\hat{p}^2 = +1$ が $\hat{q}'^2 = -1$ に変わる）

   これにより、**$R_{\rm usual}^\dagger$** は次のように表現されます：
   $$
   R_{\rm usual}^\dagger = \exp\!\left( -\frac{\phi}{2}\hat{p} \right) = \cosh\frac{\phi}{2} - \hat{p}\sinh\frac{\phi}{2}
   $$
   を dual map で再解釈すると
   $$
   R_{\rm usual}^\dagger = \cos\frac{\phi}{2} - \hat{q}'\sin\frac{\phi}{2}
   $$
   （dual sector での純粋回転として振る舞う）

**実装上のポイント**：
- `R.dagger()` メソッドは **逆元計算 + dual map による再解釈** を同時に行う。
- 単なる algebraic inverse は `R.inverse()` として別に実装（通常のClifford reversion / conjugate）。
- ねじれ不整合計算では **必ず dagger変換**（†）を使用：
  $$
  \Omega = R_{\rm usual}^\dagger R_{\rm dual}
  $$

#### 4.3 サンドイッチ積の4パターン（daggerを正しく反映）

| パターン | 対象ベクトル | 適用ローター          | 変換式                                      | 物理的意味 |
|----------|--------------|-----------------------|---------------------------------------------|------------|
| (1) 標準 | 通常 $X$   | $R_{\rm usual}$     | $\tilde{X} = R_{\rm usual}\, X\, R_{\rm usual}^\dagger$ | 通常時空での純粋ローレンツ変換 |
| (2) 完全 | 通常 $X$   | $R_{\rm total}$     | $\tilde{X} = R_{\rm total}\, X\, R_{\rm total}^\dagger$ | 通常＋双対効果を含む完全変換 |
| (3)      | 双対 $X'$  | $R_{\rm dual}$      | $\tilde{X}' = R_{\rm dual}\, X'\, R_{\rm dual}^\dagger$ | 双対時空での純粋回転 |
| (4) 特徴的 | 双対 $X'$  | $R_{\rm usual}$     | $\tilde{X}' = R_{\rm usual}\, X'\, R_{\rm usual}^\dagger$ | **retrogradeブースト**（時間反転効果） |

**通常の四元数共役 $^*$** との関係：
- パターン(1)では unit rotor の場合 $R_{\rm usual}^* = R_{\rm usual}^\dagger$ となり、既存の四元数ライブラリがそのまま使えます。
- ただし dagger変換の本質は **dual map（右×i）** による再解釈であることを忘れないでください。

---

### 5. ねじれ不整合（Torsional Mismatch）と重力スカラー $J$（dagger反映済）

**ねじれ不整合ローター**（dagger変換を使用）：
$$
\Omega = R_{\rm usual}^\dagger\, R_{\rm dual}
$$

以降の $J$ の計算、物理的意味、作用積分は変更ありません。

---

### 実装チェックリスト（dagger対応更新版）

1. 双四元数クラス  
   - `inverse()`：純粋な multiplicative inverse（exp(-B)）  
   - `dagger()`：**逆元 + dual map（右×i）による再解釈**（$\hat{p} \to -\hat{q}'$）

2. 通常／双対ベクトル抽出関数  
3. ベルソル生成（usual / dual 別）  
4. **サンドイッチ積関数**（4パターン、dagger対応）  
5. $\Omega = R_{\rm usual}.dagger() \cdot R_{\rm dual}$ → $\log \Omega$ → $J$  
6. 粒子集合の並列更新

### 4. ベルソル（Versor / Rotor）とローレンツ変換

すべての変換は**サンドイッチ積**で表現されます：

$$
\tilde{X} = R_{\rm total}\, X\, R_{\rm total}^\dagger
$$

#### 4.1 $R^\dagger$ の厳密定義
二重時空理論では、**unit rotor $R$ に対して $R^\dagger$ は $R$ の逆元 $R^{-1}$ と完全に一致**します。  
具体的には、指数形式で生成子（bivector）の符号を反転させたものです。

- 通常ローターの場合：
  $$
  R_{\rm usual} = \exp\!\left( \frac{\phi}{2}\hat{p} \right) \quad \Rightarrow \quad
  R_{\rm usual}^\dagger = \exp\!\left( -\frac{\phi}{2}\hat{p} \right) = \cosh\frac{\phi}{2} - \hat{p}\sinh\frac{\phi}{2}
  $$
  （$\hat{p}^2 = +1$）

- 双対ローターの場合：
  $$
  R_{\rm dual} = \exp\!\left( \frac{\theta}{2}\hat{q} \right) \quad \Rightarrow \quad
  R_{\rm dual}^\dagger = \exp\!\left( -\frac{\theta}{2}\hat{q} \right) = \cos\frac{\theta}{2} - \hat{q}\sin\frac{\theta}{2}
  $$
  （$\hat{q}^2 = -1$）

**実装上のポイント**：
- 四元数／双四元数クラスに `dagger()` または `inverse()` メソッドを実装。
- 指数関数の引数の**符号を反転**するだけで計算可能（Taylor展開 or 公式使用）。
- 通常の四元数共役（`conjugate()` または `*`）とは**同一ではない**場合があるが、unit rotor では $R^\dagger = R^{-1}$ として動作します。論文では「standard conjugate-inverse」と呼んでいます。

#### 4.2 完全ローター
$$
R_{\rm total} = R_{\rm usual}\, R_{\rm dual}
$$
（生成子が可換なので因子化可能）

#### 4.3 サンドイッチ積の4パターン（重要！）
通常時空ベクトル $X$ と双対時空ベクトル $X'$ のそれぞれに対して、$R_{\rm usual}$ と $R_{\rm dual}$ を適用する4通りの組み合わせがあります。以下に表で整理しました。

| パターン | 対象ベクトル | 適用ローター | 変換式 | 物理的意味 |
|----------|--------------|--------------|--------|------------|
| (1) **標準** | 通常 $X$ | $R_{\rm usual}$ | $\tilde{X} = R_{\rm usual}\, X\, R_{\rm usual}^\dagger$ | 通常時空での純粋ローレンツ変換（ブースト・回転） |
| (2) | 通常 $X$ | $R_{\rm total}$ | $\tilde{X} = R_{\rm total}\, X\, R_{\rm total}^\dagger$ | 完全変換（通常＋双対効果を含む） |
| (3) | 双対 $X'$ | $R_{\rm dual}$ | $\tilde{X}' = R_{\rm dual}\, X'\, R_{\rm dual}^\dagger$ | 双対時空での純粋回転 |
| (4) **特徴的** | 双対 $X'$ | $R_{\rm usual}$ | $\tilde{X}' = R_{\rm usual}\, X'\, R_{\rm usual}^\dagger$ | **逆方向（retrograde）ブースト**（論文で強調） |

**実装Tips**：
- 通常時空の物理量（位置・速度など）は主にパターン(1)または(2)を使用。
- 双対時空成分を扱うときはパターン(3)または(4)。特に(4)は「双対時空に通常ブーストをかけると時間の矢印が逆向きに作用する」のが鍵。
- 標準的な四元数共役を使った場合：
  $$
  \tilde{X} = R_{\rm usual}\, X\, R_{\rm usual}^*
  $$
  はパターン(1)と**完全に一致**します（unit rotor では $R_{\rm usual}^* = R_{\rm usual}^\dagger$）。  
  したがって、既存の四元数ライブラリを拡張するだけで容易に実装できます。

**完全ローターの場合**：
$$
R_{\rm total}^\dagger = R_{\rm dual}^\dagger\, R_{\rm usual}^\dagger
$$
（逆順に注意）

これにより、**1回のサンドイッチ積で両時空を同時に変換**できます。

すべての変換は**サンドイッチ積**で表現：
$$
\tilde{X} = R_{\rm total}\, X\, R_{\rm total}^\dagger
$$

**完全ローター**：
$$
R_{\rm total} = R_{\rm usual}\, R_{\rm dual}
$$
$$
R_{\rm total} = \exp\!\left( \frac{\phi}{2}\hat{p} + \frac{\theta}{2}\hat{q} \right)
$$
- $\hat{p} = p_1 iI + p_2 iJ + p_3 iK$　（$\hat{p}^2 = +1$：通常時空のブースト）
- $\hat{q} = q_1 I + q_2 J + q_3 K$　（$\hat{q}^2 = -1$：双対時空の回転）

**指数関数展開**：
$$
R_{\rm usual} = \cosh\frac{\phi}{2} + \hat{p}\sinh\frac{\phi}{2}
$$
$$
R_{\rm dual} = \cos\frac{\theta}{2} + \hat{q}\sin\frac{\theta}{2}
$$

**逆**：$R^\dagger$ は符号を反転させた指数（双対写像で $\hat{p}i = -\hat{q}'$ となる）。

**実装のポイント**：
- 双四元数の指数関数は **Taylor展開** または **クォータニオン指数公式** を拡張して使用（双四元数も可換部分で分離可能）。
- ローレンツ変換は行列を使わず、**純粋代数演算**のみで完結（高速）。

---

### 5. ねじれ不整合（Torsional Mismatch）と重力スカラー J

**重力の本質**：粒子内在の通常ローターと双対ローターの**相対角度**。

**ねじれ不整合ローター**：
$$
\Omega = R_{\rm usual}^\dagger R_{\rm dual}
$$

**ねじれバイベクトル**：
$$
\Omega_{\rm biv} = \log \Omega \in \mathfrak{so}(3,1) \oplus \mathfrak{so}(3,1)
$$

**Killing形式による不変スカラー**（これが重力の源）：
$$
J = \frac{1}{16} B(\Omega_{\rm biv}, \Omega_{\rm biv}) = \frac{1}{2} \sum_{a=1}^3 (\alpha_a^2 - \beta_a^2)
$$
（$\alpha_a$: 通常ブースト成分、$\beta_a$: 双対回転成分）

**物理的意味**：
- $J = 0$　→　自由落下（完全同期）
- $J > 0$　→　通常ブースト支配 → 引力（GRの通常重力）
- $J < 0$　→　双対回転支配 → 斥力（核力・反重力層）

**作用積分**：
$$
S = \frac{c^4}{16\pi G} \int J \, d^4x
$$
これは **Teleparallel Equivalent of GR (TEGR)** と完全に等価。GRの全解（Schwarzschild, Kerr, FLRW, 重力波など）が再現されます。

**シミュレーションでの使い方**：
- 各粒子の状態：$(R_{\rm usual}, R_{\rm dual})$（または $\phi_a, \theta_a$)
- 相互作用：近傍粒子のローターから $\Omega$ を計算 → $J$ → 力として加速度更新
- 慣性力も同一メカニズム（加速で $R_{\rm usual}$ が変化しても $R_{\rm dual}$ が追従しきれない）

---

### 6. 基本的な粒子内力学（Dual-Rotor Dynamics）

粒子内在の有効作用（非相対論的近似）：
$$
S_{\rm particle} = \int \left[ \frac12 \sum_a \dot{\phi}_a^2 - \frac12 \sum_a \dot{\theta}_a^2 - \frac{m}{2} \sum_a (\phi_a - \theta_a)^2 \right] dt
$$

**運動方程式**（Euler-Lagrange）：
$$
\ddot{\phi}_a = m (\phi_a - \theta_a), \quad -\ddot{\theta}_a = m (\phi_a - \theta_a)
$$
$$
\ddot{\delta\phi}_a + m \delta\phi_a = 0 \quad (\delta\phi_a = \phi_a - \theta_a)
$$

**解**：単振動（Compton周波数 $\sqrt{m}$)。低エネルギーでは de Broglie 位相として量子波動が出現。

**双対時空の離散化**（量子重力対応）：
$$
\theta_a \in \frac{2\pi}{N}\mathbb{Z} \quad (N \text{は巨大素数})
$$
通常時空の $\phi_a$ は連続のまま。これにより特異点は自然に除去され、Diophantine制約で有限性が保証されます。

**実装推奨**：
- 各粒子に `Biquaternion` クラス（usual_rotor, dual_rotor）
- 時間発展：4次Runge-Kutta または 指数マップでローター更新
- 力計算：近傍粒子との $\Omega$ → $J$ → 力ベクトル（Killing formの勾配）
- スケール不変性を利用して、核スケール・銀河スケール・恒星スケールを同一コードで扱える

---

### 7. 実装チェックリスト（プログラマ用）

1. 双四元数クラス（加減乗除、共役、ノルム、指数・対数）
2. 通常／双対ベクトル抽出関数
3. ベルソル生成（$\hat{p}$, $\hat{q}$ 単位化 → exp）
4. サンドイッチ積関数
5. $\Omega = R_{\rm usual}^\dagger R_{\rm dual}$、$\log \Omega$、Killing form $J$
6. 粒子集合の並列更新（N-body）
7. 可視化：$J$ の符号で引力／斥力層を色分け

これで**重力・慣性・核力・電子殻・回転曲線・時間遅れ**まですべて再現可能です。

---

**参考**：  
- 完全乗法表・詳細証明は `double-spacetime-theory.tex` https://github.com/hypernumbernet/dual-spacetime-doc/ を参照  
