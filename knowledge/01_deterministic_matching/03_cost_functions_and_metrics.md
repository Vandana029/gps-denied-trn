# Chapter 1.3: The Fingerprint Comparator — MAD, MSD, and Correlation Metrics

> **Phase**: 1 — Deterministic Matching  
> **Subphase**: 1.3 — MAD & MSD Cost Functions & Metrics  
> **Goal**: Master the mathematical cost functions used to compare a live 1D measured terrain elevation profile against candidate terrain slices from a Digital Elevation Model (DEM), analyze their sensitivity to sensor noise and vertical atmospheric bias, and establish the mathematical invariants required for robust matching.

---

## 🏔️ 1. The Story: The Search for the True Valley

Imagine your autonomous reconnaissance aircraft has just flown a $3\,\text{km}$ track over an uncharted mountainous zone in a GPS-denied corridor. Onboard memory holds a 1D tape of 101 elevation readings taken at $30\,\text{m}$ intervals:

$$\mathbf{h}_{\text{meas}} = [h_0, h_1, h_2, \dots, h_{100}]^T$$

The aircraft knows it is somewhere inside a $10 \times 10\,\text{km}$ search sector. Onboard, the flight computer has a high-resolution DEM of that sector. 

If we hypothetically test a thousand different candidate flight lines across this map, each candidate flight line yields a reference elevation profile:

$$\mathbf{h}_{\text{ref}} = [h_{\text{ref}, 0}, h_{\text{ref}, 1}, \dots, h_{\text{ref}, 100}]^T$$

Now comes the moment of truth for the guidance computer:
> *How do we numerically evaluate how closely $\mathbf{h}_{\text{meas}}$ matches $\mathbf{h}_{\text{ref}}$?*

If our comparison metric is too sensitive to occasional sensor noise spikes (e.g., radar pings bouncing off a high-voltage power transmission tower or tree canopy), the vehicle picks the wrong valley.

Even worse: if a regional weather front causes the barometric pressure to drop, the vehicle's measured elevation might be uniformly shifted upward by $+20\,\text{meters}$. If our comparison metric cannot handle this vertical bias, **the true physical position might produce a terrible score, and the aircraft will localize to an entirely wrong mountain range.**

To solve this, we must build a rigorous mathematical toolkit of **Profile Cost Functions and Similarity Metrics**.

---

## 🧮 2. Mathematical Formulations of Matching Metrics

Let:
- $\mathbf{h}_{\text{meas}} \in \mathbb{R}^N$ be the measured terrain profile vector.
- $\mathbf{h}_{\text{ref}} \in \mathbb{R}^N$ be a candidate terrain profile sampled from the DEM along a candidate trajectory.
- $N$ be the number of sample points in the profile.
- $e_i = h_{\text{meas}, i} - h_{\text{ref}, i}$ be the elevation residual at the $i$-th sample point.

```text
Elevation
   ▲
   │        Measured Profile h_meas
   │         /───\          /\
   │        /     \   /\   /  \
   │       /       \_/  \_/    \
   │      :  |   |   |   |   |  :
   │      :  e0  e1  e2  e3  e4 :  ◄── Residuals e_i = h_meas[i] - h_ref[i]
   │      :  |   |   |   |   |  :
   │     /───\          /\      :
   │    /     \   /\   /  \     :
   │   /       \_/  \_/    \____:
   │  Candidate Reference Profile h_ref
   └───────────────────────────────────────► Along-Track Index i
```

---

### 2.1 Mean Absolute Difference (MAD) — The $L_1$ Norm

The **Mean Absolute Difference (MAD)** computes the average absolute distance between the two profiles:

$$\text{MAD}(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}) = \frac{1}{N} \sum_{i=0}^{N-1} |h_{\text{meas}, i} - h_{\text{ref}, i}| = \frac{1}{N} \|\mathbf{h}_{\text{meas}} - \mathbf{h}_{\text{ref}}\|_1$$

#### Physical & Mathematical Properties:
1. **Units**: Meters ($\text{m}$).
2. **Global Minimum**: $\text{MAD} \ge 0$, with $\text{MAD} = 0$ if and only if $\mathbf{h}_{\text{meas}} \equiv \mathbf{h}_{\text{ref}}$ (identical match in a noiseless world).
3. **Robustness to Outliers**: Because the error penalty grows linearly ($|e_i|$), a single spurious altimeter glitch (e.g. radar pinging a crane or water tower: $e_{k} = 50\,\text{m}$) does not dominate the entire sum. Its contribution is simply $50 / N$.
4. **Optimization Geometry**: The $L_1$ norm has a sharp, V-shaped minimum at zero error, but its derivative is non-smooth (subgradient) at $e_i = 0$.

---

### 2.2 Mean Squared Difference (MSD) — The $L_2^2$ Norm

The **Mean Squared Difference (MSD)** computes the average squared residual:

$$\text{MSD}(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}) = \frac{1}{N} \sum_{i=0}^{N-1} (h_{\text{meas}, i} - h_{\text{ref}, i})^2 = \frac{1}{N} \|\mathbf{h}_{\text{meas}} - \mathbf{h}_{\text{ref}}\|_2^2$$

#### Root Mean Squared Error (RMSE):
To restore physical units of meters:
$$\text{RMSE}(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}) = \sqrt{\text{MSD}(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}})}$$

#### Physical & Mathematical Properties:
1. **Connection to Maximum Likelihood Estimation (MLE)**:  
   If altimeter measurement noise is zero-mean additive white Gaussian noise $v_i \sim \mathcal{N}(0, \sigma^2)$, the joint probability density (likelihood) of observing $\mathbf{h}_{\text{meas}}$ given candidate reference $\mathbf{h}_{\text{ref}}$ is:
   $$P(\mathbf{h}_{\text{meas}} \mid \mathbf{h}_{\text{ref}}) = \prod_{i=0}^{N-1} \frac{1}{\sqrt{2\pi\sigma^2}} \exp\left( -\frac{(h_{\text{meas}, i} - h_{\text{ref}, i})^2}{2\sigma^2} \right)$$
   Taking the negative log-likelihood:
   $$-\ln P(\mathbf{h}_{\text{meas}} \mid \mathbf{h}_{\text{ref}}) = \frac{N}{2} \ln(2\pi\sigma^2) + \frac{1}{2\sigma^2} \sum_{i=0}^{N-1} (h_{\text{meas}, i} - h_{\text{ref}, i})^2$$
   Notice that maximizing likelihood is **strictly mathematically equivalent to minimizing MSD**:
   $$\arg\max_{\mathbf{x}} P(\mathbf{h}_{\text{meas}} \mid \mathbf{h}_{\text{ref}}(\mathbf{x})) \equiv \arg\min_{\mathbf{x}} \text{MSD}(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}(\mathbf{x}))$$
2. **Sensitivity to Outliers**: Because errors are squared ($e_i^2$), an outlier of $50\,\text{m}$ contributes $2500 / N$ to the cost! A single faulty reading can heavily distort the correlation surface.
3. **Smoothness**: MSD is smooth and infinitely differentiable ($C^\infty$), making it ideal for gradient-based descent algorithms and Kalman filter measurement updates.

---

## ⚡ 3. The Fatal Vulnerability: The Barometric Bias Problem

In Chapter 1.2, we discovered that barometric altimeters drift with atmospheric weather fronts. 

Suppose the true aircraft clearance over ground is $h_{\text{true}}(t)$, but due to a regional pressure drop, the barometer reads $+15\,\text{m}$ high:
$$h_{\text{meas}, i} = h_{\text{true}, i} + b, \quad \text{where } b = +15\,\text{m}$$

Now evaluate what happens when we compare $\mathbf{h}_{\text{meas}}$ against the **100% physically correct ground-truth path** $\mathbf{h}_{\text{ref}} = \mathbf{h}_{\text{true}}$:

$$\text{MAD}_{\text{true}} = \frac{1}{N} \sum_{i=0}^{N-1} |(h_{\text{true}, i} + 15) - h_{\text{true}, i}| = \frac{1}{N} \sum_{i=0}^{N-1} 15 = 15\,\text{meters!}$$

$$\text{MSD}_{\text{true}} = \frac{1}{N} \sum_{i=0}^{N-1} 15^2 = 225\,\text{meters}^2!$$

Now imagine another candidate flight path $2\,\text{km}$ away that happens to fly over an elevated plateau where the average terrain elevation is naturally $15\,\text{m}$ higher than our true valley, but has relatively flat contours. 

Naive MAD and MSD might score the wrong plateau **lower** than the true valley simply because the mean elevation matches the biased reading!

> **Core GNC Axiom**: In terrain matching, absolute elevation numbers are untrustworthy due to barometric drift. **Only the relative shape (the profile slope and variance) contains true geographic information.**

---

## 🛡️ 4. Bias-Invariant Cost Functions

To conquer barometric drift, we introduce **Zero-Mean Metrics**.

### 4.1 Mean-Centered Profile Vectors
We define the sample mean of a profile vector:
$$\mu_{\text{meas}} = \frac{1}{N} \sum_{i=0}^{N-1} h_{\text{meas}, i}, \qquad \mu_{\text{ref}} = \frac{1}{N} \sum_{i=0}^{N-1} h_{\text{ref}, i}$$

The zero-mean (mean-centered) profiles are:
$$\tilde{\mathbf{h}}_{\text{meas}} = \mathbf{h}_{\text{meas}} - \mu_{\text{meas}} \mathbf{1}, \qquad \tilde{\mathbf{h}}_{\text{ref}} = \mathbf{h}_{\text{ref}} - \mu_{\text{ref}} \mathbf{1}$$

### 4.2 Zero-Mean Mean Squared Difference (ZMSD)
$$\text{ZMSD}(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}) = \frac{1}{N} \sum_{i=0}^{N-1} \left( (h_{\text{meas}, i} - \mu_{\text{meas}}) - (h_{\text{ref}, i} - \mu_{\text{ref}}) \right)^2$$

#### 🔬 Mathematical Proof of Bias Invariance:
Let $h_{\text{meas}, i} = h_{\text{true}, i} + b$, where $b \in \mathbb{R}$ is any arbitrary constant bias.
Then:
$$\mu_{\text{meas}} = \frac{1}{N} \sum_{i=0}^{N-1} (h_{\text{true}, i} + b) = \left(\frac{1}{N} \sum_{i=0}^{N-1} h_{\text{true}, i}\right) + b = \mu_{\text{true}} + b$$

Substituting into the zero-mean term:
$$\tilde{h}_{\text{meas}, i} = h_{\text{meas}, i} - \mu_{\text{meas}} = (h_{\text{true}, i} + b) - (\mu_{\text{true}} + b) = h_{\text{true}, i} - \mu_{\text{true}} \equiv \tilde{h}_{\text{true}, i}$$

The bias $b$ cancels out with mathematical precision!
$$\text{ZMSD}(\mathbf{h}_{\text{true}} + b, \mathbf{h}_{\text{true}}) = 0.0$$

Whether the barometer is biased by $+10\,\text{m}$, $-50\,\text{m}$, or $+100\,\text{m}$, ZMSD evaluates purely the geometric undulations of the terrain.

---

### 4.3 Zero-Mean Mean Absolute Difference (ZMAD)
Similarly, the zero-mean $L_1$ metric is:
$$\text{ZMAD}(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}) = \frac{1}{N} \sum_{i=0}^{N-1} |(h_{\text{meas}, i} - \mu_{\text{meas}}) - (h_{\text{ref}, i} - \mu_{\text{ref}})|$$

Combines the outlier robustness of $L_1$ with the bias-immunity of mean centering.

---

## 📈 5. Cross-Correlation Metrics

Rather than measuring difference (distance metrics where $0$ is best), we can measure **correlation** (similarity metrics where $+1$ is best).

### 5.1 Normalized Cross-Correlation (NCC)
$$\text{NCC}(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}) = \frac{\mathbf{h}_{\text{meas}}^T \mathbf{h}_{\text{ref}}}{\|\mathbf{h}_{\text{meas}}\|_2 \|\mathbf{h}_{\text{ref}}\|_2} = \frac{\sum_{i=0}^{N-1} h_{\text{meas}, i} h_{\text{ref}, i}}{\sqrt{\sum_{i=0}^{N-1} h_{\text{meas}, i}^2} \sqrt{\sum_{i=0}^{N-1} h_{\text{ref}, i}^2}}$$

By the Cauchy-Schwarz inequality, $\text{NCC} \in [-1, 1]$.
- $\text{NCC} = +1$: The vectors are strictly collinear ($\mathbf{h}_{\text{meas}} = \alpha \mathbf{h}_{\text{ref}}$ with $\alpha > 0$).
- $\text{NCC} = 0$: Orthogonal profiles (uncorrelated).
- $\text{NCC} = -1$: Inverted profiles.

⚠️ **Limitation**: Standard NCC handles scale multiplication, but is **not** invariant to additive vertical bias ($+b$).

---

### 5.2 Zero-Mean Normalized Cross-Correlation (ZNCC) / Pearson Correlation
The gold standard of classical cross-correlation:

$$\text{ZNCC}(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}) = \frac{\sum_{i=0}^{N-1} (h_{\text{meas}, i} - \mu_{\text{meas}})(h_{\text{ref}, i} - \mu_{\text{ref}})}{\sqrt{\sum_{i=0}^{N-1} (h_{\text{meas}, i} - \mu_{\text{meas}})^2} \sqrt{\sum_{i=0}^{N-1} (h_{\text{ref}, i} - \mu_{\text{ref}})^2}}$$

Using vector notation:
$$\text{ZNCC} = \frac{\tilde{\mathbf{h}}_{\text{meas}}^T \tilde{\mathbf{h}}_{\text{ref}}}{\|\tilde{\mathbf{h}}_{\text{meas}}\|_2 \|\tilde{\mathbf{h}}_{\text{ref}}\|_2}$$

#### 💎 The Dual Invariance Theorem:
ZNCC is invariant under **any affine transformation**:
$$\mathbf{h}' = \alpha \mathbf{h} + \beta, \quad \text{for any } \alpha > 0, \beta \in \mathbb{R}$$
- Immune to barometric bias $\beta$.
- Immune to linear altimeter scaling error $\alpha$.
- Perfectly bounded in the intuitive range $[-1.0, +1.0]$.

#### ⚠️ The Flat-Terrain Singularity (Division by Zero):
If an aircraft flies over a completely flat plain (e.g. calm lake or salt flat):
$$h_i = \text{const} \implies \tilde{h}_i = 0 \implies \|\tilde{\mathbf{h}}\|_2 = 0$$
The denominator becomes $0 \cdot 0 = 0$!
- **Physical Meaning**: A flat plain has **zero terrain information content**. It is mathematically impossible to localize on a flat sheet because all points look identical.
- **Avionics Safeguard**: In code, we must enforce a variance threshold ($\sigma_h^2 < \epsilon$): if terrain variance is near zero, reject ZNCC correlation or return $0.0$ and raise an insufficient roughness flag!

---

## 📊 6. Comprehensive Metric Comparison Matrix

| Metric | Formula Type | Range | Best Score | Outlier Sensitivity | Immune to Bias $+b$? | Immune to Scale $\times \alpha$? | Primary Aerospace Application |
|---|---|---|---|---|---|---|---|
| **MAD** | Difference ($L_1$) | $[0, \infty)$ | $0.0$ | Low (Robust) | ❌ No | ❌ No | Fast baseline search, noisy sensors |
| **MSD** | Difference ($L_2^2$) | $[0, \infty)$ | $0.0$ | High | ❌ No | ❌ No | Optimal under Gaussian noise (MLE) |
| **RMSE** | Difference ($L_2$) | $[0, \infty)$ | $0.0$ | Moderate | ❌ No | ❌ No | Human-interpretable physical residual (meters) |
| **ZMAD** | Zero-Mean $L_1$ | $[0, \infty)$ | $0.0$ | Low (Robust) | ✅ Yes | ❌ No | Real-world altimeter matching with bias |
| **ZMSD** | Zero-Mean $L_2^2$ | $[0, \infty)$ | $0.0$ | Moderate | ✅ Yes | ❌ No | TERCOM difference matching |
| **NCC** | Correlation | $[-1, 1]$ | $+1.0$ | Moderate | ❌ No | ✅ Yes | Optical / Radar intensity matching |
| **ZNCC** | Normalized Pearson | $[-1, 1]$ | $+1.0$ | Moderate | ✅ Yes | ✅ Yes | Classical TERCOM correlation standard |

---

## 🏛️ 7. Software Architecture Blueprint for Subphase 1.3

We will create a clean, purely vectorized module in `src/terrain_matching/core/metrics.py`:

```python
"""Terrain profile matching metrics and cost functions.

Provides robust, vectorized distance and similarity metrics between 1D elevation
profiles, supporting both raw and bias-invariant matching.
"""

def mean_absolute_difference(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute MAD (L1 cost) between two elevation profiles."""

def mean_squared_difference(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute MSD (L2^2 cost) between two elevation profiles."""

def root_mean_squared_difference(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute RMSE in meters between two elevation profiles."""

def zero_mean_msd(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute bias-invariant Zero-Mean MSD (ZMSD)."""

def zero_mean_mad(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute bias-invariant Zero-Mean MAD (ZMAD)."""

def normalized_cross_correlation(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute Normalized Cross-Correlation (NCC) in [-1, 1]."""

def zero_mean_normalized_cross_correlation(
    h_meas: np.ndarray, 
    h_ref: np.ndarray,
    eps: float = 1e-9
) -> float:
    """Compute Zero-Mean Normalized Cross-Correlation (ZNCC) in [-1, 1]."""
```

---

## 💡 8. Pedagogical Check & Conceptual Walkthrough (Step 2 Solutions)

Here are the mathematical solutions and avionics takeaways for our key conceptual scenarios:

### 1. The Weather Front Scenario (+30 m Atmospheric Shift)
**Question**: Suppose our drone flies over rolling terrain ($100\,\text{m}$ to $300\,\text{m}$). A sudden atmospheric cold front drops barometric pressure, causing the altimeter to uniformly overestimate ground elevation by $+30\,\text{m}$ across all samples ($h_{\text{meas}, i} = h_{\text{true}, i} + 30$). What happens to MSD, ZMSD, and ZNCC when evaluated against the true DEM track ($\mathbf{h}_{\text{ref}} = \mathbf{h}_{\text{true}}$)?

- **MSD Calculation**:
  The residual at every point is $e_i = (h_{\text{true}, i} + 30) - h_{\text{true}, i} = 30\,\text{m}$.
  $$\text{MSD} = \frac{1}{N} \sum_{i=0}^{N-1} (30)^2 = \frac{1}{N} \cdot N \cdot 900 = \mathbf{900.0\,\text{m}^2} \quad (\text{RMSE} = 30.0\,\text{m})$$
  *Takeaway*: Naive MSD reports an enormous error of $900\,\text{m}^2$ despite evaluating the 100% correct geographic location! If a distant incorrect track happens to sit on a flat plateau that is naturally $30\,\text{m}$ higher, raw MSD will falsely select the wrong location.
- **ZMSD Calculation**:
  The sample means are $\mu_{\text{meas}} = \mu_{\text{true}} + 30$ and $\mu_{\text{ref}} = \mu_{\text{true}}$.
  The mean-centered profiles are $\tilde{h}_{\text{meas}, i} = (h_{\text{true}, i} + 30) - (\mu_{\text{true}} + 30) = h_{\text{true}, i} - \mu_{\text{true}} = \tilde{h}_{\text{true}, i}$.
  $$\text{ZMSD} = \frac{1}{N} \sum_{i=0}^{N-1} (\tilde{h}_{\text{meas}, i} - \tilde{h}_{\text{ref}, i})^2 = \frac{1}{N} \sum_{i=0}^{N-1} 0^2 = \mathbf{0.0\,\text{m}^2}$$
- **ZNCC Calculation**:
  Since $\tilde{\mathbf{h}}_{\text{meas}} \equiv \tilde{\mathbf{h}}_{\text{ref}}$, the vectors are collinear:
  $$\text{ZNCC} = \frac{\|\tilde{\mathbf{h}}_{\text{true}}\|_2^2}{\|\tilde{\mathbf{h}}_{\text{true}}\|_2 \|\tilde{\mathbf{h}}_{\text{true}}\|_2} = \mathbf{+1.0}$$
  *Takeaway*: Both ZMSD ($0.0$) and ZNCC ($+1.0$) completely reject constant vertical pressure bias and accurately identify the true track based on topographic shape.

---

### 2. The Outlier Trap (Radar Spike Sensitivity)
**Question**: A 100-point profile matches the DEM perfectly ($e_i = 0\,\text{m}$) across 99 points, but at sample $k=50$, the radar altimeter strikes a guy-wire or mast, reporting a clearance spike with residual $e_{50} = 100\,\text{m}$. What are the MAD, MSD, and RMSE scores, and why does $L_2^2$ punish this glitch more severely?

- **MAD Calculation**:
  $$\text{MAD} = \frac{1}{100} \left[ (99 \times 0) + |100| \right] = \frac{100}{100} = \mathbf{1.0\,\text{m}}$$
- **MSD & RMSE Calculation**:
  $$\text{MSD} = \frac{1}{100} \left[ (99 \times 0^2) + (100)^2 \right] = \frac{10,000}{100} = \mathbf{100.0\,\text{m}^2}$$
  $$\text{RMSE} = \sqrt{\text{MSD}} = \sqrt{100.0} = \mathbf{10.0\,\text{m}}$$
- **Mathematical & Physical Insight**:
  $L_1$ has a linear cost function $f(e) = |e|$, giving the $100\,\text{m}$ spike exactly $100\times$ the weight of a $1\,\text{m}$ error. $L_2^2$ has a quadratic cost function $f(e) = e^2$, giving the same spike $10,000\times$ the penalty! Consequently, $\text{RMSE} = 10.0\,\text{m}$ is $10\times$ larger than $\text{MAD} = 1.0\,\text{m}$. For terrains with occasional canopy or man-made clutter, MAD provides significantly better outlier resilience.

---

### 3. The Salt Flat Hazard (Singularity & Avionics Safeguards)
**Question**: If an aircraft flies over the Bonneville Salt Flats where terrain elevation is constant ($h_i = c$), why does ZNCC fail, and what must our software do?

- **Mathematical Singularity**:
  Mean centering a constant profile yields $\mu = c \implies \tilde{h}_i = c - c = 0$ for all $i$.
  The norm is $\|\tilde{\mathbf{h}}\|_2 = \sqrt{\sum 0^2} = 0.0$.
  $$\text{ZNCC} = \frac{\tilde{\mathbf{h}}_{\text{meas}}^T \tilde{\mathbf{h}}_{\text{ref}}}{\|\tilde{\mathbf{h}}_{\text{meas}}\|_2 \|\tilde{\mathbf{h}}_{\text{ref}}\|_2} = \frac{0}{0 \cdot \|\tilde{\mathbf{h}}_{\text{ref}}\|_2} = \frac{0}{0} \implies \mathbf{NaN \quad (\text{Division by Zero})}$$
- **Physical Meaning**: A flat plain has **zero terrain information content (roughness)**. Every location looks identical, making terrain-aided localization mathematically impossible.
- **Avionics Software Safeguards**:
  1. **Epsilon / Variance Guard**: Before computing the denominator, check if profile sample variance $\sigma^2 = \frac{1}{N} \|\tilde{\mathbf{h}}\|_2^2 < \epsilon$ (e.g. $\epsilon = 10^{-6}\,\text{m}^2$). If flat, bypass division and safely return $0.0$ (uncorrelated).
  2. **Roughness Gating**: In the flight navigation loop, flag regions of low terrain roughness ($\sigma_h < \sigma_{\text{threshold}}$) as unmatchable, preventing invalid fixes from corrupting the Kalman filter or INS dead reckoning.

---

## 📚 9. Curated Aerospace Literature

1. 📄 **Golden, J. P. (1980)**: *"Terrain Contour Matching (TERCOM) Applications"*. SPIE Image Processing for Missile Guidance, Vol. 238, pp. 10-18.  
   *(The definitive declassified paper detailing the MAD vs MSD vs Mean-Centered correlation algorithms evaluated for the AGM-86 ALCM and BGM-109 Tomahawk).*
2. 📄 **Johnson, C. W. (1991)**: *"TERCOM: A Technical Analysis and History"*. Naval Surface Warfare Center Technical Report.  
   *(Analyzes the mathematical impact of barometric vertical bias and seasonal foliage variations on correlation cost surfaces).*
3. 📖 **Kayton, M. & Fried, W. R. (1997)**: *"Avionics Navigation Systems"*, 2nd Ed.  
   *Chapter 6 & Chapter 15: Cross-correlation processing, peak-to-sidelobe ratio (PSR), and false-fix rejection criteria.*
