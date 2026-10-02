# Chapter 1.5: The Mirage of the Mountains — Ambiguity Analysis & Correlation Surfaces

> **Phase**: 1 — Deterministic Matching  
> **Subphase**: 1.5 — Ambiguity Analysis & Correlation Surfaces  
> **Goal**: Master the topography of 2D correlation cost surfaces, formulate quantitative metrics to detect false fixes and multi-modal ambiguities (Peak-to-Sidelobe Ratio, Peak-to-Second-Peak Ratio, and Hessian curvature), and construct the avionics fix-rejection filter that safeguards aircraft against erroneous terrain updates.

---

## 🏔️ 1. The Story: The Cruise Missile That Followed a Ghost

### 💡 In Simple Terms: The Fatal Mirage
Imagine hiking through a desert of repetitive, wind-sculpted sand dunes. Every single ridge looks nearly identical to the ridge behind it and the ridge in front of it. 
If your GPS was jammed and you tried to recognize a specific dune from a photograph, you might find **five different dunes that match your photo almost perfectly**.

Now imagine a tactical cruise missile flying at 800 km/h in thick fog. Its radar altimeter reads a terrain profile. The computer scans the map and finds:
- Candidate Location A has a cost of $0.5\,\text{meters}$.
- Candidate Location B (just $800\,\text{meters}$ to the left) has a cost of $0.6\,\text{meters}$.

Location A is chosen by naive software because $0.5 < 0.6$. But in reality, Location B was the true path, and a tiny gust of wind or sensor noise nudged Location A's score slightly ahead. 

The missile resets its navigation computer believing it is at Location A. Correcting for this $800\,\text{meter}$ false fix, the flight control system veers hard into a granite ridge.

This catastrophic failure mode is called a **False Fix** or **Ambiguity Trap**. To prevent this, every military and aerospace terrain navigation system must include an **Ambiguity Analyzer**: an algorithm that answers:
> *"Is this position fix a sharp, unmistakable, solitary needle — or is it surrounded by dangerous competitor peaks?"*

---

### 🚀 The Operational Problem:
When the matching engine (from Chapter 1.4) computes a 2D cost surface $\mathbf{J}(y_c, x_c)$, it produces a complete 2D landscape of matching scores. 

Simply finding the minimum $(x^*, y^*) = \arg\min \mathbf{J}$ is **necessary, but never sufficient for safety**. 

Before the flight computer allows a terrain fix to update the flight guidance trajectory, it must subject the correlation surface to rigorous statistical and geometric scrutiny.

```text
       UNAMBIGUOUS FIX (Sharp Needle)             AMBIGUOUS FIX (Twin Valleys / Rivals)
       
              Cost Surface J                              Cost Surface J
                   ▲                                           ▲
                   │   \               /                       │   \       /   \       /
                   │    \             /                        │    \     /     \     /
                   │     \     *     /                         │     \ * /       \ o /
                   │      \   / \   /                          │      \ /         \ /
                   └───────\─/───\─/──────► Coord              └───────V───────────V──────► Coord
                        Solitary Minimum                             Global     Dangerous
                           (Accepted)                               Minimum     Rival Peak
                                                                   (REJECTED AS AMBIGUOUS!)
```

---

## 🧩 2. Real-World Analogy: The Neon Jacket in a Stadium

### 💡 In Simple Terms:
- **Case 1 (High Confidence / Unambiguous)**:  
  You are looking for a lost friend in a football stadium of 50,000 spectators wearing black coats. Your friend is wearing an electric-neon green jacket. You scan the crowd and immediately spot a single, brilliant point of green. The contrast between your friend (the peak) and the background crowd (the sidelobes) is massive. You are 100% confident.
- **Case 2 (Ambiguous / Multi-Modal)**:  
  Now imagine you are looking for your friend in the student section of a university match where 5,000 students are all wearing identical red and white striped jerseys. You spot 10 different people who match the description. Picking one at random has a 90% chance of being catastrophic. An intelligent person says: *"I cannot be sure. I will not make a decision until I get more distinctive information."*

---

## 📐 3. The Topography of Correlation Surfaces

### 💡 In Simple Terms: What Does the Error Landscape Look Like?
When you plot the matching error across a 2D grid of $(X, Y)$ coordinates, the resulting 3D map looks like a physical terrain of its own:
- High errors look like tall mountain walls (bad candidate locations).
- Low errors look like bowls, valleys, or trenches (good candidate locations).
- The exact shape of this error bowl tells us everything about how well the terrain can guide us!

---

### 🔬 The Technical Typology of Cost Surfaces:

```text
  1. ISOTROPIC ROUGH TERRAIN            2. LINEAR VALLEY / CANYON           3. REPETITIVE ROLLING DUNES
  (Isolated peak / distinct cone)       (Flat bottom along river line)      (Parallel sinusoidal ridges)

             ▲ North                               ▲ North                             ▲ North
             │                                     │                                   │
          ╭─────╮                               ═════════════                       ( ( ( ( ) ) ) )
         │   •   │  ◄── Sharp Bowl              ───── • ───── ◄── Trough             •   •   •   • ◄── Periodic
          ╰─────╯       (Strong in X & Y)       ═════════════     (Observ. in Y,     (Multi-modal
             └──────────────► East                 └──────────────► East but flat in X)   false peaks)
```

1. **Isotropic Rough Terrain (The Bullseye Bowl)**:
   - Formed over rugged, multi-directional topography (craters, isolated volcanic peaks, jagged foothills).
   - Cost contours are closed concentric circles or compact ellipses.
   - **Observability**: Complete 2D observability. The aircraft position is tightly bounded in both East ($X$) and North ($Y$).
2. **Linear Valleys & Canyons (The Degenerate Trough)**:
   - Formed when an aircraft flies along a long, uniform canyon, riverbed, or ridgeline.
   - Across the canyon (cross-track), the canyon walls rise steeply $\implies$ Error explodes rapidly (sharp curvature in cross-track).
   - Along the canyon floor (along-track), the elevation is flat and uniform $\implies$ Error remains near zero for hundreds of meters!
   - **Observability**: Strong cross-track localization, but **zero along-track localization**. The system can tell if it drifted left or right, but cannot tell how far forward it has traveled.
3. **Repetitive Topography (Multi-Modal Sinusoids)**:
   - Formed over parallel agricultural furrows, sand dunes, or uniform rolling hills.
   - Produces periodic waves of multiple local minima spaced at the terrain's spatial wavelength $\lambda$.
   - **Observability**: High risk of catastrophic cycle slip (locking onto a false neighbor).

---

## 🔍 4. Quantitative Ambiguity Metrics

To build a software filter, we must convert visual shapes into **hard numerical thresholds**.

---

### 4.1 Peak-to-Sidelobe Ratio (PSR)

#### 💡 In Simple Terms: How Tall is the Tower Above the Noise Floor?
Imagine a cell tower standing in a flat desert versus a tower standing in the middle of a forest of trees of almost equal height.
- If the tower is 100 meters tall and the surrounding ground is flat sand (mean height 0, standard deviation small), the tower sticks out like a sore thumb.
- **PSR measures how many standard deviations the best peak sticks out above the surrounding background terrain.**

---

#### 🔬 The Technical Formulation:
Let:
- $\mathbf{p}^* = (x^*, y^*)$ be the global optimum coordinate.
- $C^* = \mathbf{J}(y^*, x^*)$ be the optimal cost value.
- $R_{\text{excl}}$ be the **exclusion radius** (e.g. $100\,\text{m}$ to $200\,\text{m}$) surrounding the peak.
- $\mathcal{S}_{\text{sidelobe}} = \{ (x, y) \mid \|\mathbf{p} - \mathbf{p}^*\|_2 > R_{\text{excl}} \}$ be the **sidelobe region** (the rest of the search window).

We compute the mean and standard deviation of the sidelobe costs:
$$\mu_{\text{side}} = \frac{1}{|\mathcal{S}_{\text{side}}|} \sum_{\mathbf{p} \in \mathcal{S}_{\text{side}}} \mathbf{J}(\mathbf{p})$$
$$\sigma_{\text{side}} = \sqrt{\frac{1}{|\mathcal{S}_{\text{side}}|} \sum_{\mathbf{p} \in \mathcal{S}_{\text{side}}} \left( \mathbf{J}(\mathbf{p}) - \mu_{\text{side}} \right)^2}$$

#### A. For Similarity Metrics (ZNCC, where peak is a maximum):
$$\text{PSR}_{\text{ZNCC}} = \frac{C^* - \mu_{\text{side}}}{\sigma_{\text{side}} + \epsilon}$$

#### B. For Difference Metrics (MAD, MSD, where peak is a minimum valley):
We measure the **Valley-to-Sidelobe Ratio (VSR)**, looking at how deeply the valley plunges below the mean noise floor:
$$\text{PSR}_{\text{MAD}} = \frac{\mu_{\text{side}} - C^*}{\sigma_{\text{side}} + \epsilon}$$

- **Aerospace Standard**: A reliable terrain fix typically requires $\text{PSR} \ge 4.0 \text{ to } 6.0$ (the fix must be at least 4 to 6 standard deviations more prominent than background ambiguity).

---

### 4.2 Peak-to-Second-Peak Ratio (P2PR / Multi-Modal Ambiguity Ratio)

#### 💡 In Simple Terms: How Close is the Runner-Up?
In a race, if the gold medalist crosses the finish line 10 seconds ahead of the silver medalist, there is no dispute. But if the gold and silver medalists cross the line separated by 0.001 seconds, it is a photo-finish — a gust of wind could have changed the outcome!
- P2PR compares the winner against the second-best distinct candidate.
- If the runner-up is almost as good as the winner, we have a dangerous multi-modal ambiguity!

---

#### 🔬 The Technical Formulation:
We identify the **second-best distinct local optimum** outside the exclusion radius $R_{\text{excl}}$ of the global fix:
$$\mathbf{p}_{\text{second}}^* = \arg\min_{\mathbf{p} \in \mathcal{S}_{\text{side}}} \mathbf{J}(\mathbf{p}) \quad (\text{for MAD/MSD})$$
$$C_{\text{second}}^* = \mathbf{J}(\mathbf{p}_{\text{second}}^*)$$

We define the **Multi-Modal Ambiguity Ratio (MAR)**:
- **For Difference Metrics** (MAD / MSD / ZMSD):
  $$\text{MAR} = \frac{C_{\text{second}}^*}{C^* + \epsilon}$$
  - Since lower is better, $C_{\text{second}}^* > C^*$. 
  - If $\text{MAR} \approx 1.0$, the runner-up is nearly identical to the global fix $\implies$ **High Risk!**
  - If $\text{MAR} \ge 2.0$, the runner-up has twice the error of the best fix $\implies$ **Clear Winner!**
- **For Similarity Metrics** (ZNCC):
  $$\Delta C = C^* - C_{\text{second}}^*$$
  - If $\Delta C \le 0.05$, the two peaks have almost identical correlation $\implies$ **High Risk!**

---

### 4.3 Hessian Matrix & Curvature Analysis (Directional Sharpness)

#### 💡 In Simple Terms: Is the Hole a Round Bowl or a Long Trench?
Imagine placing a marble into a bowl.
- If the bowl is round like a cereal bowl, the marble rolls straight to the center from all directions.
- If the bowl is shaped like a half-pipe skateboard ramp, the marble rolls easily to the bottom of the ramp in one direction, but can slide freely along the flat groove in the other direction.
- The **Hessian matrix** measures the curvature (steepness) of the bowl in every direction.

---

#### 🔬 The Technical Formulation:
Around the optimal coordinate $\mathbf{p}^* = (x^*, y^*)$, we approximate the cost surface using a 2D second-order Taylor expansion:
$$\mathbf{J}(\mathbf{p}) \approx \mathbf{J}(\mathbf{p}^*) + \nabla \mathbf{J}^T (\mathbf{p} - \mathbf{p}^*) + \frac{1}{2} (\mathbf{p} - \mathbf{p}^*)^T \mathbf{H} (\mathbf{p} - \mathbf{p}^*)$$

At the local minimum, the gradient vanishes ($\nabla \mathbf{J} \approx \mathbf{0}$), leaving the curvature governed by the **Hessian Matrix**:
$$\mathbf{H} = \begin{bmatrix} \frac{\partial^2 \mathbf{J}}{\partial x^2} & \frac{\partial^2 \mathbf{J}}{\partial x \partial y} \\ \frac{\partial^2 \mathbf{J}}{\partial y \partial x} & \frac{\partial^2 \mathbf{J}}{\partial y^2} \end{bmatrix}$$

We compute the eigenvalues of $\mathbf{H}$: $\lambda_1 \ge \lambda_2 \ge 0$.
- $\lambda_1$ represents the curvature along the steepest direction.
- $\lambda_2$ represents the curvature along the shallowest direction.
- **Condition Number / Anisotropy**: $\kappa = \frac{\lambda_1}{\lambda_2 + \epsilon}$.
  - If $\kappa \approx 1.0$: Perfectly isotropic round bowl (equal accuracy in X and Y).
  - If $\kappa \gg 10.0$: Elongated canyon trench (strong cross-track fix, poor along-track fix).

*(Preview for Phase 2 & 3: The inverse Hessian $\mathbf{H}^{-1}$ is proportional to the **measurement error covariance matrix $\mathbf{R}_{\text{TRN}}$** passed into the Extended Kalman Filter!)*

---

## 🛡️ 5. The Avionics Fix-Rejection Filter

### 💡 In Simple Terms: The 4-Door Security Checkpoint
Before any terrain fix is allowed to adjust the aircraft's autopilot, it must pass through 4 security checkpoints:
1. **Did we fly over actual hills?** (Terrain roughness check: if flat like a runway, reject!).
2. **Is the best score actually good?** (Cost magnitude check: if the best score is still high error, reject!).
3. **Does the winner stand out above the background crowd?** (PSR check: must be $\ge 4.0\sigma$).
4. **Is the silver medalist far behind?** (Ambiguity margin: no photo-finishes allowed!).

If the fix fails **even one** checkpoint, the autopilot drops the fix, stays on safe dead reckoning, and waits for rougher, more distinctive mountains ahead.

---

### 🔬 The Complete GNC Acceptance Rule:

$$\text{ACCEPT FIX} \iff \begin{cases} 
\text{Var}(\mathbf{h}_{\text{meas}}) \ge \sigma_{h, \min}^2 & \text{(Sufficient terrain relief)} \\
C^* \le C_{\max} & \text{(Low residual error)} \\
\text{PSR} \ge \text{PSR}_{\min} & \text{(Prominent global peak)} \\
\text{MAR} \ge \text{MAR}_{\min} & \text{(No close rival peaks)}
\end{cases}$$

---

## 🏛️ 6. Software Architecture Blueprint for Subphase 1.5

We will create `src/terrain_matching/core/ambiguity.py`:

```python
"""Ambiguity analysis and correlation surface assessment for terrain matching.

Computes Peak-to-Sidelobe Ratio (PSR), multi-modal ambiguity margins,
and local surface curvature to validate position fix confidence.
"""

@dataclass(frozen=True)
class AmbiguityReport:
    """Quantitative assessment of correlation surface confidence and ambiguity.

    Attributes:
        is_fix_acceptable: True if all safety criteria are satisfied.
        rejection_reason: Explanation if rejected, or None if accepted.
        psr: Peak-to-Sidelobe Ratio (in standard deviations).
        second_best_coord: Coordinate of the strongest rival local minimum outside exclusion zone.
        second_best_cost: Cost value of the second-best candidate.
        ambiguity_ratio: Ratio between second-best and best cost (MAR).
        curvature_eigenvalues: Eigenvalues (lambda_1, lambda_2) of local Hessian matrix.
        anisotropy: Ratio lambda_1 / lambda_2 indicating directional elongation.
    """

class AmbiguityAnalyzer:
    """Evaluates correlation surfaces to accept or reject terrain-aided fixes."""

    def __init__(
        self,
        exclusion_radius_m: float = 60.0,
        min_psr: float = 4.0,
        min_ambiguity_ratio: float = 1.3,
        max_acceptable_cost: float = 5.0,
    ) -> None:
        ...

    def analyze(
        self,
        match_result: MatchResult,
        measured_profile_variance: float | None = None,
        min_profile_variance: float = 4.0,
    ) -> AmbiguityReport:
        """Perform comprehensive ambiguity analysis on a MatchResult."""
```

---

## 💡 7. Pedagogical Check & Conceptual Walkthrough (Step 2 Solutions)

Here are the detailed mathematical derivations and physical insights for our three pedagogical check questions:

---

### Question 1: The Exclusion Radius Mystery ($R_{\text{excl}}$)

#### 💡 In Simple Terms: Finding K2 Instead of Everest's Shoulder
Imagine you ask a computer: *"What is the second-highest point on Earth?"*
If the computer looks 2 inches to the left of the summit of Mount Everest, it finds a pebble that is 1 millimeter lower than the peak. Technically, it is lower. But that pebble is not a second mountain—it is still Mount Everest! 
To find the *true* second mountain (K2, hundreds of miles away in the Karakoram range), you must draw a big circle around Mount Everest and say: *"Ignore everything on Everest's slopes. Look outside this circle!"*

#### 🔬 The Technical Solution:
- **Why $R_{\text{excl}}$ is mandatory**:
  Because digital elevation models and cost surfaces $\mathbf{J}(x, y)$ are spatially continuous, the cost values at grid nodes immediately adjacent to the global minimum $(x^* \pm \delta x, y^* \pm \delta y)$ belong to the **same continuous potential well (the mainlobe)**.
- **What happens if $R_{\text{excl}} = 0$**:
  1. **False Rival**: The algorithm will pick an adjacent pixel on the slope of the main peak as $\mathbf{p}_{\text{second}}^*$. Since adjacent pixels have nearly identical costs ($C_{\text{second}}^* \approx C^* + \Delta \epsilon$), the ambiguity ratio collapses to:
     $$\text{MAR} = \frac{C_{\text{second}}^*}{C^*} \approx 1.0$$
     The filter would falsely flag **every single peak as ambiguous and reject 100% of valid fixes!**
  2. **Contaminated Sidelobes**: The steep walls of the main peak would be included in the sidelobe statistics, artificially inflating $\sigma_{\text{side}}$ and collapsing the Peak-to-Sidelobe Ratio ($\text{PSR}$).
- **Avionics Standard**: Setting $R_{\text{excl}} \approx 2 \text{ to } 3$ grid cells (or the terrain's spatial correlation length, e.g. $60\,\text{m} - 100\,\text{m}$) excludes the mainlobe and forces the algorithm to search for genuinely distinct, competing topological valleys.

---

### Question 2: The Riverbed Dilemma (Hessian Curvature)

#### 💡 In Simple Terms: Flying in a Half-Pipe
Imagine an aircraft flying deep inside the Grand Canyon.
- If the aircraft drifts just $20\,\text{meters}$ left or right (cross-track), it hits the sheer vertical canyon walls. The altimeter sees the ground instantly shoot up hundreds of meters. The computer screams: *"You are off-center!"* The left-right position is known with razor-sharp accuracy.
- But if the aircraft drifts $200\,\text{meters}$ forward or backward along the smooth, flat riverbed (along-track), the altitude beneath it barely changes at all. The computer is **virtually blind** to forward/backward drift!

#### 🔬 The Technical Solution:
The Hessian matrix has eigenvalues:
$$\lambda_1 = 8.5 \times 10^{-2}, \qquad \lambda_2 = 1.2 \times 10^{-5}$$
- **Anisotropy / Condition Number**:
  $$\kappa = \frac{\lambda_1}{\lambda_2} = \frac{8.5 \times 10^{-2}}{1.2 \times 10^{-5}} \approx \mathbf{7,083}$$
  The curvature across the canyon is over **7,000 times steeper** than along the canyon!
- **Uncertainty Ellipse Axes**:
  In state estimation, the covariance semi-axes are inversely proportional to curvature: $\sigma \propto \frac{1}{\sqrt{\lambda}}$.
  - **Cross-Track Axis ($\lambda_1$)**: $\sigma_{\text{cross}} \propto \frac{1}{\sqrt{8.5 \times 10^{-2}}} \approx 3.43\,\text{m}$ (sharp, tight localization).
  - **Along-Track Axis ($\lambda_2$)**: $\sigma_{\text{along}} \propto \frac{1}{\sqrt{1.2 \times 10^{-5}}} \approx 288.67\,\text{m}$ ($84\times$ wider uncertainty!).
- **Avionics Conclusion**: The cost surface forms an elongated degenerate trough. The aircraft's cross-track coordinate is well-observed, but along-track position is **unobservable**. An intelligent EKF will only update the cross-track state and maintain dead-reckoning uncertainty along-track.

---

### Question 3: The Sand Dune Cycle Slip

#### 💡 In Simple Terms: The Barcode Jump
Imagine reading a barcode made of identical black-and-white stripes spaced every $200\,\text{meters}$. If your position drifts by $210\,\text{meters}$, the sensor looks down, sees a stripe, and thinks: *"Ah! I am only $10\,\text{meters}$ off!"*
It snaps to the wrong stripe. The aircraft is now off by an entire $200\,\text{meter}$ block, but the computer believes it is on course. This is called a **Cycle Slip**.

#### 🔬 The Technical Solution:
- **Periodic Cost Surfaces**:
  A periodic terrain $h(x) = A \sin\left(\frac{2\pi}{\lambda} x\right)$ with wavelength $\lambda = 200\,\text{m}$ creates a periodic correlation surface:
  $$\mathbf{J}(x) \approx \mathbf{J}(x \pm k\lambda), \quad k \in \{1, 2, 3, \dots\}$$
- **The Failure Mode**:
  Every local minimum has almost identical elevation residuals. A small gust of wind or $0.2\,\text{m}$ of radar noise can cause a secondary minimum at $x^* \pm 200\,\text{m}$ to score slightly lower than the true minimum.
- **The Resulting Error**:
  The navigation computer suffers an **integer wavelength position jump**:
  $$\text{Position Error} = k \cdot \lambda = \pm 200\,\text{m}, \pm 400\,\text{m}, \dots$$
- **Avionics Defense**: The **Ambiguity Ratio (MAR)** check will flag that $C_{\text{second}}^* \approx C^*$ ($\text{MAR} \approx 1.0$), immediately recognize the multi-modal ambiguity, and **reject the fix** before the autopilot can be corrupted.

---

## 📚 8. Curated Aerospace Literature

1. 📄 **Golden, J. P. (1980)**: *"Terrain Contour Matching (TERCOM) Applications"*. SPIE Image Processing for Missile Guidance, Vol. 238, pp. 10-18.  
   *(Section on false-fix probability, thresholding, and correlation peak-to-sidelobe ratios).*
2. 📄 **Kayton, M. & Fried, W. R. (1997)**: *"Avionics Navigation Systems"*, 2nd Ed. John Wiley & Sons.  
   *(Chapter 6: Surface roughness requirements and the mathematical derivation of fix acceptance criteria).*
3. 📄 **Baird, C. A. & Abramson, M. R. (1984)**: *"A Comparison of TERCOM and SITAN"*. IEEE PLANS.  
   *(Analyzes correlation surface valley geometry, unobservable directions, and how non-linear particle filters resolve multi-modal ambiguity).*
