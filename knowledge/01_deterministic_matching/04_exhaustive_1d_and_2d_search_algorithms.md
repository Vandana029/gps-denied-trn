# Chapter 1.4: Finding the Needle in the Mountain — Exhaustive 1D & 2D Search Algorithms

> **Phase**: 1 — Deterministic Matching  
> **Subphase**: 1.4 — Exhaustive 1D & 2D Search Algorithms  
> **Goal**: Master the mathematical formulation and algorithmic mechanics of deterministic terrain matching. Construct the search algorithms that slide a measured elevation profile across 1D flight corridors and evaluate 2D planar search grids across a Digital Elevation Model (DEM) to find the global position fix.

---

## 🏔️ 1. The Story: Blind in the Sierra Nevada

### 💡 In Simple Terms: What is the mission here?
Imagine you are blindfolded inside a car driving on a hilly country road. You cannot see out the windows, and your phone has no GPS. But you have two things:
1. An altimeter in your hand that records your height every second. Over the last mile, you wrote down a wavy graph of numbers: *"up 20m, down 10m, sharp rise 50m, down 30m"*. That wavy line of numbers is your **fingerprint**.
2. A detailed paper topographic map in your glovebox showing all the hills and valleys in the county.

Your job is simple: **Slide that wavy line across the paper map until it sits on top of a hill-path that matches your numbers exactly.** The moment the waves match, you open your eyes and say: *"I know exactly where we are on Earth!"*

---

### 🚀 The Operational Scenario:
An autonomous reconnaissance aircraft is penetrating a radar-jammed valley along the spine of the Sierra Nevada. High winds and turbulent updrafts have caused its dead-reckoning inertial navigation system to drift. 

The onboard computer knows its general heading ($\psi = 45^\circ$, North-East) and knows its true position lies somewhere within an uncertainty bounding box of $4\,\text{km} \times 4\,\text{km}$.

During the last 15 seconds of flight, the radar and barometric altimeter triad sampled 51 ground elevations spaced every $30\,\text{meters}$:
$$\mathbf{h}_{\text{meas}} = [h_0, h_1, \dots, h_{50}]^T$$

The computer now holds a $1.5\,\text{km}$ continuous topological contour strip in its RAM.

Somewhere within that $4 \times 4\,\text{km}$ mountain map lies the **one physical ground path** that generated this exact sequence of peaks, ridges, saddles, and gullies.

The mission of the **Terrain Matching Engine** is to systematically slide this measured strip over candidate ground paths across the map, score each candidate using our cost metrics (from Chapter 1.3), and pinpoint the exact coordinates $(x^*, y^*)$ of the aircraft:

```text
                  2D SEARCH BOUNDING BOX (e.g. 4 km x 4 km)
     ┌──────────────────────────────────────────────────────────────┐
     │  Candidate (x_c, y_c)                                        │
     │      ▲                                                       │
     │      │   Candidate Flight Strip                              │
     │      └───► ░░░░░░░░░░░░░░░░░░░░░░░ (Score: MAD = 42.5 m)    │
     │                                                              │
     │                                                              │
     │            TRUE PHYSICAL FLIGHT STRIP                        │
     │            ███████████████████████████ (Score: MAD = 0.0 m)  │
     │            ▲                                                 │
     │            │ Optimal Fix: (x*, y*)                           │
     │                                                              │
     │                                                              │
     │                                                              │
     └──────────────────────────────────────────────────────────────┘
```

---

## 🧩 2. Real-World Analogy: The Serrated Key

### 💡 In Simple Terms:
Think of a physical brass house key with jagged teeth cut along its edge.
- The **DEM (map)** is like a lock cylinder containing hundreds of internal pins at different heights.
- The **measured profile $\mathbf{h}_{\text{meas}}$** is the cut pattern of ridges and notches on your key.
- If you slide the key into the wrong slot or stop halfway, the teeth collide with the pins — the key refuses to turn (**High Error / High Cost**).
- But when you slide the key into the exact matching chamber, every single tooth drops into its corresponding notch simultaneously ($e_i = 0$) — the lock turns effortlessly (**Zero Error / Perfect Match**).
- In **1D**, we slide our key forward and backward along one single groove.
- In **2D**, we slide our key across an entire chessboard of candidate slots until we find the one slot where it clicks!

---

## 📐 3. Mathematical Formulations of Terrain Search

### 3.1 1D Along-Track Corridor Search (Sliding Window)

#### 💡 In Simple Terms: Sliding a Tape Measure along a Train Track
Suppose our aircraft is flying down a narrow mountain canyon or along a railroad line. We know for sure we are on that line (we haven't drifted left or right), but we don't know **how far along the track we have traveled**.
- The map has a long elevation profile of the whole canyon ($10\,\text{km}$ long).
- Our aircraft measured a short slice ($1.5\,\text{km}$ long).
- To find where we are, we take our $1.5\,\text{km}$ ruler, place it at the beginning of the canyon ($0\,\text{km}$), and check the error. Then we slide it forward by $30\,\text{m}$ and check again. We repeat this slide-and-check process until we find the spot where the error is smallest!

#### 🔬 The Technical Math:
Suppose we extract a long 1D reference elevation profile from the DEM along the known corridor:
$$\mathbf{H}_{\text{corridor}} = [H_0, H_1, H_2, \dots, H_{M-1}]^T \in \mathbb{R}^M$$
Our live measured profile has length $N$, where $N \ll M$.

For every discrete along-track shift index $k \in \{0, 1, \dots, M - N\}$:
1. **Extract candidate slice** of length $N$:
   $$\mathbf{h}_{\text{ref}}(k) = [H_k, H_{k+1}, \dots, H_{k+N-1}]^T$$
2. **Compute the matching cost** using metric $\mathcal{M}$:
   $$C(k) = \mathcal{M}\left(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}(k)\right)$$
3. **Identify the optimal along-track index** $k^*$:
   $$k^* = \arg\min_k C(k) \quad (\text{for difference metrics: MAD, MSD, ZMSD})$$
   $$k^* = \arg\max_k C(k) \quad (\text{for similarity metrics: NCC, ZNCC})$$

The corresponding along-track ground coordinate is:
$$s^* = s_{\text{start}} + k^* \cdot \Delta s$$

---

### 3.2 2D Planar Grid Search (Spatial Raster Matching)

#### 💡 In Simple Terms: Dropping a Cookie Cutter Across a Giant Map
In real life, the aircraft drifts in **both** directions: East-West ($X$) and North-South ($Y$).
- Imagine your flight path as a wire bent into the exact shape of your aircraft's ground track (e.g. flying Northeast for $1.5\,\text{km}$).
- We create a grid of thousands of candidate starting pins across our map (e.g., every $30\,\text{meters}$ in East and North).
- For every pin $(x_c, y_c)$, we place the start of our wire at that pin, lay it down in the flight direction, look up the mountain heights under the wire on our DEM, and compare those heights with what the altimeter actually measured.
- We record the error score on a 2D heat map. The pin that produces the lowest error is our estimated physical location $(x^*, y^*)$!

```text
             (x_c, y_c + dy)         (x_c + dx, y_c + dy)
                   O───────────────────────────O
                   │                           │
                   │      Flight Path Wire     │
                   │      (Relative Offsets)   │
                   │      *─────*─────*─────*  │
                   │     /                     │
                   │    /                      │
                   O───*───────────────────────O
                (x_c, y_c)               (x_c + dx, y_c)
```

#### 🔬 The Technical Math:
Let the nominal flight trajectory have duration $T$ and discrete sample count $N$:
$$\mathbf{p}_{\text{nom}}(t_i) = \begin{bmatrix} x_{\text{nom}, i} \\ y_{\text{nom}, i} \end{bmatrix}, \quad i = 0, 1, \dots, N-1$$

We decompose the trajectory into a **start position** plus an array of **relative displacement vectors**:
$$\Delta x_i = x_{\text{nom}, i} - x_{\text{nom}, 0}$$
$$\Delta y_i = y_{\text{nom}, i} - y_{\text{nom}, 0}$$
$$\Delta \mathbf{p}_i = [\Delta x_i, \Delta y_i]^T, \quad \text{with } \Delta \mathbf{p}_0 = [0, 0]^T$$

#### Defining the 2D Search Space:
Let the horizontal search bounding box be centered at a prior estimate $(x_0, y_0)$ with search radii $(R_x, R_y)$ and grid search step sizes $(\delta x, \delta y)$:
$$x_c \in [x_0 - R_x, x_0 + R_x], \quad \text{spaced by } \delta x$$
$$y_c \in [y_0 - R_y, y_0 + R_y], \quad \text{spaced by } \delta y$$

This generates a discrete 2D grid of candidate start coordinates of size $N_y \times N_x$.

#### The Evaluation Loop:
For each candidate start point $(x_c, y_c)$:
1. **Reconstruct Candidate Ground Path**:
   $$\mathbf{p}_{\text{cand}, i}(x_c, y_c) = \begin{bmatrix} x_c + \Delta x_i \\ y_c + \Delta y_i \end{bmatrix}, \quad i = 0, 1, \dots, N-1$$
2. **Sample DEM Topography**:
   Query the DEM at all $N$ coordinates using Bilinear Interpolation:
   $$\mathbf{h}_{\text{ref}}(x_c, y_c) = \left[ h_{\text{dem}}(\mathbf{p}_{\text{cand}, 0}), \dots, h_{\text{dem}}(\mathbf{p}_{\text{cand}, N-1}) \right]^T$$
3. **Compute Matching Cost**:
   $$\mathbf{J}(y_c, x_c) = \mathcal{M}\left(\mathbf{h}_{\text{meas}}, \mathbf{h}_{\text{ref}}(x_c, y_c)\right)$$

The complete 2D matrix $\mathbf{J} \in \mathbb{R}^{N_y \times N_x}$ is the **Correlation Surface** (or **Cost Surface**).

#### The Global Fix:
The estimated horizontal location of the aircraft is the coordinate pair that minimizes the cost surface:
$$(x^*, y^*) = \arg\min_{(x_c, y_c)} \mathbf{J}(y_c, x_c)$$

---

## ⚡ 4. Computational Complexity & The Curse of Dimensionality

### 💡 In Simple Terms: How Many Guesses Can the Computer Make in 1 Second?
If you search an area of $3\,\text{km} \times 3\,\text{km}$ testing every $30\,\text{meters}$, that gives:
$$101 \times 101 = 10,201 \text{ candidate locations!}$$
At every location, your profile has $50$ points. That means $10,201 \times 50 \approx 510,000$ elevation queries.
- If you write a slow Python `for` loop, it might take 20 seconds — your aircraft would crash before getting a fix!
- But if we vectorize the math using **NumPy matrix operations**, the entire half-million calculations execute in **~30 milliseconds**!
- In Phase 1, we can afford to test every single spot because we assume the aircraft heading is known. (In Phase 2, if heading is also unknown, we must test angles too, which is why we will learn how to shrink the search box using Kalman filter covariances).

---

### 🔬 The Technical Complexity Math:
Let:
- $N$ = Number of profile samples (e.g. 50 samples).
- $W_x, W_y$ = Search window widths in meters (e.g. $3000\,\text{m} \times 3000\,\text{m}$).
- $\delta x, \delta y$ = Grid search resolution (e.g. $30\,\text{m}$ matching DEM grid spacing).

The number of candidate positions evaluated is:
$$N_{\text{candidates}} = \left(\frac{W_x}{\delta x} + 1\right) \times \left(\frac{W_y}{\delta y} + 1\right) = 101 \times 101 = 10,201\,\text{candidates}$$

At each candidate position:
- We perform $N = 50$ bilinear interpolations.
- We evaluate an $N$-element vectorized cost function (e.g. ZMSD or MAD).

Total bilinear queries:
$$N_{\text{total\_queries}} = 10,201 \times 50 \approx 510,000\,\text{interpolations}$$

On a modern CPU (e.g., Python + NumPy vectorization), half a million bilinear interpolations takes **~10 to 50 milliseconds**. In embedded C++20 with AVX2/SIMD vector instructions, this executes in **under 2 milliseconds**!

#### ⚠️ Why Exhaustive Search is Feasible in Phase 1:
In Phase 1, our search space is strictly 2D $(x, y)$ with known heading $\psi$ and known groundspeed $V$. 
*(In Phase 2 and Phase 3, heading uncertainty adds a third search dimension $\psi$, expanding the search space into a 3D manifold $(x, y, \psi)$, which we will optimize using bounded INS uncertainty ellipses!)*

---

## 🛡️ 5. Edge Handling: Trajectory Truncation & DEM Boundaries

### 💡 In Simple Terms: What Happens When We Fly Off the Edge of the Map?
Imagine you place a candidate pin near the top-right corner of your paper map. The aircraft flies Northeast, so after 3 seconds, the candidate flight path flies **off the edge of the paper into thin air**!
- We don't have map data out there.
- If the computer tries to read map data outside the boundaries, it will crash with an out-of-bounds error.
- Therefore, any candidate pin whose flight path steps off the map must be marked as **strictly invalid**.
- We give it the worst possible score: $+\infty$ for difference metrics, or $-1.0$ for correlation metrics. That way, the computer will never pick an off-map ghost location!

```text
    ┌───────────────────────────┐  ◄── DEM Boundary
    │                           │
    │      Candidate Path       │
    │      (x_c, y_c) ───►──────┼────────► [OFF MAP!] -> Assign Cost = +Infinity
    │                           │
    └───────────────────────────┘
```

---

### 🔬 The Technical Handling:
If candidate path $\mathbf{p}_{\text{cand}}(x_c, y_c)$ exits the DEM:
1. **Invalid Candidate**: Any candidate whose path leaves the map cannot be verified against terrain ground truth.
2. **Avionics Handling**: The matching engine marks this candidate coordinate as **invalid** and assigns it a penalty cost:
   - For difference metrics (MAD, MSD, ZMSD): $\mathbf{J}(y_c, x_c) = +\infty$.
   - For correlation metrics (NCC, ZNCC): $\mathbf{J}(y_c, x_c) = -1.0$.
3. **Safe Search Window Bounding**:
   To avoid wasting CPU cycles, the search grid bounds can be automatically clipped:
   $$x_{\min} \ge \text{DEM.x\_min} - \min(\Delta x)$$
   $$x_{\max} \le \text{DEM.x\_max} - \max(\Delta x)$$

---

## 🏛️ 6. Software Architecture Blueprint for Subphase 1.4

### 💡 In Simple Terms: What Are We Actually Coding?
We are going to build a Python module `src/terrain_matching/core/matcher.py` that contains:
1. `MatchResult`: A neat results box that returns:
   - Where the aircraft is: `best_coord = (x*, y*)`
   - How confident the match is: `best_cost`
   - How far off it was from reality: `position_error` (in meters)
   - The entire 2D cost surface so we can plot beautiful 3D landscapes of the error!
2. `DeterministicMatcher`: The engine itself with two functions:
   - `match_1d_corridor`: For sliding a profile along a single straight line.
   - `match_2d_grid`: For checking a full $(x, y)$ grid box over the map.

---

### 🔬 The Technical Code Interface:
```python
"""Deterministic 1D and 2D terrain matching search engines.

Provides exhaustive sliding-window corridor matching and 2D spatial grid search
over Digital Elevation Models using vectorized cost and similarity metrics.
"""

@dataclass(frozen=True)
class MatchResult:
    """Encapsulates the outcome of a terrain matching search.

    Attributes:
        best_coord: Estimated (x, y) coordinates of the trajectory start in meters.
        best_cost: Metric score at the optimal coordinate.
        true_coord: Ground truth (x, y) coordinates (if known for verification).
        position_error: Euclidean distance error ||best_coord - true_coord|| in meters.
        search_grid_x: 1D array of evaluated X coordinates.
        search_grid_y: 1D array of evaluated Y coordinates.
        cost_surface: 2D array of shape (len(y), len(x)) containing matching scores.
        metric_name: Name of the cost metric used.
        execution_time_s: Elapsed search runtime in seconds.
    """

class DeterministicMatcher:
    """Exhaustive terrain matching engine over 2D DEMs."""

    def __init__(self, dem: DigitalElevationModel) -> None:
        self.dem = dem

    def match_1d_corridor(
        self,
        measured_profile: np.ndarray,
        corridor_start: tuple[float, float],
        heading_deg: float,
        corridor_length_m: float,
        step_size_m: float,
        metric: str = "mad",
    ) -> MatchResult:
        """Sliding-window along-track matching along a single flight line."""

    def match_2d_grid(
        self,
        measured_profile: np.ndarray,
        relative_offsets_xy: np.ndarray,
        search_bounds: tuple[float, float, float, float],  # (x_min, x_max, y_min, y_max)
        step_size_m: float,
        metric: str = "mad",
        true_coord: tuple[float, float] | None = None,
    ) -> MatchResult:
        """Exhaustive 2D spatial raster search across a rectangular bounding box."""
```

---

## 💡 7. Pedagogical Check & Conceptual Walkthrough (Step 2 Solutions)

Here are the complete step-by-step mathematical answers and physical insights for our three pedagogical check questions:

---

### Question 1: The Exact Recovery Theorem

#### 💡 In Simple Terms:
If the world is completely perfect (no sensor noise, no weather pressure bias) and our search grid happens to test the exact spot where the aircraft started:
- What should our error score be?
- Can the computer find the aircraft with zero millimeter error?

#### 🔬 The Technical Solution:
- **At the true location $(x_{\text{true}}, y_{\text{true}})$**:
  Because the simulation is deterministic, the sampled candidate profile $\mathbf{h}_{\text{ref}}(x_{\text{true}}, y_{\text{true}})$ is mathematically identical to the measured profile $\mathbf{h}_{\text{meas}}$:
  $$h_{\text{ref}, i} \equiv h_{\text{meas}, i}, \quad \forall i \in \{0, 1, \dots, N-1\}$$
- **Expected Metric Values**:
  - $\text{MAD}(x^*, y^*) = \frac{1}{N} \sum |0| = \mathbf{0.0\,\text{m}}$
  - $\text{MSD}(x^*, y^*) = \frac{1}{N} \sum 0^2 = \mathbf{0.0\,\text{m}^2}$
  - $\text{ZNCC}(x^*, y^*) = \frac{\tilde{\mathbf{h}}^T \tilde{\mathbf{h}}}{\|\tilde{\mathbf{h}}\|_2 \|\tilde{\mathbf{h}}\|_2} = \mathbf{+1.0}$
- **Position Error**:
  $$\text{Error} = \|\mathbf{p}^* - \mathbf{p}_{\text{true}}\| = \sqrt{(x^* - x_{\text{true}})^2 + (y^* - y_{\text{true}})^2} = \mathbf{0.0\,\text{meters}}$$
- **GNC Conclusion**: In an idealized world, terrain matching is an **exact inverse problem**. The true flight path forms a sharp global delta-spike minimum on the correlation surface.

---

### Question 2: Grid Resolution vs. Off-Grid Aircraft (Discretization Error)

#### 💡 In Simple Terms:
What happens if the drone started at coordinates $(x = 105\,\text{m}, y = 215\,\text{m})$, but our computer only checks points on a $50\,\text{m}$ grid (like checking $0, 50, 100, 150, 200, 250$)?
- The drone is sitting between the grid pins! The computer will pick the closest pin $(100, 200)$.
- Because the closest pin is not the exact spot, the elevations will be slightly different. The error will **never reach exactly $0.0$**!
- What is the worst-case distance between any random point and the nearest grid pin?

#### 🔬 The Technical Solution:
- **Nearest Grid Node**:
  The closest multiples of $50\,\text{m}$ to $(105.0, 215.0)$ are:
  $$x_{\text{nearest}} = \text{round}(105.0 / 50.0) \times 50.0 = 2 \times 50.0 = \mathbf{100.0\,\text{m}}$$
  $$y_{\text{nearest}} = \text{round}(215.0 / 50.0) \times 50.0 = 4 \times 50.0 = \mathbf{200.0\,\text{m}}$$
  The nearest grid point is $(100.0, 200.0)\,\text{m}$.
  The distance to this nearest node is:
  $$\Delta r = \sqrt{(105 - 100)^2 + (215 - 200)^2} = \sqrt{5^2 + 15^2} = \sqrt{25 + 225} = \sqrt{250} \approx \mathbf{15.81\,\text{meters}}$$
- **Maximum Theoretical Discretization Error**:
  The worst-case scenario occurs when an aircraft sits precisely at the center of a grid cell:
  $$\Delta x_{\max} = \frac{\delta x}{2} = 25.0\,\text{m}, \quad \Delta y_{\max} = \frac{\delta y}{2} = 25.0\,\text{m}$$
  The maximum distance error to the nearest node is the half-diagonal:
  $$r_{\max} = \sqrt{\left(\frac{\delta x}{2}\right)^2 + \left(\frac{\delta y}{2}\right)^2} = \frac{\delta}{\sqrt{2}} = \frac{50}{\sqrt{2}} \approx \mathbf{35.36\,\text{meters}}$$
- **Can the cost ever reach exactly $0.0$?**
  **No.** Because the nearest tested node is offset by $15.81\,\text{m}$ from the true start point, the candidate profile extracted at $(100, 200)$ is spatially shifted relative to the actual terrain flown over. The slopes and heights will slightly differ, so $\text{MAD} > 0$ and $\text{MSD} > 0$.
- **Avionics Insight**: The spatial resolution of your search grid $\delta$ establishes a fundamental lower bound on your localization accuracy. To achieve sub-grid accuracy (e.g. $< 1\,\text{meter}$), production systems perform **sub-pixel quadratic surface fitting** around the minimum or switch to continuous Kalman filtering (SITAN, Phase 3).

---

### Question 3: Metric Directionality ($\arg\min$ vs. $\arg\max$)

#### 💡 In Simple Terms:
- For **difference metrics** like MAD or MSD: A score of $0$ means perfect match, and $100$ means terrible match. So we want the **lowest** score ($\arg\min$, like golf!).
- For **similarity metrics** like NCC or ZNCC: A score of $+1.0$ means identical shape, and $-1.0$ means completely upside down. So we want the **highest** score ($\arg\max$, like basketball!).
- When a path goes off the edge of the map, what fake score do we give it?
  - For difference metrics: We give it $+\infty$ (the worst possible score).
  - For correlation metrics: We give it $-1.0$ (the worst possible correlation).

#### 🔬 The Technical Solution:
- **Optimization Direction**:
  - **Difference Metrics** (`mad`, `msd`, `rmse`, `zmad`, `zmsd`):
    $$(x^*, y^*) = \arg\min_{(x_c, y_c)} \mathbf{J}(y_c, x_c)$$
  - **Correlation Metrics** (`ncc`, `zncc`):
    $$(x^*, y^*) = \arg\max_{(x_c, y_c)} \mathbf{J}(y_c, x_c)$$
- **Boundary Penalty Handling**:
  - For difference metrics: Assign $\mathbf{J}(y_c, x_c) = +\infty$ (`np.inf`).
  - For correlation metrics: Assign $\mathbf{J}(y_c, x_c) = -1.0$.
  This guarantees that boundary-violating points can never be falsely selected as the optimal global fix.

---

## 📚 8. Curated Aerospace Literature

1. 📄 **Golden, J. P. (1980)**: *"Terrain Contour Matching (TERCOM) Applications"*. SPIE Image Processing for Missile Guidance, Vol. 238, pp. 10-18.  
   *(Details the 2D correlation matrix computation and search window sizing for cruise missile terminal fixes).*
2. 📄 **Hostetler, L. D. & Kohler, R. D. (1983)**: *"Transponder-Aided Terrain-Following Guidance and SITAN"*. Sandia Report SAND82-1405.  
   *(Contrasts batch exhaustive search algorithms against recursive state-space Kalman filtering).*
3. 📄 **Baird, C. A. (1984)**: *"TERCOM-Aided Inertial Navigation Systems"*. IEEE Aerospace and Electronic Systems Magazine.  
   *(Mathematical formulation of trajectory offset arrays $\Delta \mathbf{p}_i$ and correlation surface landscapes).*
