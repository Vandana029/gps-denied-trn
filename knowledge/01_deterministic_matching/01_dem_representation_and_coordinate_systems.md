# Chapter 1.1: Representing Earth in Silicon — DEMs & Coordinate Systems

> **Phase**: 1 — Deterministic Matching  
> **Subphase**: 1.1 — DEM Representation & Coordinate Grid Transformations  
> **Goal**: Master how a continuous 3D planetary landscape is digitized into a computer's memory, how coordinates are mapped without off-by-one errors, and how to smoothly sample elevations using bilinear interpolation.

---

## 🏔️ 1. The Story: Digitizing Planet Earth

Imagine you are an autonomous systems engineer sitting inside the flight software laboratory of an aerospace defense program. 

The vehicle you are programming is an autonomous aircraft tasked with flying low over rugged terrain without GPS. Before the mission, the mission planning team uploads a **geographical map** into the drone's solid-state drive.

Inside the drone's memory, there are no actual dirt, rocks, trees, or valleys. There is only a **2D matrix of numbers**:

```text
[ [ 1420.5,  1423.2,  1428.0,  ... ],
  [ 1418.1,  1421.4,  1426.5,  ... ],
  [ 1415.0,  1419.0,  1424.1,  ... ] ]
```

Every number represents **elevation in meters above Mean Sea Level (MSL)** at that specific spot on Earth. This 2D array is called a **Digital Elevation Model (DEM)**.

Now, here is the engineering puzzle:
- The computer's DEM matrix is **discrete** (a grid of isolated numbers spaced 30 meters apart).
- But the physical airplane moves through a **continuous** world: it can fly to $(x = 124.37\,\text{m}, y = 89.12\,\text{m})$.

If our software cannot bridge the gap between continuous physics and discrete memory grids with extreme precision, our terrain matching algorithms will stumble, create false spikes, and fail.

---

## 🌐 2. The Coordinate Transformation Problem

Let's establish our local navigation coordinate frame.

In local navigation, we use Cartesian coordinates measured in **meters**:
- $+X$: Points **East**.
- $+Y$: Points **North**.
- The origin $(X = 0, Y = 0)$ is placed at a reference landmark (e.g., the bottom-left or top-left of our mission sector).

Let's define our DEM bounding box:
- Map extends horizontally from $X_{\min}$ to $X_{\max}$ (Width: $W = X_{\max} - X_{\min}$).
- Map extends vertically from $Y_{\min}$ to $Y_{\max}$ (Height: $H = Y_{\max} - Y_{\min}$).
- The grid has $R$ rows and $C$ columns.
- The grid cell spacing (resolution) is $\Delta x$ (meters per column) and $\Delta y$ (meters per row).

```text
(X_min, Y_max) ──────────────────────────────── (X_max, Y_max)
   [Row 0, Col 0]                                  [Row 0, Col C-1]
         │                                                │
         │                ▲ +Y (North)                    │
         │                │                               │
         │                │                               │
         │                └───▶ +X (East)                 │
         │                                                │
         │                                                │
   [Row R-1, Col 0]                                [Row R-1, Col C-1]
(X_min, Y_min) ──────────────────────────────── (X_max, Y_min)
```

### ⚠️ The Classic Inversion Trap: Why Rows Go Down While North Goes Up!
Notice the subtle geometric trap that has bitten thousands of robotics engineers:
1. In physical space, **North ($+Y$) points UP**.
2. In computer image/matrix arrays, **Row 0 is at the TOP, and Row index $r$ increases DOWNWARDS**.

Therefore:
- When the aircraft flies **East** ($+X$), column index $c$ **increases**.
- When the aircraft flies **North** ($+Y$), row index $r$ **DECREASES**!

### The Exact Conversion Formulas

#### 1. From Continuous World $(x, y)$ to Floating-Point Grid Coordinates $(r_f, c_f)$:
$$\text{Floating Column } c_f = \frac{x - X_{\min}}{\Delta x}$$

$$\text{Floating Row } r_f = \frac{Y_{\max} - y}{\Delta y}$$

#### 2. From Discrete Grid Indices $[r, c]$ to World Coordinates $(x, y)$ at the post center:
$$x = X_{\min} + c \cdot \Delta x$$

$$y = Y_{\max} - r \cdot \Delta y$$

---

## 📐 3. The Interpolation Dilemma: How to Find Height Between Posts

Suppose our grid resolution is $\Delta x = \Delta y = 30\,\text{meters}$.  
Our drone is flying at position $(x = 45\,\text{m}, y = 45\,\text{m})$.

Looking at our grid:
$$c_f = \frac{45 - 0}{30} = 1.5$$
$$r_f = \frac{Y_{\max} - 45}{30} = 2.5$$

The drone is floating exactly in the center between 4 grid posts! What is the terrain height beneath it?

```text
       c_0 = 1                c_1 = 2
r_0 = 2  Q_11 (100m) ─────────── Q_12 (120m)
              │                    │
              │     • (1.5, 2.5)   │
              │    Drone is here   │
              │                    │
r_1 = 3  Q_21 (110m) ─────────── Q_22 (150m)
```

### Option A: Nearest-Neighbor (Why it is dangerous)
Nearest-neighbor simply rounds $1.5$ and $2.5$ to the nearest integer.  
* **Problem**: As the drone moves smoothly forward, the measured terrain elevation suddenly **jumps in discrete staircases**.
* In Phase 3 (SITAN / Kalman filtering), we must compute the slope (gradient $\frac{\partial h}{\partial x}$) of the terrain. A staircase has a derivative of **zero** on the flat steps and **infinity** at the cliff edges! Nearest-neighbor destroys gradient-based filtering.

---

### Option B: Bilinear Interpolation (The Aerospace Standard)
Bilinear interpolation performs two linear interpolations in one direction, followed by one linear interpolation in the perpendicular direction. It produces a **smooth, continuous surface**.

#### Step-by-Step Mathematical Derivation:

1. **Find the 4 integer bounding corners**:
   $$c_0 = \lfloor c_f \rfloor, \quad c_1 = c_0 + 1$$
   $$r_0 = \lfloor r_f \rfloor, \quad r_1 = r_0 + 1$$

2. **Calculate the fractional offsets** ($u, v \in [0, 1)$):
   $$u = c_f - c_0 \quad (\text{horizontal fraction})$$
   $$v = r_f - r_0 \quad (\text{vertical fraction})$$

3. **Retrieve the 4 corner elevations from the matrix**:
   $$Q_{11} = \mathbf{M}[r_0, c_0] \quad (\text{Top-Left})$$
   $$Q_{12} = \mathbf{M}[r_0, c_1] \quad (\text{Top-Right})$$
   $$Q_{21} = \mathbf{M}[r_1, c_0] \quad (\text{Bottom-Left})$$
   $$Q_{22} = \mathbf{M}[r_1, c_1] \quad (\text{Bottom-Right})$$

4. **Interpolate along columns first (Horizontal)**:
   - Elevation at the top edge ($r = r_0$):
     $$h_{\text{top}} = (1 - u) Q_{11} + u Q_{12}$$
   - Elevation at the bottom edge ($r = r_1$):
     $$h_{\text{bottom}} = (1 - u) Q_{21} + u Q_{22}$$

5. **Interpolate along rows (Vertical)**:
   Combine $h_{\text{top}}$ and $h_{\text{bottom}}$ using vertical fraction $v$:
   $$h(x, y) = (1 - v) h_{\text{top}} + v h_{\text{bottom}}$$

#### Expanding into the Master Bilinear Formula:
$$
h(x, y) = (1 - u)(1 - v) Q_{11} + u(1 - v) Q_{12} + (1 - u) v Q_{21} + u v Q_{22}
$$

Notice the geometric beauty of this formula: **Each corner's weight is equal to the area of the diagonally opposite rectangle!**
- If the drone is very close to the top-left ($u \to 0, v \to 0$), the weight $(1-u)(1-v) \to 1$, so $h(x, y) \to Q_{11}$.

---

## 🛡️ 4. Edge Cases & Defensive Software Engineering

When deploying this in autonomous flight, we must handle two real-world failure modes:

1. **Boundary Clipping (Out-of-Bounds Queries)**:
   - What happens if the drone flies to $X = -5\,\text{m}$ (off the western edge of the DEM)?
   - *Software policy*: The class should provide clear options: raise a `ValueError("Flight path outside DEM bounds")` or clamp to the boundary with a warning flag.
2. **Extreme Slopes & Cliffs**:
   - In canyons, elevation changes abruptly. We must ensure our interpolator handles float precision cleanly without NaN propagation.

---

## 🏗️ 5. Software Architecture Blueprint

To make our implementation clean, reusable, and tested, we will build two modular components in `src/terrain_matching/`:

```text
src/terrain_matching/
├── core/
│   ├── __init__.py
│   └── dem.py                <── DigitalElevationModel class
└── simulation/
    ├── __init__.py
    └── terrain_generator.py  <── Synthetic landscape generator (Hills, Ridges, Flats)
```

### The `DigitalElevationModel` Class Blueprint
```python
class DigitalElevationModel:
    def __init__(
        self,
        elevation: np.ndarray,      # 2D array of shape (R, C)
        x_min: float,
        y_max: float,
        dx: float,
        dy: float
    ) -> None:
        ...

    def world_to_grid(self, x: float, y: float) -> tuple[float, float]:
        """Convert continuous metric coordinates (x, y) to floating (row, col)."""

    def grid_to_world(self, row: float, col: float) -> tuple[float, float]:
        """Convert grid (row, col) to continuous metric coordinates (x, y)."""

    def get_elevation(self, x: float, y: float, method: str = "bilinear") -> float:
        """Sample terrain elevation at continuous world coordinate (x, y)."""
```

### The `TerrainGenerator` Blueprint
For reproducible testing in Phase 1, we will generate synthetic terrains:
1. **Gaussian Mountain Ridge**: Perfect for proving that a ridge produces a single, sharp correlation minimum.
2. **Multi-frequency Rolling Hills (Sinusoidal / Perlin-style)**: Realistic landscape with multiple hills and valleys.
3. **Flat Salt Plain**: Used to demonstrate the "Desert Paradox" and verify ambiguity detection.

---

## 📚 Recommended Literature & Reading Materials

1. **USGS (U.S. Geological Survey)**: *"Standards for Digital Elevation Models"*, National Mapping Program Technical Instructions. *(The official specification of DEM standards).*
2. **Press, W. H., et al.**: *"Numerical Recipes: The Art of Scientific Computing"*, Chapter 3 (Interpolation and Extrapolation). Cambridge University Press.
3. 📄 **Farr, T. G., et al. (2007)**: *"The Shuttle Radar Topography Mission (SRTM)"*. Reviews of Geophysics, 45(2). *(How NASA mapped 80% of Earth's landmass using space shuttle radar interferometry).*
