# Chapter 1.2: The Aerial Tape Measure — Altimeter Modeling & Profile Sampling

> **Phase**: 1 — Deterministic Matching  
> **Subphase**: 1.2 — Altimeter Modeling & Profile Sampling  
> **Goal**: Master the physics and geometry of the airborne altimeter triad, formulate how flight trajectories are parameterized, and construct the continuous-to-discrete sampling pipeline that extracts 1D terrain elevation profiles from a 2D DEM.

---

## ✈️ 1. The Story: Feeling the Earth from 5,000 Feet

Imagine you are seated in the telemetry station of a flight-test facility in the Mojave Desert. A subscale autonomous test vehicle is screaming over the desert scrub at 200 meters per second (approx. 720 km/h, Mach 0.6). 

The onboard GPS has been intentionally commanded into blackout (jammed/denied). 

To keep from drifting off course or flying into a granite mountain ridge, the flight computer must continuously answer one fundamental question:
> *"What does the terrain directly beneath my belly look like right now?"*

The aircraft cannot touch the ground. It cannot see stars or horizons in thick clouds or sandstorms. Instead, it relies on two complementary sensors working in tandem:

```text
               ┌────────────────────────┐
               │   Aircraft at (x, y)   │ ◄─── Flight Altitude z_baro (above Sea Level)
               └───────────┬────────────┘
                           │
                           │ ◄── Radar/LiDAR Clearance h_radar (AGL)
                           ▼
               ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
             ▲▲▲▲                      ▲▲▲▲ ◄── Terrain Surface h_terrain
            ▲▲                            ▲▲
  ═══════════════════════════════════════════════════════════════ Datum: Mean Sea Level (MSL = 0)
```

1. **The Barometric / Inertial Vertical Sensor**: Measures the aircraft's absolute altitude above Mean Sea Level (MSL), denoted $z_{\text{baro}}(t)$.
2. **The Down-Looking Radar / LiDAR Altimeter**: Shoots radio frequency pulses or laser photons straight down toward nadir, measuring the clearance distance between the aircraft belly and the ground (Above Ground Level, AGL), denoted $h_{\text{radar}}(t)$.

By subtracting the clearance from the absolute altitude, the flight computer reconstructs the ground elevation beneath it:
$$h_{\text{terrain}}(t) = z_{\text{baro}}(t) - h_{\text{radar}}(t)$$

As the aircraft flies along a flight path for a few kilometers, it accumulates a sequential tape of terrain heights:
$$\mathbf{h} = [h_0, h_1, h_2, \dots, h_{N-1}]$$

This sequence is the **Terrain Profile**. In Terrain-Relative Navigation, this 1D profile is our primary "fingerprint." If we slide this fingerprint across our onboard digital map (DEM), there should be only one place on Earth where the elevations match up perfectly!

---

## 🛰️ 2. The Altimeter Triad & Sensor Geometry

In professional aerospace systems engineering, we refer to the measurement architecture as the **Altimeter Triad**:

| Sensor Component | Physical Principle | Reference Datum | Variable Symbol | Typical Update Rate |
|---|---|---|---|---|
| **Barometric Altimeter** | Hydrostatic atmospheric pressure lapse rate | Mean Sea Level (MSL) geoid | $z_{\text{baro}}$ | 10 – 50 Hz |
| **Radar / Laser Altimeter** | Two-way time-of-flight (ToF) of EM pulse | Local ground surface directly below | $h_{\text{radar}}$ or $h_{\text{AGL}}$ | 10 – 100 Hz |
| **Digital Elevation Model (DEM)** | Pre-surveyed digital topographic map | Mean Sea Level (MSL) | $h_{\text{dem}}(x, y)$ | Static onboard database |

### The Fundamental TRN Equation

Under ideal conditions (nadir pointing, zero sensor noise, zero barometric bias):
$$h_{\text{terrain}}(t) = z_{\text{baro}}(t) - h_{\text{radar}}(t) \equiv h_{\text{dem}}(x(t), y(t))$$

Where:
- $(x(t), y(t))$ is the aircraft's true horizontal position in East-North-Up (ENU) coordinates.
- $z(t)$ is the true aircraft altitude above MSL.
- $h_{\text{radar}}(t) = z(t) - h_{\text{dem}}(x(t), y(t))$.
- $z_{\text{baro}}(t) = z(t)$.

Notice the mathematical elegance: **the altitude of the aircraft $z(t)$ cancels out completely!**
$$h_{\text{terrain}}(t) = z(t) - [z(t) - h_{\text{dem}}(x(t), y(t))] = h_{\text{dem}}(x(t), y(t))$$

Whether the aircraft is flying at 500 meters or 2,000 meters above sea level, whether it is climbing, descending, or riding updrafts, the difference between the barometric reading and the radar clearance isolates the **pure, invariant shape of the Earth beneath it**.

---

## 📈 3. Trajectory Parameterization & Sampling

An aircraft trajectory in 3D Euclidean space is a continuous vector function of time:
$$\mathbf{p}(t) = \begin{bmatrix} x(t) \\ y(t) \\ z(t) \end{bmatrix}, \quad t \in [0, T]$$

### 3.1 Kinematics of Flight
For an aircraft flying with groundspeed $V(t)$ and heading angle $\psi(t)$ (measured clockwise from True North, or counter-clockwise from East depending on convention — in ENU coordinates, heading $\theta$ counter-clockwise from East):
$$\dot{x}(t) = V(t) \cos \theta(t)$$
$$\dot{y}(t) = V(t) \sin \theta(t)$$
$$\dot{z}(t) = v_z(t) \quad (\text{climb/descent rate})$$

For our Phase 1 deterministic pipeline, we consider two primary trajectory archetypes:
1. **Straight-Line Cruise**: Constant speed $V$, constant heading $\theta$, constant altitude $z_0$:
   $$x(t) = x_0 + V \cos(\theta) \cdot t$$
   $$y(t) = y_0 + V \sin(\theta) \cdot t$$
   $$z(t) = z_0$$
2. **Curvilinear Waypoint Flight (Piecewise / Spline)**: Aircraft maneuvering between waypoints $\mathbf{W}_0, \mathbf{W}_1, \dots, \mathbf{W}_M$.

### 3.2 Discrete Sampling: The Nyquist-Shannon Constraint on Terrain
The onboard altimeter samples at a discrete sampling frequency $f_s = \frac{1}{\Delta t}$ (e.g., 10 Hz, meaning $\Delta t = 0.1\,\text{s}$).

At each discrete time step $k = 0, 1, 2, \dots, N-1$:
$$t_k = k \cdot \Delta t$$
$$\mathbf{p}_k = \mathbf{p}(t_k) = [x_k, y_k, z_k]^T$$

The physical distance traveled between two consecutive samples along the ground is:
$$\Delta s_k = \sqrt{(x_{k} - x_{k-1})^2 + (y_{k} - y_{k-1})^2} \approx V \cdot \Delta t$$

#### ⚠️ The Spatial Aliasing Hazard:
In Chapter 1.1, our DEM had a grid resolution of $\Delta x = \Delta y = 30\,\text{meters}$.
- If an aircraft flies at $V = 150\,\text{m/s}$ with an altimeter sampling at $f_s = 1\,\text{Hz}$:
  $$\Delta s = 150\,\text{m/s} \times 1.0\,\text{s} = 150\,\text{meters!}$$
  The aircraft jumps across **5 full DEM pixels** between consecutive samples! All high-frequency topography (sharp ridges, gullies) between those samples is aliased and lost.
- If the altimeter instead samples at $f_s = 10\,\text{Hz}$:
  $$\Delta s = 150\,\text{m/s} \times 0.1\,\text{s} = 15\,\text{meters}$$
  The sampling interval is **half the grid cell size**, satisfying the Nyquist spatial sampling criterion for the DEM grid!

---

## 🔍 4. The Profile Sampling Pipeline

To simulate and extract a terrain profile from our digital world, the software executes a 4-stage pipeline:

```text
  ┌─────────────────────────┐
  │  Trajectory Generator   │ ──▶ Generates continuous 3D points p_k = (x_k, y_k, z_k)
  └────────────┬────────────┘
               │
               ▼
  ┌─────────────────────────┐
  │    DEM Query Engine     │ ──▶ Bilinear interpolation on DEM at (x_k, y_k)
  │  (Bilinear Interp)      │     Yields true terrain height: h_dem(k)
  └────────────┬────────────┘
               │
               ▼
  ┌─────────────────────────┐
  │   Altimeter Simulator   │ ──▶ Computes clearance: h_radar(k) = z_k - h_dem(k)
  │                         │     Computes baro altitude: z_baro(k) = z_k
  └────────────┬────────────┘
               │
               ▼
  ┌─────────────────────────┐
  │ Terrain Profile Output  │ ──▶ Reconstructed: h_meas(k) = z_baro(k) - h_radar(k)
  │                         │     Profile vector: h = [h_0, h_1, ..., h_{N-1}]
  └─────────────────────────┘
```

### Profile Data Structure
A clean `TerrainProfile` object must encapsulate:
1. `timestamps`: 1D array of floats $[t_0, t_1, \dots, t_{N-1}]$ (seconds).
2. `trajectory`: 2D array of coordinates of shape $(N, 3)$ representing $[x_k, y_k, z_k]$.
3. `h_radar`: 1D array of clearance distances (meters).
4. `z_baro`: 1D array of barometric altitudes (meters).
5. `elevations`: 1D array of reconstructed ground heights (meters) where $\mathbf{h} = \mathbf{z}_{\text{baro}} - \mathbf{h}_{\text{radar}}$.
6. `distance_along_track`: Cumulative distance traveled along the ground $s_k = \sum_{i=1}^{k} \|\Delta \mathbf{p}_{xy, i}\|$.

---

## ⚠️ 5. Real-World Avionics Imperfections (Preview for Phase 2 & 3)

In Phase 1, our simulator is a **deterministic, perfect-world laboratory**. However, as a GNC engineer, you must always understand the physical failure modes of real avionics:

### 1. Radar Beam Divergence (Footprint Averaging)
A real radar altimeter does not project an infinitely thin mathematical ray. It radiates an electromagnetic beam with a finite cone beamwidth (typically $\theta_{3\text{dB}} \approx 15^\circ - 45^\circ$).
- At $h = 1000\,\text{m}$ AGL, a $30^\circ$ cone illuminates a circular footprint on the ground of diameter:
  $$D = 2 \cdot h \cdot \tan\left(\frac{\theta}{2}\right) \approx 2 \cdot 1000 \cdot \tan(15^\circ) \approx 536\,\text{meters!}$$
- Over rough mountains, the leading-edge detector triggers on the **closest point inside the cone** (the nearest peak), not the point directly at nadir!

### 2. Vehicle Attitude Coupling (Roll $\phi$ & Pitch $\theta$)
If the aircraft banks into a $30^\circ$ roll turn:
$$\text{Slant Range} = \frac{h_{\text{AGL}}}{\cos(\phi) \cos(\theta)}$$
If the flight software fails to compensate for vehicle attitude using the IMU rotation matrix, the radar altimeter will report a clearance that is far too large, corrupting the profile.

### 3. Barometric Drift (Weather Fronts)
Barometric altimeters compute altitude from ambient static air pressure $P$:
$$z_{\text{baro}} = \frac{T_0}{L} \left[ 1 - \left(\frac{P}{P_0}\right)^{\frac{R \cdot L}{g_0 \cdot M}} \right]$$
If a weather front causes a local atmospheric pressure drop of only 3 millibars (hPa), the altimeter will shift by $\approx 27\,\text{meters}$! This introduces a constant vertical bias $\delta z_{\text{baro}}$ across the profile.

*(We will systematically model and conquer every one of these physical phenomena when we reach Phase 2 and Phase 3!)*

---

## 📐 6. Software Architecture Blueprint for Subphase 1.2

We will implement two modular, production-grade components in `src/terrain_matching/`:

### 1. `src/terrain_matching/simulation/trajectory.py`
```python
@dataclass
class TrajectoryPoint:
    t: float
    x: float
    y: float
    z: float

class FlightTrajectory:
    """Represents a continuous parameterized flight trajectory."""
    
    @classmethod
    def create_linear(
        cls,
        start_point: tuple[float, float, float],
        velocity_ms: float,
        heading_deg: float,
        duration_s: float,
        sample_rate_hz: float
    ) -> "FlightTrajectory":
        """Generates a straight-and-level trajectory."""
```

### 2. `src/terrain_matching/simulation/altimeter.py`
```python
@dataclass
class TerrainProfile:
    timestamps: np.ndarray      # shape (N,)
    trajectory: np.ndarray      # shape (N, 3): columns (x, y, z)
    h_radar: np.ndarray         # shape (N,)
    z_baro: np.ndarray          # shape (N,)
    elevations: np.ndarray      # shape (N,): z_baro - h_radar
    distance_along_track: np.ndarray  # shape (N,)

class AltimeterSimulator:
    """Simulates the altimeter triad over a given DEM and trajectory."""
    
    def __init__(self, dem: DigitalElevationModel) -> None:
        self.dem = dem
        
    def sample_profile(
        self,
        trajectory: FlightTrajectory,
        radar_noise_std: float = 0.0,
        baro_noise_std: float = 0.0,
        baro_bias: float = 0.0
    ) -> TerrainProfile:
        """Extracts an altimeter profile along the flight path."""
```

---

## 💡 7. Pedagogical Walkthrough & Physical Intuition Q&A

### 1. The Fundamental Invariant & Sensor Errors

#### A. Why does vehicle altitude cancel out?
Think of holding a long measuring tape from a ladder:
- The ladder puts your hands at a height of **$10\,\text{m}$ above sea level** ($z_{\text{baro}}$).
- You drop the tape down to the top of a hill, and it reads **$4\,\text{m}$ to the ground** ($h_{\text{radar}}$).
- The hill's height is obviously:
  $$10\,\text{m} - 4\,\text{m} = 6\,\text{m}$$

Now imagine an updraft pushes the aircraft $2\,\text{m}$ higher:
- Your baro altitude becomes **$12\,\text{m}$**.
- But because you are higher, the clearance down to the hill also increases to **$6\,\text{m}$**.
- The calculated hill height is still:
  $$12\,\text{m} - 6\,\text{m} = 6\,\text{m}$$

> **Key Takeaway**: Because the vehicle's true altitude appears with a positive sign in $z_{\text{baro}}$ and equally inside $h_{\text{radar}}$, subtracting them eliminates the vehicle’s vertical motion entirely. You are left purely with the shape of the mountain or hill below.

#### B. Baro Drift vs. Radar Noise (+15 m Baro Bias)
- **Radar Noise (Zero-mean white noise)**: Every radar ping flickers slightly (e.g. $+0.2\,\text{m}, -0.3\,\text{m}, +0.1\,\text{m}$). It looks like fuzzy static on top of your mountain profile.
- **Baro Bias ($+15\,\text{m}$ shift)**: Barometers read air pressure. If the regional weather pressure changes, your barometer might believe you are $15\,\text{m}$ higher than you actually are for the entire flight.
- **What happens to $h_{\text{terrain}}$?**  
  Every single point of your sampled profile is shifted upward by $+15\,\text{m}$ across the board:
  $$h_{\text{terrain}}(t) = (z_{\text{true}} + 15) - h_{\text{radar}} = h_{\text{true\_terrain}} + 15\,\text{m}$$
  The *shape* (peaks and valleys) is identical, but the entire elevation curve is lifted vertically by $15\,\text{m}$.  
  *(Crucial preview for Chapter 1.3: This is why a standard difference metric like Mean Absolute Difference struggles unless we use zero-mean or bias-invariant metrics!)*

---

### 2. Continuous Flight to Discrete Grid Sampling

- **The Problem**: A digital map (DEM) is a grid of discrete posts spaced every $30\,\text{m}$ (like trees planted in rows on an orchard at $x = 0, 30, 60, 90\,\dots$).
- When the aircraft flies at $v = 150\,\text{m/s}$ sampling at $f_s = 10\,\text{Hz}$, it takes a reading every $15\,\text{m}$:
  $$\Delta s = \frac{v}{f_s} = \frac{150}{10} = 15\,\text{m}$$
- Readings occur at $x = 0\,\text{m}, 15\,\text{m}, 30\,\text{m}, 45\,\text{m}\,\dots$
- At $x = 15\,\text{m}$, there is **no direct map post**—you are floating midway between the post at $0\,\text{m}$ and the post at $30\,\text{m}$!
- **How Bilinear Interpolation fixes this**:  
  It looks at the 4 neighboring grid posts surrounding the vehicle $(x, y)$, stretches a smooth hyperbolic paraboloid patch between them, and computes the exact height by taking a weighted average based on how close you are to each post. This allows you to query the DEM at **any continuous coordinate $(x, y)$ smoothly**.

---

### 3. Pitch & Roll: Why Tilting Matters

- When flying straight and level, the radar beam shoots straight down (at **nadir**). If the ground is $200\,\text{m}$ below you, the beam travels $200\,\text{m}$.
- But if the aircraft banks into a steep turn with roll angle $\phi$ and pitch angle $\theta$:
  1. **Slant Range**: The beam travels along a diagonal hypotenuse:
     $$h_{\text{slant}} = \frac{h_{\text{AGL}}}{\cos(\theta)\cos(\phi)}$$
     The sensor measures a longer distance than the true vertical clearance.
  2. **Ground Footprint Offset**: The laser spot strikes the ground at an offset $(x + \Delta x, y + \Delta y)$ far off to the side, rather than directly beneath the aircraft belly.

*(In Phase 1, we assume stabilized nadir-pointing; in Phases 2 and 3, we project slant range vectors using attitude direction cosine matrices).*

---

## 📚 8. Curated Aerospace Literature

1. 📄 **Hostetler, L. D. & Kohler, R. D. (1983)**: *"Transponder-Aided Terrain-Following Guidance and SITAN"*. Sandia National Laboratories Report SAND82-1405.  
   *(The definitive Sandia technical foundation for profile sampling and altimeter modeling in cruise vehicles).*
2. 📄 **Golden, J. P. (1980)**: *"Terrain Contour Matching (TERCOM) Applications"*. SPIE Image Processing for Missile Guidance, Vol. 238, pp. 10-18.  
   *(Details the length, spacing, and geometry of sampled terrain strips required for reliable cross-correlation fixes).*
3. 📖 **Kayton, M. & Fried, W. R. (1997)**: *"Avionics Navigation Systems"*, 2nd Ed. John Wiley & Sons.  
   *Chapter 6 (Altimeters and Depth Sensors) — covers radar beam cone geometry, leading-edge vs pulse tracking, and barometric lapse rate equations.*
