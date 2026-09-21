# Chapter 0.3: The Math & Coordinate Frames — Step-by-Step

> **Topic**: Demystifying the Mathematics Behind Terrain-Relative Navigation.  
> **Prerequisites**: High-school algebra and basic calculus. No scary Greek symbols without a friendly explanation!

---

## 📐 1. Why Do Errors Grow Quadratically ($t^2$) and Cubically ($t^3$)?

In navigation papers, you often see:  
*"Accelerometer bias causes quadratic error growth $\frac{1}{2} b t^2$, and gyro bias causes cubic error growth $\frac{1}{6} g \epsilon t^3$."*

Let's demystify exactly where these numbers come from. It is nothing more than basic calculus!

### A. The Accelerometer Bias ($t^2$) Derivation
Suppose an accelerometer has a tiny, constant error $b_a = 0.01\,\text{m/s}^2$ (it thinks you are accelerating slightly forward, but you are not).

1. **Acceleration Error**:
   $$a_{\text{error}}(t) = b_a$$
2. **Velocity Error** (Integrate acceleration once):
   $$v_{\text{error}}(t) = \int_0^t a_{\text{error}}(\tau) d\tau = \int_0^t b_a d\tau = b_a \cdot t$$
   *(After 100 seconds, the computer thinks you are moving at $0.01 \times 100 = 1\,\text{m/s}$!)*
3. **Position Error** (Integrate velocity again):
   $$p_{\text{error}}(t) = \int_0^t v_{\text{error}}(\tau) d\tau = \int_0^t (b_a \tau) d\tau = \frac{1}{2} b_a t^2$$

#### The Concrete Numbers:
Look at what happens over time with a tiny bias of just $0.01\,\text{m/s}^2$:
- After **10 seconds**: $\frac{1}{2}(0.01)(10^2) = \mathbf{0.5\,\text{meters}}$ (Tiny, no big deal)
- After **1 minute (60s)**: $\frac{1}{2}(0.01)(3600) = \mathbf{18\,\text{meters}}$ (Noticeable)
- After **10 minutes (600s)**: $\frac{1}{2}(0.01)(360,000) = \mathbf{1,800\,\text{meters}}$ (**Nearly 2 kilometers of error!**)

---

### B. The Gyroscope Tilt ($t^3$) Derivation
A gyroscope measures rotation. If a gyroscope has a small drift $\epsilon$ (e.g., $0.001\,\text{rad/s}$), it slowly miscalculates the drone's tilt angle:

$$\theta_{\text{error}}(t) = \epsilon \cdot t$$

Now, here is the aerospace catch: **Gravity ($g \approx 9.81\,\text{m/s}^2$) points straight down.**  
If the drone computer thinks it is tilted by angle $\theta_{\text{error}}$, it calculates that a component of gravity is pulling horizontally:

$$a_{\text{horizontal}} = g \cdot \sin(\theta_{\text{error}}) \approx g \cdot (\epsilon \cdot t) \quad (\text{using small-angle approx } \sin x \approx x)$$

Now integrate that horizontal acceleration to find position error:
1. **Velocity Error**:
   $$v_{\text{error}}(t) = \int_0^t (g \epsilon \tau) d\tau = \frac{1}{2} g \epsilon t^2$$
2. **Position Error**:
   $$p_{\text{error}}(t) = \int_0^t \left( \frac{1}{2} g \epsilon \tau^2 \right) d\tau = \frac{1}{6} g \epsilon t^3$$

Because of that $t^3$ power, gyro drift will completely destroy an inertial navigation system within minutes if not corrected by an absolute anchor like Terrain Matching!

---

## 🧭 2. Coordinate Frames Made Visual

An aircraft navigates between three different worlds (coordinate frames):

```
1. Global Earth Frame (WGS-84 / ECEF)
   - Latitude, Longitude, Altitude above sea level.
   - Example: 37.7749° N, 122.4194° W, 450m.
           │
           ▼
2. Local Navigation Frame (NED: North - East - Down)
   - A flat tangent plane placed directly where the mission started.
   - +X points North (meters)
   - +Y points East (meters)
   - +Z points Down towards Earth center (meters)
           │
           ▼
3. Body Frame (The Drone's Belly)
   - Attached to the aircraft fuselage.
   - +X points out the nose (Forward)
   - +Y points out the right wing (Right)
   - +Z points out the floor/landing gear (Down)
```

### What is the Rotation Matrix $\mathbf{R}_b^n$?
In plain English: The Rotation Matrix is a 3x3 mathematical machine that takes an arrow described in the **Body Frame** and translates it into the **Local Navigation (NED) Frame**.

$$\mathbf{v}_{\text{NED}} = \mathbf{R}_b^n \cdot \mathbf{v}_{\text{Body}}$$

#### Why Does This Matter for Our Altimeter?
Our radar altimeter is mounted on the belly of the drone. In the body frame, its laser beam shoots straight down along body $+Z$:

$$\mathbf{v}_{\text{beam, body}} = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$

If the drone is banking (rolling $25^\circ$) to turn around a mountain:
- The laser beam is **NOT** pointing straight down to Earth! It is shooting diagonally into the hillside.
- By multiplying by $\mathbf{R}_b^n$, our navigation computer knows the true 3D vector of the laser beam in the world, so we can calculate the exact $(X, Y, Z)$ ground intersection point!

---

## 🗺️ 3. The Digital Elevation Model (DEM): Matrices vs. Physical Space

A Digital Elevation Model (DEM) is stored as a 2D NumPy array or C++ matrix:

$$\mathbf{M} \in \mathbb{R}^{\text{rows} \times \text{cols}}$$

Here is the most common bug engineers make:
- In Cartesian physics: $(X, Y)$ usually means $X = \text{East}$ and $Y = \text{North}$.
- In Computer Science matrices: $\mathbf{M}[r, c]$ where $r = \text{Row (Vertical)}$ and $c = \text{Column (Horizontal)}$.
- Notice that in images/matrices, **Row 0 is at the top and rows increase downwards**, whereas in North-East coordinates, **North increases upwards**!

```
        c = 0        c = 1        c = 2        ...     c = (Cols-1)  ──▶ East (+X)
r = 0  [120.5m,     122.0m,      125.1m,       ... ]  (Northmost edge)
r = 1  [118.2m,     120.8m,      124.0m,       ... ]
r = 2  [115.0m,     119.3m,      122.5m,       ... ]
  │
  ▼
South (-Y)
```

### The Transformation Formula:
If your DEM starts at physical origin $(X_{\text{origin}}, Y_{\text{origin}})$ with grid spacing $\Delta x$ and $\Delta y$ (e.g., 30 meters per pixel):

$$\text{Column } c = \frac{X_{\text{drone}} - X_{\text{origin}}}{\Delta x}$$

$$\text{Row } r = \frac{Y_{\text{origin}} - Y_{\text{drone}}}{\Delta y}$$

---

## 📐 4. Bilinear Interpolation: Finding Elevation Between Grid Points

A drone does not fly in 30-meter teleporting steps. It flies smoothly at real coordinates like $X = 142.7\,\text{m}, Y = 89.3\,\text{m}$.  
That means its position lands **between four grid posts** in our DEM:

```
      (r, c) = Q11 ───────────── (r, c+1) = Q12
           │                         │
           │        • (x, y)         │
           │     Drone is here       │
           │                         │
     (r+1, c) = Q21 ───────────── (r+1, c+1) = Q22
```

How do we find the height at $(x, y)$?  
We use a **weighted average** of the four surrounding heights, where the closer the drone is to a corner post, the more weight that post gets!

Let normalized offsets be:
$$\alpha = c_{\text{float}} - \lfloor c_{\text{float}} \rfloor \in [0, 1)$$
$$\beta = r_{\text{float}} - \lfloor r_{\text{float}} \rfloor \in [0, 1)$$

Then:
$$h(x, y) = (1-\beta) \left[ (1-\alpha) Q_{11} + \alpha Q_{12} \right] + \beta \left[ (1-\alpha) Q_{21} + \alpha Q_{22} \right]$$

This gives a smooth, continuous terrain surface!

---

## 📚 Recommended Reading, Papers & Online Textbooks

1. **Farrell, J. A. (2008)**. *"Aided Navigation: GPS with High Rate Sensors"*, McGraw-Hill. *(Chapters 2 & 5 give the best step-by-step coordinate transformation and inertial error propagation derivations in the literature).*
2. **Titterton, D. H., & Weston, J. L. (2004)**. *"Strapdown Inertial Navigation Technology"*, IET Radar, Sonar and Navigation.
3. 📺 **Brian Douglas (Control Systems Lectures)**: *"Understanding the Kalman Filter and Navigation Frames"* (YouTube - fantastic visual animated engineering series).
