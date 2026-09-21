# 🛰️ Terrain-Relative Navigation (TRN): From Classical Matching to Modern Deep Visual Odometry

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Status: Phase 1 In Progress](https://img.shields.io/badge/status-Phase%201%20In%20Progress-orange.svg)]()

A comprehensive, mathematically rigorous implementation of **Terrain-Relative Navigation (TRN)** and **Terrain Contour Matching** systems for autonomous aerial and space vehicles operating in **GPS-denied / electronic-warfare environments**.

Built progressively from deterministic cross-correlation to classical batch TERCOM, stochastic Kalman filtering (SITAN), and state-of-the-art deep cross-view visual matching.

---

## 🧭 Why Terrain-Relative Navigation?

In real-world defense, aerospace, and planetary exploration missions (such as Mars 2020 Lander Vision System or cruise missiles):
1. **GPS/GNSS is vulnerable**: Jamming, spoofing, atmospheric distortion, or planetary absence make satellite-based positioning unreliable or impossible.
2. **Inertial Navigation Systems (INS) drift**: Accelerometers and gyroscopes accumulate sensor bias over time through numerical integration ($v = \int a \, dt$, $p = \int v \, dt$), leading to quadratic error growth.
3. **The Terrain is the Ground Truth**: Earth (or Mars) topography does not lie. By measuring the terrain profile beneath the craft and correlating it against an onboard Digital Elevation Model (DEM) or satellite orthophoto, the vehicle can bound its drift and maintain pinpoint localization accuracy without external signals.

---

## 🗺️ The 4-Phase Progressive Architecture

```
                                    THE TRN EVOLUTION
                                    
  [Phase 1] Deterministic Matching   ──▶ Perfect world, exhaustive cost search (MAD / MSD / NCC)
         │
  [Phase 2] Classical TERCOM         ──▶ INS dead reckoning drift, strip roughness, bounded search
         │
  [Phase 3] Stochastic SITAN         ──▶ Continuous recursive filtering (EKF / PF), terrain linearization
         │
  [Phase 4] Deep Cross-View Vision   ──▶ Camera-to-satellite matching, homography, deep feature transformers
```

Detailed curriculum and real-time execution status are tracked in [TRN_MASTER_GUIDE.md](TRN_MASTER_GUIDE.md).

---

## 🌐 The Navigation Hierarchy: Where Does TRN Fit?

To build an unshakeable autonomous navigation system, multiple paradigms are organized into a tiered fusion pipeline:

| Paradigm | Frequency | Error Over Time | Global Map Needed? | Role in System |
|---|---|---|---|---|
| **Inertial Navigation (INS)** | 200–1000 Hz | Drifts rapidly ($t^2, t^3$) | No | High-rate, ultra-smooth attitude and velocity propagation. |
| **Visual-Inertial Odometry (VIO)** | 30–60 Hz | Drifts slowly (~1% distance) | No | Bounds short-term INS drift by tracking local frame-to-frame visual features. |
| **Terrain-Relative Navigation (TRN)** | 1–10 Hz | **ZERO DRIFT** | Yes (DEM) | **Absolute Anchor**: Matches ground elevation profiles to reset accumulated drift. |
| **Cross-View Satellite Matching** | 1–5 Hz | **ZERO DRIFT** | Yes (Satellite) | **Visual Anchor**: Matches live drone camera to georeferenced satellite orthophotos. |

```
 [IMU (500 Hz)] ──▶ INS Dead Reckoning ──┐
                                          │
 [Camera (30 Hz)] ──▶ Visual Odometry ───┼──▶ [ Hierarchical Sensor Fusion ] ──▶ Unjammable,
                                          │     (Factor Graphs / ES-EKF)         Zero-Drift Pose
 [Radar + DEM (1 Hz)] ──▶ TRN Fixes ─────┘
```

---

## 📂 Repository Structure

```text
terrain_matching/
├── TRN_MASTER_GUIDE.md         # Master curriculum, pedagogical protocol & live progress tracker
├── README.md                   # Public project overview & architectural introduction
├── .gitignore                  # Git ignore rules
├── knowledge/                  # Exhaustive theoretical & mathematical derivations
│   ├── 00_foundations/         # 4-part foundation: History, Jargons, Sensors, Math
│   ├── 01_deterministic_matching/
│   ├── 02_tercom/
│   ├── 03_sitan_kalman_filtering/
│   └── 04_cross_view_deep_matching/
├── src/                        # Modular, production-grade source code
│   └── terrain_matching/
│       ├── core/               # Math utilities, coordinate transforms, interpolators
│       ├── simulation/         # Synthetic DEMs, flight dynamics, sensor simulators
│       ├── deterministic/      # Phase 1: MAD, MSD, 1D/2D correlators
│       ├── tercom/             # Phase 2: Batch matching & INS drift model
│       ├── sitan/              # Phase 3: EKF & Particle Filter TRN
│       └── visual/             # Phase 4: Cross-view deep feature matchers
├── tests/                      # Pytest automated test suites for every phase
└── data/                       # Synthetic and real-world DEM / map datasets
```

---

## 🔭 Future Scope & Engineering Roadmap

- [ ] **Multi-Sensor Factor Graph Optimization**: Fusing IMU, VIO, and TRN batch fixes using GTSAM.
- [ ] **Real-Time C++20 Embedded Engine**: Low-latency vectorized DEM matching with SIMD/AVX2 instructions.
- [ ] **TensorRT / ONNX Deployment**: Edge acceleration of deep cross-view feature matchers on NVIDIA Jetson.
- [ ] **Terrain Suitability & Information Analysis**: Mission planner computing optimal TRN waypoints based on terrain roughness.

---

## 📚 Landmark Papers & Core Literature

- **Hostetler, L. D. (1978)**. *"Optimal Terrain-Aided Navigation Systems"*, AIAA Guidance and Control.
- **Golden, J. P. (1980)**. *"Terrain Contour Matching (TERCOM) Applications"*, SPIE, Vol. 238.
- **Forster, C., et al. (2017)**. *"On-Manifold Preintegration for Real-Time Visual--Inertial Odometry"*, IEEE Transactions on Robotics.
- **Johnson, A. E., et al. (2021)**. *"Lander Vision System for Mars 2020: Autonomous Terrain Relative Navigation for Touchdown"*, IEEE Aerospace.
- **Groves, P. D. (2015)**. *"Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"*, Artech House.

---

## 🚀 Getting Started

### Prerequisites
- Python 3.10+
- `numpy`, `scipy`, `matplotlib`, `pytest`

```bash
# Clone the repository
git clone https://github.com/<your-username>/terrain-relative-navigation.git
cd terrain-relative-navigation

# Create virtual environment
python -m venv .venv
.venv\Scripts\activate   # Windows

# Install dependencies
pip install -r requirements.txt
```

---

## 📖 Learning in Public

Follow the live development and read the mathematical explanations in the [`knowledge/`](knowledge/) directory. Every algorithm is backed by formal mathematical derivations, error bounds analysis, and unit tests.

---

## 📜 License
This project is licensed under the MIT License - see the LICENSE file for details.
