# Terrain-Relative Navigation (TRN) Master Guide & Progress Tracker

> **Single Source of Truth** for the entire Terrain Matching Mastery Curriculum.  
> This document governs agent behavior, curriculum architecture, mathematical prerequisites, implementation milestones, and real-time project progress.

---

## 🤖 Agent Operating Protocol & Persona Contract

> **CRITICAL DIRECTIVE FOR ANY AI AGENT / MODEL JOINING THIS CONVERSATION:**  
> Read this section before responding or executing any work.

1. **Role & Demeanor**:
   - You are a **distinguished Senior Principal Aerospace & Autonomous Systems Scientist** specializing in GNSS-denied navigation, state estimation, and terrain-aided navigation (TRN / SITAN / TERCOM / Deep Matchers).
   - You are an **exceptionally patient, rigorous, and inspiring teacher**.
   - Your student has a **Master's in Robotics and AI/ML**, knows Python and C++, but needs deliberate refreshers connecting foundational math and robotics concepts directly to physical terrain-aided navigation.
   - **Never rush.** Never assume understanding without verification. Never skip mathematical derivations or hand-wave physical intuition.

2. **The 5-Step Progressive Workflow (MANDATORY FOR EVERY SUBPHASE)**:
   For every subphase across all 4 phases (e.g. 1.1, 1.2, 1.3...), the agent and student must strictly execute this 5-step loop:

```
                      THE 5-STEP SUBPHASE LOOP
                      
  Step 1: Theory & Intuition Chapter in knowledge/
          (Story -> Real-World Analogy -> Step-by-Step Math -> Software Architecture -> Curated Papers)
                                     │
                                     ▼
  Step 2: Pedagogical Check & Conceptual Walkthrough
          (Discuss in chat, verify understanding of math, failure modes, and edge cases)
                                     │
                                     ▼
  Step 3: Clean, Modular Implementation in src/
          (Strict typing, docstrings, clean architecture, zero shortcuts)
                                     │
                                     ▼
  Step 4: Automated Testing & Verification in tests/
          (Comprehensive pytest test suites, edge case verification)
                                     │
                                     ▼
  Step 5: Progress Update & Semantic Git Commit
          (Update TRN_MASTER_GUIDE.md and provide exact conventional git commit message)
```

3. **No Unprompted Fast-Forwarding**:
   - Do **not** generate full-phase code implementations in one single reply.
   - Break every phase down into bite-sized, interactive learning units.
   - Always wait for the student's confirmation on Step 2 before writing code in Step 3.

---

## 🗺️ Architectural Roadmap: The 4 Progressive Phases

```
[Phase 1: Deterministic Matching]
  │   - 2D Digital Elevation Models (DEMs)
  │   - Profile extraction & Altimeter models
  │   - MAD (Mean Absolute Difference) & MSD (Mean Squared Difference)
  │   - Exhaustive grid search & Ambiguity surfaces
  ▼
[Phase 2: Classical TERCOM (Batch Matching with INS Drift)]
  │   - Inertial Navigation Systems (INS) Dead Reckoning
  │   - Double integration & Quadratic error propagation
  │   - Search window bounding boxes & Uncertainty ellipses
  │   - Terrain roughness metrics (Terrain Information Content)
  │   - Position fix generation & Batch trajectory update
  ▼
[Phase 3: Continuous Stochastic Filtering (SITAN)]
  │   - State-space navigation error dynamics
  │   - Sensor noise characterization (Gaussian white noise vs biases)
  │   - Non-linear terrain measurement models: h(x, y)
  │   - Extended Kalman Filter (EKF) with local terrain gradient linearization
  │   - Measurement Jacobians & Divergence detection
  │   - Non-linear alternatives: Particle Filters (SITAN-PF) & Rao-Blackwellization
  ▼
[Phase 4: Modern Cross-View Deep Visual Matching]
  │   - GPS-denied passive vision (Down-looking UAV camera vs Satellite Orthophoto)
  │   - Perspective vs Orthographic projective geometry
  │   - Scale, rotation, illumination, and seasonal domain shift
  │   - Feature detectors/descriptors & Homography estimation via RANSAC
  │   - Deep matchers (SuperPoint + LightGlue / Cross-View Transformers)
  │   - Geo-localization pipeline integration
```

---

## 📚 Detailed Curriculum Breakdown

### Phase 1: The Deterministic Foundation (Perfect World Simulator)
- **Concept**: If we have a known elevation grid and a sequence of clean altimeter measurements, how do we locate the vehicle deterministically?
- **Key Math**:
  - Matrix indexing vs Cartesian coordinates $(x, y) \leftrightarrow [row, col]$.
  - Discrete cost functions: Mean Absolute Difference ($MAD$), Mean Squared Difference ($MSD$), Normalized Cross-Correlation ($NCC$).
  - Surface correlation landscapes & global vs local minima.
- **Physical Dynamics**:
  - Flat/constant-altitude flight, down-looking laser/radar altimeter subtracting terrain height from barometric altitude:
    $$h_{terrain}(t) = z_{baro}(t) - h_{altimeter}(t)$$
- **Deliverables**:
  - `knowledge/01_deterministic_matching/` notes.
  - Synthetic DEM generator (Perlin noise, Gaussian hills, flat plains).
  - 1D profile matcher & 2D trajectory correlation engine.
  - Ambiguity & cost surface visualizer.
  - Full test suite verifying exact recovery under zero noise.

---

### Phase 2: TERCOM Pipeline (Batch Processing with INS Drift)
- **Concept**: Aircraft do not know their true path; they rely on INS (accelerometers + gyroscopes) which drift quadratically over time. TERCOM collects a batch of readings over an area of sufficient contour variation to compute a single correction fix.
- **Key Math**:
  - Inertial dead reckoning: $v(t) = \int a(t)dt$, $p(t) = \int v(t)dt$.
  - Drift modeling: Constant accelerometer bias $b_a \implies \Delta p(t) = \frac{1}{2} b_a t^2$.
  - Search window bounds determined by $3\sigma_{INS}$ error bounds.
  - Terrain suitability metrics: Standard deviation of elevation ($\sigma_h$), terrain roughness index ($TRI$), spatial auto-correlation length.
- **Physical Dynamics**:
  - Vehicle flies an INS-estimated waypoint track.
  - Radar altimeter samples terrain profile over a "TERCOM strip" (typically 20 to 100 samples).
  - Batch correlation matches strip over the INS uncertainty window.
  - Position fix resets the INS error accumulator.
- **Deliverables**:
  - `knowledge/02_tercom/` notes.
  - INS simulation engine with customizable noise and bias.
  - Strip terrain selection algorithm (roughness analysis).
  - TERCOM batch correlator with bounded search window.
  - Trajectory reset simulator & before/after error plots.

---

### Phase 3: Continuous Stochastic Filtering (SITAN Pipeline)
- **Concept**: Sandia Inertial Terrain-Aided Navigation (SITAN). Rather than waiting to collect long batches, update the vehicle state continuously at every altimeter tick using optimal recursive Bayesian filtering.
- **Key Math**:
  - State vector formulation: $\mathbf{x} = [\delta x, \delta y, \delta z, \delta v_x, \delta v_y, \delta v_z, b_a, \dots]^T$.
  - Linear state transition model: $\mathbf{x}_k = \mathbf{F}_{k-1}\mathbf{x}_{k-1} + \mathbf{w}_{k-1}$.
  - Non-linear measurement model: $z_k = h(x_k, y_k) + v_k$.
  - First-order Taylor linearization: $\mathbf{H}_k = \left[ \frac{\partial h}{\partial x}, \frac{\partial y}{\partial y}, -1, 0, \dots \right]$.
  - Terrain gradient computation via central differences / bilinear interpolation.
  - Extended Kalman Filter (EKF) prediction and update steps.
  - Fallback / Divergence detection when terrain linearity breaks (rough cliffs, gullies).
  - Sequential Monte Carlo (Particle Filtering) for multi-modal terrain matching.
- **Physical Dynamics**:
  - Real-time continuous flight over variable terrain.
  - Continuous covariance shrinking over rough terrain; covariance growth over flat terrain.
- **Deliverables**:
  - `knowledge/03_sitan_kalman_filtering/` notes.
  - Continuous state-space simulator.
  - Local DEM interpolator & gradient calculator.
  - SITAN EKF engine with adaptive covariance gating.
  - SITAN Particle Filter (PF) comparison module.

---

### Phase 4: Modern Visual Cross-View Matching (Deep TRN)
- **Concept**: Replacing or augmenting active radar/LiDAR with a passive optical camera. Match live oblique/downward UAV video/stills against satellite orthophotos in GPS-denied environments.
- **Key Math & ML**:
  - Projective Geometry: Pinhole camera model, intrinsic matrix $\mathbf{K}$, extrinsic matrix $[\mathbf{R} | \mathbf{t}]$.
  - Planar Homography: $\mathbf{x}_{sat} \sim \mathbf{H} \mathbf{x}_{uav}$.
  - Perspective-n-Point (PnP) problem.
  - Robust estimation via RANSAC.
  - Deep local feature extraction and matching (SuperPoint + LightGlue / LoFTR).
  - Cross-view geo-localization and metric pose recovery.
- **Physical Dynamics**:
  - UAV cruising at altitude with camera looking nadir or slightly oblique.
  - Real-time visual odometry fused with satellite reference tile matching.
- **Deliverables**:
  - `knowledge/04_cross_view_deep_matching/` notes.
  - Synthetic flight camera projection from textured DEMs.
  - Homography estimation & RANSAC geometric verification pipeline.
  - Deep matching integration with geographic coordinate mapping.
  - End-to-end GPS-denied visual navigation pipeline demonstration.

---

## 🌐 The Navigation Spectrum: How Concepts Interlock

In autonomous aerospace and robotics, multiple navigation paradigms work together in a hierarchical stack:

```
[Level 1: High Rate Propagator] (500 Hz)
  • Inertial Navigation Systems (INS) / Dead Reckoning (DR)
  • Integrates IMU (accelerometers & gyroscopes)
  • Provides smooth, high-rate attitude & velocity
  • Subject to quadratic (t^2) and cubic (t^3) drift

[Level 2: High-Rate Local Relative Estimator] (30 - 60 Hz)
  • Optical Flow (OF) & Visual-Inertial Odometry (VIO)
  • Tracks consecutive local camera frames (Same-View Matching)
  • Bounds short-term INS drift to ~0.5% - 1% of distance traveled
  • Still slowly drifts without global anchors; blind in fog/clouds/darkness

[Level 3: Global Absolute Anchor] (1 - 5 Hz)
  • Terrain-Relative Navigation (TRN: TERCOM & SITAN)
  • Cross-View Visual Matching (Drone camera vs. Satellite orthophotos)
  • Zero Accumulated Drift: Directly correlates with permanent Earth geography!
  • Resets accumulated INS and VIO error to zero!
```

### Production Architecture & Future Scope
In production-grade avionics, these subsystems are tightly fused using:
1. **Error-State Extended Kalman Filter (ES-EKF)** or **Factor Graph Optimization (GTSAM)**.
2. **Terrain Information Indexing**: Pre-flight calculation of terrain roughness to plan optimal "fix points" where TRN confidence is maximized.
3. **Real-time C++20 Embedded Port**: Microsecond-level vector search with SIMD/AVX2.
4. **Edge TensorRT / ONNX Runtime**: GPU-accelerated deep cross-view matching on embedded edge computers (e.g. NVIDIA Jetson Orin).

---

## 🎯 Current Status & Progress Tracker

| Phase | Module / Topic | Status | Knowledge Chapter | Code Implementation | Tests |
|---|---|---|---|---|---|
| **Phase 0** | The Story of Navigation & Why GPS Fails | 🟢 **COMPLETED** | [00_the_story_of_navigation_why_gps_fails.md](knowledge/00_foundations/00_the_story_of_navigation_why_gps_fails.md) | N/A | N/A |
| **Phase 0** | Navigation Concepts Demystified (DR, INS, VIO, TRN, SLAM) | 🟢 **COMPLETED** | [01_navigation_jargons_and_concepts_demystified.md](knowledge/00_foundations/01_navigation_jargons_and_concepts_demystified.md) | N/A | N/A |
| **Phase 0** | Sensor Physics & Altimeter Triad (Simplified) | 🟢 **COMPLETED** | [02_sensor_physics_and_altimeter_triad_simplified.md](knowledge/00_foundations/02_sensor_physics_and_altimeter_triad_simplified.md) | N/A | N/A |
| **Phase 0** | Math & Coordinate Frames Step-by-Step | 🟢 **COMPLETED** | [03_math_and_coordinate_frames_demystified.md](knowledge/00_foundations/03_math_and_coordinate_frames_demystified.md) | N/A | N/A |
| **Phase 0** | Foundations Synthesis & Bridge to Phase 1 | 🟢 **COMPLETED** | [04_foundations_synthesis_and_bridge_to_phase1.md](knowledge/00_foundations/04_foundations_synthesis_and_bridge_to_phase1.md) | N/A | N/A |
| **Phase 1** | 1.1 DEM Representation & Coordinate Grid Transformations | 🟢 **COMPLETED** | [01_dem_representation_and_coordinate_systems.md](knowledge/01_deterministic_matching/01_dem_representation_and_coordinate_systems.md) | [dem.py](src/terrain_matching/core/dem.py), [terrain_generator.py](src/terrain_matching/simulation/terrain_generator.py) | [test_dem.py](tests/test_dem.py) |
| **Phase 1** | 1.2 Altimeter Modeling & Profile Sampling | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 1** | 1.3 MAD & MSD Cost Functions & Metrics | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 1** | 1.4 Exhaustive 1D & 2D Search Algorithms | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 1** | 1.5 Ambiguity Analysis & Correlation Surfaces | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 2** | 2.1 INS Mechanics & Quadratic Error Drift | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 2** | 2.2 Terrain Information Content (Roughness Index) | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 2** | 2.3 Bounded Search Windows ($3\sigma$ Ellipses) | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 2** | 2.4 Batch TERCOM Correlator & Position Reset | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 3** | 3.1 State-Space Error Dynamics for TRN | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 3** | 3.2 DEM Gradient Linearization & Jacobians | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 3** | 3.3 SITAN Extended Kalman Filter (EKF) | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 3** | 3.4 Particle Filtering (SITAN-PF) for Multi-Modal Slopes | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 4** | 4.1 Pinhole Cameras & Projective Homography | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 4** | 4.2 Feature Detectors & RANSAC Geometry | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 4** | 4.3 Deep Cross-View Feature Matching | ⚪ Pending | [Pending] | [Pending] | [Pending] |
| **Phase 4** | 4.4 End-to-End GPS-Denied Drone Flight Pipeline | ⚪ Pending | [Pending] | [Pending] | [Pending] |

*Legend: ⚪ Pending | 🟡 IN PROGRESS | 🟢 COMPLETED*

---

## 🛠️ Public Repository Standards & Git Workflow

### Branching Strategy
- `main`: Production-ready, fully tested, documented code.
- `feat/phase1-deterministic-matching`: Phase 1 development branch.
- `feat/phase2-tercom-batch`: Phase 2 development branch.
- `feat/phase3-sitan-filtering`: Phase 3 development branch.
- `feat/phase4-visual-matching`: Phase 4 development branch.

### Commit Convention
Follow Conventional Commits:
- `docs(knowledge): ...` for theoretical notes and math derivations.
- `feat(core): ...` for new algorithms or modules.
- `test(core): ...` for unit tests and validation suites.
- `refactor(core): ...` for performance optimizations.

---

## 📍 Where We Are Right Now
- **Current Milestone**: Phase 1: Deterministic Matching ──▶ **Subphase 1.1 Complete! Ready for Subphase 1.2**.
- **Active Task**: Reviewing 3D/2D visualization artifacts of Subphase 1.1 and committing.
- **Next Up**: Subphase 1.2: Altimeter Modeling & Profile Sampling (Step 1: Theory chapter in `knowledge/`).
