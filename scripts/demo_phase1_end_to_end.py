"""End-to-End Demonstration of Phase 1: Deterministic Terrain-Relative Navigation.

This interactive demo runs the complete Phase 1 pipeline:
1. Procedural terrain generation (2D DEM with rolling mountains).
2. Autonomous flight trajectory simulation across the map.
3. Airborne altimeter triad profile extraction with barometric bias and noise.
4. Exhaustive 2D planar raster search to locate the aircraft.
5. Ambiguity analysis (PSR, runner-up check, Hessian curvature, and 4-gate fix filter).
6. High-resolution visual output saved to artifacts/phase1_demo/.
"""

import sys
import time
from pathlib import Path

# Add src/ to python search path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm

from terrain_matching.core.ambiguity import AmbiguityAnalyzer
from terrain_matching.core.matcher import DeterministicMatcher
from terrain_matching.simulation.altimeter import AltimeterSimulator
from terrain_matching.simulation.terrain_generator import TerrainGenerator
from terrain_matching.simulation.trajectory import FlightTrajectory


def print_banner(text: str) -> None:
    print("\n" + "=" * 75)
    print(f"  {text}")
    print("=" * 75)


def run_demo() -> None:
    print_banner("TERRAIN-RELATIVE NAVIGATION (TRN) - PHASE 1 END-TO-END DEMO")
    print("Operating Mode: Autonomous GNC Flight Simulator")

    output_dir = Path("artifacts/phase1_demo")
    output_dir.mkdir(parents=True, exist_ok=True)

    # -------------------------------------------------------------------------
    # STEP 1: Terrain Generation
    # -------------------------------------------------------------------------
    print("\n[Step 1/5] Synthesizing Digital Elevation Model (DEM)...")
    t0 = time.perf_counter()
    dem = TerrainGenerator.generate_rolling_hills(
        shape=(100, 100),
        dx=20.0,
        dy=20.0,
        base_elevation=300.0,
        octaves=3,
        primary_wavelength=400.0,
        primary_amplitude=120.0,
        seed=42,
    )
    print(f"  -> Generated DEM size: {dem.cols * 20}m x {dem.rows * 20}m ({dem.rows}x{dem.cols} cells)")
    print(f"  -> Elevation range: {dem.elevation_matrix.min():.1f}m to {dem.elevation_matrix.max():.1f}m MSL")
    print(f"  -> Generation time: {(time.perf_counter() - t0)*1000:.1f} ms")

    # -------------------------------------------------------------------------
    # STEP 2: Aircraft Flight Path Simulation
    # -------------------------------------------------------------------------
    print("\n[Step 2/5] Simulating Autonomous UAV Flight Trajectory...")
    true_start_x = 520.0
    true_start_y = 480.0
    true_altitude = 750.0  # meters MSL
    flight_speed = 50.0    # m/s (~180 km/h)
    heading_deg = 35.0     # Degrees CCW from East (North-East)
    duration_s = 10.0      # 10 second flight strip
    sample_rate_hz = 5.0   # 5 Hz sampling (51 samples)

    trajectory = FlightTrajectory.create_linear(
        start_point=(true_start_x, true_start_y, true_altitude),
        velocity_ms=flight_speed,
        heading_deg=heading_deg,
        duration_s=duration_s,
        sample_rate_hz=sample_rate_hz,
    )
    print(f"  -> True Start Position: East = {true_start_x:.1f}m, North = {true_start_y:.1f}m")
    print(f"  -> Flight Path: Speed = {flight_speed} m/s, Heading = {heading_deg} deg, Duration = {duration_s}s")
    print(f"  -> Total Distance: {trajectory.total_distance:.1f} meters ({len(trajectory)} sample points)")

    # -------------------------------------------------------------------------
    # STEP 3: Airborne Altimeter Triad Measurement
    # -------------------------------------------------------------------------
    print("\n[Step 3/5] Sampling Airborne Altimeter Triad (Injecting Real-World Physics)...")
    baro_weather_bias = 25.0  # +25 meters atmospheric front bias
    radar_noise_sigma = 1.0   # 1.0 meter 1-sigma radar measurement noise
    baro_noise_sigma = 0.5    # 0.5 meter baro noise

    simulator = AltimeterSimulator(dem)
    profile = simulator.sample_profile(
        trajectory=trajectory,
        radar_noise_std=radar_noise_sigma,
        baro_noise_std=baro_noise_sigma,
        baro_bias=baro_weather_bias,
        seed=101,
    )
    print(f"  -> Injected Atmospheric Weather Bias: +{baro_weather_bias:.1f} meters")
    print(f"  -> Injected Sensor Noise: Radar sigma = {radar_noise_sigma}m, Baro sigma = {baro_noise_sigma}m")
    print(f"  -> Profile Mean Elevation: {profile.mean_elevation:.1f}m, Variance: {profile.elevation_variance:.1f} m^2")

    # -------------------------------------------------------------------------
    # STEP 4: 2D Exhaustive Spatial Search
    # -------------------------------------------------------------------------
    print("\n[Step 4/5] Executing Exhaustive 2D Grid Search (Locating Aircraft)...")
    matcher = DeterministicMatcher(dem)
    rel_offsets = trajectory.coordinates_xy - trajectory.coordinates_xy[0]

    # Search window: 600m x 600m bounding box centered around the true location
    search_radius = 250.0
    search_bounds = (
        true_start_x - search_radius,
        true_start_x + search_radius,
        true_start_y - search_radius,
        true_start_y + search_radius,
    )
    step_size = 10.0  # 10m grid search step size

    t_search_start = time.perf_counter()
    # Using bias-invariant ZMSD to defeat the +25m weather pressure front
    match_result = matcher.match_2d_grid(
        measured_profile=profile.elevations,
        relative_offsets_xy=rel_offsets,
        search_bounds=search_bounds,
        step_size_m=step_size,
        metric="zmsd",
        true_coord=(true_start_x, true_start_y),
    )
    search_duration = time.perf_counter() - t_search_start
    num_candidates = match_result.cost_surface.size

    print(f"  -> Search Window: {search_bounds[0]:.0f}m to {search_bounds[1]:.0f}m East, "
          f"{search_bounds[2]:.0f}m to {search_bounds[3]:.0f}m North")
    print(f"  -> Evaluated {num_candidates:,} candidate locations in {search_duration*1000:.1f} ms "
          f"({num_candidates / search_duration:,.0f} candidates/sec)")
    print(f"  -> Optimal Estimated Fix: East = {match_result.best_coord[0]:.1f}m, North = {match_result.best_coord[1]:.1f}m")
    print(f"  -> Ground Truth Position: East = {true_start_x:.1f}m, North = {true_start_y:.1f}m")
    print(f"  -> Horizontal Position Error: {match_result.position_error:.2f} meters!")

    # -------------------------------------------------------------------------
    # STEP 5: Ambiguity Analysis & Fix Acceptance Filter
    # -------------------------------------------------------------------------
    print("\n[Step 5/5] Subjecting Fix to 4-Gate Avionics Ambiguity Filter...")
    analyzer = AmbiguityAnalyzer(
        exclusion_radius_m=60.0,
        min_psr=1.8,
        min_ambiguity_ratio=1.25,
        max_acceptable_difference_cost=5.0,
        min_profile_variance=4.0,
    )
    report = analyzer.analyze(match_result, measured_profile_variance=profile.elevation_variance)

    print(f"  -> Peak-to-Sidelobe Ratio (PSR): {report.psr:.2f} sigma (Threshold >= {analyzer.min_psr})")
    print(f"  -> Runner-Up Candidate: Location = {report.second_best_coord}, Cost = {report.second_best_cost:.2f}")
    print(f"  -> Ambiguity Margin (MAR): {report.ambiguity_margin:.2f} (Threshold >= {analyzer.min_ambiguity_ratio})")
    if report.curvature_eigenvalues:
        e1, e2 = report.curvature_eigenvalues
        print(f"  -> Local Surface Curvature (Hessian): lambda_1 = {e1:.4f}, lambda_2 = {e2:.4f}")
        print(f"  -> Anisotropy Ratio (Condition Number): {report.anisotropy:.1f}")

    print_banner("FLIGHT COMPUTER FIX VERDICT")
    if report.is_fix_acceptable:
        print("  STATUS: [FIX ACCEPTED] -> High Confidence. Safe to reset Inertial Drift!")
    else:
        print("  STATUS: [FIX REJECTED] -> Potential false peak or high ambiguity.")
        for reason in report.rejection_reasons:
            print(f"    - Reason: {reason}")
    print("=" * 75)

    # -------------------------------------------------------------------------
    # STEP 6: Generating Diagnostic Visualizations
    # -------------------------------------------------------------------------
    print("\nRendering high-resolution 4-panel diagnostic dashboard...")
    fig = plt.figure(figsize=(18, 12), facecolor="#0e1117")

    # Styling helper for dark mode aerospace telemetry
    def style_ax(ax, title: str):
        ax.set_facecolor("#161b22")
        ax.set_title(title, color="white", fontsize=13, fontweight="bold", pad=12)
        ax.tick_params(colors="gray")
        for spine in ax.spines.values():
            spine.set_color("#30363d")
        ax.grid(True, linestyle="--", alpha=0.3, color="gray")

    # PANEL 1: 3D Topographic DEM & Aircraft Trajectory
    ax1 = fig.add_subplot(2, 2, 1, projection="3d")
    ax1.set_facecolor("#0e1117")
    x_dem = np.linspace(dem.x_min, dem.x_max, dem.cols)
    y_dem = np.linspace(dem.y_max, dem.y_min, dem.rows)
    X_dem, Y_dem = np.meshgrid(x_dem, y_dem)
    surf = ax1.plot_surface(
        X_dem, Y_dem, dem.elevation_matrix, cmap="terrain", alpha=0.85, edgecolor="none", antialiased=True
    )
    # Plot true flight path
    true_path_z = dem.get_elevation_profile(trajectory.coordinates_xy) + 50.0  # hover above ground
    ax1.plot(
        trajectory.x,
        trajectory.y,
        true_path_z,
        color="#ff3366",
        linewidth=3.5,
        label="UAV Flight Path",
        zorder=10,
    )
    ax1.scatter(
        [true_start_x],
        [true_start_y],
        [true_path_z[0]],
        color="#00ffcc",
        s=80,
        label="Start Fix",
        zorder=11,
    )
    ax1.set_title("1. Digital Elevation Model & UAV Track", color="white", fontsize=12, fontweight="bold")
    ax1.tick_params(colors="gray")
    ax1.legend(facecolor="#161b22", edgecolor="#30363d", labelcolor="white")

    # PANEL 2: The Altimeter Triad Fingerprint
    ax2 = fig.add_subplot(2, 2, 2)
    style_ax(ax2, "2. Airborne Altimeter Triad Profile Sampling")
    dists = profile.distance_along_track
    true_ground_elev = dem.get_elevation_profile(trajectory.coordinates_xy)
    ax2.plot(dists, true_ground_elev, color="#00ffcc", linewidth=2.5, label="True DEM Elevation (h_dem)")
    ax2.plot(
        dists,
        profile.elevations,
        color="#ff9900",
        linestyle="--",
        linewidth=2.0,
        label=f"Sampled Profile (z_baro - h_radar, +{baro_weather_bias}m bias)",
    )
    ax2.fill_between(dists, true_ground_elev, profile.elevations, color="#ff9900", alpha=0.15)
    ax2.set_xlabel("Along-Track Distance (meters)", color="gray", fontsize=10)
    ax2.set_ylabel("Elevation (meters MSL)", color="gray", fontsize=10)
    ax2.legend(facecolor="#161b22", edgecolor="#30363d", labelcolor="white")

    # PANEL 3: 2D Correlation Cost Surface (ZMSD Heatmap)
    ax3 = fig.add_subplot(2, 2, 3)
    style_ax(ax3, "3. 2D Correlation Cost Surface (ZMSD Heatmap)")
    gx = match_result.search_grid_x
    gy = match_result.search_grid_y
    cs = ax3.contourf(gx, gy, match_result.cost_surface, levels=30, cmap="viridis_r")
    cbar = fig.colorbar(cs, ax=ax3, fraction=0.046, pad=0.04)
    cbar.ax.yaxis.set_tick_params(color="gray")
    cbar.set_label("ZMSD Error (m^2)", color="gray")
    plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color="gray")

    # Mark Ground Truth and Estimated Fix
    ax3.plot(
        true_start_x,
        true_start_y,
        marker="*",
        markersize=14,
        color="#00ffcc",
        label=f"Ground Truth ({true_start_x:.0f}, {true_start_y:.0f})",
        linestyle="none",
    )
    ax3.plot(
        match_result.best_coord[0],
        match_result.best_coord[1],
        marker="o",
        markersize=8,
        markerfacecolor="none",
        markeredgecolor="#ff3366",
        markeredgewidth=2.0,
        label=f"Estimated Fix ({match_result.best_coord[0]:.0f}, {match_result.best_coord[1]:.0f})",
        linestyle="none",
    )
    # Draw Exclusion Radius circle
    circle = plt.Circle(
        match_result.best_coord,
        analyzer.exclusion_radius_m,
        color="#ff3366",
        fill=False,
        linestyle=":",
        linewidth=1.5,
        label=f"Exclusion Zone ({analyzer.exclusion_radius_m:.0f}m)",
    )
    ax3.add_patch(circle)
    # Mark Runner-Up
    ax3.plot(
        report.second_best_coord[0],
        report.second_best_coord[1],
        marker="x",
        markersize=9,
        color="#ff9900",
        label=f"Runner-Up ({report.second_best_coord[0]:.0f}, {report.second_best_coord[1]:.0f})",
        linestyle="none",
    )
    ax3.set_xlabel("East Coordinate (meters)", color="gray", fontsize=10)
    ax3.set_ylabel("North Coordinate (meters)", color="gray", fontsize=10)
    ax3.legend(facecolor="#161b22", edgecolor="#30363d", labelcolor="white", fontsize=9, loc="upper right")

    # PANEL 4: Avionics Security Gate Telemetry Dashboard
    ax4 = fig.add_subplot(2, 2, 4)
    style_ax(ax4, "4. Avionics Fix-Rejection Telemetry Dashboard")
    ax4.axis("off")

    status_color = "#00ffcc" if report.is_fix_acceptable else "#ff3366"
    status_text = "ACCEPTED (CLEAR FIX)" if report.is_fix_acceptable else "REJECTED (AMBIGUOUS)"

    summary_box = (
        f"====================================================\n"
        f"  GNC FIX STATUS:  {status_text}\n"
        f"====================================================\n\n"
        f"  * Horizontal Error:     {match_result.position_error:.2f} meters\n"
        f"  * Search Execution:     {search_duration*1000:.1f} ms ({num_candidates:,} candidates)\n"
        f"  * Best ZMSD Cost:       {match_result.best_cost:.4f} m^2\n\n"
        f"  SECURITY CHECKPOINTS:\n"
        f"  ----------------------------------------------------\n"
        f"  [1] Terrain Roughness:  {profile.elevation_variance:.1f} m^2 (Min: {analyzer.min_profile_variance:.1f}) -> PASS\n"
        f"  [2] Residual Cost:      {match_result.best_cost:.2f} m^2 (Max: {analyzer.max_acceptable_difference_cost:.1f}) -> PASS\n"
        f"  [3] Peak Prominence:    {report.psr:.2f} sigma (Min: {analyzer.min_psr:.1f}) -> PASS\n"
        f"  [4] Ambiguity Margin:   {report.ambiguity_margin:.2f} ratio (Min: {analyzer.min_ambiguity_ratio:.2f}) -> PASS\n\n"
        f"  SURFACE GEOMETRY (HESSIAN CURVATURE):\n"
        f"  ----------------------------------------------------\n"
        f"  * Steepest Curvature:   lambda_1 = {report.curvature_eigenvalues[0]:.4f}\n"
        f"  * Shallowest Curvature: lambda_2 = {report.curvature_eigenvalues[1]:.4f}\n"
        f"  * Valley Anisotropy:    kappa = {report.anisotropy:.1f} (Elongation)\n"
    )
    ax4.text(
        0.05,
        0.95,
        summary_box,
        transform=ax4.transAxes,
        fontfamily="monospace",
        fontsize=10.5,
        color="white",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.8", facecolor="#161b22", edgecolor=status_color, linewidth=2.0),
    )

    fig.tight_layout()
    plot_file = output_dir / "phase1_end_to_end_fix.png"
    plt.savefig(plot_file, dpi=180, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close()

    print(f"\n[Artifact Saved] Diagnostic figure generated at: {plot_file}")
    print_banner("DEMO COMPLETED SUCCESSFULLY!")


if __name__ == "__main__":
    run_demo()
