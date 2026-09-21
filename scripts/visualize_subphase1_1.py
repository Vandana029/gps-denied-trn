"""Visualizer script for Subphase 1.1 DEM representation and bilinear interpolation.

Generates 3D and 2D visual plots of synthetic terrain models and renders
a close-up demonstration of bilinear interpolation between grid posts.
"""

import sys
from pathlib import Path

# Add src/ to python path so script runs directly
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import matplotlib.pyplot as plt
import numpy as np
from terrain_matching.core.dem import DigitalElevationModel
from terrain_matching.simulation.terrain_generator import TerrainGenerator


def main() -> None:
    output_dir = Path("artifacts/subphase_1_1")
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Generating synthetic terrains...")

    # 1. Generate Gaussian Ridge (50x50 grid, 1000m x 1000m)
    gaussian_dem = TerrainGenerator.generate_gaussian_ridge(
        shape=(50, 50),
        base_elevation=100.0,
        peak_height=400.0,
        center_x=500.0,
        center_y=500.0,
        sigma_x=150.0,
        sigma_y=150.0,
        dx=20.0,
        dy=20.0,
    )

    # 2. Generate Rolling Hills (50x50 grid)
    hills_dem = TerrainGenerator.generate_rolling_hills(
        shape=(50, 50),
        base_elevation=250.0,
        primary_wavelength=400.0,
        primary_amplitude=70.0,
        dx=20.0,
        dy=20.0,
        seed=101,
    )

    # -------------------------------------------------------------
    # Plot 1: 3D Topographic Surfaces
    # -------------------------------------------------------------
    fig = plt.figure(figsize=(16, 7))

    # Subplot 1: Gaussian Mountain
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    x_g = np.linspace(gaussian_dem.x_min, gaussian_dem.x_max, gaussian_dem.cols)
    y_g = np.linspace(gaussian_dem.y_max, gaussian_dem.y_min, gaussian_dem.rows)
    X_g, Y_g = np.meshgrid(x_g, y_g)

    surf1 = ax1.plot_surface(
        X_g, Y_g, gaussian_dem.elevation_matrix, cmap="terrain", edgecolor="none", alpha=0.9
    )
    ax1.set_title("Synthetic Gaussian Peak (Single Prominent Mountain)", fontsize=13, pad=15)
    ax1.set_xlabel("East X (m)")
    ax1.set_ylabel("North Y (m)")
    ax1.set_zlabel("Elevation MSL (m)")
    fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=10, label="Altitude (m)")

    # Subplot 2: Multi-Frequency Rolling Hills
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    x_h = np.linspace(hills_dem.x_min, hills_dem.x_max, hills_dem.cols)
    y_h = np.linspace(hills_dem.y_max, hills_dem.y_min, hills_dem.rows)
    X_h, Y_h = np.meshgrid(x_h, y_h)

    surf2 = ax2.plot_surface(
        X_h, Y_h, hills_dem.elevation_matrix, cmap="gist_earth", edgecolor="none", alpha=0.9
    )
    ax2.set_title("Multi-Frequency Rolling Hills (3 Superimposed Octaves)", fontsize=13, pad=15)
    ax2.set_xlabel("East X (m)")
    ax2.set_ylabel("North Y (m)")
    ax2.set_zlabel("Elevation MSL (m)")
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10, label="Altitude (m)")

    plt.tight_layout()
    plot1_path = output_dir / "terrain_3d_surfaces.png"
    plt.savefig(plot1_path, dpi=200)
    plt.close()
    print(f"Saved 3D terrain plot to: {plot1_path}")

    # -------------------------------------------------------------
    # Plot 2: 2D Contour Topographic Navigation Charts
    # -------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    c1 = ax1.contourf(X_g, Y_g, gaussian_dem.elevation_matrix, levels=25, cmap="terrain")
    lines1 = ax1.contour(X_g, Y_g, gaussian_dem.elevation_matrix, levels=10, colors="black", linewidths=0.5)
    ax1.clabel(lines1, inline=True, fontsize=8)
    ax1.set_title("Topographic Contour Map: Gaussian Peak", fontsize=12)
    ax1.set_xlabel("East X (meters)")
    ax1.set_ylabel("North Y (meters)")
    fig.colorbar(c1, ax=ax1, label="Elevation (m)")

    c2 = ax2.contourf(X_h, Y_h, hills_dem.elevation_matrix, levels=25, cmap="terrain")
    lines2 = ax2.contour(X_h, Y_h, hills_dem.elevation_matrix, levels=10, colors="black", linewidths=0.5)
    ax2.clabel(lines2, inline=True, fontsize=8)
    ax2.set_title("Topographic Contour Map: Rolling Hills", fontsize=12)
    ax2.set_xlabel("East X (meters)")
    ax2.set_ylabel("North Y (meters)")
    fig.colorbar(c2, ax=ax2, label="Elevation (m)")

    plt.tight_layout()
    plot2_path = output_dir / "topographic_contour_maps.png"
    plt.savefig(plot2_path, dpi=200)
    plt.close()
    print(f"Saved 2D contour map to: {plot2_path}")

    # -------------------------------------------------------------
    # Plot 3: Bilinear Interpolation Microscope (Seeing the math in action)
    # -------------------------------------------------------------
    # Create the exact 2x2 cell from our test: Q11=100, Q12=120, Q21=140, Q22=160
    mini_grid = np.array([
        [100.0, 120.0],  # y=100m (Row 0): x=0m -> 100m, x=50m -> 120m
        [140.0, 160.0],  # y=50m  (Row 1): x=0m -> 140m, x=50m -> 160m
    ])
    mini_dem = DigitalElevationModel(mini_grid, x_min=0.0, y_max=100.0, dx=50.0, dy=50.0)

    # Sample a dense 60x60 grid inside this single 50m x 50m cell
    dense_x = np.linspace(0.0, 50.0, 60)
    dense_y = np.linspace(50.0, 100.0, 60)
    dense_X, dense_Y = np.meshgrid(dense_x, dense_y)
    dense_Z = np.empty_like(dense_X)

    for i in range(60):
        for j in range(60):
            dense_Z[i, j] = mini_dem.get_elevation(dense_X[i, j], dense_Y[i, j], method="bilinear")

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(1, 1, 1, projection="3d")

    # Plot the smooth interpolated sheet
    surf = ax.plot_surface(dense_X, dense_Y, dense_Z, cmap="viridis", alpha=0.7, edgecolor="gray", linewidth=0.2)

    # Plot the 4 corner grid posts
    corners_x = [0.0, 50.0, 0.0, 50.0]
    corners_y = [100.0, 100.0, 50.0, 50.0]
    corners_z = [100.0, 120.0, 140.0, 160.0]
    ax.scatter(corners_x, corners_y, corners_z, color="red", s=80, label="Grid Posts (In Memory)", zorder=10)

    # Plot the drone at the center: x=25m, y=75m, z=130m
    drone_elev = mini_dem.get_elevation(25.0, 75.0, method="bilinear")
    ax.scatter([25.0], [75.0], [drone_elev], color="magenta", s=150, marker="^", label=f"Drone @ (25, 75) = {drone_elev:.1f}m", zorder=15)

    ax.set_title("Bilinear Interpolation Microscope: Smooth Surface Between 4 Posts", fontsize=12)
    ax.set_xlabel("East X (m)")
    ax.set_ylabel("North Y (m)")
    ax.set_zlabel("Elevation MSL (m)")
    ax.legend(loc="upper left")

    plt.tight_layout()
    plot3_path = output_dir / "bilinear_interpolation_microscope.png"
    plt.savefig(plot3_path, dpi=200)
    plt.close()
    print(f"Saved Bilinear microscope plot to: {plot3_path}")
    print("\nVisualizer finished successfully! Check artifacts/subphase_1_1/")


if __name__ == "__main__":
    main()
