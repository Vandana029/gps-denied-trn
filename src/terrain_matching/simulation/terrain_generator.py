"""Synthetic terrain generator for reproducible TRN simulation and testing.

This module provides tools to procedurally generate synthetic landscapes,
ranging from flat featureless plains (to test ambiguity) to Gaussian ridges
and complex multi-frequency rolling hills.
"""

from __future__ import annotations

import numpy as np
from terrain_matching.core.dem import DigitalElevationModel


class TerrainGenerator:
    """Procedural generator for synthetic Digital Elevation Models."""

    @staticmethod
    def generate_flat(
        shape: tuple[int, int] = (100, 100),
        elevation: float = 200.0,
        x_min: float = 0.0,
        y_min: float = 0.0,
        dx: float = 10.0,
        dy: float = 10.0,
        y_max: float | None = None,
    ) -> DigitalElevationModel:
        """Generate a perfectly flat terrain (e.g., salt flat or desert).

        Used to demonstrate and test the 'Desert Paradox' and ambiguity detection.

        Args:
            shape: (rows, cols) grid dimensions.
            elevation: Constant elevation in meters above sea level.
            x_min: Western boundary in meters.
            y_min: Southern boundary in meters.
            dx: Grid spacing along X in meters.
            dy: Grid spacing along Y in meters.
            y_max: Optional explicit Northern boundary.

        Returns:
            DigitalElevationModel instance with uniform elevation.
        """
        grid = np.full(shape, fill_value=float(elevation), dtype=np.float64)
        return DigitalElevationModel(grid, x_min=x_min, y_min=y_min, y_max=y_max, dx=dx, dy=dy)

    @staticmethod
    def generate_gaussian_ridge(
        shape: tuple[int, int] = (100, 100),
        base_elevation: float = 100.0,
        peak_height: float = 400.0,
        center_x: float | None = None,
        center_y: float | None = None,
        sigma_x: float = 150.0,
        sigma_y: float = 150.0,
        x_min: float = 0.0,
        y_min: float = 0.0,
        dx: float = 10.0,
        dy: float = 10.0,
        y_max: float | None = None,
    ) -> DigitalElevationModel:
        """Generate a landscape with a prominent Gaussian mountain peak or ridge.

        Ideal for deterministic matching tests where a single, sharp global minimum
        is mathematically guaranteed.

        Args:
            shape: (rows, cols) grid dimensions.
            base_elevation: Baseline plain elevation in meters.
            peak_height: Height added by the Gaussian peak at its center (meters).
            center_x: Center X of peak in meters (defaults to center of DEM).
            center_y: Center Y of peak in meters (defaults to center of DEM).
            sigma_x: Standard deviation / spread along X in meters.
            sigma_y: Standard deviation / spread along Y in meters.
            x_min: Western boundary in meters.
            y_min: Southern boundary in meters.
            dx: Grid spacing along X in meters.
            dy: Grid spacing along Y in meters.
            y_max: Optional explicit Northern boundary.

        Returns:
            DigitalElevationModel with a smooth Gaussian peak.
        """
        rows, cols = shape
        x_max = x_min + (cols - 1) * dx
        if y_max is None:
            y_max = y_min + (rows - 1) * dy
        else:
            y_min = y_max - (rows - 1) * dy

        cx = (x_min + x_max) / 2.0 if center_x is None else float(center_x)
        cy = (y_min + y_max) / 2.0 if center_y is None else float(center_y)

        # Create 1D physical coordinate vectors
        x_coords = np.linspace(x_min, x_max, cols)
        y_coords = np.linspace(y_max, y_min, rows)  # Rows go from North (y_max) to South (y_min)

        # 2D coordinate meshgrids
        xx, yy = np.meshgrid(x_coords, y_coords)

        # Compute 2D Gaussian equation:
        # z(x, y) = base + peak * exp( -((x - cx)^2 / (2*sx^2) + (y - cy)^2 / (2*sy^2)) )
        exponent = -(((xx - cx) ** 2) / (2.0 * sigma_x**2) + ((yy - cy) ** 2) / (2.0 * sigma_y**2))
        grid = float(base_elevation) + float(peak_height) * np.exp(exponent)

        return DigitalElevationModel(grid, x_min=x_min, y_min=y_min, y_max=y_max, dx=dx, dy=dy)

    @staticmethod
    def generate_rolling_hills(
        shape: tuple[int, int] = (100, 100),
        base_elevation: float = 300.0,
        octaves: int = 3,
        primary_wavelength: float = 400.0,
        primary_amplitude: float = 80.0,
        seed: int = 42,
        x_min: float = 0.0,
        y_min: float = 0.0,
        dx: float = 10.0,
        dy: float = 10.0,
        y_max: float | None = None,
    ) -> DigitalElevationModel:
        """Generate multi-frequency rolling hills using superimposed sinusoids.

        Creates realistic undulating terrain with multiple peaks, valleys, and saddles.

        Args:
            shape: (rows, cols) grid dimensions.
            base_elevation: Mean elevation of the landscape in meters.
            octaves: Number of superimposed frequency layers.
            primary_wavelength: Spatial wavelength of the largest terrain feature in meters.
            primary_amplitude: Peak-to-trough amplitude of the largest layer in meters.
            seed: Random seed for pseudo-random phase offsets.
            x_min: Western boundary in meters.
            y_min: Southern boundary in meters.
            dx: Grid spacing along X in meters.
            dy: Grid spacing along Y in meters.
            y_max: Optional explicit Northern boundary.

        Returns:
            DigitalElevationModel with rich, naturalistic terrain contours.
        """
        rows, cols = shape
        x_max = x_min + (cols - 1) * dx
        if y_max is None:
            y_max = y_min + (rows - 1) * dy
        else:
            y_min = y_max - (rows - 1) * dy

        x_coords = np.linspace(x_min, x_max, cols)
        y_coords = np.linspace(y_max, y_min, rows)
        xx, yy = np.meshgrid(x_coords, y_coords)

        rng = np.random.default_rng(seed)
        grid = np.full((rows, cols), fill_value=float(base_elevation), dtype=np.float64)

        current_amp = float(primary_amplitude)
        current_wl = float(primary_wavelength)

        for _ in range(octaves):
            phase_x = rng.uniform(0.0, 2.0 * np.pi)
            phase_y = rng.uniform(0.0, 2.0 * np.pi)
            angle = rng.uniform(0.0, np.pi)

            # Rotate coordinates slightly for natural look
            xr = xx * np.cos(angle) - yy * np.sin(angle)
            yr = xx * np.sin(angle) + yy * np.cos(angle)

            wave = (
                np.sin(2.0 * np.pi * xr / current_wl + phase_x)
                * np.cos(2.0 * np.pi * yr / current_wl + phase_y)
            )
            grid += current_amp * wave

            # Scale for next harmonic octave (higher frequency, lower amplitude)
            current_amp *= 0.5
            current_wl *= 0.5

        return DigitalElevationModel(grid, x_min=x_min, y_min=y_min, y_max=y_max, dx=dx, dy=dy)
