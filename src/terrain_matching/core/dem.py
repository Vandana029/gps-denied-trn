"""Digital Elevation Model (DEM) container and coordinate transformation engine.

This module provides the core data structures and algorithms for representing,
indexing, and sampling digital elevation models in metric Cartesian navigation frames.
"""

from __future__ import annotations

from typing import Literal
import numpy as np


class DigitalElevationModel:
    """Represents a regular 2D grid of terrain elevation values.

    The model maps continuous metric Cartesian coordinates (x = East, y = North)
    to discrete matrix rows and columns with bilinear or nearest-neighbor interpolation.

    Attributes:
        elevation: 2D NumPy array of shape (R, C) containing elevation values in meters.
        x_min: Westernmost coordinate of the DEM in meters.
        y_max: Northernmost coordinate of the DEM in meters.
        dx: Grid spacing along the East-West (X) axis in meters.
        dy: Grid spacing along the North-South (Y) axis in meters.
        rows: Number of rows in the elevation grid.
        cols: Number of columns in the elevation grid.
        x_max: Easternmost coordinate of the DEM in meters.
        y_min: Southernmost coordinate of the DEM in meters.
    """

    def __init__(
        self,
        elevation: np.ndarray,
        x_min: float = 0.0,
        y_max: float | None = None,
        dx: float = 10.0,
        dy: float = 10.0,
        y_min: float | None = None,
    ) -> None:
        """Initialize the Digital Elevation Model.

        Args:
            elevation: 2D numpy array of elevation values in meters.
            x_min: Minimum X coordinate (Western boundary) in meters. Defaults to 0.0.
            y_max: Maximum Y coordinate (Northern boundary) in meters.
            dx: Horizontal grid resolution in meters per cell (must be > 0).
            dy: Vertical grid resolution in meters per cell (must be > 0).
            y_min: Minimum Y coordinate (Southern boundary) in meters.

        Raises:
            ValueError: If elevation is not 2D, smaller than 2x2, or dx/dy <= 0.
        """
        if elevation.ndim != 2:
            raise ValueError(
                f"Elevation array must be 2D, got shape {elevation.shape}"
            )
        if elevation.shape[0] < 2 or elevation.shape[1] < 2:
            raise ValueError(
                f"Elevation array must be at least 2x2 for interpolation, got {elevation.shape}"
            )
        if dx <= 0.0 or dy <= 0.0:
            raise ValueError(
                f"Grid resolutions dx and dy must be strictly positive, got dx={dx}, dy={dy}"
            )

        self._elevation = np.asarray(elevation, dtype=np.float64)
        self._rows, self._cols = self._elevation.shape
        self._x_min = float(x_min)
        self._dx = float(dx)
        self._dy = float(dy)

        # Derived boundary limits
        self._x_max = self._x_min + (self._cols - 1) * self._dx

        if y_max is not None:
            self._y_max = float(y_max)
            self._y_min = self._y_max - (self._rows - 1) * self._dy
        elif y_min is not None:
            self._y_min = float(y_min)
            self._y_max = self._y_min + (self._rows - 1) * self._dy
        else:
            # Default to natural origin at y_min = 0.0
            self._y_min = 0.0
            self._y_max = (self._rows - 1) * self._dy

    @property
    def elevation_matrix(self) -> np.ndarray:
        """Return the raw elevation matrix."""
        return self._elevation

    @property
    def rows(self) -> int:
        """Number of rows (North-South samples)."""
        return self._rows

    @property
    def cols(self) -> int:
        """Number of columns (East-West samples)."""
        return self._cols

    @property
    def dx(self) -> float:
        """Cell spacing along X (East) in meters."""
        return self._dx

    @property
    def dy(self) -> float:
        """Cell spacing along Y (North) in meters."""
        return self._dy

    @property
    def x_min(self) -> float:
        """Westernmost bound (meters)."""
        return self._x_min

    @property
    def x_max(self) -> float:
        """Easternmost bound (meters)."""
        return self._x_max

    @property
    def y_min(self) -> float:
        """Southernmost bound (meters)."""
        return self._y_min

    @property
    def y_max(self) -> float:
        """Northernmost bound (meters)."""
        return self._y_max

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """Return (x_min, x_max, y_min, y_max) in meters."""
        return (self._x_min, self._x_max, self._y_min, self._y_max)

    def world_to_grid(self, x: float, y: float) -> tuple[float, float]:
        """Convert continuous Cartesian world coordinates (x, y) to floating grid indices (row, col).

        Note:
            Matrix rows increase downwards (South), while physical Y increases upwards (North).

        Args:
            x: Easting in meters.
            y: Northing in meters.

        Returns:
            Tuple of (row_float, col_float).
        """
        col_float = (x - self._x_min) / self._dx
        row_float = (self._y_max - y) / self._dy
        return (row_float, col_float)

    def grid_to_world(self, row: float, col: float) -> tuple[float, float]:
        """Convert grid indices (row, col) to continuous Cartesian world coordinates (x, y).

        Args:
            row: Floating or integer row index.
            col: Floating or integer column index.

        Returns:
            Tuple of (x, y) in meters.
        """
        x = self._x_min + col * self._dx
        y = self._y_max - row * self._dy
        return (x, y)

    def in_bounds(self, x: float, y: float) -> bool:
        """Check whether continuous coordinates (x, y) lie within the DEM boundaries.

        Args:
            x: Easting in meters.
            y: Northing in meters.

        Returns:
            True if within bounds, False otherwise.
        """
        return (self._x_min <= x <= self._x_max) and (self._y_min <= y <= self._y_max)

    def get_elevation(
        self,
        x: float,
        y: float,
        method: Literal["bilinear", "nearest"] = "bilinear",
    ) -> float:
        """Sample elevation at continuous metric coordinate (x, y).

        Args:
            x: East coordinate in meters.
            y: North coordinate in meters.
            method: Interpolation method: 'bilinear' (default) or 'nearest'.

        Returns:
            Interpolated terrain elevation in meters above sea level.

        Raises:
            ValueError: If (x, y) is out of bounds or method is unknown.
        """
        if not self.in_bounds(x, y):
            raise ValueError(
                f"Coordinates (x={x:.2f}, y={y:.2f}) are out of DEM bounds "
                f"[X: {self._x_min:.1f} to {self._x_max:.1f}, Y: {self._y_min:.1f} to {self._y_max:.1f}]"
            )

        row_f, col_f = self.world_to_grid(x, y)

        if method == "nearest":
            r = int(np.clip(np.round(row_f), 0, self._rows - 1))
            c = int(np.clip(np.round(col_f), 0, self._cols - 1))
            return float(self._elevation[r, c])

        if method == "bilinear":
            # Identify integer corner indices
            c0 = int(np.floor(col_f))
            r0 = int(np.floor(row_f))

            # Handle edge boundary case where coordinate is exactly at max limit
            c0 = min(c0, self._cols - 2)
            r0 = min(r0, self._rows - 2)
            c1 = c0 + 1
            r1 = r0 + 1

            # Fractional offsets in [0, 1]
            u = float(col_f - c0)
            v = float(row_f - r0)

            # Retrieve four bounding corner elevations
            q11 = self._elevation[r0, c0]
            q12 = self._elevation[r0, c1]
            q21 = self._elevation[r1, c0]
            q22 = self._elevation[r1, c1]

            # Bilinear formula: weighted average of four corners
            elev = (
                (1.0 - u) * (1.0 - v) * q11
                + u * (1.0 - v) * q12
                + (1.0 - u) * v * q21
                + u * v * q22
            )
            return float(elev)

        raise ValueError(f"Unknown interpolation method '{method}', expected 'bilinear' or 'nearest'.")

    def get_elevation_profile(
        self,
        coordinates: np.ndarray,
        method: Literal["bilinear", "nearest"] = "bilinear",
    ) -> np.ndarray:
        """Sample elevations along a flight trajectory of (N, 2) coordinates.

        Args:
            coordinates: (N, 2) array where column 0 is X (East) and column 1 is Y (North).
            method: Interpolation method ('bilinear' or 'nearest').

        Returns:
            1D array of shape (N,) containing sampled elevations in meters.
        """
        coords = np.asarray(coordinates, dtype=np.float64)
        if coords.ndim != 2 or coords.shape[1] != 2:
            raise ValueError(f"Coordinates must have shape (N, 2), got {coords.shape}")

        elevations = np.empty(coords.shape[0], dtype=np.float64)
        for i in range(coords.shape[0]):
            elevations[i] = self.get_elevation(coords[i, 0], coords[i, 1], method=method)
        return elevations
