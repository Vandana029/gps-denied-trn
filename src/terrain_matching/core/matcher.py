"""Deterministic 1D and 2D terrain matching search engines.

This module provides exhaustive sliding-window corridor matching and 2D spatial raster search
over Digital Elevation Models (DEMs) using vectorized cost and similarity metrics.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable, Literal
import numpy as np

from terrain_matching.core.dem import DigitalElevationModel
from terrain_matching.core.metrics import (
    mean_absolute_difference,
    mean_squared_difference,
    normalized_cross_correlation,
    root_mean_squared_difference,
    zero_mean_mad,
    zero_mean_msd,
    zero_mean_normalized_cross_correlation,
)

MetricName = Literal["mad", "msd", "rmse", "zmad", "zmsd", "ncc", "zncc"]

_METRIC_REGISTRY: dict[str, tuple[Callable[[np.ndarray, np.ndarray], float], bool]] = {
    # metric_name: (function, is_minimization)
    "mad": (mean_absolute_difference, True),
    "msd": (mean_squared_difference, True),
    "rmse": (root_mean_squared_difference, True),
    "zmad": (zero_mean_mad, True),
    "zmsd": (zero_mean_msd, True),
    "ncc": (normalized_cross_correlation, False),
    "zncc": (zero_mean_normalized_cross_correlation, False),
}


@dataclass(frozen=True)
class MatchResult:
    """Encapsulates the complete result of a terrain matching search.

    Attributes:
        best_coord: Estimated (x, y) coordinates of the trajectory start point in meters.
        best_cost: Metric score at the optimal coordinate.
        metric_name: Name of the evaluation metric used (e.g. 'mad', 'zncc').
        search_grid_x: 1D array of evaluated X coordinates in meters.
        search_grid_y: 1D array of evaluated Y coordinates in meters.
        cost_surface: 2D array of shape (len(search_grid_y), len(search_grid_x)) with matching scores.
        true_coord: Ground truth (x, y) coordinate, if provided for validation.
        position_error: Euclidean distance error ||best_coord - true_coord|| in meters.
        execution_time_s: Elapsed search runtime in seconds.
    """

    best_coord: tuple[float, float]
    best_cost: float
    metric_name: str
    search_grid_x: np.ndarray
    search_grid_y: np.ndarray
    cost_surface: np.ndarray
    true_coord: tuple[float, float] | None = None
    position_error: float | None = None
    execution_time_s: float = 0.0


class DeterministicMatcher:
    """Exhaustive terrain matching engine operating over 2D Digital Elevation Models."""

    def __init__(self, dem: DigitalElevationModel) -> None:
        """Initialize the matcher with a reference DEM.

        Args:
            dem: DigitalElevationModel instance containing terrain elevations.
        """
        self._dem = dem

    @property
    def dem(self) -> DigitalElevationModel:
        """Reference Digital Elevation Model."""
        return self._dem

    @staticmethod
    def get_supported_metrics() -> list[str]:
        """Return list of supported metric identifier strings."""
        return list(_METRIC_REGISTRY.keys())

    def match_1d_corridor(
        self,
        measured_profile: np.ndarray,
        corridor_start: tuple[float, float],
        heading_deg: float,
        corridor_length_m: float,
        step_size_m: float,
        sample_spacing_m: float,
        metric: MetricName = "mad",
        true_coord: tuple[float, float] | None = None,
    ) -> MatchResult:
        """Perform a sliding-window along-track profile match along a 1D corridor.

        Slides the measured profile along a single flight direction to estimate the
        along-track position of the aircraft.

        Args:
            measured_profile: 1D array of N measured terrain elevations in meters.
            corridor_start: (x0, y0) starting coordinate of the search corridor in meters.
            heading_deg: Flight heading angle in degrees (measured CCW from East / +X axis).
            corridor_length_m: Total length of the search corridor in meters (must be > 0).
            step_size_m: Search step interval along the corridor in meters (must be > 0).
            sample_spacing_m: Distance between consecutive profile samples in meters (must be > 0).
            metric: Metric name ('mad', 'msd', 'rmse', 'zmad', 'zmsd', 'ncc', 'zncc').
            true_coord: Optional true (x, y) start coordinate for error calculation.

        Returns:
            MatchResult containing the best 1D location, cost, and 1D cost curve.

        Raises:
            ValueError: If inputs are invalid or metric is unknown.
        """
        start_time = time.perf_counter()
        metric_key = metric.lower()
        if metric_key not in _METRIC_REGISTRY:
            raise ValueError(f"Unknown metric '{metric}'. Supported: {self.get_supported_metrics()}")

        metric_fn, is_minimization = _METRIC_REGISTRY[metric_key]
        meas = np.asarray(measured_profile, dtype=np.float64)
        n_samples = len(meas)
        if n_samples < 2:
            raise ValueError(f"Measured profile must have at least 2 points, got {n_samples}")
        if corridor_length_m <= 0.0 or step_size_m <= 0.0 or sample_spacing_m <= 0.0:
            raise ValueError("Corridor length, step size, and sample spacing must be positive.")

        # Unit vector along corridor heading
        heading_rad = np.deg2rad(heading_deg)
        u_x = np.cos(heading_rad)
        u_y = np.sin(heading_rad)

        # Profile span length along track
        profile_span_m = (n_samples - 1) * sample_spacing_m
        profile_offsets_s = np.arange(n_samples) * sample_spacing_m

        # Along-track shift candidate start positions
        max_shift = corridor_length_m - profile_span_m
        if max_shift < 0.0:
            raise ValueError(
                f"Corridor length ({corridor_length_m:.1f}m) is shorter than profile span ({profile_span_m:.1f}m)"
            )

        candidate_shifts_s = np.arange(0.0, max_shift + step_size_m / 2.0, step_size_m)
        n_candidates = len(candidate_shifts_s)

        penalty_val = np.inf if is_minimization else -1.0
        cost_curve = np.full(n_candidates, penalty_val, dtype=np.float64)
        coords_x = np.zeros(n_candidates, dtype=np.float64)
        coords_y = np.zeros(n_candidates, dtype=np.float64)

        x0, y0 = corridor_start
        for i, s_shift in enumerate(candidate_shifts_s):
            start_x = x0 + s_shift * u_x
            start_y = y0 + s_shift * u_y
            coords_x[i] = start_x
            coords_y[i] = start_y

            # Coordinates for all samples along this candidate slice
            cand_x = start_x + profile_offsets_s * u_x
            cand_y = start_y + profile_offsets_s * u_y
            cand_coords = np.column_stack((cand_x, cand_y))

            # Bounds check
            if (
                self._dem.in_bounds(cand_x.min(), cand_y.min())
                and self._dem.in_bounds(cand_x.max(), cand_y.max())
            ):
                ref_profile = self._dem.get_elevation_profile(cand_coords, method="bilinear")
                cost_curve[i] = metric_fn(meas, ref_profile)

        # Optimal shift index
        if is_minimization:
            best_idx = int(np.argmin(cost_curve))
        else:
            best_idx = int(np.argmax(cost_curve))

        best_coord = (float(coords_x[best_idx]), float(coords_y[best_idx]))
        best_cost = float(cost_curve[best_idx])

        pos_error = None
        if true_coord is not None:
            pos_error = float(np.hypot(best_coord[0] - true_coord[0], best_coord[1] - true_coord[1]))

        elapsed = time.perf_counter() - start_time
        # Package 1D cost curve into a (1, n_candidates) surface for uniform API
        return MatchResult(
            best_coord=best_coord,
            best_cost=best_cost,
            metric_name=metric_key,
            search_grid_x=coords_x,
            search_grid_y=coords_y,
            cost_surface=cost_curve.reshape(1, -1),
            true_coord=true_coord,
            position_error=pos_error,
            execution_time_s=elapsed,
        )

    def match_2d_grid(
        self,
        measured_profile: np.ndarray,
        relative_offsets_xy: np.ndarray,
        search_bounds: tuple[float, float, float, float],
        step_size_m: float,
        metric: MetricName = "mad",
        true_coord: tuple[float, float] | None = None,
    ) -> MatchResult:
        """Perform an exhaustive 2D spatial raster search across a rectangular bounding box.

        Tests candidate trajectory start positions (x_c, y_c) over a discrete 2D grid,
        evaluating DEM elevations along the relative track for each candidate.

        Args:
            measured_profile: 1D array of N measured terrain elevations in meters.
            relative_offsets_xy: 2D array of shape (N, 2) containing relative (dx, dy)
                offsets from the trajectory start point in meters.
            search_bounds: Tuple of (x_min, x_max, y_min, y_max) defining the 2D search window.
            step_size_m: Grid spacing interval in meters for both X and Y axes.
            metric: Metric name ('mad', 'msd', 'rmse', 'zmad', 'zmsd', 'ncc', 'zncc').
            true_coord: Optional true (x, y) start coordinate for error calculation.

        Returns:
            MatchResult containing the best 2D coordinate, best cost, and full 2D cost surface.

        Raises:
            ValueError: If arguments are invalid or dimension mismatch occurs.
        """
        start_time = time.perf_counter()
        metric_key = metric.lower()
        if metric_key not in _METRIC_REGISTRY:
            raise ValueError(f"Unknown metric '{metric}'. Supported: {self.get_supported_metrics()}")

        metric_fn, is_minimization = _METRIC_REGISTRY[metric_key]
        meas = np.asarray(measured_profile, dtype=np.float64)
        offsets = np.asarray(relative_offsets_xy, dtype=np.float64)

        if meas.ndim != 1:
            raise ValueError(f"measured_profile must be 1D, got shape {meas.shape}")
        if offsets.ndim != 2 or offsets.shape[1] != 2:
            raise ValueError(f"relative_offsets_xy must have shape (N, 2), got {offsets.shape}")
        if len(meas) != len(offsets):
            raise ValueError(
                f"Length mismatch: measured_profile has {len(meas)} points, offsets has {len(offsets)}"
            )
        if step_size_m <= 0.0:
            raise ValueError(f"step_size_m must be positive, got {step_size_m}")

        x_min, x_max, y_min, y_max = search_bounds
        if x_min >= x_max or y_min >= y_max:
            raise ValueError(f"Invalid search bounds: x=[{x_min}, {x_max}], y=[{y_min}, {y_max}]")

        grid_x = np.arange(x_min, x_max + step_size_m / 2.0, step_size_m)
        grid_y = np.arange(y_min, y_max + step_size_m / 2.0, step_size_m)
        nx = len(grid_x)
        ny = len(grid_y)

        penalty_val = np.inf if is_minimization else -1.0
        cost_surface = np.full((ny, nx), penalty_val, dtype=np.float64)

        dx_min, dx_max = float(offsets[:, 0].min()), float(offsets[:, 0].max())
        dy_min, dy_max = float(offsets[:, 1].min()), float(offsets[:, 1].max())

        # Raster evaluation loop
        for j, yc in enumerate(grid_y):
            # Check if Y-extent of trajectory exceeds DEM bounds
            if not (self._dem.in_bounds(self._dem.x_min, yc + dy_min) and 
                    self._dem.in_bounds(self._dem.x_min, yc + dy_max)):
                continue

            for i, xc in enumerate(grid_x):
                # Check if X-extent of trajectory exceeds DEM bounds
                if not (self._dem.in_bounds(xc + dx_min, yc) and 
                        self._dem.in_bounds(xc + dx_max, yc)):
                    continue

                # Reconstruct candidate ground track
                cand_coords = np.column_stack((xc + offsets[:, 0], yc + offsets[:, 1]))

                # Extract reference profile via bilinear interpolation
                ref_profile = self._dem.get_elevation_profile(cand_coords, method="bilinear")
                cost_surface[j, i] = metric_fn(meas, ref_profile)

        # Find global optimal coordinate
        if is_minimization:
            opt_idx = np.unravel_index(np.argmin(cost_surface), cost_surface.shape)
        else:
            opt_idx = np.unravel_index(np.argmax(cost_surface), cost_surface.shape)

        best_y = float(grid_y[opt_idx[0]])
        best_x = float(grid_x[opt_idx[1]])
        best_coord = (best_x, best_y)
        best_cost = float(cost_surface[opt_idx])

        pos_error = None
        if true_coord is not None:
            pos_error = float(np.hypot(best_x - true_coord[0], best_y - true_coord[1]))

        elapsed = time.perf_counter() - start_time
        return MatchResult(
            best_coord=best_coord,
            best_cost=best_cost,
            metric_name=metric_key,
            search_grid_x=grid_x,
            search_grid_y=grid_y,
            cost_surface=cost_surface,
            true_coord=true_coord,
            position_error=pos_error,
            execution_time_s=elapsed,
        )
