"""Altimeter triad simulation and terrain profile sampling.

This module models airborne barometric and radar/LiDAR altimeters over digital elevation models,
extracting 1D terrain elevation profiles and modeling sensor measurement physics and errors.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal
import numpy as np

from terrain_matching.core.dem import DigitalElevationModel
from terrain_matching.simulation.trajectory import FlightTrajectory


@dataclass(frozen=True)
class TerrainProfile:
    """Encapsulates a sampled 1D terrain elevation profile from flight sensors.

    Attributes:
        timestamps: 1D array of shape (N,) with sample timestamps in seconds.
        trajectory: 2D array of shape (N, 3) representing aircraft (x, y, z) positions in meters.
        h_radar: 1D array of shape (N,) clearance measurements Above Ground Level (AGL) in meters.
        z_baro: 1D array of shape (N,) barometric altitude measurements above Sea Level in meters.
        elevations: 1D array of shape (N,) reconstructed ground elevations (z_baro - h_radar) in meters.
        distance_along_track: 1D array of shape (N,) cumulative horizontal distance traveled in meters.
    """

    timestamps: np.ndarray
    trajectory: np.ndarray
    h_radar: np.ndarray
    z_baro: np.ndarray
    elevations: np.ndarray
    distance_along_track: np.ndarray

    def __post_init__(self) -> None:
        """Validate array dimensions and consistency."""
        n = len(self.timestamps)
        if self.trajectory.shape != (n, 3):
            raise ValueError(f"Trajectory shape {self.trajectory.shape} does not match N={n}")
        for name, arr in [
            ("h_radar", self.h_radar),
            ("z_baro", self.z_baro),
            ("elevations", self.elevations),
            ("distance_along_track", self.distance_along_track),
        ]:
            if arr.ndim != 1 or len(arr) != n:
                raise ValueError(f"{name} shape {arr.shape} does not match N={n}")

    def __len__(self) -> int:
        """Number of profile samples."""
        return len(self.timestamps)

    @property
    def x(self) -> np.ndarray:
        """Aircraft East coordinates of shape (N,)."""
        return self.trajectory[:, 0]

    @property
    def y(self) -> np.ndarray:
        """Aircraft North coordinates of shape (N,)."""
        return self.trajectory[:, 1]

    @property
    def z(self) -> np.ndarray:
        """Aircraft true altitude of shape (N,)."""
        return self.trajectory[:, 2]

    @property
    def total_distance(self) -> float:
        """Total profile ground length in meters."""
        return float(self.distance_along_track[-1])

    @property
    def mean_elevation(self) -> float:
        """Mean elevation of the terrain profile in meters."""
        return float(np.mean(self.elevations))

    @property
    def elevation_variance(self) -> float:
        """Variance of the terrain profile in meters squared (roughness indicator)."""
        return float(np.var(self.elevations))


class AltimeterSimulator:
    """Simulates the airborne altimeter triad over a Digital Elevation Model.

    Calculates true clearance Above Ground Level (AGL) and true barometric altitude,
    with options to inject realistic sensor noise (Gaussian white noise) and hydrostatic
    barometric pressure bias.
    """

    def __init__(self, dem: DigitalElevationModel) -> None:
        """Initialize the AltimeterSimulator.

        Args:
            dem: DigitalElevationModel instance representing ground topography.
        """
        self._dem = dem

    @property
    def dem(self) -> DigitalElevationModel:
        """Reference Digital Elevation Model."""
        return self._dem

    def sample_profile(
        self,
        trajectory: FlightTrajectory,
        radar_noise_std: float = 0.0,
        baro_noise_std: float = 0.0,
        baro_bias: float = 0.0,
        interp_method: Literal["bilinear", "nearest"] = "bilinear",
        seed: int | None = None,
    ) -> TerrainProfile:
        """Sample a 1D terrain profile along a 3D flight trajectory.

        Args:
            trajectory: FlightTrajectory object containing timestamps and (x, y, z) path.
            radar_noise_std: 1-sigma standard deviation of Gaussian white noise on radar clearance in meters.
            baro_noise_std: 1-sigma standard deviation of Gaussian white noise on barometric altitude in meters.
            baro_bias: Constant vertical bias added to barometric altitude in meters (e.g. pressure change).
            interp_method: Interpolation mode ('bilinear' or 'nearest') for DEM sampling.
            seed: Optional integer seed for pseudo-random number generation reproducibility.

        Returns:
            TerrainProfile object containing measurements and reconstructed elevations.

        Raises:
            ValueError: If trajectory extends outside the DEM bounds or noise std is negative.
        """
        if radar_noise_std < 0.0 or baro_noise_std < 0.0:
            raise ValueError("Noise standard deviations must be non-negative.")

        rng = np.random.default_rng(seed)

        # 1. Query true ground elevation directly beneath aircraft
        coords_xy = trajectory.coordinates_xy
        true_dem_elevation = self._dem.get_elevation_profile(coords_xy, method=interp_method)

        # 2. Compute true aircraft clearances and barometric altitudes
        true_aircraft_z = trajectory.z
        true_radar_clearance = true_aircraft_z - true_dem_elevation

        # 3. Inject sensor noise & bias
        radar_noise = (
            rng.normal(0.0, radar_noise_std, size=len(trajectory))
            if radar_noise_std > 0.0
            else np.zeros(len(trajectory))
        )
        baro_noise = (
            rng.normal(0.0, baro_noise_std, size=len(trajectory))
            if baro_noise_std > 0.0
            else np.zeros(len(trajectory))
        )

        measured_h_radar = true_radar_clearance + radar_noise
        measured_z_baro = true_aircraft_z + baro_noise + baro_bias

        # 4. Reconstruct ground elevation: h_meas = z_baro - h_radar
        reconstructed_elevations = measured_z_baro - measured_h_radar

        return TerrainProfile(
            timestamps=trajectory.timestamps.copy(),
            trajectory=trajectory.positions.copy(),
            h_radar=measured_h_radar,
            z_baro=measured_z_baro,
            elevations=reconstructed_elevations,
            distance_along_track=trajectory.distance_along_track.copy(),
        )
