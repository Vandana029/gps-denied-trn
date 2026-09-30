"""Flight trajectory representation and generation for terrain matching simulations.

This module models continuous and discrete 3D aircraft trajectories in metric Cartesian
coordinates (East-North-Up), enabling altimeter sampling and navigation simulation.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class TrajectoryPoint:
    """Represents a discrete state along a flight path.

    Attributes:
        t: Timestamp in seconds from trajectory start.
        x: East coordinate in meters.
        y: North coordinate in meters.
        z: Altitude above Mean Sea Level (MSL) in meters.
    """

    t: float
    x: float
    y: float
    z: float

    @property
    def position_xy(self) -> tuple[float, float]:
        """Horizontal Cartesian coordinate (x, y)."""
        return (self.x, self.y)

    @property
    def position_xyz(self) -> tuple[float, float, float]:
        """Full 3D Cartesian position (x, y, z)."""
        return (self.x, self.y, self.z)


class FlightTrajectory:
    """Encapsulates a time-parameterized 3D flight trajectory.

    Stores timestamps and 3D positions sampled at a known rate, and provides
    kinematic properties such as along-track distance, groundspeed, and heading.
    """

    def __init__(self, timestamps: np.ndarray, positions: np.ndarray) -> None:
        """Initialize a FlightTrajectory.

        Args:
            timestamps: 1D array of shape (N,) containing monotonically increasing time in seconds.
            positions: 2D array of shape (N, 3) where columns are (x, y, z) in meters.

        Raises:
            ValueError: If array dimensions mismatch, points < 2, or timestamps not monotonically increasing.
        """
        timestamps = np.asarray(timestamps, dtype=np.float64)
        positions = np.asarray(positions, dtype=np.float64)

        if timestamps.ndim != 1:
            raise ValueError(f"Timestamps must be 1D array, got shape {timestamps.shape}")
        if positions.ndim != 2 or positions.shape[1] != 3:
            raise ValueError(f"Positions must have shape (N, 3), got {positions.shape}")
        if len(timestamps) != positions.shape[0]:
            raise ValueError(
                f"Length mismatch: {len(timestamps)} timestamps vs {positions.shape[0]} positions."
            )
        if len(timestamps) < 2:
            raise ValueError(f"Trajectory must contain at least 2 points, got {len(timestamps)}.")
        if np.any(np.diff(timestamps) <= 0.0):
            raise ValueError("Timestamps must be strictly monotonically increasing.")

        self._timestamps = timestamps
        self._positions = positions

        # Precompute along-track cumulative ground distance (XY plane)
        delta_xy = np.diff(self._positions[:, :2], axis=0)
        step_distances = np.hypot(delta_xy[:, 0], delta_xy[:, 1])
        self._distance_along_track = np.concatenate(([0.0], np.cumsum(step_distances)))

    @property
    def timestamps(self) -> np.ndarray:
        """Timestamps array of shape (N,) in seconds."""
        return self._timestamps

    @property
    def positions(self) -> np.ndarray:
        """3D Cartesian positions of shape (N, 3) in meters (x, y, z)."""
        return self._positions

    @property
    def x(self) -> np.ndarray:
        """East coordinates of shape (N,) in meters."""
        return self._positions[:, 0]

    @property
    def y(self) -> np.ndarray:
        """North coordinates of shape (N,) in meters."""
        return self._positions[:, 1]

    @property
    def z(self) -> np.ndarray:
        """Altitude coordinates of shape (N,) in meters MSL."""
        return self._positions[:, 2]

    @property
    def coordinates_xy(self) -> np.ndarray:
        """Horizontal (x, y) coordinates of shape (N, 2)."""
        return self._positions[:, :2]

    @property
    def distance_along_track(self) -> np.ndarray:
        """Cumulative horizontal distance traveled along-track of shape (N,) in meters."""
        return self._distance_along_track

    @property
    def total_distance(self) -> float:
        """Total horizontal distance traveled in meters."""
        return float(self._distance_along_track[-1])

    @property
    def duration(self) -> float:
        """Total duration of flight in seconds."""
        return float(self._timestamps[-1] - self._timestamps[0])

    def __len__(self) -> int:
        """Number of discrete points in trajectory."""
        return len(self._timestamps)

    def __getitem__(self, index: int) -> TrajectoryPoint:
        """Retrieve a specific TrajectoryPoint by index."""
        return TrajectoryPoint(
            t=float(self._timestamps[index]),
            x=float(self._positions[index, 0]),
            y=float(self._positions[index, 1]),
            z=float(self._positions[index, 2]),
        )

    @classmethod
    def create_linear(
        cls,
        start_point: tuple[float, float, float],
        velocity_ms: float,
        heading_deg: float,
        duration_s: float,
        sample_rate_hz: float,
        climb_rate_ms: float = 0.0,
    ) -> FlightTrajectory:
        """Generate a constant-velocity linear trajectory.

        Args:
            start_point: Initial position (x0, y0, z0) in meters.
            velocity_ms: Constant horizontal groundspeed in m/s (must be > 0).
            heading_deg: Flight heading angle in degrees (measured counter-clockwise from East / +X axis).
            duration_s: Total flight time in seconds (must be > 0).
            sample_rate_hz: Sampling frequency in Hertz (must be > 0).
            climb_rate_ms: Vertical climb/descent velocity in m/s. Defaults to 0.0 (level flight).

        Returns:
            FlightTrajectory instance.

        Raises:
            ValueError: If velocity_ms <= 0, duration_s <= 0, or sample_rate_hz <= 0.
        """
        if velocity_ms <= 0.0:
            raise ValueError(f"Velocity must be positive, got {velocity_ms}")
        if duration_s <= 0.0:
            raise ValueError(f"Duration must be positive, got {duration_s}")
        if sample_rate_hz <= 0.0:
            raise ValueError(f"Sample rate must be positive, got {sample_rate_hz}")

        num_points = int(np.round(duration_s * sample_rate_hz)) + 1
        if num_points < 2:
            num_points = 2

        timestamps = np.linspace(0.0, duration_s, num_points)
        dt = timestamps

        heading_rad = np.deg2rad(heading_deg)
        vx = velocity_ms * np.cos(heading_rad)
        vy = velocity_ms * np.sin(heading_rad)
        vz = climb_rate_ms

        x0, y0, z0 = start_point
        x = x0 + vx * dt
        y = y0 + vy * dt
        z = z0 + vz * dt

        positions = np.column_stack((x, y, z))
        return cls(timestamps=timestamps, positions=positions)
