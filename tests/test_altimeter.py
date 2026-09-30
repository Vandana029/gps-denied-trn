"""Unit tests for FlightTrajectory, AltimeterSimulator, and TerrainProfile.

Validates trajectory generation, altimeter triad physics, noise injection,
and continuous profile extraction over digital elevation models.
"""

import numpy as np
import pytest
from terrain_matching.core.dem import DigitalElevationModel
from terrain_matching.simulation.altimeter import AltimeterSimulator, TerrainProfile
from terrain_matching.simulation.trajectory import FlightTrajectory, TrajectoryPoint


class TestFlightTrajectory:
    """Test suite for flight trajectory kinematics and representation."""

    def test_trajectory_creation_and_properties(self) -> None:
        """Verify explicit trajectory initialization and basic properties."""
        timestamps = np.array([0.0, 1.0, 2.0, 3.0])
        positions = np.array(
            [
                [0.0, 0.0, 500.0],
                [10.0, 0.0, 500.0],
                [20.0, 0.0, 500.0],
                [30.0, 0.0, 500.0],
            ]
        )
        traj = FlightTrajectory(timestamps, positions)

        assert len(traj) == 4
        assert traj.duration == 3.0
        assert traj.total_distance == pytest.approx(30.0)
        np.testing.assert_allclose(traj.x, [0.0, 10.0, 20.0, 30.0])
        np.testing.assert_allclose(traj.y, [0.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(traj.z, [500.0, 500.0, 500.0, 500.0])
        np.testing.assert_allclose(traj.distance_along_track, [0.0, 10.0, 20.0, 30.0])

    def test_trajectory_point_indexing(self) -> None:
        """Verify indexing returns a frozen TrajectoryPoint dataclass."""
        timestamps = np.array([0.0, 1.0])
        positions = np.array([[100.0, 200.0, 300.0], [150.0, 250.0, 350.0]])
        traj = FlightTrajectory(timestamps, positions)

        pt0 = traj[0]
        assert isinstance(pt0, TrajectoryPoint)
        assert pt0.t == 0.0
        assert pt0.x == 100.0
        assert pt0.y == 200.0
        assert pt0.z == 300.0
        assert pt0.position_xy == (100.0, 200.0)
        assert pt0.position_xyz == (100.0, 200.0, 300.0)

    def test_create_linear_eastbound(self) -> None:
        """Verify straight-and-level flight traveling East (heading 0 deg)."""
        traj = FlightTrajectory.create_linear(
            start_point=(0.0, 100.0, 1000.0),
            velocity_ms=100.0,
            heading_deg=0.0,  # +X East
            duration_s=10.0,
            sample_rate_hz=10.0,  # 101 points
        )

        assert len(traj) == 101
        assert traj.duration == pytest.approx(10.0)
        assert traj.total_distance == pytest.approx(1000.0)
        assert traj.x[0] == pytest.approx(0.0)
        assert traj.x[-1] == pytest.approx(1000.0)
        assert np.all(np.isclose(traj.y, 100.0))
        assert np.all(np.isclose(traj.z, 1000.0))

    def test_create_linear_northbound(self) -> None:
        """Verify flight traveling North (heading 90 deg CCW from East)."""
        traj = FlightTrajectory.create_linear(
            start_point=(50.0, 0.0, 800.0),
            velocity_ms=50.0,
            heading_deg=90.0,  # +Y North
            duration_s=4.0,
            sample_rate_hz=5.0,  # 21 points
        )

        assert len(traj) == 21
        assert traj.total_distance == pytest.approx(200.0)
        assert np.all(np.isclose(traj.x, 50.0))
        assert traj.y[0] == pytest.approx(0.0)
        assert traj.y[-1] == pytest.approx(200.0)

    def test_create_linear_climbing(self) -> None:
        """Verify constant climb rate adds correctly to altitude."""
        traj = FlightTrajectory.create_linear(
            start_point=(0.0, 0.0, 500.0),
            velocity_ms=100.0,
            heading_deg=0.0,
            duration_s=10.0,
            sample_rate_hz=2.0,
            climb_rate_ms=15.0,  # 15 m/s climb -> 150m gain
        )
        assert traj.z[0] == pytest.approx(500.0)
        assert traj.z[-1] == pytest.approx(650.0)

    def test_trajectory_validation_errors(self) -> None:
        """Verify invalid trajectory arguments raise informative ValueErrors."""
        with pytest.raises(ValueError, match="monotonically increasing"):
            FlightTrajectory(
                timestamps=np.array([0.0, 0.0]),
                positions=np.zeros((2, 3)),
            )

        with pytest.raises(ValueError, match="at least 2 points"):
            FlightTrajectory(
                timestamps=np.array([0.0]),
                positions=np.zeros((1, 3)),
            )

        with pytest.raises(ValueError, match="Velocity must be positive"):
            FlightTrajectory.create_linear(
                start_point=(0, 0, 100),
                velocity_ms=-10.0,
                heading_deg=0.0,
                duration_s=5.0,
                sample_rate_hz=10.0,
            )


class TestAltimeterSimulator:
    """Test suite for AltimeterSimulator and the altimeter triad equations."""

    @pytest.fixture
    def test_dem(self) -> DigitalElevationModel:
        """Create a 5x5 linear ramp DEM for deterministic ground truth.

        Elevation = x + y (meters).
        Grid: x in [0, 200], y in [0, 200], dx=50, dy=50.
        """
        x_coords = np.linspace(0, 200, 5)
        y_coords = np.linspace(200, 0, 5)  # Row 0 is North (y=200)
        xx, yy = np.meshgrid(x_coords, y_coords)
        grid = xx + yy  # Linear plane
        return DigitalElevationModel(elevation=grid, x_min=0.0, y_max=200.0, dx=50.0, dy=50.0)

    def test_fundamental_trn_invariant_zero_noise(self, test_dem: DigitalElevationModel) -> None:
        """Verify that under zero noise, h_reconstructed = z_baro - h_radar == h_dem exactly."""
        traj = FlightTrajectory.create_linear(
            start_point=(20.0, 20.0, 1000.0),
            velocity_ms=40.0,
            heading_deg=45.0,  # Flying diagonally North-East
            duration_s=3.0,
            sample_rate_hz=10.0,
        )

        sim = AltimeterSimulator(test_dem)
        profile = sim.sample_profile(traj, radar_noise_std=0.0, baro_noise_std=0.0, baro_bias=0.0)

        # Expected true elevation at any point along trajectory: x(t) + y(t)
        expected_elevations = traj.x + traj.y

        np.testing.assert_allclose(profile.elevations, expected_elevations, atol=1e-5)
        # Check that clearance + elevation equals aircraft flight altitude
        np.testing.assert_allclose(profile.h_radar + profile.elevations, traj.z, atol=1e-5)

    def test_vehicle_altitude_invariance(self, test_dem: DigitalElevationModel) -> None:
        """Verify aircraft climb or descent does NOT alter reconstructed terrain profile."""
        # Flight 1: Level at 1000m
        traj_level = FlightTrajectory.create_linear(
            start_point=(10.0, 50.0, 1000.0),
            velocity_ms=50.0,
            heading_deg=0.0,
            duration_s=2.0,
            sample_rate_hz=10.0,
            climb_rate_ms=0.0,
        )
        # Flight 2: Steep climb from 800m to 1100m over identical horizontal ground track
        traj_climbing = FlightTrajectory.create_linear(
            start_point=(10.0, 50.0, 800.0),
            velocity_ms=50.0,
            heading_deg=0.0,
            duration_s=2.0,
            sample_rate_hz=10.0,
            climb_rate_ms=150.0,
        )

        sim = AltimeterSimulator(test_dem)
        prof_level = sim.sample_profile(traj_level)
        prof_climbing = sim.sample_profile(traj_climbing)

        # The reconstructed terrain profiles MUST be identical
        np.testing.assert_allclose(prof_level.elevations, prof_climbing.elevations, atol=1e-5)

    def test_barometric_bias_effect(self, test_dem: DigitalElevationModel) -> None:
        """Verify that a +20m barometric bias shifts the entire profile upward by exactly 20m."""
        traj = FlightTrajectory.create_linear(
            start_point=(0.0, 100.0, 800.0),
            velocity_ms=50.0,
            heading_deg=0.0,
            duration_s=2.0,
            sample_rate_hz=10.0,
        )

        sim = AltimeterSimulator(test_dem)
        clean_prof = sim.sample_profile(traj, baro_bias=0.0)
        biased_prof = sim.sample_profile(traj, baro_bias=20.0)

        # Biased elevations should be clean elevations + 20.0m
        np.testing.assert_allclose(biased_prof.elevations, clean_prof.elevations + 20.0, atol=1e-5)

    def test_radar_and_baro_gaussian_noise(self, test_dem: DigitalElevationModel) -> None:
        """Verify Gaussian noise injection matches statistical expectations."""
        traj = FlightTrajectory.create_linear(
            start_point=(10.0, 100.0, 800.0),
            velocity_ms=1.8,
            heading_deg=0.0,
            duration_s=100.0,  # 1001 points for statistical sample size, ends at x=190m (< 200m)
            sample_rate_hz=10.0,
        )

        sim = AltimeterSimulator(test_dem)
        radar_sigma = 3.0
        baro_sigma = 2.0
        profile = sim.sample_profile(
            traj,
            radar_noise_std=radar_sigma,
            baro_noise_std=baro_sigma,
            seed=42,
        )

        # Difference between measured elevations and true ground elevations
        true_elev = test_dem.get_elevation_profile(traj.coordinates_xy)
        errors = profile.elevations - true_elev

        # Combined theoretical variance: sigma_total^2 = sigma_radar^2 + sigma_baro^2 = 9 + 4 = 13
        expected_sigma = np.sqrt(radar_sigma**2 + baro_sigma**2)
        sample_std = np.std(errors)

        assert np.isclose(np.mean(errors), 0.0, atol=0.5)
        assert np.isclose(sample_std, expected_sigma, rtol=0.15)

    def test_out_of_bounds_trajectory_raises(self, test_dem: DigitalElevationModel) -> None:
        """Verify sampling raises ValueError if trajectory flies out of DEM coverage."""
        traj = FlightTrajectory.create_linear(
            start_point=(150.0, 100.0, 800.0),
            velocity_ms=100.0,
            heading_deg=0.0,  # Reaches x = 450m, well beyond x_max = 200m
            duration_s=3.0,
            sample_rate_hz=10.0,
        )

        sim = AltimeterSimulator(test_dem)
        with pytest.raises(ValueError, match="out of DEM bounds"):
            sim.sample_profile(traj)
