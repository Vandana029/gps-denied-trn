"""Unit tests for DeterministicMatcher and MatchResult.

Validates 1D along-track corridor matching, 2D spatial raster search,
exact recovery under zero noise, barometric bias immunity, off-grid discretization bounds,
and boundary truncation edge cases.
"""

import numpy as np
import pytest

from terrain_matching.core.dem import DigitalElevationModel
from terrain_matching.core.matcher import DeterministicMatcher, MatchResult
from terrain_matching.simulation.altimeter import AltimeterSimulator
from terrain_matching.simulation.terrain_generator import TerrainGenerator
from terrain_matching.simulation.trajectory import FlightTrajectory


@pytest.fixture
def terrain_and_simulator() -> tuple[DigitalElevationModel, AltimeterSimulator]:
    """Create a realistic 2D rolling hills DEM for testing."""
    dem = TerrainGenerator.generate_rolling_hills(
        shape=(100, 100),
        dx=20.0,
        dy=20.0,
        base_elevation=200.0,
        octaves=3,
        primary_wavelength=300.0,
        primary_amplitude=60.0,
        seed=123,
    )
    sim = AltimeterSimulator(dem)
    return dem, sim


class Test1DCorridorMatching:
    """Test suite for 1D sliding-window along-track corridor matching."""

    def test_1d_corridor_exact_recovery(
        self, terrain_and_simulator: tuple[DigitalElevationModel, AltimeterSimulator]
    ) -> None:
        dem, sim = terrain_and_simulator
        matcher = DeterministicMatcher(dem)

        # Fly along heading 0 deg (East) starting at (200.0, 500.0)
        true_start = (200.0, 500.0)
        traj = FlightTrajectory.create_linear(
            start_point=(true_start[0], true_start[1], 1000.0),
            velocity_ms=50.0,
            heading_deg=0.0,
            duration_s=6.0,  # 300m flight
            sample_rate_hz=2.0,  # 13 samples, spacing 25m
        )
        profile = sim.sample_profile(traj, radar_noise_std=0.0, baro_noise_std=0.0)

        # Search corridor starting at (0.0, 500.0) of length 800m
        result = matcher.match_1d_corridor(
            measured_profile=profile.elevations,
            corridor_start=(0.0, 500.0),
            heading_deg=0.0,
            corridor_length_m=800.0,
            step_size_m=20.0,
            sample_spacing_m=25.0,
            metric="mad",
            true_coord=true_start,
        )

        assert isinstance(result, MatchResult)
        assert result.best_coord[0] == pytest.approx(true_start[0], abs=1e-5)
        assert result.best_coord[1] == pytest.approx(true_start[1], abs=1e-5)
        assert result.best_cost == pytest.approx(0.0, abs=1e-5)
        assert result.position_error == pytest.approx(0.0, abs=1e-5)

    def test_1d_corridor_with_baro_bias_using_zncc(
        self, terrain_and_simulator: tuple[DigitalElevationModel, AltimeterSimulator]
    ) -> None:
        dem, sim = terrain_and_simulator
        matcher = DeterministicMatcher(dem)

        true_start = (300.0, 400.0)
        traj = FlightTrajectory.create_linear(
            start_point=(true_start[0], true_start[1], 1000.0),
            velocity_ms=40.0,
            heading_deg=90.0,  # North
            duration_s=5.0,
            sample_rate_hz=2.0,
        )
        # Inject +25m barometric bias
        biased_profile = sim.sample_profile(traj, baro_bias=25.0)

        result = matcher.match_1d_corridor(
            measured_profile=biased_profile.elevations,
            corridor_start=(300.0, 100.0),
            heading_deg=90.0,
            corridor_length_m=800.0,
            step_size_m=20.0,
            sample_spacing_m=20.0,
            metric="zncc",
            true_coord=true_start,
        )

        # ZNCC must achieve perfect correlation of +1.0 and locate the exact start
        assert result.best_coord[0] == pytest.approx(true_start[0], abs=1e-5)
        assert result.best_coord[1] == pytest.approx(true_start[1], abs=1e-5)
        assert result.best_cost == pytest.approx(1.0, rel=1e-5)
        assert result.position_error == pytest.approx(0.0, abs=1e-5)


class Test2DGridMatching:
    """Test suite for 2D spatial raster search."""

    def test_2d_exact_recovery_zero_noise(
        self, terrain_and_simulator: tuple[DigitalElevationModel, AltimeterSimulator]
    ) -> None:
        dem, sim = terrain_and_simulator
        matcher = DeterministicMatcher(dem)

        # Flight diagonal trajectory starting at on-grid node (400.0, 400.0)
        true_start = (400.0, 400.0)
        traj = FlightTrajectory.create_linear(
            start_point=(true_start[0], true_start[1], 800.0),
            velocity_ms=50.0,
            heading_deg=45.0,
            duration_s=6.0,
            sample_rate_hz=2.0,
        )
        profile = sim.sample_profile(traj)

        # Relative offsets from start point
        rel_offsets = traj.coordinates_xy - traj.coordinates_xy[0]

        # Search box: [200, 600] in X and Y with step size 20m (hits 400.0 exactly)
        result = matcher.match_2d_grid(
            measured_profile=profile.elevations,
            relative_offsets_xy=rel_offsets,
            search_bounds=(200.0, 600.0, 200.0, 600.0),
            step_size_m=20.0,
            metric="mad",
            true_coord=true_start,
        )

        assert result.best_coord == pytest.approx(true_start, abs=1e-5)
        assert result.best_cost == pytest.approx(0.0, abs=1e-5)
        assert result.position_error == pytest.approx(0.0, abs=1e-5)
        assert result.cost_surface.shape == (21, 21)

    def test_2d_barometric_bias_rejection_zmsd(
        self, terrain_and_simulator: tuple[DigitalElevationModel, AltimeterSimulator]
    ) -> None:
        dem, sim = terrain_and_simulator
        matcher = DeterministicMatcher(dem)

        true_start = (500.0, 500.0)
        traj = FlightTrajectory.create_linear(
            start_point=(true_start[0], true_start[1], 900.0),
            velocity_ms=40.0,
            heading_deg=30.0,
            duration_s=6.0,
            sample_rate_hz=2.0,
        )
        # Inject +40m barometric bias
        biased_profile = sim.sample_profile(traj, baro_bias=40.0)
        rel_offsets = traj.coordinates_xy - traj.coordinates_xy[0]

        # Using ZMSD
        result_zmsd = matcher.match_2d_grid(
            measured_profile=biased_profile.elevations,
            relative_offsets_xy=rel_offsets,
            search_bounds=(300.0, 700.0, 300.0, 700.0),
            step_size_m=20.0,
            metric="zmsd",
            true_coord=true_start,
        )

        assert result_zmsd.best_coord == pytest.approx(true_start, abs=1e-5)
        assert result_zmsd.best_cost == pytest.approx(0.0, abs=1e-5)
        assert result_zmsd.position_error == pytest.approx(0.0, abs=1e-5)

    def test_2d_off_grid_discretization_bounds(
        self, terrain_and_simulator: tuple[DigitalElevationModel, AltimeterSimulator]
    ) -> None:
        dem, sim = terrain_and_simulator
        matcher = DeterministicMatcher(dem)

        # Off-grid start point: (412.0, 418.0)
        true_start = (412.0, 418.0)
        traj = FlightTrajectory.create_linear(
            start_point=(true_start[0], true_start[1], 800.0),
            velocity_ms=50.0,
            heading_deg=0.0,
            duration_s=5.0,
            sample_rate_hz=2.0,
        )
        profile = sim.sample_profile(traj)
        rel_offsets = traj.coordinates_xy - traj.coordinates_xy[0]

        # Search with coarse step size 20m (evaluates 400, 420, etc.)
        # Closest grid node to (412, 418) is (420.0, 420.0)
        step = 20.0
        result = matcher.match_2d_grid(
            measured_profile=profile.elevations,
            relative_offsets_xy=rel_offsets,
            search_bounds=(300.0, 500.0, 300.0, 500.0),
            step_size_m=step,
            metric="mad",
            true_coord=true_start,
        )

        # Error must be bounded by half-diagonal: step / sqrt(2) = 20 / 1.414 = 14.14m
        max_theoretical_error = step / np.sqrt(2.0)
        assert result.position_error is not None
        assert result.position_error <= max_theoretical_error + 1e-3
        # Off-grid match cannot be exactly zero cost
        assert result.best_cost > 0.0

    def test_boundary_truncation_penalty(
        self, terrain_and_simulator: tuple[DigitalElevationModel, AltimeterSimulator]
    ) -> None:
        dem, sim = terrain_and_simulator
        matcher = DeterministicMatcher(dem)

        traj = FlightTrajectory.create_linear(
            start_point=(500.0, 500.0, 800.0),
            velocity_ms=50.0,
            heading_deg=0.0,
            duration_s=4.0,
            sample_rate_hz=2.0,
        )
        profile = sim.sample_profile(traj)
        rel_offsets = traj.coordinates_xy - traj.coordinates_xy[0]

        # Search box placed at edge where paths will extend beyond dem.x_max (2000m)
        # Search box X in [1800, 1950], traj extends by 200m -> exits DEM
        result = matcher.match_2d_grid(
            measured_profile=profile.elevations,
            relative_offsets_xy=rel_offsets,
            search_bounds=(1700.0, 1950.0, 400.0, 600.0),
            step_size_m=50.0,
            metric="mad",
        )

        # Points where trajectory left DEM must have cost == inf
        assert np.isinf(result.cost_surface[-1, -1])


class TestMatcherValidation:
    """Test input validations and edge cases."""

    def test_invalid_metric_name_raises(
        self, terrain_and_simulator: tuple[DigitalElevationModel, AltimeterSimulator]
    ) -> None:
        dem, _ = terrain_and_simulator
        matcher = DeterministicMatcher(dem)
        with pytest.raises(ValueError, match="Unknown metric"):
            matcher.match_2d_grid(
                measured_profile=np.ones(10),
                relative_offsets_xy=np.zeros((10, 2)),
                search_bounds=(0, 100, 0, 100),
                step_size_m=10.0,
                metric="invalid_metric",  # type: ignore
            )

    def test_invalid_search_bounds_raises(
        self, terrain_and_simulator: tuple[DigitalElevationModel, AltimeterSimulator]
    ) -> None:
        dem, _ = terrain_and_simulator
        matcher = DeterministicMatcher(dem)
        with pytest.raises(ValueError, match="Invalid search bounds"):
            matcher.match_2d_grid(
                measured_profile=np.ones(10),
                relative_offsets_xy=np.zeros((10, 2)),
                search_bounds=(100, 50, 0, 100),  # x_min > x_max
                step_size_m=10.0,
            )
