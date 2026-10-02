"""Unit tests for AmbiguityAnalyzer and AmbiguityReport.

Validates Peak-to-Sidelobe Ratio (PSR), Multi-Modal Ambiguity Margin (MAR),
Hessian surface curvature, and the 4-gate avionics fix acceptance filter.
"""

import numpy as np
import pytest

from terrain_matching.core.ambiguity import AmbiguityAnalyzer, AmbiguityReport
from terrain_matching.core.matcher import DeterministicMatcher, MatchResult
from terrain_matching.simulation.altimeter import AltimeterSimulator
from terrain_matching.simulation.terrain_generator import TerrainGenerator
from terrain_matching.simulation.trajectory import FlightTrajectory


@pytest.fixture
def terrain_and_matcher() -> tuple[DeterministicMatcher, AltimeterSimulator]:
    """Create a multi-octave rolling hills terrain and matcher."""
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
    matcher = DeterministicMatcher(dem)
    return matcher, sim


class TestAmbiguityAnalyzerOnRealisticTerrain:
    """Test suite using realistic 2D simulation."""

    def test_unambiguous_fix_is_accepted(
        self, terrain_and_matcher: tuple[DeterministicMatcher, AltimeterSimulator]
    ) -> None:
        matcher, sim = terrain_and_matcher
        true_start = (500.0, 500.0)
        traj = FlightTrajectory.create_linear(
            start_point=(true_start[0], true_start[1], 800.0),
            velocity_ms=50.0,
            heading_deg=45.0,
            duration_s=6.0,
            sample_rate_hz=2.0,
        )
        profile = sim.sample_profile(traj, radar_noise_std=0.0, baro_noise_std=0.0)
        rel_offsets = traj.coordinates_xy - traj.coordinates_xy[0]

        match_res = matcher.match_2d_grid(
            measured_profile=profile.elevations,
            relative_offsets_xy=rel_offsets,
            search_bounds=(300.0, 700.0, 300.0, 700.0),
            step_size_m=20.0,
            metric="mad",
            true_coord=true_start,
        )

        analyzer = AmbiguityAnalyzer(
            exclusion_radius_m=60.0,
            min_psr=3.0,
            min_ambiguity_ratio=1.2,
            max_acceptable_difference_cost=2.0,
        )
        report = analyzer.analyze(match_res, measured_profile_variance=profile.elevation_variance)

        assert isinstance(report, AmbiguityReport)
        assert report.is_fix_acceptable is True
        assert len(report.rejection_reasons) == 0
        assert report.psr >= 3.0
        # Runner-up must be strictly outside exclusion radius (60m)
        dist_to_second = np.hypot(
            report.second_best_coord[0] - match_res.best_coord[0],
            report.second_best_coord[1] - match_res.best_coord[1],
        )
        assert dist_to_second > 60.0
        assert report.ambiguity_margin > 1.2
        assert report.curvature_eigenvalues is not None


class TestAvionicsFixRejectionGates:
    """Test individual security gates of the fix-rejection filter."""

    def test_insufficient_terrain_roughness_rejected(self) -> None:
        """Gate 1: Profile elevation variance below threshold is rejected."""
        # Create a mock MatchResult with good cost and PSR
        ny, nx = 21, 21
        x = np.linspace(0, 400, nx)
        y = np.linspace(0, 400, ny)
        xx, yy = np.meshgrid(x, y)
        cost_surface = 10.0 + (xx - 200) ** 2 / 1000 + (yy - 200) ** 2 / 1000
        cost_surface[10, 10] = 0.0  # Sharp minimum

        match_res = MatchResult(
            best_coord=(200.0, 200.0),
            best_cost=0.0,
            metric_name="mad",
            search_grid_x=x,
            search_grid_y=y,
            cost_surface=cost_surface,
        )

        analyzer = AmbiguityAnalyzer(min_profile_variance=10.0)
        # Pass a flat terrain variance (e.g. 0.5 m^2 < 10.0)
        report = analyzer.analyze(match_res, measured_profile_variance=0.5)

        assert report.is_fix_acceptable is False
        assert any("Insufficient terrain roughness" in r for r in report.rejection_reasons)

    def test_excessive_residual_cost_rejected(self) -> None:
        """Gate 2: High residual error at the best location is rejected."""
        ny, nx = 21, 21
        x = np.linspace(0, 400, nx)
        y = np.linspace(0, 400, ny)
        xx, yy = np.meshgrid(x, y)
        # Minimum is 15.0m (exceeds allowable 5.0m threshold)
        cost_surface = 15.0 + (xx - 200) ** 2 / 1000 + (yy - 200) ** 2 / 1000

        match_res = MatchResult(
            best_coord=(200.0, 200.0),
            best_cost=15.0,
            metric_name="mad",
            search_grid_x=x,
            search_grid_y=y,
            cost_surface=cost_surface,
        )

        analyzer = AmbiguityAnalyzer(max_acceptable_difference_cost=5.0)
        report = analyzer.analyze(match_res)

        assert report.is_fix_acceptable is False
        assert any("Excessive residual error" in r for r in report.rejection_reasons)

    def test_multi_modal_ambiguity_rejected(self) -> None:
        """Gate 4: A rival peak with nearly identical cost is rejected."""
        ny, nx = 31, 31
        x = np.linspace(0, 600, nx)
        y = np.linspace(0, 600, ny)
        cost_surface = np.full((ny, nx), 20.0)

        # Primary minimum at (200, 200) with cost 1.0
        cost_surface[10, 10] = 1.0
        # Dangerous rival minimum at (400, 400) with cost 1.05 (outside 60m exclusion zone)
        cost_surface[20, 20] = 1.05

        match_res = MatchResult(
            best_coord=(200.0, 200.0),
            best_cost=1.0,
            metric_name="mad",
            search_grid_x=x,
            search_grid_y=y,
            cost_surface=cost_surface,
        )

        analyzer = AmbiguityAnalyzer(min_ambiguity_ratio=1.3)
        report = analyzer.analyze(match_res)

        assert report.is_fix_acceptable is False
        assert report.ambiguity_margin == pytest.approx(1.05 / 1.0, rel=1e-2)
        assert any("High multi-modal ambiguity" in r for r in report.rejection_reasons)


class TestCurvatureAndAnisotropy:
    """Test Hessian curvature calculations."""

    def test_symmetric_isotropic_bowl_anisotropy(self) -> None:
        """An isotropic circular bowl should have anisotropy close to 1.0."""
        nx, ny = 21, 21
        x = np.linspace(-100, 100, nx)  # dx = 10m
        y = np.linspace(-100, 100, ny)  # dy = 10m
        xx, yy = np.meshgrid(x, y)
        # Isotropic quadratic bowl: J = x^2 + y^2
        cost_surface = xx**2 + yy**2

        match_res = MatchResult(
            best_coord=(0.0, 0.0),
            best_cost=0.0,
            metric_name="msd",
            search_grid_x=x,
            search_grid_y=y,
            cost_surface=cost_surface,
        )

        analyzer = AmbiguityAnalyzer()
        report = analyzer.analyze(match_res)

        assert report.curvature_eigenvalues is not None
        assert report.anisotropy is not None
        assert report.anisotropy == pytest.approx(1.0, rel=1e-3)

    def test_elongated_canyon_trench_anisotropy(self) -> None:
        """An elongated canyon trench has high anisotropy (lambda_1 >> lambda_2)."""
        nx, ny = 21, 21
        x = np.linspace(-100, 100, nx)
        y = np.linspace(-100, 100, ny)
        xx, yy = np.meshgrid(x, y)
        # Trench: steep in Y (canyon walls), flat in X (river bed)
        cost_surface = 0.01 * xx**2 + 10.0 * yy**2

        match_res = MatchResult(
            best_coord=(0.0, 0.0),
            best_cost=0.0,
            metric_name="msd",
            search_grid_x=x,
            search_grid_y=y,
            cost_surface=cost_surface,
        )

        analyzer = AmbiguityAnalyzer()
        report = analyzer.analyze(match_res)

        assert report.anisotropy is not None
        # Anisotropy should be ~ 10.0 / 0.01 = 1000
        assert report.anisotropy > 100.0


class TestAmbiguityAnalyzerInputValidation:
    """Test validation errors for invalid analyzer parameters."""

    def test_negative_exclusion_radius_raises(self) -> None:
        with pytest.raises(ValueError, match="exclusion_radius_m must be positive"):
            AmbiguityAnalyzer(exclusion_radius_m=-10.0)

    def test_1d_match_result_raises(self) -> None:
        """Passing a 1D corridor search result should raise ValueError."""
        match_res = MatchResult(
            best_coord=(0.0, 0.0),
            best_cost=0.0,
            metric_name="mad",
            search_grid_x=np.array([0.0, 10.0]),
            search_grid_y=np.array([0.0]),
            cost_surface=np.zeros((1, 2)),
        )
        analyzer = AmbiguityAnalyzer()
        with pytest.raises(ValueError, match="Ambiguity analysis requires a 2D cost surface"):
            analyzer.analyze(match_res)
