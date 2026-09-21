"""Unit tests for DigitalElevationModel and TerrainGenerator."""

import numpy as np
import pytest
from terrain_matching.core.dem import DigitalElevationModel
from terrain_matching.simulation.terrain_generator import TerrainGenerator


class TestDigitalElevationModel:
    """Test suite for DEM coordinate transformations and interpolation."""

    @pytest.fixture
    def simple_dem(self) -> DigitalElevationModel:
        """Create a 3x3 DEM for exact analytical testing.

        Matrix layout:
        Row 0 (North, y=100m): [ 100.0,  120.0,  140.0 ]
        Row 1 (Mid,   y=50m):  [ 140.0,  160.0,  180.0 ]
        Row 2 (South, y=0m):   [ 180.0,  200.0,  220.0 ]
        Col:                      0       1       2
        X:                       0m      50m     100m
        """
        grid = np.array(
            [
                [100.0, 120.0, 140.0],
                [140.0, 160.0, 180.0],
                [180.0, 200.0, 220.0],
            ],
            dtype=np.float64,
        )
        return DigitalElevationModel(
            elevation=grid,
            x_min=0.0,
            y_max=100.0,
            dx=50.0,
            dy=50.0,
        )

    def test_dimensions_and_bounds(self, simple_dem: DigitalElevationModel) -> None:
        """Verify rows, cols, and derived bounds."""
        assert simple_dem.rows == 3
        assert simple_dem.cols == 3
        assert simple_dem.x_min == 0.0
        assert simple_dem.x_max == 100.0
        assert simple_dem.y_min == 0.0
        assert simple_dem.y_max == 100.0
        assert simple_dem.bounds == (0.0, 100.0, 0.0, 100.0)

    def test_coordinate_transforms_invertibility(self, simple_dem: DigitalElevationModel) -> None:
        """Verify that world_to_grid and grid_to_world are exact mathematical inverses."""
        test_points = [(0.0, 100.0), (50.0, 50.0), (100.0, 0.0), (25.0, 75.0)]
        for x, y in test_points:
            r, c = simple_dem.world_to_grid(x, y)
            x_rec, y_rec = simple_dem.grid_to_world(r, c)
            assert pytest.approx(x_rec, abs=1e-9) == x
            assert pytest.approx(y_rec, abs=1e-9) == y

    def test_north_south_inversion_property(self, simple_dem: DigitalElevationModel) -> None:
        """Verify that moving North (+Y) decreases row index."""
        r_south, _ = simple_dem.world_to_grid(x=50.0, y=0.0)
        r_north, _ = simple_dem.world_to_grid(x=50.0, y=100.0)
        assert r_south > r_north
        assert r_south == 2.0
        assert r_north == 0.0

    def test_bounds_checking(self, simple_dem: DigitalElevationModel) -> None:
        """Verify inside and outside bounds queries."""
        assert simple_dem.in_bounds(50.0, 50.0) is True
        assert simple_dem.in_bounds(0.0, 0.0) is True
        assert simple_dem.in_bounds(100.0, 100.0) is True
        assert simple_dem.in_bounds(-1.0, 50.0) is False
        assert simple_dem.in_bounds(50.0, 101.0) is False

        with pytest.raises(ValueError, match="out of DEM bounds"):
            simple_dem.get_elevation(-10.0, 50.0)

    def test_grid_post_exact_elevation(self, simple_dem: DigitalElevationModel) -> None:
        """Sampling directly on grid posts must return exact matrix values."""
        assert simple_dem.get_elevation(0.0, 100.0) == 100.0   # Row 0, Col 0
        assert simple_dem.get_elevation(50.0, 100.0) == 120.0  # Row 0, Col 1
        assert simple_dem.get_elevation(50.0, 50.0) == 160.0   # Row 1, Col 1
        assert simple_dem.get_elevation(100.0, 0.0) == 220.0   # Row 2, Col 2

    def test_bilinear_interpolation_center_quadrant(self, simple_dem: DigitalElevationModel) -> None:
        """Verify bilinear interpolation in the center of the top-left cell.

        Corner values: Q11=100, Q12=120, Q21=140, Q22=160.
        Center point: x=25m, y=75m (u=0.5, v=0.5).
        Expected value: average of all four = 130.0.
        """
        elev = simple_dem.get_elevation(25.0, 75.0, method="bilinear")
        assert pytest.approx(elev, abs=1e-9) == 130.0

    def test_bilinear_interpolation_edge_midpoint(self, simple_dem: DigitalElevationModel) -> None:
        """Verify interpolation along cell edges.

        Top edge between (0, 100) and (50, 100):
        At x=25m, y=100m, elevation should be halfway between 100 and 120 = 110.0.
        """
        elev = simple_dem.get_elevation(25.0, 100.0, method="bilinear")
        assert pytest.approx(elev, abs=1e-9) == 110.0

    def test_nearest_neighbor_interpolation(self, simple_dem: DigitalElevationModel) -> None:
        """Verify nearest neighbor snapping behavior."""
        # Closer to (0, 100) -> 100.0
        assert simple_dem.get_elevation(10.0, 90.0, method="nearest") == 100.0
        # Closer to (50, 100) -> 120.0
        assert simple_dem.get_elevation(40.0, 90.0, method="nearest") == 120.0

    def test_get_elevation_profile(self, simple_dem: DigitalElevationModel) -> None:
        """Verify vectorized path sampling across coordinates."""
        flight_path = np.array([
            [0.0, 100.0],
            [25.0, 75.0],
            [50.0, 50.0],
        ])
        profile = simple_dem.get_elevation_profile(flight_path)
        assert len(profile) == 3
        assert pytest.approx(profile[0]) == 100.0
        assert pytest.approx(profile[1]) == 130.0
        assert pytest.approx(profile[2]) == 160.0

    def test_invalid_initialization(self) -> None:
        """Verify error handling on invalid initialization parameters."""
        with pytest.raises(ValueError, match="Elevation array must be 2D"):
            DigitalElevationModel(np.array([1, 2, 3]))

        with pytest.raises(ValueError, match="at least 2x2"):
            DigitalElevationModel(np.array([[100]]))

        with pytest.raises(ValueError, match="must be strictly positive"):
            DigitalElevationModel(np.ones((3, 3)), dx=-1.0)


class TestTerrainGenerator:
    """Test suite for procedural terrain generation."""

    def test_generate_flat(self) -> None:
        """Verify flat terrain generation."""
        dem = TerrainGenerator.generate_flat(shape=(20, 20), elevation=250.0)
        assert dem.rows == 20
        assert dem.cols == 20
        assert np.all(dem.elevation_matrix == 250.0)
        assert dem.get_elevation(50.0, 50.0) == 250.0

    def test_generate_gaussian_ridge(self) -> None:
        """Verify Gaussian ridge peak location and decay."""
        dem = TerrainGenerator.generate_gaussian_ridge(
            shape=(51, 51),
            base_elevation=100.0,
            peak_height=400.0,
            center_x=500.0,
            center_y=500.0,
            x_min=0.0,
            y_max=1000.0,
            dx=20.0,
            dy=20.0,
        )
        # Peak must be at center (500m, 500m) with height base + peak = 500.0m
        peak_elev = dem.get_elevation(500.0, 500.0)
        assert pytest.approx(peak_elev, rel=1e-3) == 500.0

        # Far edges should approach base elevation 100.0m
        corner_elev = dem.get_elevation(0.0, 0.0)
        assert corner_elev < 150.0

    def test_generate_rolling_hills(self) -> None:
        """Verify rolling hills generation has rich variation."""
        dem = TerrainGenerator.generate_rolling_hills(
            shape=(40, 40),
            base_elevation=300.0,
            primary_amplitude=50.0,
            seed=123,
        )
        assert dem.rows == 40
        assert dem.cols == 40
        std_dev = np.std(dem.elevation_matrix)
        assert std_dev > 10.0  # Significant contour variation
        assert np.min(dem.elevation_matrix) < 300.0
        assert np.max(dem.elevation_matrix) > 300.0
