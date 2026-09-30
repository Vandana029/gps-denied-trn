"""Unit tests for terrain matching metrics and cost functions.

Validates mathematical correctness, invariance properties (vertical bias, scale),
outlier robustness, and edge cases (flat-terrain singularities, dimension errors).
"""

import numpy as np
import pytest

from terrain_matching.core.metrics import (
    mean_absolute_difference,
    mean_squared_difference,
    normalized_cross_correlation,
    root_mean_squared_difference,
    zero_mean_mad,
    zero_mean_msd,
    zero_mean_normalized_cross_correlation,
)


@pytest.fixture
def synthetic_undulating_profile() -> tuple[np.ndarray, np.ndarray]:
    """Provide a reference profile and a matching identical profile with realistic relief."""
    x = np.linspace(0, 1000, 101)
    # Undulating terrain: hills of varying height (100m - 350m)
    h_ref = 200.0 + 80.0 * np.sin(2 * np.pi * x / 400.0) + 40.0 * np.cos(2 * np.pi * x / 150.0)
    h_meas = h_ref.copy()
    return h_meas, h_ref


class TestExactMatch:
    """Tests when measured and reference profiles are identical."""

    def test_exact_match_metrics(self, synthetic_undulating_profile: tuple[np.ndarray, np.ndarray]) -> None:
        h_meas, h_ref = synthetic_undulating_profile

        assert mean_absolute_difference(h_meas, h_ref) == pytest.approx(0.0, abs=1e-12)
        assert mean_squared_difference(h_meas, h_ref) == pytest.approx(0.0, abs=1e-12)
        assert root_mean_squared_difference(h_meas, h_ref) == pytest.approx(0.0, abs=1e-12)
        assert zero_mean_mad(h_meas, h_ref) == pytest.approx(0.0, abs=1e-12)
        assert zero_mean_msd(h_meas, h_ref) == pytest.approx(0.0, abs=1e-12)
        assert normalized_cross_correlation(h_meas, h_ref) == pytest.approx(1.0, rel=1e-7)
        assert zero_mean_normalized_cross_correlation(h_meas, h_ref) == pytest.approx(1.0, rel=1e-7)


class TestBarometricBiasInvariance:
    """Tests sensitivity vs invariance to constant atmospheric pressure shift."""

    def test_vertical_bias_effect(self, synthetic_undulating_profile: tuple[np.ndarray, np.ndarray]) -> None:
        _, h_ref = synthetic_undulating_profile
        bias = 30.0  # +30m vertical shift from cold front
        h_biased = h_ref + bias

        # 1. Raw difference metrics fail (measure the bias directly)
        assert mean_absolute_difference(h_biased, h_ref) == pytest.approx(30.0, rel=1e-7)
        assert mean_squared_difference(h_biased, h_ref) == pytest.approx(900.0, rel=1e-7)
        assert root_mean_squared_difference(h_biased, h_ref) == pytest.approx(30.0, rel=1e-7)

        # 2. Raw NCC is degraded by the vertical shift
        assert normalized_cross_correlation(h_biased, h_ref) < 1.0

        # 3. Bias-invariant metrics completely reject the vertical shift
        assert zero_mean_mad(h_biased, h_ref) == pytest.approx(0.0, abs=1e-12)
        assert zero_mean_msd(h_biased, h_ref) == pytest.approx(0.0, abs=1e-12)
        assert zero_mean_normalized_cross_correlation(h_biased, h_ref) == pytest.approx(1.0, rel=1e-7)


class TestScaleAndAffineInvariance:
    """Tests scale and affine transformation behavior for correlation metrics."""

    def test_positive_scale_invariance(self, synthetic_undulating_profile: tuple[np.ndarray, np.ndarray]) -> None:
        _, h_ref = synthetic_undulating_profile
        scale = 2.5
        h_scaled = scale * h_ref

        # NCC and ZNCC are invariant to positive multiplicative scaling
        assert normalized_cross_correlation(h_scaled, h_ref) == pytest.approx(1.0, rel=1e-7)
        assert zero_mean_normalized_cross_correlation(h_scaled, h_ref) == pytest.approx(1.0, rel=1e-7)

    def test_affine_transformation_invariance(
        self, synthetic_undulating_profile: tuple[np.ndarray, np.ndarray]
    ) -> None:
        _, h_ref = synthetic_undulating_profile
        scale = 1.75
        bias = -45.0
        h_affine = scale * h_ref + bias

        # ZNCC is invariant to any affine transform: h' = alpha * h + beta (alpha > 0)
        assert zero_mean_normalized_cross_correlation(h_affine, h_ref) == pytest.approx(1.0, rel=1e-7)

    def test_inverted_profile_correlation(
        self, synthetic_undulating_profile: tuple[np.ndarray, np.ndarray]
    ) -> None:
        _, h_ref = synthetic_undulating_profile
        # Invert the terrain relief
        h_inverted = -h_ref
        assert zero_mean_normalized_cross_correlation(h_inverted, h_ref) == pytest.approx(-1.0, rel=1e-7)


class TestOutlierSensitivity:
    """Tests L1 vs L2 behavior under isolated sensor measurement glitches."""

    def test_single_spike_outlier(self) -> None:
        n = 100
        h_ref = np.zeros(n)
        h_meas = np.zeros(n)
        h_meas[50] = 100.0  # Single 100m spike error

        mad = mean_absolute_difference(h_meas, h_ref)
        msd = mean_squared_difference(h_meas, h_ref)
        rmse = root_mean_squared_difference(h_meas, h_ref)

        # Expected: MAD = 100 / 100 = 1.0m
        assert mad == pytest.approx(1.0, rel=1e-7)
        # Expected: MSD = 10000 / 100 = 100.0m^2
        assert msd == pytest.approx(100.0, rel=1e-7)
        # Expected: RMSE = sqrt(100.0) = 10.0m
        assert rmse == pytest.approx(10.0, rel=1e-7)

        # Demonstrates that RMSE is an order of magnitude more sensitive to outliers than MAD
        assert rmse == pytest.approx(10.0 * mad, rel=1e-7)


class TestSingularitiesAndEdgeCases:
    """Tests flat-terrain singularities, zero norms, and numerical stability."""

    def test_flat_terrain_zncc_returns_zero(self) -> None:
        """Verify flat terrain (zero variance) returns 0.0 and does NOT raise NaN or crash."""
        n = 50
        h_flat = np.full(n, 1280.0)  # Salt flat at 1280m MSL
        h_ref_undulating = 1280.0 + np.sin(np.linspace(0, 10, n))

        # Both combinations should safely return 0.0
        assert zero_mean_normalized_cross_correlation(h_flat, h_ref_undulating) == 0.0
        assert zero_mean_normalized_cross_correlation(h_flat, h_flat) == 0.0

    def test_zero_vector_ncc_returns_zero(self) -> None:
        """Verify all-zero profile for raw NCC returns 0.0 rather than NaN."""
        zeros = np.zeros(20)
        ones = np.ones(20)
        assert normalized_cross_correlation(zeros, ones) == 0.0
        assert normalized_cross_correlation(zeros, zeros) == 0.0


class TestInputValidation:
    """Tests array dimensions, length mismatches, and types."""

    def test_length_mismatch_raises(self) -> None:
        h1 = np.ones(10)
        h2 = np.ones(12)
        with pytest.raises(ValueError, match="Profile length mismatch"):
            mean_squared_difference(h1, h2)

    def test_non_1d_array_raises(self) -> None:
        h1 = np.ones((10, 2))
        h2 = np.ones((10, 2))
        with pytest.raises(ValueError, match="must be 1D arrays"):
            mean_absolute_difference(h1, h2)

    def test_insufficient_points_raises(self) -> None:
        h1 = np.array([100.0])
        h2 = np.array([100.0])
        with pytest.raises(ValueError, match="at least 2 points"):
            zero_mean_msd(h1, h2)
