"""Terrain profile matching metrics and cost functions.

This module provides robust, vectorized mathematical distance and similarity metrics
to compare 1D terrain elevation profiles for terrain-relative navigation (TRN).
Includes standard differences (MAD, MSD, RMSE) and bias-invariant formulations
(ZMAD, ZMSD, NCC, ZNCC) designed to eliminate barometric atmospheric pressure drift.
"""

from __future__ import annotations

import numpy as np


def _validate_profile_inputs(h_meas: np.ndarray, h_ref: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Validate and convert inputs to 1D float64 numpy arrays of equal length.

    Args:
        h_meas: Measured elevation profile from flight sensors.
        h_ref: Candidate reference elevation profile from DEM.

    Returns:
        Tuple of (validated_h_meas, validated_h_ref) as 1D float64 arrays.

    Raises:
        ValueError: If arrays have different shapes, are not 1D, or have fewer than 2 points.
    """
    meas = np.asarray(h_meas, dtype=np.float64)
    ref = np.asarray(h_ref, dtype=np.float64)

    if meas.ndim != 1 or ref.ndim != 1:
        raise ValueError(
            f"Profiles must be 1D arrays, got h_meas.shape={meas.shape} and h_ref.shape={ref.shape}"
        )
    if len(meas) != len(ref):
        raise ValueError(
            f"Profile length mismatch: h_meas has {len(meas)} points, h_ref has {len(ref)} points"
        )
    if len(meas) < 2:
        raise ValueError(f"Profiles must contain at least 2 points for matching, got {len(meas)}")

    return meas, ref


def mean_absolute_difference(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute the Mean Absolute Difference (MAD, L1 cost) between two profiles.

    Formula:
        MAD = (1 / N) * sum(|h_meas[i] - h_ref[i]|)

    Args:
        h_meas: 1D array of measured terrain elevations in meters.
        h_ref: 1D array of reference DEM elevations in meters.

    Returns:
        Average absolute residual in meters. Lower is better (0.0 is exact match).
    """
    meas, ref = _validate_profile_inputs(h_meas, h_ref)
    return float(np.mean(np.abs(meas - ref)))


def mean_squared_difference(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute the Mean Squared Difference (MSD, L2^2 cost) between two profiles.

    Formula:
        MSD = (1 / N) * sum((h_meas[i] - h_ref[i])^2)

    Equivalent to negative log-likelihood under zero-mean additive Gaussian white noise.

    Args:
        h_meas: 1D array of measured terrain elevations in meters.
        h_ref: 1D array of reference DEM elevations in meters.

    Returns:
        Mean squared error in meters squared (m^2). Lower is better (0.0 is exact match).
    """
    meas, ref = _validate_profile_inputs(h_meas, h_ref)
    return float(np.mean((meas - ref) ** 2))


def root_mean_squared_difference(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute Root Mean Squared Error (RMSE) between two elevation profiles.

    Formula:
        RMSE = sqrt(MSD) = sqrt((1 / N) * sum((h_meas[i] - h_ref[i])^2))

    Restores physical units of meters to the L2 residual.

    Args:
        h_meas: 1D array of measured terrain elevations in meters.
        h_ref: 1D array of reference DEM elevations in meters.

    Returns:
        Root mean squared error in meters. Lower is better (0.0 is exact match).
    """
    return float(np.sqrt(mean_squared_difference(h_meas, h_ref)))


def zero_mean_msd(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute the bias-invariant Zero-Mean Mean Squared Difference (ZMSD).

    Mean-centers both profiles before computing the MSD, rendering the metric
    completely immune to constant barometric altimeter pressure bias.

    Formula:
        tilde_h_meas = h_meas - mean(h_meas)
        tilde_h_ref  = h_ref  - mean(h_ref)
        ZMSD = (1 / N) * sum((tilde_h_meas[i] - tilde_h_ref[i])^2)

    Args:
        h_meas: 1D array of measured terrain elevations in meters.
        h_ref: 1D array of reference DEM elevations in meters.

    Returns:
        Zero-mean squared error in meters squared. Lower is better (0.0 is exact match).
    """
    meas, ref = _validate_profile_inputs(h_meas, h_ref)
    tilde_meas = meas - np.mean(meas)
    tilde_ref = ref - np.mean(ref)
    return float(np.mean((tilde_meas - tilde_ref) ** 2))


def zero_mean_mad(h_meas: np.ndarray, h_ref: np.ndarray) -> float:
    """Compute the bias-invariant Zero-Mean Mean Absolute Difference (ZMAD).

    Combines the outlier robustness of the L1 norm with the vertical bias immunity
    of mean centering.

    Formula:
        tilde_h_meas = h_meas - mean(h_meas)
        tilde_h_ref  = h_ref  - mean(h_ref)
        ZMAD = (1 / N) * sum(|tilde_h_meas[i] - tilde_h_ref[i]|)

    Args:
        h_meas: 1D array of measured terrain elevations in meters.
        h_ref: 1D array of reference DEM elevations in meters.

    Returns:
        Zero-mean absolute error in meters. Lower is better (0.0 is exact match).
    """
    meas, ref = _validate_profile_inputs(h_meas, h_ref)
    tilde_meas = meas - np.mean(meas)
    tilde_ref = ref - np.mean(ref)
    return float(np.mean(np.abs(tilde_meas - tilde_ref)))


def normalized_cross_correlation(
    h_meas: np.ndarray,
    h_ref: np.ndarray,
    eps: float = 1e-9,
) -> float:
    """Compute Normalized Cross-Correlation (NCC) between two profiles.

    Formula:
        NCC = dot(h_meas, h_ref) / (norm(h_meas) * norm(h_ref))

    Bounded in [-1.0, 1.0]. Scale-invariant, but sensitive to additive vertical bias.

    Args:
        h_meas: 1D array of measured terrain elevations in meters.
        h_ref: 1D array of reference DEM elevations in meters.
        eps: Small non-negative threshold to prevent division by zero for zero-norm vectors.

    Returns:
        Correlation score in [-1.0, 1.0], where +1.0 indicates perfect collinearity.
        Returns 0.0 if either profile has norm < eps.
    """
    meas, ref = _validate_profile_inputs(h_meas, h_ref)
    norm_meas = float(np.linalg.norm(meas))
    norm_ref = float(np.linalg.norm(ref))

    denominator = norm_meas * norm_ref
    if denominator < eps:
        return 0.0

    score = float(np.dot(meas, ref) / denominator)
    return float(np.clip(score, -1.0, 1.0))


def zero_mean_normalized_cross_correlation(
    h_meas: np.ndarray,
    h_ref: np.ndarray,
    eps: float = 1e-9,
) -> float:
    """Compute Zero-Mean Normalized Cross-Correlation (ZNCC / Pearson correlation).

    The gold standard similarity metric for classical TERCOM matching.
    Invariant to both additive vertical bias (barometric shift) and positive linear scaling.

    Formula:
        tilde_h_meas = h_meas - mean(h_meas)
        tilde_h_ref  = h_ref  - mean(h_ref)
        ZNCC = dot(tilde_h_meas, tilde_h_ref) / (norm(tilde_h_meas) * norm(tilde_h_ref))

    Args:
        h_meas: 1D array of measured terrain elevations in meters.
        h_ref: 1D array of reference DEM elevations in meters.
        eps: Minimum standard deviation threshold to guard against flat-terrain singularities.

    Returns:
        Correlation score in [-1.0, 1.0], where +1.0 indicates identical profile shape.
        Returns 0.0 if terrain variation (standard deviation) is below eps (flat plains).
    """
    meas, ref = _validate_profile_inputs(h_meas, h_ref)
    tilde_meas = meas - np.mean(meas)
    tilde_ref = ref - np.mean(ref)

    norm_meas = float(np.linalg.norm(tilde_meas))
    norm_ref = float(np.linalg.norm(tilde_ref))

    denominator = norm_meas * norm_ref
    if denominator < eps:
        # Flat-terrain singularity safeguard: zero terrain roughness
        return 0.0

    score = float(np.dot(tilde_meas, tilde_ref) / denominator)
    return float(np.clip(score, -1.0, 1.0))
