"""Ambiguity analysis and correlation surface evaluation for terrain matching.

This module provides quantitative metrics to detect false fixes, multi-modal ambiguities,
and directional observability on 2D terrain correlation cost surfaces.
Implements Peak-to-Sidelobe Ratio (PSR), Multi-Modal Ambiguity Ratio (MAR),
Hessian surface curvature eigenvalues, and an avionics fix-rejection filter.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from terrain_matching.core.matcher import MatchResult


@dataclass(frozen=True)
class AmbiguityReport:
    """Quantitative assessment of correlation surface confidence and fix reliability.

    Attributes:
        is_fix_acceptable: True if all safety acceptance checkpoints are satisfied.
        rejection_reasons: List of human-readable failure reasons if rejected (empty if accepted).
        psr: Peak-to-Sidelobe Ratio in standard deviations.
        second_best_coord: (x, y) coordinate of the strongest rival optimum outside exclusion zone.
        second_best_cost: Metric score of the second-best candidate.
        ambiguity_margin: Relative margin between best and second-best candidate.
        curvature_eigenvalues: Optional tuple of (lambda_1, lambda_2) eigenvalues of local Hessian.
        anisotropy: Ratio lambda_1 / lambda_2 indicating directional elongation (condition number).
        sidelobe_mean: Mean value of background sidelobe costs.
        sidelobe_std: Standard deviation of background sidelobe costs.
    """

    is_fix_acceptable: bool
    rejection_reasons: list[str] = field(default_factory=list)
    psr: float = 0.0
    second_best_coord: tuple[float, float] = (0.0, 0.0)
    second_best_cost: float = 0.0
    ambiguity_margin: float = 0.0
    curvature_eigenvalues: tuple[float, float] | None = None
    anisotropy: float | None = None
    sidelobe_mean: float = 0.0
    sidelobe_std: float = 0.0


class AmbiguityAnalyzer:
    """Evaluates 2D correlation surfaces to accept or reject terrain-aided position fixes."""

    def __init__(
        self,
        exclusion_radius_m: float = 60.0,
        min_psr: float = 4.0,
        min_ambiguity_ratio: float = 1.3,
        max_acceptable_difference_cost: float = 5.0,
        min_acceptable_correlation_cost: float = 0.85,
        min_profile_variance: float = 4.0,
    ) -> None:
        """Initialize the AmbiguityAnalyzer with safety thresholds.

        Args:
            exclusion_radius_m: Radius around global fix excluded from sidelobe statistics.
            min_psr: Minimum Peak-to-Sidelobe Ratio in standard deviations to accept fix.
            min_ambiguity_ratio: Minimum ratio (for diff) or delta (for corr) over second-best peak.
            max_acceptable_difference_cost: Maximum allowable cost for MAD/MSD/ZMSD in meters or m^2.
            min_acceptable_correlation_cost: Minimum allowable correlation for NCC/ZNCC (e.g. 0.85).
            min_profile_variance: Minimum required terrain elevation variance in meters squared.

        Raises:
            ValueError: If thresholds are invalid.
        """
        if exclusion_radius_m <= 0.0:
            raise ValueError(f"exclusion_radius_m must be positive, got {exclusion_radius_m}")
        if min_psr <= 0.0:
            raise ValueError(f"min_psr must be positive, got {min_psr}")

        self.exclusion_radius_m = exclusion_radius_m
        self.min_psr = min_psr
        self.min_ambiguity_ratio = min_ambiguity_ratio
        self.max_acceptable_difference_cost = max_acceptable_difference_cost
        self.min_acceptable_correlation_cost = min_acceptable_correlation_cost
        self.min_profile_variance = min_profile_variance

    def analyze(
        self,
        match_result: MatchResult,
        measured_profile_variance: float | None = None,
    ) -> AmbiguityReport:
        """Perform comprehensive ambiguity analysis on a 2D MatchResult.

        Args:
            match_result: MatchResult object produced by DeterministicMatcher.
            measured_profile_variance: Optional variance of the measured elevation profile in m^2.

        Returns:
            AmbiguityReport detailing confidence metrics and pass/fail decision.

        Raises:
            ValueError: If match_result cost surface is 1D (corridor search) or empty.
        """
        cost_surface = match_result.cost_surface
        grid_x = match_result.search_grid_x
        grid_y = match_result.search_grid_y

        if cost_surface.ndim != 2 or cost_surface.shape[0] < 2 or cost_surface.shape[1] < 2:
            raise ValueError("Ambiguity analysis requires a 2D cost surface of shape (Ny >= 2, Nx >= 2).")

        metric = match_result.metric_name.lower()
        is_minimization = metric in ("mad", "msd", "rmse", "zmad", "zmsd")

        best_x, best_y = match_result.best_coord
        best_cost = match_result.best_cost

        # 1. Coordinate distances from the optimal fix across 2D grid
        xx, yy = np.meshgrid(grid_x, grid_y)
        dist_from_peak = np.hypot(xx - best_x, yy - best_y)

        # 2. Build sidelobe mask excluding circular mainlobe and invalid/penalty cells
        if is_minimization:
            valid_cells = np.isfinite(cost_surface) & (cost_surface < 1e9)
        else:
            valid_cells = np.isfinite(cost_surface) & (cost_surface > -0.999)

        sidelobe_mask = (dist_from_peak > self.exclusion_radius_m) & valid_cells
        rejection_reasons: list[str] = []

        if not np.any(sidelobe_mask):
            return AmbiguityReport(
                is_fix_acceptable=False,
                rejection_reasons=["Search window too small or no valid sidelobe cells outside exclusion zone."],
                psr=0.0,
                second_best_coord=(0.0, 0.0),
                second_best_cost=float(np.inf if is_minimization else -1.0),
                ambiguity_margin=0.0,
            )

        sidelobe_costs = cost_surface[sidelobe_mask]
        sidelobe_mean = float(np.mean(sidelobe_costs))
        sidelobe_std = float(np.std(sidelobe_costs))

        # 3. Peak-to-Sidelobe Ratio (PSR)
        eps = 1e-9
        if is_minimization:
            # Valley-to-sidelobe: how deeply best cost plunges below sidelobe mean
            psr = float((sidelobe_mean - best_cost) / (sidelobe_std + eps))
        else:
            # Peak-to-sidelobe: how high best correlation stands above sidelobe mean
            psr = float((best_cost - sidelobe_mean) / (sidelobe_std + eps))

        # 4. Identify second-best candidate outside exclusion zone
        # Mask out everything except sidelobe region with worst possible value
        masked_surface = cost_surface.copy()
        if is_minimization:
            masked_surface[~sidelobe_mask] = np.inf
            second_idx = np.unravel_index(np.argmin(masked_surface), masked_surface.shape)
        else:
            masked_surface[~sidelobe_mask] = -np.inf
            second_idx = np.unravel_index(np.argmax(masked_surface), masked_surface.shape)

        second_y = float(grid_y[second_idx[0]])
        second_x = float(grid_x[second_idx[1]])
        second_coord = (second_x, second_y)
        second_cost = float(cost_surface[second_idx])

        # 5. Multi-Modal Ambiguity Margin (MAR)
        if is_minimization:
            # Ratio of runner-up cost to best cost (must be >= threshold, e.g. 1.3)
            ambiguity_margin = float(second_cost / (best_cost + eps))
        else:
            # Absolute difference in correlation (must be >= threshold, e.g. 0.15)
            ambiguity_margin = float(best_cost - second_cost)

        # 6. Local Hessian Curvature around global optimum
        curvature_eigenvalues, anisotropy = self._compute_local_curvature(
            cost_surface, grid_x, grid_y, best_x, best_y
        )

        # 7. Evaluate the 4 Safety Checkpoints
        # Checkpoint 1: Terrain roughness
        if measured_profile_variance is not None:
            if measured_profile_variance < self.min_profile_variance:
                rejection_reasons.append(
                    f"Insufficient terrain roughness: variance {measured_profile_variance:.2f} m^2 "
                    f"< threshold {self.min_profile_variance:.2f} m^2."
                )

        # Checkpoint 2: Residual cost magnitude
        if is_minimization:
            if best_cost > self.max_acceptable_difference_cost:
                rejection_reasons.append(
                    f"Excessive residual error: cost {best_cost:.2f} "
                    f"> threshold {self.max_acceptable_difference_cost:.2f}."
                )
        else:
            if best_cost < self.min_acceptable_correlation_cost:
                rejection_reasons.append(
                    f"Insufficient correlation: score {best_cost:.3f} "
                    f"< threshold {self.min_acceptable_correlation_cost:.3f}."
                )

        # Checkpoint 3: Peak-to-Sidelobe Prominence
        if psr < self.min_psr:
            rejection_reasons.append(
                f"Low peak prominence: PSR {psr:.2f} sigma < threshold {self.min_psr:.2f} sigma."
            )

        # Checkpoint 4: Multi-Modal Ambiguity (Runner-Up Check)
        if is_minimization:
            if ambiguity_margin < self.min_ambiguity_ratio:
                rejection_reasons.append(
                    f"High multi-modal ambiguity: runner-up at {second_coord} with cost {second_cost:.2f} "
                    f"is too close (ratio {ambiguity_margin:.2f} < threshold {self.min_ambiguity_ratio:.2f})."
                )
        else:
            min_corr_margin = 0.10
            if ambiguity_margin < min_corr_margin:
                rejection_reasons.append(
                    f"High multi-modal ambiguity: runner-up correlation delta {ambiguity_margin:.3f} "
                    f"< threshold {min_corr_margin:.3f}."
                )

        is_acceptable = len(rejection_reasons) == 0

        return AmbiguityReport(
            is_fix_acceptable=is_acceptable,
            rejection_reasons=rejection_reasons,
            psr=psr,
            second_best_coord=second_coord,
            second_best_cost=second_cost,
            ambiguity_margin=ambiguity_margin,
            curvature_eigenvalues=curvature_eigenvalues,
            anisotropy=anisotropy,
            sidelobe_mean=sidelobe_mean,
            sidelobe_std=sidelobe_std,
        )

    def _compute_local_curvature(
        self,
        cost_surface: np.ndarray,
        grid_x: np.ndarray,
        grid_y: np.ndarray,
        best_x: float,
        best_y: float,
    ) -> tuple[tuple[float, float] | None, float | None]:
        """Estimate 2D Hessian matrix eigenvalues around the global optimum using finite differences."""
        # Find nearest grid indices
        j_opt = int(np.argmin(np.abs(grid_y - best_y)))
        i_opt = int(np.argmin(np.abs(grid_x - best_x)))

        ny, nx = cost_surface.shape
        # Need at least 1 pixel padding in every direction to compute central differences
        if j_opt <= 0 or j_opt >= ny - 1 or i_opt <= 0 or i_opt >= nx - 1:
            return None, None

        # Check if local 3x3 patch contains infs or nans
        patch = cost_surface[j_opt - 1 : j_opt + 2, i_opt - 1 : i_opt + 2]
        if not np.all(np.isfinite(patch)):
            return None, None

        dx = float(grid_x[1] - grid_x[0])
        dy = float(grid_y[1] - grid_y[0])

        # Second derivatives via central differences
        h_xx = (cost_surface[j_opt, i_opt + 1] - 2 * cost_surface[j_opt, i_opt] + cost_surface[j_opt, i_opt - 1]) / (dx**2)
        h_yy = (cost_surface[j_opt + 1, i_opt] - 2 * cost_surface[j_opt, i_opt] + cost_surface[j_opt - 1, i_opt]) / (dy**2)
        h_xy = (
            cost_surface[j_opt + 1, i_opt + 1]
            - cost_surface[j_opt + 1, i_opt - 1]
            - cost_surface[j_opt - 1, i_opt + 1]
            + cost_surface[j_opt - 1, i_opt - 1]
        ) / (4 * dx * dy)

        hessian = np.array([[h_xx, h_xy], [h_xy, h_yy]], dtype=np.float64)
        eigenvalues = np.linalg.eigvalsh(hessian)  # sorted ascending
        e1, e2 = float(eigenvalues[1]), float(eigenvalues[0])  # lambda_max, lambda_min

        anisotropy = float(np.abs(e1) / (np.abs(e2) + 1e-9))
        return (e1, e2), anisotropy
