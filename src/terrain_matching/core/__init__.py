"""Core terrain matching package."""

from terrain_matching.core.ambiguity import AmbiguityAnalyzer, AmbiguityReport
from terrain_matching.core.dem import DigitalElevationModel
from terrain_matching.core.matcher import DeterministicMatcher, MatchResult
from terrain_matching.core.metrics import (
    mean_absolute_difference,
    mean_squared_difference,
    root_mean_squared_difference,
    zero_mean_mad,
    zero_mean_msd,
    normalized_cross_correlation,
    zero_mean_normalized_cross_correlation,
)

__all__ = [
    "DigitalElevationModel",
    "DeterministicMatcher",
    "MatchResult",
    "AmbiguityAnalyzer",
    "AmbiguityReport",
    "mean_absolute_difference",
    "mean_squared_difference",
    "root_mean_squared_difference",
    "zero_mean_mad",
    "zero_mean_msd",
    "normalized_cross_correlation",
    "zero_mean_normalized_cross_correlation",
]
