"""Simulation package for terrain matching."""

from terrain_matching.simulation.terrain_generator import TerrainGenerator
from terrain_matching.simulation.trajectory import FlightTrajectory, TrajectoryPoint
from terrain_matching.simulation.altimeter import AltimeterSimulator, TerrainProfile

__all__ = [
    "TerrainGenerator",
    "FlightTrajectory",
    "TrajectoryPoint",
    "AltimeterSimulator",
    "TerrainProfile",
]
