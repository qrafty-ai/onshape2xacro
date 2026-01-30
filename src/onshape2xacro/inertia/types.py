"""Data types for inertia computation."""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class InertialProperties:
    """Physical properties for a robot link."""

    mass: float  # kg
    com: Tuple[float, float, float]  # center of mass (m)
    ixx: float
    iyy: float
    izz: float
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

    def to_yaml_dict(self) -> dict:
        """Convert to inertials.yaml format."""
        return {
            "mass": self.mass,
            "origin": {
                "xyz": f"{self.com[0]} {self.com[1]} {self.com[2]}",
                "rpy": "0 0 0",
            },
            "inertia": {
                "ixx": self.ixx,
                "iyy": self.iyy,
                "izz": self.izz,
                "ixy": self.ixy,
                "ixz": self.ixz,
                "iyz": self.iyz,
            },
        }
