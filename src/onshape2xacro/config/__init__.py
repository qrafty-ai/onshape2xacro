import yaml
from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass, field


@dataclass
class ConfigOverride:
    """Robot parameter overrides from YAML."""

    joint_limits: Dict[str, Dict[str, float]] = field(default_factory=dict)
    inertials: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    dynamics: Dict[str, Dict[str, float]] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Optional[Path]) -> "ConfigOverride":
        """Load overrides from YAML file."""
        if not path or not path.exists():
            return cls()

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        return cls(
            joint_limits=data.get("joint_limits", {}),
            inertials=data.get("inertials", {}),
            dynamics=data.get("dynamics", {}),
        )

    def get_joint_limit(self, name: str, default: Dict[str, float]) -> Dict[str, float]:
        """Get overridden joint limit or return default."""
        if name in self.joint_limits:
            res = default.copy()
            res.update(self.joint_limits[name])
            return res
        return default

    def get_inertial(self, name: str, default: Dict[str, Any]) -> Dict[str, Any]:
        """Get overridden inertial or return default."""
        if name in self.inertials:
            res = default.copy()
            res.update(self.inertials[name])
            return res
        return default
