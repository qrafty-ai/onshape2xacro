"""Write computed inertial properties to config files."""

from pathlib import Path
from typing import Dict

import yaml

from .types import InertialProperties


class InertiaConfigWriter:
    """Writes inertial properties to YAML config files."""

    def write(
        self,
        properties: Dict[str, InertialProperties],
        config_dir: Path,
    ) -> Path:
        """
        Write inertial properties to inertials.yaml.

        Args:
            properties: Map of link name to InertialProperties
            config_dir: Directory to write config file

        Returns:
            Path to written file
        """
        config_dir.mkdir(parents=True, exist_ok=True)

        inertials = {name: props.to_yaml_dict() for name, props in properties.items()}

        output_path = config_dir / "inertials.yaml"
        with open(output_path, "w") as f:
            yaml.dump({"inertials": inertials}, f, default_flow_style=False)

        return output_path
