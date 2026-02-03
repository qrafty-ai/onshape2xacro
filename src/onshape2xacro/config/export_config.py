from __future__ import annotations
import yaml
from pathlib import Path
from typing import Dict, Any
from dataclasses import dataclass, field, asdict


@dataclass
class ExportOptions:
    name: str = "robot"
    visual_mesh_format: str = "obj"
    output: Path = field(default_factory=lambda: Path("output"))


@dataclass
class ExportConfiguration:
    export: ExportOptions = field(default_factory=ExportOptions)
    mate_values: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    link_names: Dict[str, str] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path) -> ExportConfiguration:
        if not path.exists():
            return cls()

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        export_data = data.get("export", {})
        if "output" in export_data:
            export_data["output"] = Path(export_data["output"])

        return cls(
            export=ExportOptions(**export_data),
            mate_values=data.get("mate_values", {}),
            link_names=data.get("link_names", {}),
        )

    def save(self, path: Path) -> None:
        data = asdict(self)
        data["export"]["output"] = str(data["export"]["output"])

        path.parent.mkdir(parents=True, exist_ok=True)

        with open(path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def merge_cli_overrides(
        self,
        name: str | None = None,
        output: Path | None = None,
        visual_mesh_format: str | None = None,
    ) -> None:
        if name:
            self.export.name = name
        if output:
            self.export.output = output
        if visual_mesh_format:
            self.export.visual_mesh_format = visual_mesh_format
