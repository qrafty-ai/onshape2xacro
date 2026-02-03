from __future__ import annotations
import yaml
from pathlib import Path
from typing import Any
from dataclasses import dataclass, field, asdict
from typing import Literal


@dataclass
class CoACDOptions:
    threshold: float = 0.08
    resolution: int = 1000
    max_convex_hull: int = 8
    preprocess: bool = True
    seed: int = 42


@dataclass
class CollisionOptions:
    method: Literal["fast", "coacd"] = "fast"
    coacd: CoACDOptions = field(default_factory=CoACDOptions)


@dataclass
class ExportOptions:
    name: str = "robot"
    visual_mesh_format: Literal["glb", "dae", "obj", "stl"] = "obj"
    collision_option: CollisionOptions = field(default_factory=CollisionOptions)
    output: Path = field(default_factory=lambda: Path("output"))


@dataclass
class ExportConfiguration:
    export: ExportOptions = field(default_factory=ExportOptions)
    mate_values: dict[str, dict[str, Any]] = field(default_factory=dict)
    link_names: dict[str, str] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path) -> ExportConfiguration:
        if not path.exists():
            return cls()

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        export_data = data.get("export", {})
        if "output" in export_data:
            export_data["output"] = Path(export_data["output"])

        collision_data = export_data.get("collision_option", {})
        if "coacd" in collision_data:
            collision_data["coacd"] = CoACDOptions(**collision_data["coacd"])

        # Support old collision_mesh_method if present
        if "collision_mesh_method" in export_data:
            if "method" not in collision_data:
                collision_data["method"] = export_data.pop("collision_mesh_method")
            else:
                export_data.pop("collision_mesh_method")

        export_data["collision_option"] = CollisionOptions(**collision_data)

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
        visual_mesh_format: Literal["glb", "dae", "obj", "stl"] | None = None,
        collision_method: Literal["fast", "coacd"] | None = None,
    ) -> None:
        if name:
            self.export.name = name
        if output:
            self.export.output = output
        if visual_mesh_format:
            self.export.visual_mesh_format = visual_mesh_format
        if collision_method:
            self.export.collision_option.method = collision_method
