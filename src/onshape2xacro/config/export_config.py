from __future__ import annotations
import yaml
from pathlib import Path
from typing import Any
from dataclasses import dataclass, field, asdict
from typing import Literal


@dataclass
class CoACDOptions:
    threshold: float = 0.05
    resolution: int = 2000
    max_convex_hull: int = 32
    preprocess: bool = True
    seed: int = 42
    max_workers: int = 10


@dataclass
class CollisionOptions:
    method: Literal["fast", "coacd"] = "fast"
    coacd: CoACDOptions = field(default_factory=CoACDOptions)


@dataclass
class VisualMeshOptions:
    formats: list[str] = field(default_factory=lambda: ["obj"])
    max_size_mb: float = 10.0


@dataclass
class ExportOptions:
    name: str = "robot"
    visual_option: VisualMeshOptions = field(default_factory=VisualMeshOptions)
    collision_option: CollisionOptions = field(default_factory=CollisionOptions)
    output: Path = field(default_factory=lambda: Path("output"))
    bom: Path | None = None


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

        # Migration: old visual_mesh_format (singular) -> visual_option.formats
        if "visual_mesh_format" in export_data:
            fmt = export_data.pop("visual_mesh_format")
            visual_data = export_data.setdefault("visual_option", {})
            visual_data.setdefault("formats", [fmt])

        # Migration: old visual_mesh_formats (flat list) -> visual_option.formats
        if "visual_mesh_formats" in export_data:
            fmts = export_data.pop("visual_mesh_formats")
            visual_data = export_data.setdefault("visual_option", {})
            visual_data.setdefault("formats", fmts)

        # Parse visual_option tree
        visual_data = export_data.get("visual_option", {})
        if isinstance(visual_data, dict):
            valid_keys = VisualMeshOptions.__annotations__.keys()
            visual_data = {k: v for k, v in visual_data.items() if k in valid_keys}
            export_data["visual_option"] = VisualMeshOptions(**visual_data)

        if "output" in export_data:
            export_data["output"] = Path(export_data["output"])
        if "bom" in export_data and export_data["bom"]:
            export_data["bom"] = Path(export_data["bom"])

        collision_data = export_data.get("collision_option", {})
        if "coacd" in collision_data:
            # Sanitize keys: replace hyphens with underscores
            coacd_data = {
                k.replace("-", "_"): v for k, v in collision_data["coacd"].items()
            }
            # Filter out keys that don't belong to CoACDOptions to be safe
            valid_keys = CoACDOptions.__annotations__.keys()
            coacd_data = {k: v for k, v in coacd_data.items() if k in valid_keys}
            collision_data["coacd"] = CoACDOptions(**coacd_data)

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
        if data["export"].get("bom"):
            data["export"]["bom"] = str(data["export"]["bom"])

        path.parent.mkdir(parents=True, exist_ok=True)

        with open(path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def merge_cli_overrides(
        self,
        name: str | None = None,
        output: Path | None = None,
        visual_mesh_formats: list[str] | None = None,
        collision_method: Literal["fast", "coacd"] | None = None,
        bom: Path | None = None,
    ) -> None:
        if name:
            self.export.name = name
        if output:
            self.export.output = output
        if visual_mesh_formats:
            self.export.visual_option.formats = visual_mesh_formats
        if collision_method:
            self.export.collision_option.method = collision_method
        if bom:
            self.export.bom = bom
