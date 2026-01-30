"""Inertia calculation report with warnings."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
from pathlib import Path

from loguru import logger

from .types import InertialProperties


@dataclass
class PartWarning:
    """Warning for a specific part."""

    link_name: str
    part_name: str
    message: str


@dataclass
class PartDebugInfo:
    """Debug information for a single part."""

    part_id: str
    bom_match: Optional[str]
    match_type: str
    mass_source: str
    material: Optional[str]
    volume_cm3: float
    mass_kg: float
    ixx: float = 0.0
    iyy: float = 0.0
    izz: float = 0.0
    mesh_match: Optional[str] = None
    warnings: List[str] = field(default_factory=list)


@dataclass
class InertiaReport:
    """Report collecting inertia results and warnings."""

    link_properties: Dict[str, InertialProperties] = field(default_factory=dict)
    warnings: List[PartWarning] = field(default_factory=list)
    link_parts: Dict[str, List[PartDebugInfo]] = field(default_factory=dict)

    def add_warning(self, link_name: str, part_name: str, message: str) -> None:
        """Add a warning for a part."""
        self.warnings.append(
            PartWarning(
                link_name=link_name,
                part_name=part_name,
                message=message,
            )
        )

    def add_link_parts(self, link_name: str, parts: List[PartDebugInfo]) -> None:
        """Add part breakdown for a link."""
        self.link_parts[link_name] = parts

    def get_summary(self) -> str:
        """Get summary of warnings."""
        if not self.warnings:
            return "No warnings - all parts have mass properties."

        lines = [f"⚠️  {len(self.warnings)} parts with missing mass properties:"]

        # Group by link
        by_link: Dict[str, List[PartWarning]] = {}
        for w in self.warnings:
            by_link.setdefault(w.link_name, []).append(w)

        for link_name, link_warnings in by_link.items():
            lines.append(f"\n  {link_name}:")
            for w in link_warnings:
                lines.append(f"    - {w.part_name}: {w.message}")

        return "\n".join(lines)

    def print_summary(self) -> None:
        """Print warning summary to logger."""
        summary = self.get_summary()
        if self.warnings:
            logger.warning(summary)
        else:
            logger.info(summary)

    def generate_debug_table(self) -> str:
        """Generate markdown table of parts breakdown per link."""
        if not self.link_parts:
            return "No part debug information available."

        lines = ["# Inertia and Mesh Calculation Debug Report\n"]

        for link_name in sorted(self.link_parts.keys()):
            parts = self.link_parts[link_name]
            props = self.link_properties.get(link_name)

            lines.append(f"## Link: {link_name}\n")

            lines.append(
                "| Part | BOM Match | Mass Source | Material | Volume (cm³) | Mass (kg) | Mesh Match | Status |"
            )
            lines.append(
                "|------|-----------|-------------|----------|--------------|-----------|------------|--------|"
            )

            link_warnings = []
            for part in parts:
                status = "✓"
                if part.warnings:
                    status = "⚠️ " + part.warnings[0]
                    link_warnings.extend(part.warnings)

                bom_match = part.bom_match or "-"
                if part.match_type == "fuzzy" and part.bom_match:
                    bom_match = f"(fuzzy: {bom_match})"

                material = part.material or "-"
                mesh_match = part.mesh_match or "MISSING"

                lines.append(
                    f"| {part.part_id} | {bom_match} | {part.mass_source} | "
                    f"{material} | {part.volume_cm3:.2f} | {part.mass_kg:.4f} | "
                    f"{mesh_match} | {status} |"
                )

            total_mass = props.mass if props else sum(p.mass_kg for p in parts)
            lines.append(f"\n**Link Total:** {total_mass:.4f} kg")

            if link_warnings:
                lines.append(f"**Warnings:** {len(link_warnings)} parts with issues")

            lines.append("\n---\n")

        return "\n".join(lines)

    def save_debug_table(self, output_path: Path) -> None:
        """Save debug table to file."""
        table = self.generate_debug_table()
        with open(output_path, "w") as f:
            f.write(table)
        logger.info(f"Debug table saved to {output_path}")
