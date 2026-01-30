"""Inertia calculation report with warnings."""

from dataclasses import dataclass, field
from typing import Dict, List
import logging

from .types import InertialProperties

logger = logging.getLogger(__name__)


@dataclass
class PartWarning:
    """Warning for a specific part."""

    link_name: str
    part_name: str
    message: str


@dataclass
class InertiaReport:
    """Report collecting inertia results and warnings."""

    link_properties: Dict[str, InertialProperties] = field(default_factory=dict)
    warnings: List[PartWarning] = field(default_factory=list)

    def add_warning(self, link_name: str, part_name: str, message: str) -> None:
        """Add a warning for a part."""
        self.warnings.append(
            PartWarning(
                link_name=link_name,
                part_name=part_name,
                message=message,
            )
        )

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
