"""BOM (Bill of Materials) parser for Onshape CSV export."""

import csv
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional


@dataclass
class BOMEntry:
    """Single part entry from BOM."""

    name: str
    material: Optional[str]  # None if "- None -"
    mass_kg: Optional[float]  # None if "No value" or empty

    @property
    def has_mass(self) -> bool:
        return self.mass_kg is not None

    @property
    def has_material(self) -> bool:
        return self.material is not None


class BOMParser:
    """Parse Onshape BOM CSV export."""

    def parse(self, csv_path: Path) -> Dict[str, BOMEntry]:
        """
        Parse BOM CSV file.

        Args:
            csv_path: Path to CSV file exported from Onshape

        Returns:
            Dict mapping part name to BOMEntry
        """
        entries = {}

        with open(csv_path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = row["Name"].strip()
                material = self._parse_material(row["Material"])
                mass_kg = self._parse_mass(row["Mass"])

                entries[name] = BOMEntry(
                    name=name,
                    material=material,
                    mass_kg=mass_kg,
                )

        return entries

    def _parse_material(self, value: str) -> Optional[str]:
        """Parse material field, return None if not assigned."""
        value = value.strip()
        if value == "- None -" or not value:
            return None
        return value

    def _parse_mass(self, value: str) -> Optional[float]:
        """Parse mass field, return None if not calculated."""
        value = value.strip()
        if value == "No value" or not value:
            return None

        # Extract numeric value from "0.3 kg"
        match = re.match(r"([\d.]+)\s*kg", value, re.IGNORECASE)
        if match:
            return float(match.group(1))

        return None
