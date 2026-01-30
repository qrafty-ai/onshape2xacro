"""Compute mass properties from STEP geometry using CadQuery."""

from pathlib import Path
from typing import Dict, Optional, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from .bom import BOMEntry
    from .report import InertiaReport

import cadquery as cq

from .types import InertialProperties

logger = logging.getLogger(__name__)

# Material densities in kg/m³
# Sources: MatWeb, manufacturer datasheets
MATERIAL_DENSITIES = {
    # Metals
    "aluminum": 2700,
    "aluminium": 2700,
    "steel": 7850,
    "stainless steel": 8000,
    "stainless": 8000,
    "brass": 8500,
    "copper": 8960,
    "titanium": 4500,
    "iron": 7874,
    # Plastics
    "abs": 1050,
    "pla": 1250,
    "petg": 1270,
    "nylon": 1150,
    "polycarbonate": 1200,
    "pc": 1200,
    "acrylic": 1180,
    "pmma": 1180,
    "hdpe": 970,
    "pp": 905,
    "pom": 1410,
    "delrin": 1410,
    "peek": 1320,
    # Composites / Reinforced
    "ppa-cf": 1400,  # Carbon fiber reinforced PPA
    "nylon-cf": 1200,
    "carbon fiber": 1600,
    "cf": 1600,
    "fiberglass": 1900,
    "gfrp": 1900,
    "cfrp": 1550,
    # Rubber/Elastomers
    "rubber": 1100,
    "silicone": 1100,
    "tpu": 1200,
    "epdm": 1150,
    # Wood
    "wood": 700,
    "plywood": 680,
    "mdf": 750,
    # Default fallback
    "default": 1000,
}


class InertiaCalculator:
    """Computes mass and inertia properties from STEP geometry."""

    def __init__(
        self,
        default_density: float = 1000.0,
        mm_to_m: float = 0.001,
    ):
        """
        Initialize calculator.

        Args:
            default_density: Default material density in kg/m³
            mm_to_m: Conversion factor from model units to meters
        """
        self.default_density = default_density
        self.mm_to_m = mm_to_m

    def compute_from_step(
        self,
        step_path: Path,
        material: Optional[str] = None,
    ) -> InertialProperties:
        """
        Compute inertial properties from a STEP file.

        Args:
            step_path: Path to STEP file
            material: Optional material name for density lookup

        Returns:
            InertialProperties with mass, COM, and inertia tensor
        """
        # Load STEP
        model = cq.importers.importStep(str(step_path))
        shape = model.val()

        # Get density
        density = self._get_density(material)
        if density is None:
            return InertialProperties(
                mass=0.0,
                com=(0.0, 0.0, 0.0),
                ixx=0.0,
                iyy=0.0,
                izz=0.0,
            )

        # Volume in model units (mm³) -> m³
        volume_mm3 = shape.Volume()
        volume_m3 = volume_mm3 * (self.mm_to_m**3)

        # Mass
        mass = volume_m3 * density

        # Center of mass (mm -> m)
        # CadQuery's centerOfMass requires passing the shape object
        com_vec = shape.centerOfMass(shape)
        com = (
            com_vec.x * self.mm_to_m,
            com_vec.y * self.mm_to_m,
            com_vec.z * self.mm_to_m,
        )

        # Inertia tensor
        # CadQuery's matrixOfInertia returns inertia assuming density=1
        # Units: mm^5 (volume integral of r² over geometry)
        inertia_matrix = shape.matrixOfInertia(shape)

        # Scale factor: mm^5 -> m^5, then * density -> kg⋅m²
        scale = (self.mm_to_m**5) * density

        # Matrix is 3x3, extract diagonal and off-diagonal
        ixx = inertia_matrix[0][0] * scale
        iyy = inertia_matrix[1][1] * scale
        izz = inertia_matrix[2][2] * scale
        ixy = inertia_matrix[0][1] * scale
        ixz = inertia_matrix[0][2] * scale
        iyz = inertia_matrix[1][2] * scale

        return InertialProperties(
            mass=mass,
            com=com,
            ixx=ixx,
            iyy=iyy,
            izz=izz,
            ixy=ixy,
            ixz=ixz,
            iyz=iyz,
        )

    def _get_density(self, material: Optional[str]) -> Optional[float]:
        """
        Look up density for material name.

        Returns None if material is None (triggers zero-mass path).
        Returns default_density if material is unknown.
        """
        if material is None:
            return None

        # Normalize: lowercase, strip whitespace
        key = material.lower().strip()

        # Direct match
        if key in MATERIAL_DENSITIES:
            return MATERIAL_DENSITIES[key]

        # Partial match (e.g., "Aluminum 6061" matches "aluminum")
        # Prefer longer matches (e.g., "stainless steel" over "steel")
        for mat_name in sorted(MATERIAL_DENSITIES.keys(), key=len, reverse=True):
            density = MATERIAL_DENSITIES[mat_name]
            if mat_name in key or key in mat_name:
                logger.info(
                    f"Matched material '{material}' to '{mat_name}' (density={density} kg/m³)"
                )
                return density

        # Unknown material - log warning, return default
        logger.warning(
            f"Unknown material '{material}', using default density {self.default_density} kg/m³"
        )
        return self.default_density

    def _compute_solid_properties(self, solid, density: float) -> InertialProperties:
        """Compute properties for a single solid with given density."""
        volume_mm3 = solid.Volume()
        volume_m3 = volume_mm3 * (self.mm_to_m**3)
        mass = volume_m3 * density

        if mass <= 0:
            return InertialProperties(
                mass=0.0, com=(0.0, 0.0, 0.0), ixx=0.0, iyy=0.0, izz=0.0
            )

        com_vec = solid.centerOfMass(solid)
        com = (
            com_vec.x * self.mm_to_m,
            com_vec.y * self.mm_to_m,
            com_vec.z * self.mm_to_m,
        )

        inertia_matrix = solid.matrixOfInertia(solid)
        scale = (self.mm_to_m**5) * density

        return InertialProperties(
            mass=mass,
            com=com,
            ixx=inertia_matrix[0][0] * scale,
            iyy=inertia_matrix[1][1] * scale,
            izz=inertia_matrix[2][2] * scale,
            ixy=inertia_matrix[0][1] * scale,
            ixz=inertia_matrix[0][2] * scale,
            iyz=inertia_matrix[1][2] * scale,
        )

    def _compute_with_known_mass(self, solid, known_mass: float) -> InertialProperties:
        """Compute properties when mass is known from BOM."""
        volume_mm3 = solid.Volume()
        volume_m3 = volume_mm3 * (self.mm_to_m**3)

        if volume_m3 > 0:
            effective_density = known_mass / volume_m3
        else:
            effective_density = self.default_density

        return self._compute_solid_properties(solid, effective_density)

    def compute_from_step_with_bom(
        self,
        step_path: Path,
        bom_entries: Dict[str, "BOMEntry"],
        link_name: str,
        report: "InertiaReport",
    ) -> InertialProperties:
        """
        Compute inertial properties using BOM data for per-part mass.

        Priority:
        1. BOM mass (Onshape-calculated) -> use directly
        2. BOM material -> estimate from geometry × density
        3. Neither -> zero mass, add warning

        Args:
            step_path: Path to link STEP file
            bom_entries: Dict of part_name -> BOMEntry from BOM CSV
            link_name: Name of this link (for warnings)
            report: InertiaReport to collect warnings

        Returns:
            Aggregated InertialProperties for the link
        """

        model = cq.importers.importStep(str(step_path))

        # Get all solids in the compound
        try:
            solids = model.solids().vals()
        except Exception:
            # Fallback: treat whole model as single solid
            solids = [model.val()]

        if not solids:
            logger.warning(f"No solids found in {step_path}")
            return InertialProperties(
                mass=0.0, com=(0.0, 0.0, 0.0), ixx=0.0, iyy=0.0, izz=0.0
            )

        # Aggregate properties
        total_mass = 0.0
        weighted_com = [0.0, 0.0, 0.0]
        total_ixx = 0.0
        total_iyy = 0.0
        total_izz = 0.0
        total_ixy = 0.0
        total_ixz = 0.0
        total_iyz = 0.0

        for i, solid in enumerate(solids):
            # Try to match to BOM by index (STEP doesn't preserve part names well)
            # For now, use link_name as the primary key since each link STEP
            # typically contains parts that should share the link's BOM entry
            part_name = f"{link_name}_solid_{i}"

            # Try to find matching BOM entry
            bom_entry = bom_entries.get(link_name)

            if bom_entry is None:
                # Try fuzzy match on link name components
                for bom_name in bom_entries:
                    if (
                        bom_name.lower() in link_name.lower()
                        or link_name.lower() in bom_name.lower()
                    ):
                        bom_entry = bom_entries[bom_name]
                        break

            if bom_entry and bom_entry.has_mass:
                # Priority 1: Use Onshape-calculated mass
                # Divide mass among solids if multiple
                solid_mass = bom_entry.mass_kg / len(solids)
                props = self._compute_with_known_mass(solid, solid_mass)

            elif bom_entry and bom_entry.has_material:
                # Priority 2: Estimate from material density
                density = self._get_density(bom_entry.material)
                if density is None:
                    density = self.default_density
                props = self._compute_solid_properties(solid, density)

            elif bom_entry:
                # Priority 3: No mass, no material - zero mass with warning
                report.add_warning(
                    link_name, part_name, "No mass or material assigned in BOM"
                )
                props = InertialProperties(
                    mass=0.0, com=(0.0, 0.0, 0.0), ixx=0.0, iyy=0.0, izz=0.0
                )
            else:
                # No BOM entry found - use default density with warning
                report.add_warning(
                    link_name,
                    part_name,
                    "Part not found in BOM, using default density",
                )
                props = self._compute_solid_properties(solid, self.default_density)

            # Aggregate
            total_mass += props.mass
            if props.mass > 0:
                for j in range(3):
                    weighted_com[j] += props.com[j] * props.mass
            total_ixx += props.ixx
            total_iyy += props.iyy
            total_izz += props.izz
            total_ixy += props.ixy
            total_ixz += props.ixz
            total_iyz += props.iyz

        # Finalize COM (mass-weighted average)
        if total_mass > 0:
            com = tuple(c / total_mass for c in weighted_com)
        else:
            com = (0.0, 0.0, 0.0)

        return InertialProperties(
            mass=total_mass,
            com=com,
            ixx=total_ixx,
            iyy=total_iyy,
            izz=total_izz,
            ixy=total_ixy,
            ixz=total_ixz,
            iyz=total_iyz,
        )
