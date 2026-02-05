"""Compute mass properties from STEP geometry using CadQuery."""

from pathlib import Path
import re
from typing import Dict, Optional, Tuple, TYPE_CHECKING
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

    def _apply_parallel_axis_theorem(
        self, props: InertialProperties, link_com: Tuple[float, float, float]
    ) -> InertialProperties:
        """
        Apply parallel axis theorem to shift inertia from part COM to link COM.

        Args:
            props: Inertial properties about the part's own COM
            link_com: Center of mass of the combined link (x, y, z) in meters

        Returns:
            New InertialProperties with inertia shifted to link COM
        """
        dx = props.com[0] - link_com[0]
        dy = props.com[1] - link_com[1]
        dz = props.com[2] - link_com[2]

        m = props.mass

        ixx_shifted = props.ixx + m * (dy * dy + dz * dz)
        iyy_shifted = props.iyy + m * (dx * dx + dz * dz)
        izz_shifted = props.izz + m * (dx * dx + dy * dy)
        ixy_shifted = props.ixy - m * dx * dy
        ixz_shifted = props.ixz - m * dx * dz
        iyz_shifted = props.iyz - m * dy * dz

        return InertialProperties(
            mass=props.mass,
            com=props.com,
            ixx=ixx_shifted,
            iyy=iyy_shifted,
            izz=izz_shifted,
            ixy=ixy_shifted,
            ixz=ixz_shifted,
            iyz=iyz_shifted,
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

    def _validate_inertia(self, props: InertialProperties) -> list[str]:
        """
        Validate inertia tensor for physical consistency.

        Args:
            props: Inertial properties to validate

        Returns:
            List of warning messages (empty if valid)
        """
        warnings = []

        if props.ixx <= 0 or props.iyy <= 0 or props.izz <= 0:
            warnings.append(
                f"Invalid inertia: diagonal elements must be positive (ixx={props.ixx:.3e}, iyy={props.iyy:.3e}, izz={props.izz:.3e})"
            )

        if props.ixx == 0 or props.iyy == 0 or props.izz == 0:
            warnings.append(
                "Warning: one or more diagonal inertia elements is exactly zero"
            )

        if props.ixx + props.iyy < props.izz:
            warnings.append(
                f"Triangle inequality violated: ixx+iyy < izz ({props.ixx:.3e}+{props.iyy:.3e} < {props.izz:.3e})"
            )
        if props.ixx + props.izz < props.iyy:
            warnings.append(
                f"Triangle inequality violated: ixx+izz < iyy ({props.ixx:.3e}+{props.izz:.3e} < {props.iyy:.3e})"
            )
        if props.iyy + props.izz < props.ixx:
            warnings.append(
                f"Triangle inequality violated: iyy+izz < ixx ({props.iyy:.3e}+{props.izz:.3e} < {props.ixx:.3e})"
            )

        return warnings

    def compute_from_step_with_bom(
        self,
        step_path: Path,
        bom_entries: Dict[str, "BOMEntry"],
        link_name: str,
        report: "InertiaReport",
        part_metadata: list[Dict[str, str]] | None = None,
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
            part_metadata: Optional list of dicts with part_id and part_name for each solid

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

        from .report import PartDebugInfo

        part_debug_infos = []

        # Match BOM entry ONCE per link (not per solid)
        bom_entry = bom_entries.get(link_name)
        match_type = "none"
        bom_match_name = None

        if bom_entry:
            match_type = "exact"
            bom_match_name = link_name
        else:
            # Fuzzy match: find BOM entry name in link name
            for bom_name in bom_entries:
                if bom_name.lower() in link_name.lower():
                    bom_entry = bom_entries[bom_name]
                    match_type = "fuzzy"
                    bom_match_name = bom_name
                    break

        # First pass: compute properties using density for all solids
        # This gives us the relative mass distribution
        solid_props_list = []
        for i, solid in enumerate(solids):
            volume_mm3 = solid.Volume()
            volume_cm3 = volume_mm3 / 1000.0

            # Get part metadata if available
            part_meta = (
                part_metadata[i] if part_metadata and i < len(part_metadata) else None
            )
            part_meta.get("part_id") if part_meta else None
            part_name_full = part_meta.get("part_name") if part_meta else None
            mesh_match = part_meta.get("mesh_match") if part_meta else None

            # Use part_name as the primary identifier (it contains the actual part name)
            # Extract the leaf name from the full path (e.g., "sub-asm-base_1_square_base_plate_1" -> "square_base_plate")

            part_id = None
            if part_name_full:
                # Remove instance suffix _\d+$
                part_id = re.sub(r"_\d+$", "", part_name_full)
                # Try to extract leaf name by removing parent prefix
                parts = part_id.split("_")
                # Look for common assembly prefixes (parts[0] might be "sub-asm-xxx")
                if len(parts) > 2 and (
                    parts[0].startswith("sub") or parts[0].startswith("asm")
                ):
                    # Skip "sub-asm-xxx_1" prefix pattern
                    for j in range(len(parts)):
                        if parts[j].isdigit():
                            part_id = (
                                "_".join(parts[j + 1 :])
                                if j + 1 < len(parts)
                                else part_id
                            )
                            break

            if not part_id:
                part_id = f"solid_{i}"

            part_bom_entry = None
            part_match_type = "none"
            part_bom_match_name = None

            def normalize_name(name: str) -> str:
                s = name.lower()
                s = s.replace("&", "_")
                s = s.replace(".", "")
                s = re.sub(r"[^a-z0-9]", "_", s)
                s = re.sub(r"_+", "_", s)
                s = s.strip("_")
                return s

            part_bom_entry = bom_entries.get(part_id)
            if part_bom_entry:
                part_match_type = "exact"
                part_bom_match_name = part_id

            if not part_bom_entry and part_id:
                normalized_part = normalize_name(part_id)
                for bom_name in bom_entries:
                    if normalize_name(bom_name) == normalized_part:
                        part_bom_entry = bom_entries[bom_name]
                        part_match_type = "exact"
                        part_bom_match_name = bom_name
                        break

            if not part_bom_entry and part_name_full:
                # Try fuzzy match
                normalized_full = normalize_name(part_name_full)
                # Sort BOM entries by length (descending) to match longest names first
                # This prevents "screw M4" from matching "screw M4x10" if both exist (though unlikely with normalize)
                # But more importantly, avoids short random matches
                sorted_bom_names = sorted(bom_entries.keys(), key=len, reverse=True)

                for bom_name in sorted_bom_names:
                    normalized_bom = normalize_name(bom_name)
                    # Skip very short matches to avoid false positives (e.g. "a", "b", "no")
                    # But allow "4310", "J1_A" (length 4)
                    if len(normalized_bom) < 3:
                        continue

                    if (
                        normalized_bom in normalized_full
                        or normalized_full in normalized_bom
                    ):
                        part_bom_entry = bom_entries[bom_name]
                        part_match_type = "fuzzy"
                        part_bom_match_name = bom_name
                        break
                    if part_bom_entry:
                        break
                    if part_bom_entry:
                        break

            # Determine which density to use (prefer part-specific BOM, fallback to link-level)
            # CRITICAL FIX: Do NOT use bom_entry (link-level) as fallback for mass calculation.
            # If a screw is unmatched, it should NOT inherit the mass of the entire base plate.
            effective_bom = part_bom_entry

            # Priority 1: If BOM has mass, use mass/volume to get effective density
            if effective_bom and effective_bom.has_mass:
                volume_m3 = volume_cm3 * 1e-6
                if volume_m3 > 0:
                    density = effective_bom.mass_kg / volume_m3
                    material = (
                        effective_bom.material if effective_bom.has_material else None
                    )
                else:
                    # Volume is zero - this shouldn't happen but handle gracefully
                    density = self.default_density
                    material = None
            # Priority 2: If BOM has material (but no mass), use material density
            elif effective_bom and effective_bom.has_material:
                material = effective_bom.material
                density = self._get_density(material)
                if density is None:
                    density = self.default_density
            # Priority 3: Fallback to link-level material (but NEVER link-level mass)
            elif bom_entry and bom_entry.has_material:
                material = bom_entry.material
                density = self._get_density(material)
                if density is None:
                    density = self.default_density
            # Priority 4: No BOM data, use default
            else:
                material = None
                density = self.default_density

            props = self._compute_solid_properties(solid, density)
            solid_props_list.append(
                (
                    props,
                    volume_cm3,
                    material,
                    part_id,
                    part_name_full,
                    part_bom_entry,  # Store original part_bom_entry, not effective_bom
                    part_match_type,
                    part_bom_match_name,
                    mesh_match,
                )
            )

        for i, (
            props,
            volume_cm3,
            material,
            part_id,
            part_name,
            part_bom_entry,
            part_match_type,
            part_bom_match_name,
            mesh_match,
        ) in enumerate(solid_props_list):
            if not part_id:
                part_id = f"solid_{i}"
            part_warnings = []

            if part_match_type == "fuzzy":
                part_warnings.append(
                    f"Fuzzy matched to BOM entry '{part_bom_match_name}'"
                )

            if part_bom_entry and part_bom_entry.has_mass:
                mass_source = "BOM Mass"
            elif part_bom_entry and part_bom_entry.has_material:
                mass_source = "Material Density"
                if self._get_density(material) is None:
                    mass_source = "Default Density"
                    part_warnings.append(
                        f"Material '{material}' unknown, used default density"
                    )
            else:
                mass_source = "Default Density"
                if not part_bom_entry:
                    part_warnings.append("Not found in BOM")
                    report.add_warning(
                        link_name,
                        part_id,
                        "Part not found in BOM, using default density",
                    )

            final_props = props

            debug_info = PartDebugInfo(
                part_id=part_id,
                bom_match=part_bom_match_name or bom_match_name,
                match_type=part_match_type if part_match_type != "none" else match_type,
                mass_source=mass_source,
                material=material,
                volume_cm3=volume_cm3,
                mass_kg=final_props.mass,
                ixx=final_props.ixx,
                iyy=final_props.iyy,
                izz=final_props.izz,
                mesh_match=mesh_match,
                warnings=part_warnings,
            )
            part_debug_infos.append(debug_info)

            total_mass += final_props.mass
            if final_props.mass > 0:
                for j in range(3):
                    weighted_com[j] += final_props.com[j] * final_props.mass

        report.add_link_parts(link_name, part_debug_infos)

        if total_mass > 0:
            com = tuple(c / total_mass for c in weighted_com)
        else:
            com = (0.0, 0.0, 0.0)

        total_ixx = 0.0
        total_iyy = 0.0
        total_izz = 0.0
        total_ixy = 0.0
        total_ixz = 0.0
        total_iyz = 0.0

        for i, (
            props,
            volume_cm3,
            material,
            part_id,
            part_name,
            part_bom_entry,
            part_match_type,
            part_bom_match_name,
            mesh_match,
        ) in enumerate(solid_props_list):
            if part_bom_entry and part_bom_entry.has_mass:
                volume_m3 = volume_cm3 * 1e-6
                if volume_m3 > 0:
                    effective_density = part_bom_entry.mass_kg / volume_m3
                else:
                    effective_density = self.default_density

                final_props = self._compute_solid_properties(
                    solids[i], effective_density
                )
            elif part_bom_entry and part_bom_entry.has_material:
                # Recalculate using material density (in case it wasn't used in pass 1)
                mat_density = (
                    self._get_density(part_bom_entry.material) or self.default_density
                )
                final_props = self._compute_solid_properties(solids[i], mat_density)
            else:
                # If no specific part match, we MUST NOT use the link-level 'effective_bom'
                # because that assigns the mass of the entire assembly to each unmatched screw.
                # Just use default density or whatever was computed in pass 1 (which used default if no part match)
                final_props = props

            shifted_props = self._apply_parallel_axis_theorem(final_props, com)

            total_ixx += shifted_props.ixx
            total_iyy += shifted_props.iyy
            total_izz += shifted_props.izz
            total_ixy += shifted_props.ixy
            total_ixz += shifted_props.ixz
            total_iyz += shifted_props.iyz

        final_props = InertialProperties(
            mass=total_mass,
            com=com,
            ixx=total_ixx,
            iyy=total_iyy,
            izz=total_izz,
            ixy=total_ixy,
            ixz=total_ixz,
            iyz=total_iyz,
        )

        validation_warnings = self._validate_inertia(final_props)
        for warning in validation_warnings:
            report.add_warning(link_name, "link_total", warning)

        return final_props
