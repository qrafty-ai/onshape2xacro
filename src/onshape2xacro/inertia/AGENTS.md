# inertia MODULE

## OVERVIEW
BOM parsing, mass property calculation, and inertia reporting. Matches CAD parts to BOM entries for material density, computes inertia tensors from geometry.

## STRUCTURE
- `calculator.py` (660 lines): InertiaCalculator class, core computation
- `bom.py`: BOM CSV parsing, fuzzy name matching
- `report.py`: Debug report generation comparing expected vs. calculated values
- `writer.py`: Writes inertia config files
- `types.py`: InertialProperties dataclass

## WHERE TO LOOK

| Task | Symbol | Notes |
|------|--------|-------|
| Main calculator | `InertiaCalculator` | Orchestrates BOM matching + geometry calculation |
| BOM parsing | `BOMParser.parse()` | CSV → BOMEntry objects |
| Name matching | `BOMParser._fuzzy_match()` | Case-insensitive, substring matching |
| Geometry calculation | `InertiaCalculator._compute_from_geometry()` | Uses trimesh for volume/inertia |
| Debug output | `InertiaReport.to_markdown()` | Generates `inertia_debug.md` |

## CONVENTIONS

### BOM Matching
- **Fuzzy Logic**: Handles case variations, substring matches, whitespace
- **Short Name Filter**: Skips very short strings ("a", "b", "no") to avoid false positives (lines 455-460 of `calculator.py`)
- **Material Priority**: BOM material > Link default material > Fallback density
- **Required Columns**: BOM must have `Name`, `Material`, `Mass`

### Inertia Calculation
- **Uniform Distribution**: Assumes mass distributed uniformly across geometry (true for metals, NOT 3D-printed parts)
- **Coordinate Frame**: Inertia computed in link's local frame
- **Fallback Chain**: BOM mass > Geometry volume × density > Error

## ANTI-PATTERNS

- **NEVER use link-level mass**: Line 501 of `calculator.py` explicitly forbids link-level mass (allows link-level material for density only)
- **NO skipping BOM validation**: Always check `inertia_debug.md` after export

## UNIQUE PATTERNS

### Debug Reporting
- Compares BOM expected mass vs. calculated mass
- Flags mismatches with warnings
- Exports part-by-part breakdown with material sources

### Material Density Fallback
- BOM entry → Link default material → 1000 kg/m³ (water)
- Logs warnings when using fallback densities
