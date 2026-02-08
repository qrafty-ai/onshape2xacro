# test_inertia SUBDIRECTORY

## OVERVIEW
Tests for BOM parsing, fuzzy name matching, inertia calculation, and debug reporting. Validates mass property computation and material fallback logic.

## STRUCTURE
- `test_calculator_with_bom.py` (224 lines): BOM integration tests
- `test_bom.py`: BOM parser edge cases
- `test_report.py`: Debug output formatting
- Other calculator tests

## TEST PATTERNS

### BOM Matching
- **Case Variations**: "Steel" vs "STEEL" vs "steel"
- **Substring Matching**: "Aluminum 6061" matches "aluminum"
- **Short Name Filter**: "a", "b", "no" should NOT match
- **Whitespace Handling**: "Stainless Steel" vs "Stainless  Steel"

### Inertia Calculation
- **Volume × Density**: Validates mass from geometry when BOM missing
- **Fallback Chain**: BOM mass > Calculated mass > Error
- **Coordinate Frame**: Inertia tensor in local link frame

### Debug Reporting
- **Mismatch Detection**: Flags when calculated mass ≠ BOM mass
- **Material Source Tracking**: Logs where density came from (BOM vs fallback)

## EDGE CASES

- BOM with missing material column → uses fallback density
- Part not in BOM → calculates from geometry
- BOM mass conflicts with geometry → warns user
- Empty parts (zero volume)

## ANTI-PATTERNS

- **NO assuming link-level mass**: Line 501 prohibition tested
- **NO skipping BOM validation**: Tests verify error on invalid BOM structure
