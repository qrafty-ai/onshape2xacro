# test_mesh_exporters SUBDIRECTORY

## OVERVIEW
Specialized testing for STEP processing with extensive OCP C++ binding mocks. Tests color extraction, shape collection, format export.

## STRUCTURE
- `test_step.py`: Basic StepMeshExporter functionality
- `test_step_color.py` (325 lines): Dedicated color extraction tests
- `test_step_coverage.py` (178 lines): Branch coverage for error paths
- `test_step_api.py`: API integration tests

## MOCKING PATTERNS

### Core OCP Mocks
```python
# ALWAYS patch, NEVER instantiate
@patch('OCP.XCAFDoc.XCAFDoc_DocumentTool')
@patch('OCP.Quantity.Quantity_Color')
@patch('OCP.STEPCAFControl.STEPCAFControl_Reader')
```

### Document Tree Simulation
- Mock `XCAFDoc_DocumentTool.ShapeTool()` to return tool mock
- Tool mock provides `GetFreeShapes()`, `GetComponents()`, `GetShape()`
- Label mocks return other label mocks for tree traversal

### Color Tool Mocking
- Mock `XCAFDoc_ColorTool()` with `GetColor()` method
- Return `Quantity_Color` mock with `Red()`, `Green()`, `Blue()` methods
- Each color component returns float [0-1]

### Dynamic Side Effects
```python
# First call returns True (has children), second returns False (leaf)
mock_tool.GetComponents.side_effect = [True, False]
```

## EDGE CASES TESTED

- Color extraction from different label types (surface, generic, curve)
- Missing color tool → default white
- Recursive shape collection with transform accumulation
- Empty assemblies
- Invalid STEP file headers
- Multiprocess CoACD failures

## ANTI-PATTERNS

- **NO real OCP**: Tests fail with segfaults if OCP instantiated
- **NO skipping recursion breaking**: Line 132 pattern mandatory for tree mocks
