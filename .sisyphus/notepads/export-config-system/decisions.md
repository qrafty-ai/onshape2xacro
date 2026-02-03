## Decisions
- Used `dataclasses.asdict` for `save()` but manually converted `Path` to `str` for YAML compatibility.
- Implemented `merge_cli_overrides` to allow easy integration with CLI arguments (name, output, visual_mesh_format).
- Used `from __future__ import annotations` to support modern type hinting (`str | None`) while maintaining compatibility.
- Followed the existing pattern for the `load` method (checking path existence and returning a default instance).
### Link Name Override Collision Handling
- Decision: Raise `RuntimeError` if multiple links map to the same custom name.
- Rationale: URDF/Xacro requires unique link names. Silently merging or renaming would lead to unexpected robot configurations or broken kinematic chains. User intervention is required to resolve such conflicts in the configuration file.
