## Learnings
- `ExportConfiguration` dataclass handles the configuration for the export process, including CLI overrides.
- `PyYAML` is used for serialization, with manual handling for `Path` objects to ensure YAML compatibility.
- The `load` method handles non-existent files gracefully by returning a default instance, following the pattern in `ConfigOverride`.
- The `link_names` mapping is designed to override generated link names, which is useful when the heaviest part is not the desired link name.
- CLI tests use `monkeypatch` to set `sys.argv` and call `main()` directly.
- Mocking `pipeline.run_export` allows verifying the parsed and merged configuration object passed to the pipeline.
- When testing local directory exports, `cad.pickle` must be a valid pickle file if the code attempts to load it (e.g., using `pickle.dump({}, f)`).
- Failing tests currently show that `configuration.yaml` is not auto-loaded from the positional directory argument, and no validation exists yet for its presence.
- Updated `fetch-cad` to bootstrap a local export workflow by generating `configuration.yaml`.
- Integrated `ExportConfiguration` into the pipeline for better configuration management.
- Used a temporary `CondensedRobot` during fetch to discover auto-generated link names, enabling identity mapping in `link_names`.
- Replaced `mate_values.json` with a more comprehensive `configuration.yaml` while ensuring backward compatibility or transition by removing the old file generation.
- Verified that existing configuration files are not overwritten to protect user modifications.
- Export pipeline now enforces a local directory workflow: fetch-cad -> edit configuration.yaml -> export.
- Remote URL exports are deprecated in the 'export' command to ensure configuration.yaml is used.
- ExportConfiguration auto-loads from the local directory and merges CLI overrides.
- Strict mode: mate_values are strictly loaded from configuration.yaml (previously auto-generated or loaded from mate_values.json).
- Breaking import cycles by moving CLI config schemas to a separate schema.py file in the package root.
### Link Name Overrides
- Implemented link name overrides in `CondensedRobot.from_graph`.
- Use a dictionary mapping auto-generated link names to custom names.
- A collision check ensures that multiple auto-generated links are not mapped to the same custom name, preventing kinematic tree corruption.
- Overrides are applied after initial name generation and de-duplication to ensure we are mapping from stable, unique auto-generated names.

## Integration Testing and CLI Refactoring Learnings
- **CLI Parameter Changes**: Renaming parameters in a CLI configuration class (e.g., from 'url' to 'path') requires updating all direct instantiations in the test suite. Tyro and other CLI parsers depend heavily on these constructor signatures.
- **Early Validation**: Moving validation logic (like checking for the existence of 'cad.pickle' and 'configuration.yaml') from the pipeline to the CLI entry point provides better user feedback but requires careful coordination with tests that mock the filesystem or specific functions.
- **Mocking Path Objects**: When mocking `Path.is_dir()` or `Path.exists()`, it's important to handle path normalization (like single vs double slashes in URLs being treated as paths) to ensure mocks trigger correctly.
- **Integration Test Workflow**: A successful end-to-end integration test for a multi-step CLI tool should:
  1. Use `tmp_path` for isolation.
  2. Mock external API calls (Onshape) in the first phase (`fetch-cad`).
  3. Verify intermediate artifacts (`configuration.yaml`).
  4. Programmatically modify intermediate artifacts to test the "human-in-the-loop" aspect.
  5. Run the second phase (`export`) and verify final output/side effects.
