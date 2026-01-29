# STEP Download Parse Errors Implementation Plan

Goal: prevent XML payloads from being saved as STEP files and ensure valid STEP content is selected from Onshape translations.

## Tasks
1. Add a failing test that simulates translation returning only XML (manifest or TreeNode) for both externaldata and translation download and asserts `export_step` raises a clear error.
2. Update `export_step` to:
   - Only write valid STEP payloads or ZIPs containing STEP.
   - Use Content-Disposition (when available) to identify `.step`/`.stp` filenames.
   - Avoid the fallback that writes the first externaldata file blindly.
3. Run focused tests and confirm they pass:
   - `test_export_step_uses_translation_download_when_only_xml`
   - `test_export_step_extracts_step_from_zip`
   - The new XML-only failure test

## Verification
- Run the above tests.
- Run `uv run onshape2xacro export waist --output waist_xacro` and confirm no missing meshes warning.
