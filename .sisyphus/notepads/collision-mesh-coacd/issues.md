# Issues with Collision Mesh CoACD

- **2026-02-03**: User reported that CoACD is "way slower than the original method".
- Implementation updated to make it optional via `collision_mesh_method` config.
- Default is set to `"fast"`.

## Performance Learnings
- CoACD provides high accuracy but significantly impacts export time.
- Users need tuning parameters to balance speed and fidelity.
