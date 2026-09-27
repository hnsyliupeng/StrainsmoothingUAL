# Twilight Wilderness Robot — **not visually approved**

Two real EEVEE review rounds are complete. Both were rejected for specific visual reasons; neither the `.blend` nor the data audits should be presented as approved. `output/visual_review_round1.json` and `output/visual_review_round2.json` preserve the image-based findings. All review renders are 640 × 480; no high-resolution render has been made.

## Round 2: what the images actually showed

- The revised light and framing made the ground and robot easier to read, and luminous yellow-green fireflies are visible in the overview.
- The lake is still only a small blue crescent, blocked by a large dark fallen log, shoreline rocks and foreground plants. It does not yet read as a clear lake feature.
- The robot changes pose and is more legible, but the overview alone still does not show foot contact and the IK gait clearly enough for acceptance.
- The firefly close-up improved, but the aura remains a distinct green ball and the abdomen hotspot clips toward white.
- The right-side forest remains dark. The round-2 candidate is **NEEDS_REVISION**.

A third revision is now prepared in the build sources:

1. Extend the GN clear-sightline to rocks, logs and flowering plants (while retaining their distribution elsewhere and preserving the flowering annulus).
2. Shrink the aura to below insect-body scale; lower its opacity, emission and point-light energy while retaining the visibly emissive abdomen and compositor bloom.
3. Animate the existing robot inspection camera to follow the IK-rigged body; generate 640 × 480 robot close-ups at frames 1, 75 and 144 alongside the overview and insect close-up.

The build/render workflow must finish, and those new images must be inspected before marking the JEV stages complete. The workflow uses four TAA samples for diagnostic review, not final rendering.

## Editable structure

- Forest and lake use a 30 m editable terrain grid, 15 Geometry Nodes scatters fed by CC0 asset meshes, and an editable 5,120-face water mesh. The lake edge is sampled at 256 directions against the analytic terrain height field.
- The fireflies are Blender NEWTON particles with Brownian motion, directional WIND and turbulence, evaluated for 144 frames and stored as relative mesh shape keys. The round-2 data audit records 107 airborne insects, 3.658 m flock travel and a 391.45 m sum of individual paths. Geometry Nodes instance the CC0 insect and aura on those baked paths.
- The robot retains the 18-link UBTECH Alpha 1S URDF/STL hierarchy. Its leg actions are solved by real 4-link ankle/sole IK constraints with pole targets, URDF axis locks/limits and evaluated sole-clearance checks. These facts describe structure, not a visual pass.

## Files

- Editable Blender candidate (round 2 until the next build finishes): `output/Twilight_Wilderness_Robot.blend`
- Latest stage report: `output/JEV_reviews.json`
- Actual round-1 and round-2 image findings: `output/visual_review_round1.json`, `output/visual_review_round2.json`
- Scene, physics/flight and IK data audits: `output/scene_audit.json`, `output/flight_audit.json`, `output/robot_ik_audit.json`
- Rebuild stages: `build_scene.py` → `upgrade_animation.py` → `animate_flight.py` → `refine_walk.py` → `audit_flight.py` → `audit_scene.py`

Do not treat the `.blend` as visually approved or ready for high-resolution rendering until a new EEVEE image review passes.
