# Twilight Wilderness Robot — **not visually approved**

The first real EEVEE review of this candidate is complete, and it **failed visual acceptance**. The four 640 × 480 frames are still in `output/`, but they show the pre-revision candidate and will be replaced by the next build. No high-resolution render has been made. `output/visual_review_round1.json` records the image-based findings.

## Round 1: what the images actually showed

- The glowing yellow-green insects are now clearly visible in the overview; their physics-baked paths are not being confused with static markers.
- The lake is still hidden by a near-black foreground canopy. The robot is small, underlit and partially hidden by plants, so the foot plants and gait are not visually legible.
- The close-up shows an oversized green sphere dominating the real insect, and the abdomen is close to white clipping. This is not acceptable glow presentation.
- The forest/ground contrast is too high. Geometry/data audits do not override these visual failures.

A corrective revision has been applied to the **build sources** and the EEVEE workflow is rebuilding the `.blend` before generating the next review set:

1. Reframe the overview to include both the lake and the moving robot; add a GN exclusion corridor between the camera and lake and clear ferns/shrubs from that corridor and the robot walk path.
2. Raise ambient/fill illumination and lighten the packed forest-floor grading, while preserving the twilight palette.
3. Shrink the anatomical firefly aura from 12 cm to near the insect’s scale, reduce its opacity/emission and temper the flight-following lights. The abdomen emission and compositor bloom remain.

Until those new images are inspected, the scene stays **NEEDS_REVISION**. The new working preview must remain EEVEE at 640 × 480 (or another size no greater than 720p). The workflow uses four TAA samples for diagnostic review, not final rendering.

## Editable structure

- Forest and lake use a 30 m editable terrain grid, 15 Geometry Nodes scatters fed by CC0 asset meshes, and an editable 5,120-face water mesh. The lake edge is sampled at 256 directions against the analytic terrain height field.
- The fireflies are Blender NEWTON particles with Brownian motion, directional WIND and turbulence, evaluated for 144 frames and stored as relative mesh shape keys. The current data audit records 107 airborne insects, 3.658 m flock travel and a 391.45 m sum of individual paths. Geometry Nodes instance the CC0 insect and aura on those baked paths.
- The robot retains the 18-link UBTECH Alpha 1S URDF/STL hierarchy. Its leg actions are solved by real 4-link ankle/sole IK constraints with pole targets, URDF axis locks/limits and evaluated sole-clearance checks. These facts describe structure, not a visual pass.

## Files

- Editable Blender candidate (round 1 until the rebuild workflow finishes): `output/Twilight_Wilderness_Robot.blend`
- Latest stage report: `output/JEV_reviews.json`
- Actual round-1 image findings: `output/visual_review_round1.json`
- Scene, physics/flight and IK data audits: `output/scene_audit.json`, `output/flight_audit.json`, `output/robot_ik_audit.json`
- Rebuild stages: `build_scene.py` → `upgrade_animation.py` → `animate_flight.py` → `refine_walk.py` → `audit_flight.py` → `audit_scene.py`

Do not treat the `.blend` as visually approved or ready for high-resolution rendering until the rebuilt EEVEE previews pass an actual image review.
