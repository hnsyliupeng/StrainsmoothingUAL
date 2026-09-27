# Twilight wilderness robot — moving-flight candidate, not visually accepted

**This `.blend` is updated, but it is not visually approved.** The current sandbox still cannot create a 640×480 EEVEE preview because EGL/GLX reports `EGL_BAD_PARAMETER`; do not use counts or script results as a substitute for looking at a render. Do not request a high-resolution render yet.

## What was changed in this scene

- **Visible lake and nonflat land geometry:** editable 30 m terrain with a measured **2.56 m elevation range** and excavated basin; a separate glossy, moonlit water mesh (150 polygons) is clipped to *submerged* terrain. The main camera includes lake center at pixel (521, 283). All 15 GN scatters exclude the lake. The forest uses UV-preserving CC0 BFjord / Poly Haven assets; the previously sideways Poly Haven tree, fern, shrub, and rock meshes are converted from Y-up to Blender Z-up. See `forest_assets.py`, `build_scene.py` and `assets/` licenses.
- **Fireflies actually fly:** first evaluate a Blender NEWTON/Brownian/turbulence particle simulation and bake 92 initial positions. `animate_flight.py` then integrates independent Newton/drag/steering paths over 144 frames and stores 19 sampled states as mesh shape keys with linear interpolation. Both the CC0 anatomical insect (head, abdomen, wings, legs; five preserved face material groups) and the small warm halo GN-instance from the SAME moving vertices. The insect's abdomen is emissive (strength 18); eight warm point lights move with specific insects. An EEVEE compositor FOG_GLOW node exists for visible bloom. The swarm center travels **3.312 m in world space**. The unanimated simulation emitter remains hidden for provenance; the final flight plays in Blender with no script or live particle cache.
- **Robot really follows flight direction:** all 18 MIT UBTECH Alpha 1S STL visual links are parented to the 18-bone armature and their supplier URDF servo axes are keyframed. The evaluated *visible mesh*, not merely a rig control, moves **3.312 m** across the scene, in the same direction as the flying swarm (direction dot product 1.0), maintaining roughly 1.43 m behind its centroid. Independent bone channels alternate while moving, rather than stationary jogging. `terrain_profile.py` supplies the same height function to land and foot checks.
- **Materials:** actual source image atlases are packed, pine needles retain their face material group, and the robot shell has the CC0 ambientCG MetalPlates006 base color, metallic and roughness maps. These are data bindings, not proof the resulting composition looks good.

## Remaining issues / strict status

- Feet clear the terrain but a lifted foot may still hover up to **17.3 cm**: *convincing foot planting is NOT approved*. This is logged in `output/flight_audit.json`.
- A physically sized 5.1 cm firefly is only about 3 px across in the main 640px overview; the included anatomical close-up camera is for inspecting wings and abdomen separately. Do not claim the main overview shows their anatomy.
- **No preview PNG was generated or reviewed here.** `output/JEV_reviews.json` marks every stage `BLOCKED_PENDING_VISUAL`.

## Files and reproducibility

- [Editable Blender 4.5 file](output/Twilight_Wilderness_Robot.blend)
- [Evaluated 3D flight and chase audit](output/flight_audit.json) — 15 data checks, not a visual review
- [Staged JEV report](output/JEV_reviews.json)
- No `output/preview_640x480.png` exists yet.

Build in this order with Blender 4.5 bpy and xacro: `python3 build_scene.py && python3 upgrade_animation.py && python3 animate_flight.py && python3 audit_flight.py`. Each rebuild starts from scratch; `animate_flight.py` refuses to apply twice. On an OpenGL/EGL/GLX-capable Blender host, run `blender -b output/Twilight_Wilderness_Robot.blend --python render_lowres_preview.py` and inspect actual EEVEE frames 1, 75 and 144 and the firefly close-up at **no more than 720p** before any visual sign-off.
