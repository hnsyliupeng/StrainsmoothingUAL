# Twilight Wilderness Robot — revised candidate scene

**Status: candidate only / 未通过最终 JEV。** The user's screenshots of BOTH previous editions showed grossly faceted vegetation, obstructed robot, black wireframe-like trees and absent-looking materials. That visual evidence invalidates the old structural PASS claims. The rebuilt candidate must not be represented as a visually validated final scene.

## Current deliverable

- `output/Twilight_Wilderness_Robot.blend`: Blender 4.5 editable scene, EEVEE preview configured **640 × 480**. The scene is **not** a high-resolution render.
- `output/JEV_reviews.json`: all five stages **PENDING_VISUAL**. The per-stage checks establish only machine-verifiable properties; a screenshot/EEVEE visual check is still required.
- `output/scene_audit.json`: independently reopening the blend verifies terrain-foot contact, object instances, packed assets, camera framing estimates, particle source and static bake. This does not certify appearance.
- `render_lowres_preview.py`: refuses any render other than 640 × 480 EEVEE. On a machine with graphical EEVEE support: `blender -b output/Twilight_Wilderness_Robot.blend --python render_lowres_preview.py`. Its report update *still* marks human visual review pending.

### What changed after the failed screenshot

- **Stage 1 environment — PENDING_VISUAL.** Replaced the former flat-shaded CC0Tree/Kenney polygon silhouettes with MIT-licensed [HungryProton Proton Scatter demo models](https://github.com/HungryProton/scatter/tree/main/addons/proton_scatter/demos/assets): bark + alpha-branch UV pines, alpha-leaf deciduous trees, UV grass/bushes, UV rocks. Images are **packed** into the blend. 7 Geometry Nodes scatter objects remain editable, including actual LGPL-licensed redbud flower meshes. The evaluated distribution currently has 20 pines, 4 standing dead trunks, 4 deciduous trees, 83 rocks, 975 grasses, 63 bushes and 17 flowering plants. A camera-corridor exclusion removes near-center trunks (validated from evaluated instance coordinates). Deadwood mesh has been cleaned of leftover foliage edges that appeared black/wireframe in the screenshot; viewport defaults to material mode with overlays off. These changes are data-checked, **not** image-checked. **No new preview has verified these changes.**
- **Stage 2 robot — PENDING_VISUAL.** Increased the articulated frame from 31 to 99 editable parts, adding keyed bearing axles, fasteners, paired leg pistons, shin/arm armour, articulated toe caps, routed cable harnesses, service hatch, battery cooling vanes and dual optical apertures. Still needs visual inspection for scale, intersections and believability.
- **Stage 3 fireflies — PENDING_VISUAL.** Preserved Blender's Newton particle + Brownian + turbulence simulation and the 92 baked luminous static instances. Still needs a visual check of color, size and distribution.
- **Stages 4–5 lighting/delivery — PENDING_VISUAL.** Cold moonlight and low amber bounce with EEVEE 640 × 480 setup. No new low-resolution EEVEE preview was produced; this sandbox lacks an EGL/GLX rendering context. **Do not declare “结构已确认，可交由人类进行高清渲染”.**

## Source and license

The **active** tree, grass, rock, and bush source meshes and textures are the MIT-licensed Proton Scatter demo assets, with license copy at `assets/proton_scatter/LICENSE.md`. Flower source: `assets/proton_scatter/RedbudFlower.obj` from [PlantSimulationLab/Helios](https://github.com/PlantSimulationLab/Helios), LGPL-2.1 (`assets/proton_scatter/RedbudFlower_LICENSE`). Previous unused CC0Tree FBX and Open-Golf Kenney OBJ sources remain in `assets/` for provenance/rebuild history, but are **not instantiated in the revised scene**; licenses are in `assets/CC0Tree_LICENSE` and `assets/Open-Golf_LICENSE`. All active textures and models are embedded in the blend, which opens without internet or source files.

`build_scene.py` rebuilds the candidate and runs machine checks, `audit_scene.py` verifies the saved blend. **The user-provided screenshot of the old scene was a FAIL; this revised scene has no screenshot yet and is not visually approved.**
