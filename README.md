# Twilight Wilderness Robot — ongoing rebuild, **not finished**

**This repository is not a completed deliverable.** Previous screenshots provided by the user invalidated the former scene: coarse cards for foliage, indistinct materials, a box-assembled robot and uniform white firefly dots. Do not use this `.blend` for high-resolution rendering.

## Work actually done this iteration

- **Robot:** replaced the 99 procedural box/bolt parts with the [UBTECH Alpha 1S URDF digital twin](https://github.com/andresjjn/alpha1s-ros2-twin), MIT-licensed. 18 imported STL visual links (over 100,000 vertices) with 17 measured servo-axis joints are individually editable. The assembly was posed using joint transforms and scaled as one unit; the 2 feet are positioned within 2 cm of the floor. This is an asset-based machine, **not** proof of final appearance. Source visual STL, xacro and license live in `assets/alpha1s/`; source images for reference are in that directory.
- **Forest:** the current scene still uses MIT-licensed Proton Scatter demo trees, grass, bushes, and rocks, with LGPL-2.1 RedbudFlower petals. This vegetation **is not sufficiently realistic**, per user screenshot. Changing a grayscale material to a color ramp does not solve that. Scatter remains Geometry Nodes; no dense forest has been generated from scripted leaf primitives.
- **Fireflies:** Blender Newton particles with Brownian and turbulence field sampled through frame 75 and baked into 92 static instances. Their last visually reviewed version looked like white dots, so the appearance is **not accepted**.
- **Scene:** EEVEE 640×480 is configured; the current sandbox has no EGL/GLX render context. Direct OSMesa context attempt still failed to render EEVEE. No low-resolution preview PNG exists. `audit_scene.py` independently opens the scene and tests machine-readable structure, but cannot visually approve it.

[`output/JEV_reviews.json`](output/JEV_reviews.json) contains step-by-step status and `get_scene_info` equivalents. Stage 1 forest: **FAIL_VISUAL**. Stage 2 asset-based robot: **PENDING_VISUAL**. Stage 3 fireflies: **FAIL_VISUAL**. Stage 4 preview/material: **FAIL_VISUAL**. Stage 5 delivery: **BLOCKED**. This is deliberately not called "structure confirmed".

## To complete, not merely tweak

Acquire correctly licensed, high-quality tree/grass/rock assets with accompanying albedo/normal/roughness textures, replace demo vegetation, and review each stage at ≤720p EEVEE in an environment with a working OpenGL/EGL/GLX stack. The candidate `output/Twilight_Wilderness_Robot.blend` is only a checkpoint. `build_scene.py`, `robot_asset.py`, `audit_scene.py` and `render_lowres_preview.py` explain and reproduce the checkpoint; the latter refuses to render anything but 640×480 EEVEE. No high-resolution renders were made.
