# Twilight Wilderness Robot — editable candidate, **not visually approved**

The current `.blend` is structurally rebuilt, but the user’s earlier screenshot remains a rejection and this candidate has **not** been visually approved. Its old preview PNGs were deleted because they depict the rejected, pre-rebuild scene. No high-resolution render has been made. Automated audits below are evidence about scene data and motion only; they are not visual JEV approval.

## Current candidate

- **Forest and lake:** editable 30 m height-field terrain on a 128 × 128 grid (16,641 vertices; 2.562 m relief). Fifteen CC0 source meshes feed editable Geometry Nodes scatters. Tree framing exclusions were narrowed and plant scatter density increased to address the sparse clearing in the rejected screenshot. The water is an editable 5,120-face mesh whose shore is built from 256 binary-searched intersections against the analytic terrain height field, rather than the old coarse polygon clipping. Ground color retains the packed Poly Haven forest-floor map but is graded darker for twilight. These changes still need an EEVEE image review.
- **Fireflies:** `build_scene.py` runs Blender’s actual NEWTON particle system with Brownian motion, a directional WIND field and turbulence, sampling all 144 frames. 107 continuously airborne particles are baked into 144 relative shape keys on an editable vertex mesh; Geometry Nodes instance the CC0 anatomical firefly and a translucent aura at those moving points. The measured flock centroid travels **3.658 m** (aggregate individual path distance **391.45 m**). `animate_flight.py` no longer invents a second flight integrator: the robot follows the measured baked centroid, about 1.433 m behind it. The source emitter and force fields are retained hidden for provenance.
- **Bioluminescence:** the anatomical abdomen has saturated yellow-green emission (strength 4.8), a translucent 12 cm aura, 18 animated flight-following point lights, and an EEVEE compositor FOG_GLOW node. Those are implementation details—not proof the glow reads well at 640 × 480; that must be judged from the rendered image.
- **Robot and gait:** the 18-link UBTECH Alpha 1S URDF armature retains real STL link meshes. Both legs now have four-link ankle/sole IK constraints, animated world-space foot targets and knee pole targets, with the leg action curves removed. URDF axis locks and angular limits constrain each leg DOF. Evaluated sole-mesh stance clearance across frames 1–144 is **0.80–3.11 cm**; the all-phase range is **−0.24–5.04 cm** (the raised part is the authored swing arc). Maximum IK endpoint error is **0.000047 m**. The visible body follows the physical flock for 3.652 m with direction dot product 1.0. These measurements do not establish that the gait looks natural.
- **Lighting and presentation:** the ground grading, twilight exposure and forest framing were changed after inspecting the rejected screenshot. The only approved review format is EEVEE at **640 × 480** (or another size no larger than 720p).

## Visual-review blocker — approval is still pending

`output/JEV_reviews.json` marks all five stages `PENDING_VISUAL`. No current-candidate preview exists. I attempted the ≤720p EEVEE preview locally; the sandbox has no GPU device and its software EGL/llvmpipe path is too old for Blender 4.5 EEVEE (it reports OpenGL 3.1; forcing a 4.5 profile still segfaults even on a minimal EEVEE scene). I will not substitute Cycles, Workbench, a data-only audit, or the old rejected PNGs and call that a visual review.

On a working Blender 4.5 host with a compatible OpenGL/EGL/GLX context, generate only low-resolution review frames:

```bash
blender -b output/Twilight_Wilderness_Robot.blend \
  --python render_lowres_preview.py -- \
  --frames=1,75,144 --include-firefly --samples=4
```

Review the three overview frames and the animated anatomical close-up. If any of the user-reported issues remain (stiff gait, imperceptible glow, jagged shore, or floating dark geometry), fix them and repeat the JEV. **No visual sign-off has been recorded.**

## Files

- Editable candidate: `output/Twilight_Wilderness_Robot.blend` (43,401,354 bytes; 41.4 MiB)
- Staged JEV: `output/JEV_reviews.json` (all five stages remain `PENDING_VISUAL`)
- Independent non-rendering scene/data audit: `output/scene_audit.json` (22 checks pass; explicitly not visual approval)
- Physical flock / chase data audit: `output/flight_audit.json`
- Two-leg URDF IK and sole-contact audit: `output/robot_ik_audit.json`
- Build stages: `build_scene.py` → `upgrade_animation.py` → `animate_flight.py` → `refine_walk.py` → `audit_flight.py`

Do not treat the `.blend` as visually approved or ready for high-resolution rendering until the required ≤720p EEVEE review is completed.
