# Twilight Wilderness Robot — **not visually approved**

The last successful EEVEE images are **round 2**, not the current source candidate. They remain rejected for an obscured, undersized lake; a small, underlit robot without readable foot plants; and a firefly close-up whose green ball overwhelms the insect. Round 1 and round 2 findings are preserved in `output/visual_review_round1.json` and `output/visual_review_round2.json`. The current `.blend` and these older PNGs are not a visual pass. No high-resolution render has been made.

## Round 3 work now queued in the rebuild harness

- **Lake and forest sightline:** the shared terrain function now makes a broad, shallow basin (mean binary-solved shoreline radius about 4.40 m over 256 rays), and the editable water mesh follows that exact height field. GN scatter selection uses actual terrain Z versus the waterline, rather than a nominal radial approximation, so rocks, logs and plants should not float through submerged ground. The former two manually placed foreground rocks were removed. A clean foreground-to-lake corridor is retained while authored assets remain GN-scattered on dry land around the edges.
- **Robot:** the 18-link Alpha 1S URDF/STL assembly keeps a real armature and two four-link IK constraints with poles and URDF axis limits. Support targets are now corrected against evaluated sole-mesh vertices to a 3 mm target gap (0.5 mm solver tolerance); the harness rejects planted gaps outside −1 to 6 mm. Root travel still follows the measured flock path. Three animated 640 × 480 follow-cameras show the actual IK pose at frames 1, 75 and 144.
- **Fireflies:** Blender’s retained emitter must pass a direct settings check for NEWTON physics, Brownian motion, directional wind and turbulence; its 144 sampled point states are baked into shape keys and GN instances. The abdomen remains the main chromatic green emitter. Its strength was reduced to 3.0, and the separate transparent bloom prototype is now only 2.4 cm with low alpha/emission; 18 moving point lights and a restrained EEVEE FOG_GLOW remain. This is designed to make the insect read as the emitter rather than as a large green sphere, but only the next rendered close-up can confirm that.
- **Build and verification harness:** the branch workflow rebuilds the `.blend`, runs `audit_flight.py`, `audit_scene.py` and `scene_harness.py`, then creates three overview frames plus one insect and three robot close-ups. Any contract or render failure blocks publishing the candidate; failure logs are retained. `scene_contract_audit.json` and `preview_manifest.json` are data/PNG-integrity reports, not visual approval.

## Current delivery status

The round-2 `.blend` is still the last successfully published renderable candidate until the round-3 workflow completes. The new source changes have only passed local Python syntax and whitespace checks so far; the Blender build, IK contact checks and EEVEE images still need to finish and be reviewed. All review renders are restricted to 640 × 480 (≤720p). **Do not treat any geometry/count assertion as visual JEV approval.**

## Files

- Editable candidate: `output/Twilight_Wilderness_Robot.blend`
- Last image-based findings: `output/visual_review_round1.json`, `output/visual_review_round2.json`
- Stage report: `output/JEV_reviews.json`
- Non-rendering scene, flight and IK audits: `output/scene_audit.json`, `output/flight_audit.json`, `output/robot_ik_audit.json`
- New structural contract harness: `scene_harness.py`
- Rebuild: `build_scene.py` → `upgrade_animation.py` → `animate_flight.py` → `refine_walk.py` → audits → `render_lowres_preview.py`

Approval remains blocked until the newly generated low-resolution EEVEE images have been inspected and all visible issues are fixed.
