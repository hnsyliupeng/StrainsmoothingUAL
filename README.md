# Twilight Wilderness Robot — **not visually approved**

The last successful EEVEE images are **round 2**, not the current source candidate. They remain rejected for an obscured, undersized lake; a small, underlit robot without readable foot plants; and a firefly close-up whose green ball overwhelms the insect. Round 1 and round 2 findings are preserved in `output/visual_review_round1.json` and `output/visual_review_round2.json`. The current `.blend` and these older PNGs are not a visual pass. No high-resolution render has been made.

## Round 3 work now queued in the rebuild harness

- **Lake and forest sightline:** the shared terrain function now makes a broad, shallow basin (mean binary-solved shoreline radius about 4.40 m over 256 rays), and the editable water mesh follows that exact height field. GN scatter selection uses actual terrain Z versus the waterline, rather than a nominal radial approximation, so rocks, logs and plants should not float through submerged ground. The former two manually placed foreground rocks were removed. A clean foreground-to-lake corridor is retained while authored assets remain GN-scattered on dry land around the edges.
- **Robot:** the 18-link Alpha 1S URDF/STL assembly keeps a real armature and both knee poles. Four-axis position IK ends at each ankle pivot; the fifth, asymmetric ankle-roll servo is keyed from the terrain normal on its own foot bone, whose local rotation limit is aligned to and copied from the exact URDF axis/range. The actual toe offset is measured from the authored foot mesh, and stance targets are solved against evaluated sole clearance (3 mm goal, 0.5 mm tolerance); the contract rejects planted gaps outside −1 to 6 mm, ankle-pivot endpoint errors above 1 mm, or implausibly large target corrections. Root travel still follows the measured flock path. Three animated 640 × 480 follow-cameras show the actual IK pose at frames 1, 75 and 144. These latest solver changes are coded but have not yet passed the Blender harness.
- **Fireflies:** Blender’s retained emitter must pass direct settings checks for NEWTON physics, Brownian motion, directional wind and turbulence; its 144 sampled point states are baked into shape keys and GN instances. The abdomen remains the main chromatic green emitter, now set to strength 1.8; the separate transparent secondary halo is reduced to 1.1 cm with 0.4% alpha and 0.04 emission, while 18 lower-energy moving point lights and EEVEE FOG_GLOW remain. This should retain visible bioluminescence without the white hotspot/green ball seen in round 2, but the next rendered close-up must confirm it.
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
