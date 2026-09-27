"""Render <=720p EEVEE review frames for human visual inspection.

Run with Blender 4.5 + working EGL/GLX, e.g.:
  blender -b output/Twilight_Wilderness_Robot.blend --python render_lowres_preview.py -- --frames=1,75,144 --include-firefly
Or use bpy 4.5 in a graphical/Xvfb session. This script refuses high-resolution
or non-EEVEE renders. It lowers TAA samples for fast structural previews only;
it does not save those temporary render settings back into the editable blend.
Automated PNG checks do not grant visual JEV approval.
"""
import bpy
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parent
BLEND = ROOT / 'output' / 'Twilight_Wilderness_Robot.blend'
REPORT = ROOT / 'output' / 'JEV_reviews.json'

if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != BLEND.resolve():
    bpy.ops.wm.open_mainfile(filepath=str(BLEND))
scene = bpy.context.scene

single_frame = next((arg.split('=', 1)[1] for arg in sys.argv if arg.startswith('--frame=')), None)
frames_arg = next((arg.split('=', 1)[1] for arg in sys.argv if arg.startswith('--frames=')), None)
if frames_arg is not None:
    assert single_frame is None, 'Choose --frame or --frames, not both'
    frames = [int(x) for x in frames_arg.split(',') if x]
else:
    frames = [int(single_frame or 75)]
assert frames and all(f in (1, 75, 144) for f in frames), 'Approved review frames are 1, 75, 144'
include_firefly = '--camera=firefly' in sys.argv or '--include-firefly' in sys.argv
sample_arg = next((arg.split('=', 1)[1] for arg in sys.argv if arg.startswith('--samples=')), '8')
samples = int(sample_arg)
assert 1 <= samples <= 16, 'Review renders are limited to 1–16 EEVEE TAA samples'

assert scene.render.engine == 'BLENDER_EEVEE_NEXT', 'Refusing non-EEVEE engine'
w = round(scene.render.resolution_x * scene.render.resolution_percentage / 100)
h = round(scene.render.resolution_y * scene.render.resolution_percentage / 100)
assert (w, h) == (640, 480), f'Refusing unapproved preview size: {w}x{h}'
assert w <= 1280 and h <= 720, 'Refusing high-resolution render'
assert bpy.data.objects.get('Baked fireflies | 144-frame physical particle cache') is not None
assert scene.camera is not None
scene.render.image_settings.file_format = 'PNG'
scene.render.use_persistent_data = True
scene.eevee.taa_render_samples = samples
inspection_camera = bpy.data.objects.get('Camera | firefly anatomical inspection')
assert not include_firefly or inspection_camera is not None, 'Missing animated firefly close-up camera'

def path_for(frame, closeup=False):
    if closeup:
        return ROOT / 'output' / f'preview_firefly_{frame:03d}_640x480.png'
    if frame == 75:
        return ROOT / 'output' / 'preview_640x480.png'
    return ROOT / 'output' / f'preview_frame_{frame:03d}_640x480.png'

def verify_png(path):
    with path.open('rb') as f:
        header = f.read(24)
    assert header[:8] == b'\x89PNG\r\n\x1a\n', f'Preview PNG missing or invalid: {path}'
    assert struct.unpack('>II', header[16:24]) == (640, 480), f'Preview resolution mismatch: {path}'

def record_pending_visual_review(path):
    report = json.loads(REPORT.read_text())
    generated = sorted(p.name for p in (ROOT / 'output').glob('preview*640x480.png'))
    note = f' 已生成PNG头校验的640×480 EEVEE预览（TAA {samples}样本）：{", ".join(generated)}；图像尚未目视审查，未自动批准。'
    for entry in report:
        entry['JEV']['Judgement'] = 'PENDING_HUMAN_REVIEW'
        entry['JEV']['Evidence']['eevee_preview_640x480_generated'] = True
        entry['JEV']['Evidence']['eevee_preview_taa_samples'] = samples
        entry['JEV']['Evidence']['preview_image_files'] = generated
        entry['JEV']['Evidence']['full_visual_review_completed'] = False
        entry['JEV']['Verification'] = entry['JEV']['Verification'].split(' 已生成PNG头校验的640×480 EEVEE预览')[0] + note
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2))

def render_review(frame, closeup=False):
    scene.camera = inspection_camera if closeup else bpy.data.objects['Camera | forest clearing']
    scene.frame_set(frame)
    target = path_for(frame, closeup)
    scene.render.filepath = str(target)
    print(f'RENDER START {target.name} frame={frame} closeup={closeup} size={w}x{h} samples={samples}', flush=True)
    bpy.ops.render.render(write_still=True)
    verify_png(target)
    record_pending_visual_review(target)
    print(f'RENDER COMPLETE {target.name} bytes={target.stat().st_size}', flush=True)

for frame in frames:
    render_review(frame)
if include_firefly:
    render_review(75, closeup=True)

# Leave the saved file untouched and restore an informative scene state in memory.
scene.camera = bpy.data.objects['Camera | forest clearing']
scene.frame_set(75)
print('EEVEE review PNGs are ready, but visual sign-off remains pending.')
