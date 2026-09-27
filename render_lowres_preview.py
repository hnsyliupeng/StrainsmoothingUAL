"""Render and verify the mandated <=720p EEVEE preview in a GRAPHICAL Blender 4.5 environment.

Run from repo root using Blender binary: blender -b output/Twilight_Wilderness_Robot.blend --python render_lowres_preview.py
Or with bpy 4.5 and working EGL/GLX: python3 render_lowres_preview.py
The file refuses to render any high-resolution or non-EEVEE scene.
The script does NOT grant visual JEV approval: a human must inspect the output.
"""
import bpy
import json
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parent
BLEND = ROOT / 'output' / 'Twilight_Wilderness_Robot.blend'
PREVIEW = ROOT / 'output' / 'preview_640x480.png'
REPORT = ROOT / 'output' / 'JEV_reviews.json'

if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != BLEND.resolve():
    bpy.ops.wm.open_mainfile(filepath=str(BLEND))
scene = bpy.context.scene
import sys
requested = int(next((arg.split('=',1)[1] for arg in sys.argv if arg.startswith('--frame=')), '75'))
assert requested in (1,75,144), 'Only approved structure-review frames 1, 75, 144'
scene.frame_set(requested)
closeup = '--camera=firefly' in sys.argv
if closeup:
    scene.camera = bpy.data.objects['Camera | firefly anatomical inspection']
    PREVIEW = ROOT / 'output' / f'preview_firefly_{requested:03d}_640x480.png'
if requested != 75 and not closeup:
    PREVIEW = ROOT / 'output' / f'preview_frame_{requested:03d}_640x480.png'
assert scene.render.engine == 'BLENDER_EEVEE_NEXT', 'Refusing non-EEVEE engine'
w = round(scene.render.resolution_x * scene.render.resolution_percentage / 100)
h = round(scene.render.resolution_y * scene.render.resolution_percentage / 100)
assert (w, h) == (640, 480), f'Refusing unapproved preview size: {w}x{h}'
assert w <= 1280 and h <= 720, 'Refusing high-resolution render'
assert bpy.data.objects.get('Baked fireflies | frozen instances') is not None
assert scene.camera is not None
scene.render.image_settings.file_format = 'PNG'
scene.render.filepath = str(PREVIEW)
bpy.ops.render.render(write_still=True)
# Read the PNG's IHDR rather than assuming that the render completed successfully.
with PREVIEW.open('rb') as f:
    header = f.read(24)
assert header[:8] == b'\x89PNG\r\n\x1a\n', 'Preview PNG missing or invalid'
assert struct.unpack('>II', header[16:24]) == (640, 480), 'Preview resolution mismatch'
report = json.loads(REPORT.read_text())
for entry in report[3:]:
    entry['JEV']['Judgement'] = 'PENDING_HUMAN_REVIEW'
    entry['JEV']['Evidence']['eevee_preview_640x480_generated'] = True
    entry['JEV']['Verification'] += ' 已生成经 PNG 文件头核验的 640×480 EEVEE 预览；仍需人类目视复查机器人轮廓、脚部接触、植被比例、萤火虫分布及光感。'
REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2))
print('640x480 EEVEE preview ready for human visual JEV:', PREVIEW)
