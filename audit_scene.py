"""Independent Blender data audit; strictly NOT a visual JEV approval."""
import bpy, json, collections
from mathutils import Vector
from pathlib import Path
ROOT=Path(__file__).resolve().parent
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'output'/'Twilight_Wilderness_Robot.blend'))
s=bpy.context.scene;camera=s.camera
D=bpy.context.evaluated_depsgraph_get()
counts=dict(collections.Counter(i.parent.original.name for i in D.object_instances if i.is_instance))
flies=bpy.data.objects['Baked fireflies | frozen instances']; coords=[v.co for v in flies.data.vertices]
robot=list(bpy.data.collections['Robot'].objects)
feet=[bpy.data.objects[f'URDF | {side}_foot_link'] for side in ('l','r')]
def corners(o):return [o.matrix_world@Vector(c) for c in o.bound_box]
feet_base={o.name:round(min(p.z for p in corners(o)),4) for o in feet}
frame_objects=['URDF | base_link','URDF | Head_link','URDF | l_foot_link','URDF | r_foot_link']
world_to_camera=camera.matrix_world.inverted()
def screen_xy(o):
 p=world_to_camera @ (sum(corners(o),Vector())/8)
 return [round((p.x/camera.data.ortho_scale+.5)*640,1),round((.5-p.y/(camera.data.ortho_scale*.75))*480,1)]
frame_positions={name:screen_xy(bpy.data.objects[name]) for name in frame_objects}
w=round(s.render.resolution_x*s.render.resolution_percentage/100)
h=round(s.render.resolution_y*s.render.resolution_percentage/100)
source=bpy.data.objects['Source | RoseThicket_A']
checks={
 'eevee_640x480_configured':s.render.engine=='BLENDER_EEVEE_NEXT' and (w,h)==(640,480),
 'urdf_visual_meshes_18':len(robot)==18 and all(o.get('asset_source') for o in robot),
 'robot_visual_vertices_over_100k':sum(len(o.data.vertices) for o in robot)>100000,
 'robot_camera_frame':all(0<x<640 and 0<y<480 for x,y in frame_positions.values()),
 'foot_world_floor_tolerance_2cm':all(abs(z)<.02 for z in feet_base.values()),
 'fifteen_gn_environment_scatterers':len([o for o in bpy.data.collections['Environment'].objects if any(m.type=='NODES' for m in o.modifiers)])==15,
 'flowering_instances':counts.get('Flowering rose plants | GN scatter',0)>=20,
 'no_central_tree_trunks':all(not (abs(i.matrix_world.translation.x)<3.8 and -8<i.matrix_world.translation.y<5) for i in D.object_instances if i.is_instance and i.parent.original.name in {'Oaks A | GN scatter','Oaks B | GN scatter','Birches A | GN scatter','Birches B | GN scatter','Pine saplings | GN scatter'}),
 'fireflies_static_92':counts.get(flies.name)==len(flies.data.vertices)==92,
 'particle_engine_source_retained':len(bpy.data.objects['Simulation emitter | hidden after bake'].particle_systems)>0,
 'active_material_textures_packed':all(n.image and n.image.packed_file for m in bpy.data.materials if m.use_nodes and m.users for n in m.node_tree.nodes if n.type=='TEX_IMAGE'),
}
result={'kind':'non-rendering data audit; never substitute for visual JEV', 'checks':checks,
 'instances':counts,'robot_camera_pixel_positions':frame_positions,'foot_lowest_world_z_m':feet_base,
 'firefly_z_range_m':[round(min(p.z for p in coords),2),round(max(p.z for p in coords),2)],
 'full_visual_review_completed':False}
(ROOT/'output'/'scene_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
if not all(checks.values()):raise RuntimeError('Data audit failed')
