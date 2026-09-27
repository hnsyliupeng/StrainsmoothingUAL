"""Independent, non-rendering High-hardness scene audit. Never replaces visual JEV."""
import bpy, json, math, collections
from mathutils import Vector
from pathlib import Path
ROOT=Path(__file__).resolve().parent
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'output'/'Twilight_Wilderness_Robot.blend'))
s=bpy.context.scene
camera=s.camera
D=bpy.context.evaluated_depsgraph_get()
counts=dict(collections.Counter(i.parent.original.name for i in D.object_instances if i.is_instance))
flies=bpy.data.objects['Baked fireflies | frozen instances']; coords=[v.co for v in flies.data.vertices]
feet=[bpy.data.objects[f'{side} foot sole'] for side in ('L','R')]
def ground(x,y):return .13*math.sin(x*.46)*math.cos(y*.36)+.065*math.sin(y*.83+x*.17)
foot_errors={o.name:round((o.location.z-o.dimensions.z/2)-ground(o.location.x,o.location.y),5) for o in feet}
materials={o.name:[m.name for m in o.data.materials if m] for o in bpy.data.objects if o.name.startswith('Source |')}
world_to_camera=camera.matrix_world.inverted()
def screen_xy(o):
 p=world_to_camera@o.location;return [round((p.x/camera.data.ortho_scale+.5)*640,1),round((.5-p.y/(camera.data.ortho_scale*.75))*480,1)]
frame_positions={name:screen_xy(bpy.data.objects[name]) for name in ['Head sensor housing','Torso | pressure sealed frame','L foot sole','R foot sole']}
w=round(s.render.resolution_x*s.render.resolution_percentage/100)
h=round(s.render.resolution_y*s.render.resolution_percentage/100)
evidence={
 'preview_eevee_640x480_configured':s.render.engine=='BLENDER_EEVEE_NEXT' and (w,h)==(640,480),
 'camera_subject_in_frame':all(0<p[0]<640 and 0<p[1]<480 for p in frame_positions.values()),
 'feet_contact_terrain_tolerance_2cm':all(abs(v)<.02 for v in foot_errors.values()),
 'seven_geometry_nodes_environment_scatterers':len([o for o in bpy.data.collections['Environment'].objects if any(m.type=='NODES' for m in o.modifiers)])==7,
 'flowering_groundcover_instances':counts.get('Flowering understory | GN scatter',0)>=8,
 'no_central_tree_trunk':all(not (abs(i.matrix_world.translation.x)<3.8 and -8<i.matrix_world.translation.y<5) for i in D.object_instances if i.is_instance and i.parent.original.name in {'Pines | GN scatter','Dead trunks | GN scatter','Deciduous | GN scatter'}),
 'default_material_viewport':all(a.spaces.active.shading.type=='MATERIAL' and not a.spaces.active.overlay.show_overlays for sc in bpy.data.screens for a in sc.areas if a.type=='VIEW_3D'),
 'static_firefly_mesh_instances':counts.get(flies.name)==len(flies.data.vertices)==92,
 'firefly_height_layers':max(p.z for p in coords)-min(p.z for p in coords)>2,
 'self_contained_source_meshes':all(mats for mats in materials.values()) and not any(i.users>0 and i.type!='RENDER_RESULT' and not i.packed_file for i in bpy.data.images),
 'original_simulation_retained':len(bpy.data.objects['Simulation emitter | hidden after bake'].particle_systems)>0 and bpy.data.objects['Simulation force | turbulence'].field.type=='TURBULENCE',
}
result={'kind':'non-rendering structural audit, not visual JEV','checks':evidence,'instances':counts,'camera_pixel_positions':frame_positions,'feet_ground_errors_m':foot_errors,'fly_z_m':[round(min(p.z for p in coords),2),round(max(p.z for p in coords),2)],'source_materials':materials,'source_images_with_users':[(i.name,i.users) for i in bpy.data.images if i.type!='RENDER_RESULT' and i.users>0], 'full_visual_review_completed':False}
path=ROOT/'output'/'scene_audit.json';path.write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps(result,ensure_ascii=False,indent=2))
if not all(evidence.values()):raise RuntimeError('Independent structural audit failed')
