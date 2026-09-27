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
robot=[o for o in bpy.data.collections['Robot'].objects if o.type=='MESH']
rig=bpy.data.objects['Robot armature | 144-frame scanning and step pose']
insect=bpy.data.objects['Firefly | original CC0 insect mesh | source']
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
 'bone_rig_17_servos_plus_root':rig.type=='ARMATURE' and len(rig.data.bones)==18 and s.frame_end>=120,
 'rigged_robot_links':all(o.parent==rig and any(m.type=='ARMATURE' and m.object==rig for m in o.modifiers) and o.vertex_groups.get(o['urdf_link']) for o in robot),
 'authored_insect_anatomy':len(insect.data.vertices)>=900 and len(insect.data.materials)==5 and all(any(p.material_index==i for p in insect.data.polygons) for i in range(5)) and counts.get(flies.name)==len(flies.data.vertices)*2,
 'insect_glow_and_wings':insect.data.materials[1].node_tree.nodes.get('Principled BSDF').inputs['Emission Strength'].default_value>0 and all(m is not None for m in insect.data.materials),
 'robot_visual_vertices_over_100k':sum(len(o.data.vertices) for o in robot)>100000,
 'robot_camera_frame':all(0<x<640 and 0<y<480 for x,y in frame_positions.values()),
 'foot_mesh_bounds_finite':all(abs(z)<3 for z in feet_base.values()),
 'fifteen_gn_environment_scatterers':len([o for o in bpy.data.collections['Environment'].objects if any(m.type=='NODES' for m in o.modifiers)])==15,
 'inspection_cameras_present':all(bpy.data.objects.get(n) and bpy.data.objects[n].type=='CAMERA' for n in ('Camera | robot rig inspection','Camera | firefly anatomical inspection')) and camera.data.ortho_scale<=11,
 'pine_needles_have_real_face_materials':__import__('collections').Counter(p.material_index for p in bpy.data.objects['Source | pine_sapling_small_b'].data.polygons)[1]>1000 and len(bpy.data.objects['Source | pine_sapling_small_b'].data.materials)==2,
 'fallen_log_woodland_material':bpy.data.objects['Source | FallenHollowLog_A'].data.materials[0]==bpy.data.objects['Source | MatureOak_A'].data.materials[0],
 'oak_birch_b_variants_have_correct_uv_atlas':all(
     any(n.type=='TEX_IMAGE' and n.image and n.image.name.startswith('Woodland06Atlas') and n.image.packed_file
         for n in bpy.data.objects['Source | '+name].data.materials[0].node_tree.nodes)
     for name in ('MatureOak_B','SilverBirch_B')),
 'tree_scatter_has_random_z_rotation':all(
     any(n.bl_idname=='FunctionNodeRandomValue' and
         any(link.to_node.bl_idname=='ShaderNodeCombineXYZ' and link.to_socket.name=='Z'
             for link in o.modifiers[0].node_group.links if link.from_node==n)
         for n in o.modifiers[0].node_group.nodes)
     for o in bpy.data.collections['Environment'].objects
     if o.name in ('Oaks A | GN scatter','Oaks B | GN scatter','Birches A | GN scatter','Birches B | GN scatter')), 
 'flowering_instances':counts.get('Flowering rose plants | GN scatter',0)>=15,
 'no_central_tree_trunks':all(not (abs(i.matrix_world.translation.x)<3.8 and -8<i.matrix_world.translation.y<5) for i in D.object_instances if i.is_instance and i.parent.original.name in {'Oaks A | GN scatter','Oaks B | GN scatter','Birches A | GN scatter','Birches B | GN scatter','Pine saplings | GN scatter'}),
 'fireflies_92_simulation_baked_points_with_two_instanced_meshes':counts.get(flies.name)==len(flies.data.vertices)*2==184,
 'particle_engine_source_retained':len(bpy.data.objects['Simulation emitter | hidden after bake'].particle_systems)>0,
 'active_material_textures_packed':all(n.image and n.image.packed_file and all(n.image.size) for m in bpy.data.materials if m.use_nodes and m.users for n in m.node_tree.nodes if n.type=='TEX_IMAGE'),
 'all_image_references_packed':all(i.packed_file for i in bpy.data.images if i.type=='IMAGE' and i.users),
}
# Assert actual evaluated vertex motion, not merely the presence of action keys.
def posed_vertex(obj,frame):
 s.frame_set(frame);ev=obj.evaluated_get(bpy.context.evaluated_depsgraph_get());me=ev.to_mesh()
 try:return ev.matrix_world @ me.vertices[0].co
 finally:ev.to_mesh_clear()
motion=(posed_vertex(bpy.data.objects['URDF | l_forearm_link'],1)-posed_vertex(bpy.data.objects['URDF | l_forearm_link'],73)).length
checks['evaluated_arm_animation_moves_mesh']=motion>.01
s.frame_set(75)
result={'kind':'non-rendering data audit; never substitute for visual JEV', 'checks':checks,
 'instances':counts,'robot_camera_pixel_positions':frame_positions,'foot_lowest_world_z_m':feet_base,
 'firefly_z_range_m':[round(min(p.z for p in coords),2),round(max(p.z for p in coords),2)],
 'animation_test_displacement_m':round(motion,4),'frame_range':[s.frame_start,s.frame_end],'insect_mesh_vertices':len(insect.data.vertices), 'full_visual_review_completed':False}
(ROOT/'output'/'scene_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
if not all(checks.values()):raise RuntimeError('Data audit failed')
