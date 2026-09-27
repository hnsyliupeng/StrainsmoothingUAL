"""Upgrade the asset-based candidate: rigid URDF-bone rig, 144-frame performance,
authored CC0 firefly insect instances, and repair imported placeholder references.
Run: LD_LIBRARY_PATH=/tmp/bpy-libs python3 upgrade_animation.py
Does not render or assert visual approval.
"""
from pathlib import Path
from xml.etree import ElementTree as ET
import bpy, xacro, math, json
from mathutils import Vector, Matrix, Quaternion
ROOT=Path(__file__).resolve().parent
BLEND=ROOT/'output/Twilight_Wilderness_Robot.blend'
bpy.ops.wm.open_mainfile(filepath=str(BLEND))
scene=bpy.context.scene
robot=bpy.data.collections['Robot'];flies=bpy.data.collections['Fireflies'];sources=bpy.data.collections['Architecture / open-source asset prototypes']
# Imported FBX placeholder images are not connected to actual retained materials.
# Purge them rather than leaving broken, magenta paths in the deliverable.
removed=[]
for img in list(bpy.data.images):
    if img.type=='IMAGE' and not img.packed_file:
        removed.append(img.name);bpy.data.images.remove(img,do_unlink=True)
assert not any(i.type=='IMAGE' and i.users and not i.packed_file for i in bpy.data.images)
# Matte authored skin finishes are intentionally material properties (the STL
# meshes have no UVs); they are not falsely labelled as photographed PBR maps.
def finish(name, color, metal, rough):
    mat=bpy.data.materials.new(name);mat.diffuse_color=(*color,1);mat.use_nodes=True
    bs=mat.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=(*color,1)
    bs.inputs['Metallic'].default_value=metal;bs.inputs['Roughness'].default_value=rough
    return mat
# AmbientCG MetalPlates006: real CC0 color, metallic and roughness images;
# authored STL links lack UVs, so Generated coordinates supply repeatable triplanar
# surface mapping (mesh geometry remains imported and fully editable).
shell=finish('Robot / ambientCG MetalPlates006 / authored STL',(.37,.44,.44),.42,.48)
bs=shell.node_tree.nodes.get('Principled BSDF');nodes=shell.node_tree.nodes;links=shell.node_tree.links
coords=nodes.new('ShaderNodeTexCoord');mapping=nodes.new('ShaderNodeVectorMath');mapping.operation='SCALE';mapping.inputs[3].default_value=2.0;links.new(coords.outputs['Generated'],mapping.inputs[0])
for suffix,input_name in [('Color','Base Color'),('Metalness','Metallic'),('Roughness','Roughness')]:
    img=bpy.data.images.load(str(ROOT/'assets/robot_surface'/(suffix+'.jpg')),check_existing=True)
    if suffix!='Color':img.colorspace_settings.name='Non-Color'
    img.pack();assert img.packed_file and all(img.size)
    tex=nodes.new('ShaderNodeTexImage');tex.image=img;tex.projection='BOX';tex.projection_blend=.35
    links.new(mapping.outputs['Vector'],tex.inputs['Vector']);links.new(tex.outputs['Color'],bs.inputs[input_name])
joints=finish('Robot / exposed graphite actuator / authored STL',(.055,.075,.076),.36,.65)
for obj in robot.objects:
    if obj.type!='MESH':continue
    obj.data.materials.clear()
    obj.data.materials.append(joints if any(s in obj.name for s in ('ankle','hip','shoulder','knee','forearm')) else shell)
    obj.color=obj.data.materials[0].diffuse_color[:]
# Build a real edit-bone skeleton at the supplier's revolute servo-axis pivots.
# Each rigid authored STL visual mesh is weighted 100% to its corresponding
# link bone; Blender's Armature modifier therefore preserves the accurate shell.
urdf=ET.fromstring(xacro.process_file(str(ROOT/'assets/alpha1s/alpha1s.urdf.xacro')).toxml())
joints_by_child={j.find('child').get('link'):j for j in urdf.findall('joint')}
def xyz(text):return Vector(tuple(float(v) for v in (text or '0 0 0').split()))
def local_origin(e):
    if e is None:return Matrix.Identity(4)
    a,b,c=xyz(e.get('rpy'));return Matrix.Translation(xyz(e.get('xyz'))) @ Matrix.Rotation(c,4,'Z') @ Matrix.Rotation(b,4,'Y') @ Matrix.Rotation(a,4,'X')
arm=bpy.data.armatures.new('17 measured URDF servo axes | rigid link skin')
rig=bpy.data.objects.new('Robot armature | 144-frame scanning and step pose',arm);robot.objects.link(rig);rig.show_in_front=True
bpy.ops.object.select_all(action='DESELECT');rig.select_set(True);bpy.context.view_layer.objects.active=rig;bpy.ops.object.mode_set(mode='EDIT')
axes={}
for obj in [o for o in robot.objects if o.type=='MESH' and o.name.startswith('URDF |')]:
    name=obj['urdf_link'];link=next(l for l in urdf.findall('link') if l.get('name')==name)
    # Remove visual offset to recover the joint frame from the already assembled
    # (scaled and posed) link transform; use that pose as the animation rest pose.
    joint_world=obj.matrix_world @ local_origin(link.find('visual/origin')).inverted()
    head=joint_world.translation.copy()
    j=joints_by_child.get(name);axis=(joint_world.to_3x3() @ xyz(j.find('axis').get('xyz'))).normalized() if j is not None and j.find('axis') is not None else Vector((0,0,1))
    bone=arm.edit_bones.new(name);bone.head=head;bone.tail=head+axis*.17
    axes[name]=axis
for name,j in joints_by_child.items():
    parent=j.find('parent').get('link')
    if name in arm.edit_bones and parent in arm.edit_bones:
        arm.edit_bones[name].parent=arm.edit_bones[parent]
        arm.edit_bones[name].use_connect=False
bpy.ops.object.mode_set(mode='OBJECT')
for obj in list(robot.objects):
    if obj.type!='MESH' or not obj.name.startswith('URDF |'):continue
    vg=obj.vertex_groups.new(name=obj['urdf_link']);vg.add(list(range(len(obj.data.vertices))),1.0,'REPLACE')
    mod=obj.modifiers.new('Rigid URDF servo-bone deformation','ARMATURE');mod.object=rig;mod.use_deform_preserve_volume=False
    obj['rigid_bone']=obj['urdf_link']
    obj.parent=rig;obj.matrix_parent_inverse=rig.matrix_world.inverted()
# Each key is a relative delta from the audited initial supplier pose; all joint
# excursions stay well inside their URDF limits. Stand, scan, lift leg, recover.
# 1, 37, 73, 109, 144 span >120 frames and form a clear editable action.
keyframes=(1,37,73,109,144)
trajectories={
 'Head_link':(0,.20,-.19,.10,0),
 'l_shoulder_link':(0,.13,-.09,.18,0),'r_shoulder_link':(0,-.13,.08,-.15,0),
 'l_arm_link':(0,.14,-.18,.22,0),'r_arm_link':(0,-.15,.20,-.20,0),
 'l_forearm_link':(0,.18,-.12,.20,0),'r_forearm_link':(0,-.18,.12,-.18,0),
 'l_hip_roll_link':(0,.025,-.02,.03,0),'r_hip_roll_link':(0,-.025,.02,-.03,0),
 'l_knee_link':(0,.28,-.12,.32,0),'r_knee_link':(0,-.12,.28,-.16,0),
 'l_ankle_pitch_joint':(0,-.19,.10,-.22,0),'r_ankle_pitch_joint':(0,.12,-.18,.13,0),
}
for name,angles in trajectories.items():
    pb=rig.pose.bones[name];pb.rotation_mode='QUATERNION'
    rest=arm.bones[name].matrix_local.to_3x3()
    axis=rest.inverted() @ axes[name]
    for f,angle in zip(keyframes,angles):
        pb.rotation_quaternion=Quaternion(axis,angle);pb.keyframe_insert(data_path='rotation_quaternion',frame=f,group='URDF | '+name)
for bone in arm.bones:
    bone.use_deform=True
rig['urdf_source']='andresjjn/alpha1s-ros2-twin (MIT)'
rig['animation']='144 frames, independent URDF-axis rigid bone deformation; preserved rest mesh'
# Replace light bulbs with a CC0 authored firefly OBJ, with wing membranes,
# head, thorax, abdomen and legs. Preserve the *physics-baked* vertex point cloud.
path=ROOT/'assets/firefly_cc0/firefly.obj'
assert path.exists()
# Supplier MTL references broken paths; provide a minimal local MTL consisting
# only of the source's five original material names. This preserves usemtl face
# indices; afterwards all slots are replaced by explicit real Blender materials.
import tempfile
with tempfile.TemporaryDirectory() as tmp:
    clean=Path(tmp)/'firefly.obj'
    clean.write_text(''.join('mtllib firefly.mtl\n' if line.startswith('mtllib ') else line
                             for line in path.read_text().splitlines(True)))
    (Path(tmp)/'firefly.mtl').write_text(''.join('newmtl '+n+'\nKd 0.5 0.5 0.5\n' for n in ('none','yellow','tete','ailes','black')))
    before=set(bpy.data.objects);bpy.ops.wm.obj_import(filepath=str(clean))
new=[o for o in bpy.data.objects if o not in before and o.type=='MESH']
assert len(new)==1
insect=new[0]
for coll in list(insect.users_collection):coll.objects.unlink(insect)
sources.objects.link(insect);insect.name='Firefly | original CC0 insect mesh | source'
# MTL paths in source are malformed; do not use broken importer placeholders.
# The small sourced wing/head image maps remain available as real texture input.
materials=[]
for name,rgb,emission in [('none',(.055,.057,.047),0),('yellow',(.72,.44,.035),1.3),('tete',(.125,.145,.10),0),('ailes',(.22,.26,.19),0),('black',(.018,.024,.021),0)]:
    m=finish('Firefly authored mesh / '+name,rgb,.05,.73)
    bs=m.node_tree.nodes.get('Principled BSDF')
    if emission:
        bs.inputs['Emission Color'].default_value=(1,.67,.08,1);bs.inputs['Emission Strength'].default_value=emission
    if name in ('ailes','tete'):
        image=bpy.data.images.load(str(ROOT/'assets/firefly_cc0/textures'/(name+'.png')),check_existing=True);image.pack()
        tex=m.node_tree.nodes.new('ShaderNodeTexImage');tex.image=image
        m.node_tree.links.new(tex.outputs['Color'],bs.inputs['Base Color'])
    materials.append(m)
# Replace material slots IN PLACE. Clearing them resets every face's material
# index to 0 in Blender and destroys the authored wing/abdomen assignments.
assert len(insect.data.materials)==5
for index,m in enumerate(materials):insect.data.materials[index]=m
assert all(any(p.material_index==index for p in insect.data.polygons) for index in range(5))
# The OBJ bounding width is 2.04 authored units; static fireflies are 5 cm wide.
scale=.025
insect.data.transform(Matrix.Diagonal((scale,scale,scale,1)))
insect.data.transform(Matrix.Rotation(math.pi/2,4,'X')) # anatomical body lies horizontally
center=sum((v.co for v in insect.data.vertices),Vector())/len(insect.data.vertices)
for vert in insect.data.vertices:vert.co-=center
insect.location=(96,0,-30);insect.hide_render=True;insect.hide_set(True)
insect['asset_source']='LaurianeGelebart/The_Colorless_Journey, CC0 1.0'
baked=bpy.data.objects['Baked fireflies | frozen instances']
g=baked.modifiers[0].node_group
ref=next(n for n in g.nodes if n.bl_idname=='GeometryNodeObjectInfo')
ref.inputs['Object'].default_value=insect
baked['source_asset']='assets/firefly_cc0/firefly.obj | CC0'
baked['insect_anatomy']='authored wing, head, body and abdominal glow submeshes/material slots'
# The physics cache is retained for provenance, but animated presentation is
# deliberately driven by the skeleton; frozen fireflies never rely on cache.
scene.frame_end=144;scene.frame_set(75)
# Main scene in the user's screenshot made the 2m robot too small to read.
# Increase framing without inventing new forest assets. Additional edit-friendly
# diagnostic cameras make the authored insect anatomy and rig inspectable at
# <=720p, without pretending that their existence is a rendered preview.
scene.camera.data.ortho_scale=10.2
lights=bpy.data.collections['Lights']
def inspection_camera(name, location, target, width):
    bpy.ops.object.camera_add(location=location)
    camera=bpy.context.object
    for c in list(camera.users_collection):c.objects.unlink(camera)
    lights.objects.link(camera);camera.name=name
    camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.type='ORTHO';camera.data.ortho_scale=width
    camera['purpose']='Optional material/structure inspection only; not a preview render'
    return camera
inspection_camera('Camera | robot rig inspection', (3.8,-5.7,2.7), (0,0,1.05), 3.2)
fly_target=Vector(baked.data.vertices[len(baked.data.vertices)//2].co)
inspection_camera('Camera | firefly anatomical inspection',fly_target+Vector((.24,-.23,.12)),fly_target,.30)
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type=='VIEW_3D':area.spaces.active.shading.type='MATERIAL'
# Fatal for users: don't leave any live material with a missing image or blank slot.
assert all(o.data.materials and all(o.data.materials) for o in robot.objects if o.type=='MESH')
assert all(n.image and n.image.packed_file and all(n.image.size) for m in bpy.data.materials if m.users and m.use_nodes for n in m.node_tree.nodes if n.type=='TEX_IMAGE')
assert len(arm.bones)==18 and len(insect.data.vertices)>900 and len(baked.data.vertices)>=15
assert scene.render.engine=='BLENDER_EEVEE_NEXT' and scene.render.resolution_y<=720
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND),compress=True)
# The user's screenshot rejects the earlier scene. Keep the high-hardness JEV
# status synchronized with THIS upgraded file; passing assertions is data only.
report_path=ROOT/'output/JEV_reviews.json'
reviews=json.loads(report_path.read_text())
assert len(reviews)==5
summary={'objects':len(bpy.data.objects),'collections':{c.name:len(c.objects) for c in scene.collection.children_recursive},
         'geometry_nodes':[o.name for o in bpy.data.collections['Environment'].objects if any(m.type=='NODES' for m in o.modifiers)],
         'robot_parts':sum(o.type=='MESH' for o in robot.objects),'robot_armatures':1,'rig_bones':len(arm.bones),
         'animated_joint_channels':len(trajectories),'animation_frame_range':[scene.frame_start,scene.frame_end],
         'baked_fireflies':len(baked.data.vertices),'firefly_insect_source_vertices':len(insect.data.vertices),
         'render_engine':scene.render.engine,'render_size':[scene.render.resolution_x,scene.render.resolution_y]}
for index,entry in enumerate(reviews):
    if index:entry['get_scene_info']=summary
    entry['JEV']['Judgement']='FAIL_VISUAL' if index<4 else 'BLOCKED_PENDING_VISUAL'
    entry['JEV']['Evidence']['user_screenshot_rejected_original_scene']=True
    entry['JEV']['Evidence']['new_scene_eevee_preview_available']=False
    entry['JEV']['Verification']+=' 用户最新截图否定旧版材质/构图/萤火虫。新文件只通过数据审计，尚无其自身的低分辨率 EEVEE 预览，不可通过视觉审查。'
reviews[0]['JEV']['Evidence'].update({'preserved_pine_bark_twig_face_indices':True,'woodland_log_uses_correct_atlas':True})
reviews[1]['JEV']['Evidence'].update({'animated_armature_bones':len(arm.bones),'frame_end':scene.frame_end,'surface_pbr_map_count':3,'main_camera_ortho_scale_m':scene.camera.data.ortho_scale})
reviews[3]['JEV']['Evidence']['two_optional_anatomy_and_rig_inspection_cameras']=True
reviews[2]['JEV']['Evidence'].update({'original_insect_material_face_groups':len(set(p.material_index for p in insect.data.polygons)),
                                       'physics_baked_insect_instances':len(baked.data.vertices)})
report_path.write_text(json.dumps(reviews,ensure_ascii=False,indent=2)+'\n')
print('UPGRADE',json.dumps({'frame_range':[scene.frame_start,scene.frame_end], 'bones':len(arm.bones), 'animated_bones':len(trajectories), 'mesh_links':sum(o.type=='MESH' for o in robot.objects), 'firefly_vertices':len(insect.data.vertices),'baked_positions':len(baked.data.vertices), 'removed_broken_import_images':removed},ensure_ascii=False))
