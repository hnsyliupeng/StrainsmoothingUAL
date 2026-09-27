"""Stage 3 JEV continuation: follow a physically simulated, fully baked flock.

Run AFTER build_scene.py and upgrade_animation.py. build_scene.py runs Blender's
NEWTON particles, Brownian motion, directional wind, and turbulence at every
frame, then stores all evaluated positions as editable shape keys in the .blend.
This script reads that cache (it does not invent or re-integrate motion), makes
anatomical fireflies glow, animates real light sources, and keys the robot root
to the moving flock centroid. Leg articulation is solved later by refine_walk.py.
"""
import bpy
import math
import random
from pathlib import Path
from mathutils import Vector, Matrix, Quaternion
from terrain_profile import height

ROOT=Path(__file__).resolve().parent
BLEND=ROOT/'output/Twilight_Wilderness_Robot.blend'
bpy.ops.wm.open_mainfile(filepath=str(BLEND))
s=bpy.context.scene
s.frame_start=1;s.frame_end=144
flies=bpy.data.collections['Fireflies']
lights=bpy.data.collections['Lights']
sources=bpy.data.collections['Architecture / open-source asset prototypes']
robot=bpy.data.collections['Robot']
points=bpy.data.objects['Baked fireflies | 144-frame physical particle cache']
insect=bpy.data.objects['Firefly | original CC0 insect mesh | source']
rig=bpy.data.objects['Robot armature | 144-frame scanning and step pose']
assert len(points.data.vertices)>80 and len(insect.data.vertices)>900
# If re-running from an upgraded file, refuse to duplicate animated assets.
assert points.data.shape_keys and len(points.data.shape_keys.key_blocks) >= 140 and not rig.get('flight_follow_version'), 'Rebuild baseline before applying the flight follow twice'
for obj in robot.objects:
    if obj.type!='MESH':continue
    rest=obj.matrix_world.copy()
    obj.parent=rig;obj.matrix_parent_inverse=rig.matrix_world.inverted();obj.matrix_world=rest
# Orient original CC0 insect body from OBJ-Z to horizontal world-Y. The wings
# remain X-extended; the five source polygon groups and UVs are unchanged.
# upgrade_animation.py already rotates source body Z into horizontal Y.
assert (max(v.co.y for v in insect.data.vertices)-min(v.co.y for v in insect.data.vertices)) > (max(v.co.z for v in insect.data.vertices)-min(v.co.z for v in insect.data.vertices))
abdomen=insect.data.materials[1].node_tree.nodes.get('Principled BSDF')
# Saturated yellow-green bioluminescence stays chromatic through AgX instead of
# clipping into the tiny white pinpricks rejected in the last screenshot.
abdomen.inputs['Emission Color'].default_value=(.72,1.0,.075,1)
abdomen.inputs['Emission Strength'].default_value=4.8
abdomen.inputs['Base Color'].default_value=(.40,.58,.045,1)
# An enlarged, translucent aura around the true abdomen catches the compositor
# glow; the authentic ~5 cm insect anatomy remains separately visible.
halo_mat=bpy.data.materials.new('Firefly | soft yellow-green 12cm bioluminescent aura')
halo_mat.use_nodes=True
bs=halo_mat.node_tree.nodes.get('Principled BSDF')
bs.inputs['Base Color'].default_value=(.35,.76,.025,1)
bs.inputs['Emission Color'].default_value=(.65,1.0,.045,1)
bs.inputs['Emission Strength'].default_value=2.8
bs.inputs['Alpha'].default_value=.075
halo_mat.surface_render_method='BLENDED'
bpy.ops.mesh.primitive_uv_sphere_add(segments=16,ring_count=12)
halo=bpy.context.object
for c in list(halo.users_collection):c.objects.unlink(halo)
sources.objects.link(halo)
halo.name='Firefly | soft 12cm abdomen aura prototype'
halo.data.transform(Matrix.Translation(Vector((0,-.012,0))) @ Matrix.Diagonal((.062,.052,.045,1)))
halo.data.materials.append(halo_mat)
halo.location=(94,0,-30);halo.hide_render=True;halo.hide_set(True)
# Extend the ORIGINAL insect GN node tree with a second instance on the SAME
# moving points. Both wings and halo thus share the particle-baked path.
g=points.modifiers[0].node_group
instance=next(n for n in g.nodes if n.bl_idname=='GeometryNodeInstanceOnPoints')
out=next(n for n in g.nodes if n.bl_idname=='NodeGroupOutput')
halo_ref=g.nodes.new('GeometryNodeObjectInfo')
halo_ref.inputs['Object'].default_value=halo;halo_ref.inputs['As Instance'].default_value=True
halo_instance=g.nodes.new('GeometryNodeInstanceOnPoints')
halo_instance.label='Small abdominal halos follow physics-baked positions'
g.links.new(next(n for n in g.nodes if n.bl_idname=='NodeGroupInput').outputs['Geometry'],halo_instance.inputs['Points'])
g.links.new(halo_ref.outputs['Geometry'],halo_instance.inputs['Instance'])
join=g.nodes.new('GeometryNodeJoinGeometry')
g.links.new(instance.outputs['Instances'],join.inputs['Geometry'])
g.links.new(halo_instance.outputs['Instances'],join.inputs['Geometry'])
g.links.new(join.outputs['Geometry'],out.inputs['Geometry'])
# Read the actual per-frame positions sampled from Blender's NEWTON particle
# dependency graph; no second/custom integrator and no undeclared motion formula.
seed=random.Random(61239)
frames=list(range(s.frame_start,s.frame_end+1))
shape_keys=points.data.shape_keys.key_blocks
snapshots={}
centers={}
for f in frames:
    key=shape_keys.get('Blender NEWTON bake | frame %03d'%f)
    if key is None:raise RuntimeError(f'Missing physical particle bake at frame {f}')
    snapshots[f]=[key.data[i].co.copy() for i in range(len(points.data.vertices))]
    centers[f]=sum(snapshots[f],Vector())/len(snapshots[f])
pos=snapshots[1]
origin=[p.copy() for p in pos]
phase=[seed.uniform(0,2*math.pi) for _ in pos]
# Preserve the complete one-frame-per-key physical geometry bake in the .blend.
shape_action=points.data.shape_keys.animation_data.action
if shape_action:
    for curve in shape_action.fcurves:
        for key in curve.keyframe_points:key.interpolation='LINEAR'
points['bake_method']='Genuine Blender NEWTON + Brownian + directional wind + turbulence, evaluated and baked each frame 1–144 into geometry shape keys; GN instances the authored CC0 insect'
points['flight_baked_frames']=str(frames)
points['source_particle_physics']=points.get('physics','Blender NEWTON + Brownian + turbulence + wind')
# Real animated point lights illuminate nearby shells in addition to emissive
# insect abdomens and the soft compositor halo. All lights follow simulated flies.
light_count=18
for index in range(light_count):
    flyindex=round(index*(len(pos)-1)/max(1,light_count-1))
    data=bpy.data.lights.new('Bioluminescence | physical flight-following %02d'%index,'POINT')
    data.energy=6.5;data.color=(.62,1.0,.10);data.shadow_soft_size=.09;data.use_shadow=False
    light=bpy.data.objects.new(data.name,data);flies.objects.link(light)
    for f in frames:
        p=snapshots[f][flyindex]
        light.location=p+Vector((0,-.012,0));light.keyframe_insert(data_path='location',frame=f)
        data.energy=5.5+2.2*math.sin(f*.13+phase[flyindex])**2
        data.keyframe_insert(data_path='energy',frame=f)
    if light.animation_data and light.animation_data.action:
        for curve in light.animation_data.action.fcurves:
            for key in curve.keyframe_points:key.interpolation='LINEAR'
# The robot follows the moving NEWTON flock centroid with a 1.43 m chase lag.
# Its front points along local -Y, so its yaw follows the measured swarm velocity.
for f in frames:
    c=centers[f]
    next_frame=min(s.frame_end,f+1)
    prev_frame=max(s.frame_start,f-1)
    direction=(centers[next_frame]-centers[prev_frame]).xy
    if direction.length>1e-6:direction.normalize()
    else:direction=Vector((.22,-.53))
    behind=Vector((-.48,1.35,0))
    x,y=c.x+behind.x,c.y+behind.y
    rig.location=(x,y,height(x,y))
    rig.rotation_euler.z=math.atan2(direction.x,-direction.y)
    rig.keyframe_insert(data_path='location',frame=f,group='Robot follows measured NEWTON flock centroid')
    rig.keyframe_insert(data_path='rotation_euler',frame=f,group='Robot turns toward simulated insects')
# Counter-swing the authored arms; the legs are solved by the URDF-limited IK
# targets in refine_walk.py, never by independent knee-angle offsets.
pose_names=('l_arm_link','r_arm_link')
original_pose={}
for f in frames:
    s.frame_set(f)
    original_pose[f]={name:rig.pose.bones[name].rotation_quaternion.copy() for name in pose_names}
for f in frames:
    s.frame_set(f)
    phase_walk=2*math.pi*(f-1)/30
    for name in pose_names:
        pb=rig.pose.bones[name]
        side=1 if name.startswith('l_') else -1
        swing=-.18*math.sin(phase_walk)*side
        pb.rotation_quaternion=original_pose[f][name] @ Quaternion(Vector((0,1,0)),swing)
        pb.keyframe_insert(data_path='rotation_quaternion',frame=f,group='Counter-swing arms during IK walk')
rig['flight_follow_version']='baked Newton flock + root translation + URDF-axis foot IK walk'
rig['flight_chase_lag_m']=1.35
# Linear interpolation prevents root and gesture curves from easing/overshooting.
for fc in rig.animation_data.action.fcurves:
    for key in fc.keyframe_points:key.interpolation='LINEAR'
# Make lake water visibly glossy and break up its normal on a real surface.
lake=bpy.data.objects['Lake | radial waterline inside excavated basin']
water=lake.data.materials[0]
ns=water.node_tree.nodes;links=water.node_tree.links
noise=ns.new('ShaderNodeTexNoise');noise.inputs['Scale'].default_value=5.5
noise.inputs['Detail'].default_value=2
bump=ns.new('ShaderNodeBump');bump.inputs['Strength'].default_value=.12;bump.inputs['Distance'].default_value=.035
links.new(noise.outputs['Fac'],bump.inputs['Height'])
links.new(bump.outputs['Normal'],ns.get('Principled BSDF').inputs['Normal'])
# A local broad moon reflection makes the real lake basin legible at twilight.
light_data=bpy.data.lights.new('Lake | broad moon reflection','AREA');light_data.energy=240
light_data.color=(.42,.65,1);light_data.shape='DISK';light_data.size=5
lake_light=bpy.data.objects.new(light_data.name,light_data);lights.objects.link(lake_light)
lake_light.location=(3.6,3.8,6)
lake_light.rotation_euler=(Vector((3.7,3.8,-.38))-lake_light.location).to_track_quat('-Z','Y').to_euler()
# The old firefly inspection camera was aimed at a STATIC vertex near the
# *original* bake and misses the flying swarm completely. Follow an actual
# insect's animated, baked location with a small (still <=720p) anatomy shot.
inspection=bpy.data.objects['Camera | firefly anatomical inspection']
inspection.data.type='ORTHO'
inspection.data.ortho_scale=.23
# Pick a middle-height insect so the close-up shows recognizable wings,
# thorax and abdomen rather than a light-only profile.
hero=min(range(len(pos)),key=lambda i:abs(origin[i].z-1.65)+.20*abs(origin[i].x))
inspection['tracked_particle_index']=hero
inspection['purpose']='Animated close-up of one real CC0 anatomical insect; use 640x480 EEVEE'
for f in frames:
    focus=snapshots[f][hero]
    inspection.location=focus+Vector((.13,-.18,.105))
    inspection.rotation_euler=(focus-inspection.location).to_track_quat('-Z','Y').to_euler()
    inspection.keyframe_insert(data_path='location',frame=f,group='Follow the insect')
    inspection.keyframe_insert(data_path='rotation_euler',frame=f,group='Look at the insect')
# A second, slightly wider tracking shot makes both the robot's moving body
# and a real insect legible over time without changing their physical sizes.
# Scene camera remains the overview showing the lake and the height field.
# Compositor fog glow is what turns the mesh's bioluminescence into visible
# bloom in EEVEE; tiny emissive polys alone do not generate any halo on screen.
s.use_nodes=True
nodes=s.node_tree.nodes;nodes.clear();links=s.node_tree.links
rl=nodes.new('CompositorNodeRLayers');rl.location=(-350,0)
glare=nodes.new('CompositorNodeGlare');glare.glare_type='FOG_GLOW';glare.quality='HIGH'
glare.inputs['Threshold'].default_value=.80;glare.inputs['Strength'].default_value=1.25;glare.inputs['Size'].default_value=.92
glare.location=(-70,0)
s.render.use_compositing=True
out=nodes.new('CompositorNodeComposite');out.location=(250,0)
links.new(rl.outputs['Image'],glare.inputs['Image'])
links.new(glare.outputs['Image'],out.inputs['Image'])
# Use the physical foot stance to keep the low-level crossing clean. The
# average is exposed in audit_flight.py; this remains an editable authored walk.
# Keep viewport focused on the readable overview, not invisible source objects.
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type=='VIEW_3D':
            area.spaces.active.shading.type='MATERIAL'
            area.spaces.active.region_3d.view_perspective='CAMERA'
s.frame_set(75)
assert s.render.engine=='BLENDER_EEVEE_NEXT' and s.render.resolution_y<=720
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND),compress=True)
print('FLIGHT BAKED:',len(pos),'insects,',len(frames),'geometry states, full range',s.frame_start,s.frame_end,
      'centroids',tuple(round(v,2) for v in centers[1]),'->',tuple(round(v,2) for v in centers[144]))
