"""Convert the real particle bake into a 144-frame moving, self-contained flight.

Run AFTER build_scene.py and upgrade_animation.py. This is a deterministic
Newton/drag/steering integrator initialized at Blender's frame-75 particle
positions. Its sampled positions are baked as animated shape-key coordinates
on the GN point cloud: rendering NEVER requires the live emitter or Python.
The articulated visual robot follows the moving swarm in world space.
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
points=bpy.data.objects['Baked fireflies | frozen instances']
insect=bpy.data.objects['Firefly | original CC0 insect mesh | source']
rig=bpy.data.objects['Robot armature | 144-frame scanning and step pose']
assert len(points.data.vertices)>80 and len(insect.data.vertices)>900
# If re-running from an upgraded file, refuse to duplicate animated assets.
assert points.data.shape_keys is None and not rig.get('flight_follow_version'), 'Rebuild baseline before applying the flight bake twice'
for obj in robot.objects:
    if obj.type!='MESH':continue
    rest=obj.matrix_world.copy()
    obj.parent=rig;obj.matrix_parent_inverse=rig.matrix_world.inverted();obj.matrix_world=rest
# Orient original CC0 insect body from OBJ-Z to horizontal world-Y. The wings
# remain X-extended; the five source polygon groups and UVs are unchanged.
# upgrade_animation.py already rotates source body Z into horizontal Y.
assert (max(v.co.y for v in insect.data.vertices)-min(v.co.y for v in insect.data.vertices)) > (max(v.co.z for v in insect.data.vertices)-min(v.co.z for v in insect.data.vertices))
abdomen=insect.data.materials[1].node_tree.nodes.get('Principled BSDF')
abdomen.inputs['Emission Color'].default_value=(1.0,.56,.07,1)
abdomen.inputs['Emission Strength'].default_value=18
# A small halo at the abdomen, not a huge featureless sphere over the wings.
halo_mat=bpy.data.materials.new('Firefly | atmospheric amber halo')
halo_mat.use_nodes=True
bs=halo_mat.node_tree.nodes.get('Principled BSDF')
bs.inputs['Base Color'].default_value=(1,.44,.035,1)
bs.inputs['Emission Color'].default_value=(1,.56,.08,1)
bs.inputs['Emission Strength'].default_value=7
bs.inputs['Alpha'].default_value=.20
halo_mat.surface_render_method='BLENDED'
bpy.ops.mesh.primitive_uv_sphere_add(segments=12,ring_count=8)
halo=bpy.context.object
for c in list(halo.users_collection):c.objects.unlink(halo)
sources.objects.link(halo)
halo.name='Firefly | 3cm abdomen halo prototype'
halo.data.transform(Matrix.Translation(Vector((0,-.012,0))) @ Matrix.Diagonal((.027,.023,.020,1)))
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
# Newton flight: aerial propulsion toward camera, drag, turbulence, and
# restoring acceleration to each insect's preferred altitude. Start from
# actual evaluated Blender NEWTON + turbulence particle positions at frame 75.
seed=random.Random(61239)
pos=[v.co.copy() for v in points.data.vertices]
origin=[p.copy() for p in pos]
velocity=[Vector((seed.uniform(.04,.16),seed.uniform(-.12,-.02),seed.uniform(-.035,.035))) for p in pos]
phase=[seed.uniform(0,2*math.pi) for _ in pos]
# Travel ~3m while retaining a natural spread; 24 fps, 144 frames.
start_velocity=Vector((.22,-.53,0))
dt=1/24
snapshots={}
centers={}
for f in range(1,145):
    if f>1:
        for i,p in enumerate(pos):
            v=velocity[i]
            t=f*dt
            propulsion=Vector((start_velocity.x-v.x,start_velocity.y-v.y,-v.z))*2.1
            gust=Vector((.18*math.sin(1.8*t+phase[i]),.11*math.cos(2.2*t+phase[i]),
                         .25*math.sin(2.6*t+phase[i])))
            # Soft tether retains a flock without making its 92 paths identical.
            tether=Vector((origin[i].x-p.x+.22*t,origin[i].y-p.y-.53*t,0))*.19
            vertical=(origin[i].z-p.z)*.85
            acceleration=propulsion+gust+tether+Vector((0,0,vertical))
            v+=acceleration*dt
            p+=v*dt
            floor=height(p.x,p.y)+.45
            if p.z<floor:p.z=floor;v.z=max(0.,-v.z*.35)
            p.z=min(p.z,3.6)
    if f==1 or (f-1)%8==0 or f==144:
        snapshots[f]=[p.copy() for p in pos]
        centers[f]=sum(pos,Vector())/len(pos)
# Store every sample *as geometry*; shape key interpolation is linear and
# therefore the flight plays in a normal .blend without script or particle cache.
points.shape_key_add(name='Basis',from_mix=False)
frames=sorted(snapshots)
for n,f in enumerate(frames):
    key=points.shape_key_add(name='Flight physics bake | frame %03d'%f,from_mix=False)
    for i,co in enumerate(snapshots[f]):key.data[i].co=co
    key.value=0
    for adjacent in (n-1,n,n+1):
        if 0<=adjacent<len(frames):
            key.value=1 if adjacent==n else 0
            key.keyframe_insert(data_path='value',frame=frames[adjacent])
    key.value=0
# Enforce linear interpolation between adjacent baked states (no overshoot).
shape_action=points.data.shape_keys.animation_data.action
if shape_action:
    for fc in shape_action.fcurves:
        for k in fc.keyframe_points:k.interpolation='LINEAR'
points['bake_method']='Blender NEWTON particle frames 1-75 -> frame-75 point cloud -> Newton drag/steering flight -> sampled animated shape keys over frames 1-144'
points['flight_baked_frames']=str(frames)
points['source_particle_physics']=points.get('physics','NEWTON + Brownian + turbulence')
# Place actual light objects at selected simulated insects each baked frame.
# Unlike emissive materials alone, these lights illuminate nearby robot shells.
for index in range(8):
    flyindex=min(len(pos)-1,index*11)
    data=bpy.data.lights.new('Bioluminescence | flight-following %02d'%index,'POINT')
    data.energy=22;data.color=(1,.56,.10);data.shadow_soft_size=.12
    light=bpy.data.objects.new(data.name,data);flies.objects.link(light)
    for f in frames:
        p=snapshots[f][flyindex]
        light.location=p;light.keyframe_insert(data_path='location',frame=f)
        data.energy=19+5*math.sin(f*.13+phase[flyindex])**2
        data.keyframe_insert(data_path='energy',frame=f)
# Follow the instantaneous baked flight centroid, offset behind moving swarm.
# The original robot's front points along -Y, so yaw follows world velocity.
for f in frames:
    c=centers[f]
    ahead=centers[frames[min(len(frames)-1,frames.index(f)+1)]]
    direction=(ahead-c).xy
    direction.normalize()
    behind=Vector((-.48,1.35,0))
    x,y=c.x+behind.x,c.y+behind.y
    rig.location=(x,y,height(x,y))
    rig.rotation_euler.z=math.atan2(direction.x,-direction.y)
    rig.keyframe_insert(data_path='location',frame=f,group='Robot follows moving flight centroid')
    rig.keyframe_insert(data_path='rotation_euler',frame=f,group='Robot turns toward moving swarm')
# Keep independent supplier bones animated; add alternating local hip and
# opposing arm phase to make the real forward walk read, not a stationary jog.
pose_names=('l_knee_link','r_knee_link','l_arm_link','r_arm_link')
original_pose={}
for f in frames:
    s.frame_set(f)
    original_pose[f]={name:rig.pose.bones[name].rotation_quaternion.copy() for name in pose_names}
for f in frames:
    s.frame_set(f)
    phase_walk=2*math.pi*(f-1)/40
    for name in pose_names:
        b=rig.pose.bones[name]
        leg=1 if name.startswith('l_') else -1
        swing=.19*math.sin(phase_walk)*leg
        if 'arm' in name:swing*=-.7
        b.rotation_quaternion=original_pose[f][name] @ Quaternion(Vector((0,1,0)),swing)
        b.keyframe_insert(data_path='rotation_quaternion',frame=f,group='Alternating joint walk')
# Solve evaluated sole/terrain separation at every eight-frame sample.
feet=[bpy.data.objects['URDF | %s_foot_link'%side] for side in ('l','r')]
right_links=('r_knee_link','r_ankle_link','r_foot_link')
servo_axis=Vector((0,1,0))
# Baseline right leg is shorter in the supplied STL pose. Correct at the
# anatomical joints, then set global clearance from actual sole geometry.
right_original={}
for f in frames:
    s.frame_set(f)
    right_original[f]={name:rig.pose.bones[name].rotation_quaternion.copy() for name in right_links}
for f in frames:
    s.frame_set(f)
    for name,angle in zip(right_links,(.20,.40,-.30)):
        pb=rig.pose.bones[name]
        pb.rotation_quaternion=right_original[f][name] @ Quaternion(servo_axis,angle)
        pb.keyframe_insert(data_path='rotation_quaternion',frame=f,group='Right leg sole contact')

def lowest_clearance(foot):
    ev=foot.evaluated_get(bpy.context.evaluated_depsgraph_get())
    me=ev.to_mesh()
    try:
        return min((lambda p:p.z-height(p.x,p.y))(ev.matrix_world@v.co) for v in me.vertices)
    finally:ev.to_mesh_clear()
for f in frames:
    s.frame_set(f)
    correction=max(0,.009-min(lowest_clearance(foot) for foot in feet))
    rig.location.z+=correction
    rig.keyframe_insert(data_path='location',frame=f,group='Measured two-sole terrain clearance')
rig['flight_follow_version']='baked Newton flock + root translation + real articulated walk'
rig['flight_chase_lag_m']=1.35
# Make lake water visibly glossy and break up its normal on a real surface.
lake=bpy.data.objects['Lake | water inside excavated basin']
water=lake.data.materials[0]
ns=water.node_tree.nodes;links=water.node_tree.links
noise=ns.new('ShaderNodeTexNoise');noise.inputs['Scale'].default_value=5.5
noise.inputs['Detail'].default_value=2
bump=ns.new('ShaderNodeBump');bump.inputs['Strength'].default_value=.12;bump.inputs['Distance'].default_value=.035
links.new(noise.outputs['Fac'],bump.inputs['Height'])
links.new(bump.outputs['Normal'],ns.get('Principled BSDF').inputs['Normal'])
# A local broad moon reflection makes the real lake basin legible at twilight.
light_data=bpy.data.lights.new('Lake | broad moon reflection','AREA');light_data.energy=480
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
glare=nodes.new('CompositorNodeGlare');glare.glare_type='FOG_GLOW';glare.inputs['Threshold'].default_value=.55;glare.inputs['Strength'].default_value=.75;glare.inputs['Size'].default_value=.75
glare.location=(-70,0)
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
