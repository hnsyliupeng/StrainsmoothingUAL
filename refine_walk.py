"""Refine evaluated URDF foot contact against the actual terrain at 4-frame intervals.

Run after animate_flight.py. This is a deterministic articulated geometry solver,
not an assertion that rendered gait looks natural. It never rescales the robot.
"""
import bpy
from pathlib import Path
from mathutils import Vector, Quaternion
from terrain_profile import height

ROOT = Path(__file__).resolve().parent
BLEND = ROOT / 'output/Twilight_Wilderness_Robot.blend'
bpy.ops.wm.open_mainfile(filepath=str(BLEND))
scene = bpy.context.scene
rig = bpy.data.objects['Robot armature | 144-frame scanning and step pose']
assert rig.get('flight_follow_version') and not rig.get('foot_contact_refinement')
axis = Vector((0, 1, 0))
frames = list(range(1, 145))
if frames[-1] != 144:frames.append(144)
foot_objects = {side:bpy.data.objects[f'URDF | {side}_foot_link'] for side in ('l','r')}
chains = {side:[rig.pose.bones[side+'_'+part] for part in ('knee_link','ankle_pitch_joint','ankle_link','foot_link')] for side in ('l','r')}

def clearance(side):
    ev=foot_objects[side].evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh=ev.to_mesh()
    try:
        return min((lambda p:p.z-height(p.x,p.y))(ev.matrix_world @ v.co) for v in mesh.vertices)
    finally:
        ev.to_mesh_clear()

# Snapshot original curves before writing ANY new joint keys, preventing a
# newly inserted key at an early frame from contaminating later samples.
rest={}
for f in frames:
    scene.frame_set(f)
    rest[f]={side:[bone.rotation_quaternion.copy() for bone in chains[side]] for side in chains}
# Coordinate descent on genuine evaluated mesh vertices, separately for each
# leg; prefer small measured clearance AND restrained servo corrections.
trial=(-.75,-.55,-.35,-.15,0,.15,.35,.55,.75)
for f in frames:
    scene.frame_set(f)
    for side in ('l','r'):
        bones=chains[side]
        base=rest[f][side]
        for bone,quaternion in zip(bones,base):bone.rotation_quaternion=quaternion
        bpy.context.view_layer.update()
        offsets=[0.]*4
        for _ in range(2):
            for i,bone in enumerate(bones):
                candidates=[]
                for delta in trial:
                    bone.rotation_quaternion=base[i] @ Quaternion(axis,delta)
                    bpy.context.view_layer.update()
                    gap=clearance(side)
                    # Gentle penalties discourage buried feet and excessive
                    # rotations, while targeting about 1.4 cm sole clearance.
                    score=(gap-.014)**2 + .0004*delta**2 + 3*max(0,-gap-.005)**2
                    candidates.append((score,abs(delta),delta))
                delta=min(candidates)[2]
                offsets[i]=delta
                bone.rotation_quaternion=base[i] @ Quaternion(axis,delta)
                bpy.context.view_layer.update()
        for bone in bones:
            bone.keyframe_insert(data_path='rotation_quaternion',frame=f,group='Terrain foot-contact refinement')
    # Preserve the baked XY chase; Z only clears a remaining local collision.
    min_gap=min(clearance(side) for side in chains)
    rig.location.z+=max(0.,.008-min_gap)
    rig.keyframe_insert(data_path='location',frame=f,group='Terrain sole clearance')
rig['foot_contact_refinement']='evaluated foot vertex solver; every frame, original supplier mesh and URDF skeleton retained'
rig['foot_solver_frames']=str(frames)
scene.frame_set(75)
assert scene.render.engine=='BLENDER_EEVEE_NEXT' and scene.render.resolution_y<=720
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND),compress=True)
print('FOOT SOLVER keyed',len(frames),'frames for both articulated legs')
