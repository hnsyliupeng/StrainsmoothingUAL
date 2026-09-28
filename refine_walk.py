"""Build a real URDF-limited, foot-target IK walk for the moving robot.

Run after animate_flight.py. Each editable ankle/sole IK target and knee pole is
keyed every frame; URDF joint axes/limits constrain the four-DOF leg chain while
the foot link inherits ankle pose. Sole contact is measured on evaluated STL
geometry against terrain_profile.height.
This is a structural/kinematic audit, not a substitute for viewing the gait.
"""
import bpy
import json
import math
from pathlib import Path
from mathutils import Euler, Vector
from terrain_profile import height

ROOT = Path(__file__).resolve().parent
BLEND = ROOT / 'output/Twilight_Wilderness_Robot.blend'
bpy.ops.wm.open_mainfile(filepath=str(BLEND))
scene = bpy.context.scene
rig = bpy.data.objects['Robot armature | 144-frame scanning and step pose']
assert rig.get('flight_follow_version'), 'Run animate_flight.py before building IK controls'
assert not rig.get('foot_ik_version'), 'Rebuild baseline before applying IK rig twice'
assert scene.frame_start == 1 and scene.frame_end >= 120

robot = bpy.data.collections['Robot']
controls = bpy.data.collections.new('Robot Controls | URDF foot IK targets and poles')
scene.collection.children.link(controls)
foot_objects = {side: bpy.data.objects[f'URDF | {side}_foot_link'] for side in ('l', 'r')}
leg_bones = {
    side: (side + '_hip_roll_link', side + '_knee_link', side + '_ankle_pitch_joint',
           side + '_ankle_link', side + '_foot_link')
    for side in ('l', 'r')
}

# Remove only leg pose curves from the provisional stand/scan action. Arm swing,
# head scans, and the root's flight-follow path remain untouched.
leg_names = {name for chain in leg_bones.values() for name in chain}
action = rig.animation_data.action if rig.animation_data else None
if action:
    for curve in list(action.fcurves):
        if any(f'pose.bones["{name}"]' in curve.data_path for name in leg_names):
            action.fcurves.remove(curve)
for name in leg_names:
    rig.pose.bones[name].rotation_mode = 'QUATERNION'
    rig.pose.bones[name].rotation_quaternion = (1, 0, 0, 0)
bpy.context.view_layer.update()

# Capture the already-authored pelvis path before installing the IK constraints.
root_pose = {}
for frame in range(scene.frame_start, scene.frame_end + 1):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    root_pose[frame] = (rig.matrix_world.translation.copy(), float(rig.rotation_euler.z))


def angle_delta(a, b):
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def root_at(frame):
    """Linearly sample/extrapolate the continuous world-space chase transform."""
    lo = math.floor(frame)
    t = frame - lo
    if lo < scene.frame_start:
        lo = scene.frame_start
        p0, a0 = root_pose[lo]
        p1, a1 = root_pose[lo + 1]
        dp = p1 - p0
        da = angle_delta(a0, a1)
        p = p0 + dp * (frame - lo)
        a = a0 + da * (frame - lo)
        return p, a
    if lo >= scene.frame_end:
        hi = scene.frame_end
        p0, a0 = root_pose[hi - 1]
        p1, a1 = root_pose[hi]
        dp = p1 - p0
        da = angle_delta(a0, a1)
        p = p1 + dp * (frame - hi)
        a = a1 + da * (frame - hi)
        return p, a
    p0, a0 = root_pose[lo]
    p1, a1 = root_pose[lo + 1]
    return p0.lerp(p1, t), a0 + angle_delta(a0, a1) * t


def local_to_world(point, frame):
    translation, yaw = root_at(frame)
    return translation + Euler((0, 0, yaw), 'XYZ').to_matrix() @ point


def make_control(name, display_type, size, color):
    obj = bpy.data.objects.new(name, None)
    controls.objects.link(obj)
    obj.empty_display_type = display_type
    obj.empty_display_size = size
    obj.color = color
    obj.hide_render = True
    obj['purpose'] = 'Editable non-rendering Blender IK control'
    return obj

# Local foot-toe endpoints, sole height, and a forward knee-pole location are
# read from the authored URDF rig, not guessed from a box proxy.
scene.frame_set(75)
bpy.context.view_layer.update()
base_toe = {side: rig.pose.bones[side + '_foot_link'].tail.copy() for side in ('l', 'r')}
ankle_tail = {side: rig.pose.bones[side + '_ankle_link'].tail.copy() for side in ('l', 'r')}
hip_local = {}
knee_local = {}
for side in ('l', 'r'):
    hip_local[side] = rig.data.bones[side + '_hip_roll_link'].head_local.copy()
    knee_local[side] = rig.data.bones[side + '_knee_link'].head_local.copy()

def mesh_clearance(side):
    foot = foot_objects[side].evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = foot.to_mesh()
    try:
        return min((lambda p: p.z - height(p.x, p.y))(foot.matrix_world @ v.co) for v in mesh.vertices)
    finally:
        foot.to_mesh_clear()

toe_above_sole = {}
for side in ('l', 'r'):
    foot_bone = rig.pose.bones[side + '_foot_link']
    toe_world = rig.matrix_world @ foot_bone.tail
    foot = foot_objects[side].evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = foot.to_mesh()
    try:
        lowest_z = min((foot.matrix_world @ v.co).z for v in mesh.vertices)
    finally:
        foot.to_mesh_clear()
    toe_above_sole[side] = max(0.015, toe_world.z - lowest_z)

# The robot faces local -Y. A 30-frame cycle at 24 fps matches its measured
# ~0.56 m/s chase speed; the 62% stance gives brief double support and a 10 cm
# toe-clearance arc on swing.
period = 24.0
stance_fraction = 0.62
contact_clearance_target = 0.003
contact_clearance_tolerance = 0.0005
max_contact_iterations = 24
stride_half_length = 0.18
swing_height = 0.10
phase_offset = {'l': 0.0, 'r': 0.5}
IKs = {}
targets = {}
poles = {}

for side in ('l', 'r'):
    target = make_control('IK Target | ' + side + ' sole contact / ankle pivot', 'SPHERE', 0.055, (1.0, 0.34, 0.04, 1))
    pole = make_control('IK Pole | ' + side + ' knee bend', 'CIRCLE', 0.075, (0.08, 0.55, 1.0, 1))
    targets[side] = target
    poles[side] = pole
    constraint = rig.pose.bones[side + '_ankle_link'].constraints.new('IK')
    constraint.name = 'URDF leg IK | ' + side + ' | measured ankle/sole plant'
    constraint.target = target
    constraint.pole_target = pole
    constraint.chain_count = 4
    constraint.iterations = 512
    constraint.use_stretch = False
    # Solve the physically reachable ankle position only; rotational IK over-constrains
    # the URDF-limited chain and previously sent the endpoint metres off target.
    constraint.use_rotation = False
    IKs[side] = constraint
    for name in leg_bones[side]:
        rig.pose.bones[name]['IK_chain_side'] = side


def touchdown_local(side):
    return base_toe[side] + Vector((0.0, -stride_half_length, 0.0))


def target_position(side, frame):
    phase = (frame - scene.frame_start) / period + phase_offset[side]
    cycle_index = math.floor(phase)
    u = phase - cycle_index
    touchdown = scene.frame_start + (cycle_index - phase_offset[side]) * period
    next_touchdown = touchdown + period
    local = touchdown_local(side)
    previous = local_to_world(local, touchdown)
    following = local_to_world(local, next_touchdown)
    if u < stance_fraction:
        x, y = previous.x, previous.y
        z = height(x, y) + toe_above_sole[side] + contact_clearance_target
        planted = True
    else:
        swing = (u - stance_fraction) / (1.0 - stance_fraction)
        smooth = swing * swing * (3.0 - 2.0 * swing)
        point = previous.lerp(following, smooth)
        x, y = point.x, point.y
        lift = swing_height * 4.0 * swing * (1.0 - swing)
        z = height(x, y) + toe_above_sole[side] + lift + contact_clearance_target
        planted = False
    return Vector((x, y, z)), planted


def pole_position(side, frame):
    local = (hip_local[side] + knee_local[side]) * 0.5 + Vector((0.0, -0.28, 0.0))
    return local_to_world(local, frame)

# Solve sole clearance by bracketing the evaluated mesh response, rather than
# assuming one-unit target motion produces one-unit sole motion. This matters
# near URDF joint limits, where a simple proportional correction can stall or
# alternate between penetration and hover.
def solve_stance_height(target, side, frame, desired_gap):
    base_z = float(target.location.z)
    best = None
    evaluations = 0

    def sample(offset):
        nonlocal evaluations, best
        target.location.z = base_z + offset
        target.keyframe_insert(data_path='location', frame=frame,
                               group='World-space planted-foot IK')
        bpy.context.view_layer.update()
        gap = mesh_clearance(side)
        residual = gap - desired_gap
        evaluations += 1
        if best is None or abs(residual) < abs(best[1]):
            best = (offset, residual, gap)
        return residual

    residual0 = sample(0.0)
    if abs(residual0) <= contact_clearance_tolerance:
        return best[2], evaluations, best[0], True

    # The evaluated sole height should increase monotonically with target Z.
    # Expand only in the direction needed to bracket zero and cap at 32 cm.
    direction = -1.0 if residual0 > 0.0 else 1.0
    step = max(0.005, min(0.02, abs(residual0) * 1.25))
    offset0, residual_at_zero = 0.0, residual0
    bracket = None
    for _ in range(12):
        offset = direction * step
        residual = sample(offset)
        if residual_at_zero <= 0.0 <= residual:
            bracket = (offset0, offset, residual_at_zero, residual)
            break
        if residual <= 0.0 <= residual_at_zero:
            bracket = (offset, offset0, residual, residual_at_zero)
            break
        offset0, residual_at_zero = offset, residual
        step *= 1.8
        if step > 0.32:
            break

    if bracket is not None:
        lo, hi, flo, fhi = bracket
        for _ in range(max_contact_iterations):
            mid = 0.5 * (lo + hi)
            fm = sample(mid)
            if abs(fm) <= contact_clearance_tolerance:
                break
            if fm < 0.0:
                lo, flo = mid, fm
            else:
                hi, fhi = mid, fm
        converged = abs(best[1]) <= contact_clearance_tolerance
    else:
        converged = False

    # Restore the best evaluated solution so the saved F-curve and the live
    # dependency graph agree with the diagnostic residual.
    sample(best[0])
    return best[2], evaluations, best[0], converged


# The pelvis trajectory remains the measured physical flock-follow path; no
# individual joint is hand-spun. Only world-space IK controls are adjusted.
contact_errors = []
contact_samples = []
contact_solver_iterations = []
contact_solver_failures = []
target_errors = []
target_error_samples = []
for frame in range(scene.frame_start, scene.frame_end + 1):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    for side in ('l', 'r'):
        target = targets[side]
        desired_toe, planted = target_position(side, frame)
        yaw = root_at(frame)[1]
        ankle_toe_offset = Euler((0, 0, yaw), 'XYZ').to_matrix() @ (base_toe[side] - ankle_tail[side])
        target.location = desired_toe - ankle_toe_offset
        poles[side].location = pole_position(side, frame)
        # Insert the current keys BEFORE measuring: otherwise Blender evaluates
        # the prior-frame F-curve and the solver sees a stale target transform.
        target.keyframe_insert(data_path='location', frame=frame, group='World-space planted-foot IK')
        poles[side].keyframe_insert(data_path='location', frame=frame, group='Moving knee pole target')
        bpy.context.view_layer.update()
        if planted:
            gap, iterations, correction, converged = solve_stance_height(
                target, side, frame, contact_clearance_target)
            contact_solver_iterations.append(iterations)
            sample = {'frame': frame, 'side': side, 'clearance_m': gap,
                      'target_z_correction_m': correction, 'evaluations': iterations,
                      'converged': converged}
            contact_samples.append(sample)
            if not converged:
                contact_solver_failures.append(sample)
        else:
            # Enforce a 3.5 cm swing-toe safety margin against the evaluated
            # terrain while leaving the authored parabolic step arc intact.
            for _ in range(12):
                gap = mesh_clearance(side)
                if gap >= 0.035:
                    break
                target.location.z += max(0.002, 0.035 - gap)
                target.keyframe_insert(data_path='location', frame=frame, group='World-space planted-foot IK')
                bpy.context.view_layer.update()
        gap = mesh_clearance(side)
        if planted:
            contact_errors.append(gap)
        end = rig.matrix_world @ rig.pose.bones[side + '_ankle_link'].tail
        endpoint_error = (end - target.location).length
        target_errors.append(endpoint_error)
        target_error_samples.append({'frame': frame, 'side': side, 'error_m': endpoint_error,
                                     'target_xyz_m': [round(v, 6) for v in target.location],
                                     'ankle_tail_xyz_m': [round(v, 6) for v in end]})

for obj in list(targets.values()) + list(poles.values()):
    if obj.animation_data and obj.animation_data.action:
        for curve in obj.animation_data.action.fcurves:
            for key in curve.keyframe_points:
                key.interpolation = 'LINEAR'

rig['foot_ik_version'] = 'URDF joint-limited 4-link ankle IK, moving pole targets, measured terrain-planted sole meshes'
rig['foot_ik_period_frames'] = period
rig['foot_ik_stance_fraction'] = stance_fraction
rig['foot_ik_stride_length_m'] = 2.0 * stride_half_length
rig['foot_ik_solver_frames'] = scene.frame_end - scene.frame_start + 1
rig['foot_contact_clearance_range_m'] = [min(contact_errors), max(contact_errors)]
rig['foot_ik_max_endpoint_error_m'] = max(target_errors)

# Preserve the scene camera/frame, then independently verify continuous sampled
# mesh contact and target reach. Audit results are not a visual JEV approval.
scene.frame_set(75)
bpy.context.view_layer.update()
clearances = []
clearance_samples = []
for frame in range(scene.frame_start, scene.frame_end + 1):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    for side in ('l', 'r'):
        gap = mesh_clearance(side)
        clearances.append(gap)
        clearance_samples.append({'frame': frame, 'side': side, 'clearance_m': gap})
rig['foot_all_frame_clearance_range_m'] = [min(clearances), max(clearances)]
summary = {
    'IK_constraints': {side: IKs[side].name for side in ('l', 'r')},
    'IK_position_only_targets': {side: not IKs[side].use_rotation for side in ('l', 'r')},
    'IK_chain_lengths': {side: IKs[side].chain_count for side in ('l', 'r')},
    'leg_joint_axis_locks': {
        side: {name: {'locks': [rig.pose.bones[name].lock_ik_x,
                                rig.pose.bones[name].lock_ik_y,
                                rig.pose.bones[name].lock_ik_z],
                      'limits': [rig.pose.bones[name].ik_min_x,
                                 rig.pose.bones[name].ik_min_y,
                                 rig.pose.bones[name].ik_min_z,
                                 rig.pose.bones[name].ik_max_x,
                                 rig.pose.bones[name].ik_max_y,
                                 rig.pose.bones[name].ik_max_z]}
               for name in leg_bones[side]}
        for side in ('l', 'r')
    },
    'foot_clearance_range_m': [min(clearances), max(clearances)],
    'planted_foot_clearance_range_m': [min(contact_errors), max(contact_errors)],
    'max_ik_target_error_m': max(target_errors),
    'contact_clearance_target_m': contact_clearance_target,
    'contact_clearance_tolerance_m': contact_clearance_tolerance,
    'lowest_sole_sample': min(clearance_samples, key=lambda item: item['clearance_m']),
    'highest_sole_sample': max(clearance_samples, key=lambda item: item['clearance_m']),
    'lowest_planted_sample': min(contact_samples, key=lambda item: item['clearance_m']),
    'highest_planted_sample': max(contact_samples, key=lambda item: item['clearance_m']),
    'largest_ik_endpoint_error': max(target_error_samples, key=lambda item: item['error_m']),
    'stance_contact_solver': {'method': 'bracketed bisection on evaluated sole-mesh clearance',
                              'tolerance_m': contact_clearance_tolerance,
                              'max_evaluations': max(contact_solver_iterations, default=0),
                              'failed_samples': contact_solver_failures[:12],
                              'failure_count': len(contact_solver_failures)},
    'frames': scene.frame_end,
    'visual_review_completed': False,
}
(ROOT / 'output' / 'robot_ik_audit.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
print('URDF-LIMITED IK WALK DIAGNOSTIC', json.dumps(summary, ensure_ascii=False), flush=True)
assert len(IKs) == 2 and all(c.chain_count == 4 and c.pole_target for c in IKs.values())
assert min(clearances) >= -0.003 and max(clearances) < 0.15, summary['foot_clearance_range_m']
assert not contact_solver_failures, summary['stance_contact_solver']
assert min(contact_errors) >= -0.001 and max(contact_errors) <= 0.006, summary['planted_foot_clearance_range_m']
assert max(target_errors) < 0.001, summary['largest_ik_endpoint_error']
assert scene.render.engine == 'BLENDER_EEVEE_NEXT' and scene.render.resolution_y <= 720
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND), compress=True)
# Synchronize the five staged JEVs with the completed rig and immutable physics
# bake. Their current verdict remains pending until an EEVEE image is inspected.
report_path = ROOT / 'output' / 'JEV_reviews.json'
reviews = json.loads(report_path.read_text())
assert len(reviews) == 5
scene_info = {
    'objects': len(bpy.data.objects),
    'collections': {c.name: len(c.objects) for c in scene.collection.children_recursive},
    'geometry_nodes': [o.name for o in bpy.data.collections['Environment'].objects if any(m.type == 'NODES' for m in o.modifiers)],
    'robot_parts': sum(o.type == 'MESH' for o in robot.objects),
    'armature_bones': len(rig.data.bones),
    'leg_ik_constraints': sum(c.type == 'IK' for pb in rig.pose.bones for c in pb.constraints),
    'firefly_instances': len(bpy.data.objects['Baked fireflies | 144-frame physical particle cache'].data.vertices),
    'firefly_physics_frames': 144,
    'render_engine': scene.render.engine,
    'render_size': [scene.render.resolution_x, scene.render.resolution_y],
}
for entry in reviews:
    entry['get_scene_info'] = scene_info
    entry['JEV']['Judgement'] = 'PENDING_VISUAL'
    entry['JEV']['Evidence']['current_candidate_visual_review_completed'] = False
    entry['JEV']['Verification'] += ' 新候选的真实rig、物理逐帧烘焙与EEVEE设置已结构审计；没有成功的实际渲染图，因此绝不把数据断言当作视觉通过。'
    entry['Next_step_plan'] = '审查新生成的640×480 EEVEE图像；若湖面、机器人步态或萤火虫光仍不清晰则继续修改并重审。'
reviews[1]['JEV']['Evidence'].update({
    'real_armature_constraints': len(IKs) == 2,
    'urdf_limited_ik_chain_length': 4,
    'pole_targets_baked_every_frame': True,
    'two_sole_meshes_measured_each_frame': len(clearances) == 288,
    'ik_position_targets_only': all(not c.use_rotation for c in IKs.values()),
    'all_frame_foot_clearance_range_m': [min(clearances), max(clearances)],
    'stance_only_foot_clearance_range_m': [min(contact_errors), max(contact_errors)],
    'max_ik_endpoint_error_m': max(target_errors),
})
reviews[2]['JEV']['Evidence'].update({
    'blender_newton_frames_baked': 144,
    'physics_baked_firefly_instances': scene_info['firefly_instances'],
    'custom_flight_integrator_removed': True,
    'real_newton_wind_turbulence_brownian': True,
})
reviews[3]['JEV']['Evidence'].update({
    'saturated_abdomen_emission_strength': bpy.data.objects['Firefly | original CC0 insect mesh | source'].data.materials[1].node_tree.nodes.get('Principled BSDF').inputs['Emission Strength'].default_value,
    'animated_flight_following_point_lights': len([o for o in bpy.data.collections['Fireflies'].objects if o.type == 'LIGHT']),
    'high_quality_eevee_fog_glow_enabled': scene.use_nodes and scene.render.use_compositing,
    'preview_image_files': [],
})
reviews[4]['JEV']['Evidence'].update({
    'editable_blend_saved': BLEND.exists(),
    'preview_generated_and_visually_approved': False,
    'visual_review_status': 'PENDING_EEVEE_IMAGE_REVIEW',
})
report_path.write_text(json.dumps(reviews, ensure_ascii=False, indent=2) + '\n')
print('URDF-LIMITED IK WALK', json.dumps(summary, ensure_ascii=False))
