"""Build a real URDF-limited IK walk plus an ankle-roll servo for the robot.

Run after animate_flight.py. Each editable ankle-pivot target and knee pole is
keyed every frame; four proximal URDF axes are constrained by IK and the fifth
ankle-roll servo is independently keyed against its exact URDF angular limit.
Sole contact is measured on evaluated STL geometry against terrain_profile.height.
This is a structural/kinematic audit, not a substitute for viewing the gait.
"""
import bpy
import json
import math
from pathlib import Path
from mathutils import Euler, Quaternion, Vector
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


def posed_toe_world(side):
    bone = rig.pose.bones[side + '_foot_link']
    return rig.matrix_world @ (bone.matrix @ toe_local_bone[side])


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
base_toe = {}
toe_local_bone = {}
ankle_tail = {}
for side in ('l', 'r'):
    ankle_tail[side] = rig.pose.bones[side + '_ankle_link'].tail.copy()
    endpoint = rig.get(side + '_foot_toe_endpoint_armature')
    assert endpoint is not None, f'Missing authored URDF toe endpoint for {side}'
    base_toe[side] = Vector(endpoint)
    foot_bone_rest = rig.data.bones[side + '_foot_link'].matrix_local
    toe_local_bone[side] = foot_bone_rest.inverted() @ base_toe[side]
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
    toe_world = rig.matrix_world @ base_toe[side]
    foot = foot_objects[side].evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = foot.to_mesh()
    try:
        lowest_z = min((foot.matrix_world @ v.co).z for v in mesh.vertices)
    finally:
        foot.to_mesh_clear()
    toe_above_sole[side] = max(0.015, toe_world.z - lowest_z)

# The terminal visual bone's local Y axis is deliberately aligned to its true
# URDF ankle-roll axis by upgrade_animation.py. A native local-space rotation
# constraint therefore enforces the actual, asymmetric servo range even though
# Blender's position IK chain stops at the ankle pivot.
ankle_roll = {}
ankle_roll_constraints = {}
for side in ('l', 'r'):
    name = side + '_foot_link'
    pb = rig.pose.bones[name]
    axis_local = Vector(pb['urdf_joint_axis_local']).normalized()
    axis_index = max(range(3), key=lambda i: abs(axis_local[i]))
    assert abs(axis_local[axis_index]) >= 0.995, (name, tuple(axis_local))
    axis_arm = rig.data.bones[name].matrix_local.to_3x3() @ axis_local
    axis_arm.normalize()
    lower, upper = map(float, pb['urdf_joint_limit_rad'])
    constraint = pb.constraints.new('LIMIT_ROTATION')
    constraint.name = 'URDF ankle-roll limit | ' + side
    constraint.owner_space = 'LOCAL'
    component_min, component_max = ((lower, upper) if axis_local[axis_index] >= 0
                                   else (-upper, -lower))
    for index, axis_name in enumerate(('x', 'y', 'z')):
        setattr(constraint, 'use_limit_' + axis_name, True)
        setattr(constraint, 'min_' + axis_name, component_min if index == axis_index else 0.0)
        setattr(constraint, 'max_' + axis_name, component_max if index == axis_index else 0.0)
    ankle_roll_constraints[side] = constraint
    ankle_roll[side] = {'axis_local': axis_local, 'axis_arm': axis_arm,
                        'limits': (lower, upper), 'axis_index': axis_index}

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
    target = make_control('IK Target | ' + side + ' ankle pivot / sole plant', 'SPHERE', 0.055, (1.0, 0.34, 0.04, 1))
    pole = make_control('IK Pole | ' + side + ' knee bend', 'CIRCLE', 0.075, (0.08, 0.55, 1.0, 1))
    targets[side] = target
    poles[side] = pole
    # Solve the four proximal servos at the ankle pivot. Ankle roll is a
    # separately keyed, locally constrained fifth URDF servo on foot_link.
    constraint = rig.pose.bones[side + '_ankle_link'].constraints.new('IK')
    constraint.name = 'URDF leg IK | ' + side + ' | measured ankle-pivot plant'
    constraint.target = target
    constraint.pole_target = pole
    constraint.chain_count = 4
    constraint.iterations = 512
    constraint.use_stretch = False
    # Solve the physically reachable ankle-pivot position only; rotation-target
    # IK over-constrained the URDF chain and previously sent its endpoint metres away.
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


def ankle_roll_angle(side, x, y, yaw):
    """Best bounded URDF ankle-roll angle aligning the sole to terrain tilt."""
    epsilon = 0.01
    dx = (height(x + epsilon, y) - height(x - epsilon, y)) / (2.0 * epsilon)
    dy = (height(x, y + epsilon) - height(x, y - epsilon)) / (2.0 * epsilon)
    terrain_normal = Vector((-dx, -dy, 1.0)).normalized()
    root_rotation = Euler((0.0, 0.0, yaw), 'XYZ').to_quaternion()
    axis_world = (root_rotation @ ankle_roll[side]['axis_arm']).normalized()
    base_normal = Vector((0.0, 0.0, 1.0))
    base_projected = base_normal - axis_world * base_normal.dot(axis_world)
    terrain_projected = terrain_normal - axis_world * terrain_normal.dot(axis_world)
    if base_projected.length < 1e-8 or terrain_projected.length < 1e-8:
        angle = 0.0
    else:
        angle = math.atan2(axis_world.dot(base_projected.cross(terrain_projected)),
                           base_projected.dot(terrain_projected))
    lower, upper = ankle_roll[side]['limits']
    return max(lower, min(upper, angle))


def pole_position(side, frame):
    local = (hip_local[side] + knee_local[side]) * 0.5 + Vector((0.0, -0.28, 0.0))
    return local_to_world(local, frame)

# Solve sole clearance by bracketing the evaluated mesh response, rather than
# assuming one-unit target motion produces one-unit sole motion. This matters
# near URDF joint limits, where a simple proportional correction can stall or
# alternate between penetration and hover.
def solve_stance_height(target, side, frame, desired_gap):
    """Find a reachable ankle-target height using evaluated sole geometry.

    URDF-limited IK can change ankle posture as target height changes, so the
    sole-clearance response is not guaranteed monotonic. Sweep both directions,
    then locally refine the best sample instead of assuming a proportional
    correction always moves the sole in the same direction.
    """
    base_z = float(target.location.z)
    best = None
    evaluations = 0
    initial_gap = None
    sweep = []

    def sample(offset):
        nonlocal evaluations, best, initial_gap
        target.location.z = base_z + offset
        target.keyframe_insert(data_path='location', frame=frame,
                               group='World-space planted-foot IK')
        bpy.context.view_layer.update()
        gap = mesh_clearance(side)
        residual = gap - desired_gap
        evaluations += 1
        sample_record = {'offset_m': offset, 'clearance_m': gap, 'residual_m': residual}
        sweep.append(sample_record)
        if initial_gap is None and abs(offset) < 1e-12:
            initial_gap = gap
        if best is None or (abs(residual), abs(offset)) < (abs(best[1]), abs(best[0])):
            best = (offset, residual, gap)
        return residual

    coarse_offsets = [step * 0.01 for step in range(-12, 13)]
    for offset in coarse_offsets:
        sample(offset)

    # A dense local scan (0.4 mm spacing) refines the nearest physically
    # reachable root without relying on a derivative through the IK solver.
    center = best[0]
    fine_offsets = [max(-0.12, min(0.12, center + step * 0.0004))
                    for step in range(-max_contact_iterations, max_contact_iterations + 1)]
    for offset in fine_offsets:
        sample(offset)

    best_offset = best[0]
    best_gap = best[2]
    # Only accept small support-foot corrections; a larger correction usually
    # indicates a bad stride/root/rig transform, not a plausible ankle solution.
    converged = (abs(best[1]) <= contact_clearance_tolerance
                 and abs(best_offset) <= 0.12)
    sample(best_offset)  # restore the best solution into the current keyed pose
    diagnostics = {
        'frame': frame, 'side': side, 'initial_clearance_m': initial_gap,
        'clearance_m': best_gap, 'target_z_correction_m': best_offset,
        'evaluations': evaluations, 'converged': converged,
        'sampled_clearance_range_m': [min(v['clearance_m'] for v in sweep),
                                      max(v['clearance_m'] for v in sweep)],
    }
    return best_gap, evaluations, best_offset, converged, diagnostics


# The pelvis trajectory remains the measured physical flock-follow path; no
# individual joint is hand-spun. Only world-space IK controls are adjusted.
contact_errors = []
contact_samples = []
contact_solver_iterations = []
contact_solver_failures = []
ankle_roll_samples = {side: [] for side in ('l', 'r')}
toe_position_errors = []
target_errors = []
target_error_samples = []
for frame in range(scene.frame_start, scene.frame_end + 1):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    for side in ('l', 'r'):
        target = targets[side]
        desired_toe, planted = target_position(side, frame)
        yaw = root_at(frame)[1]
        roll_angle = ankle_roll_angle(side, desired_toe.x, desired_toe.y, yaw)
        ankle_roll_samples[side].append(roll_angle)
        foot_bone = rig.pose.bones[side + '_foot_link']
        foot_bone.rotation_quaternion = Quaternion(ankle_roll[side]['axis_local'], roll_angle)
        foot_bone.keyframe_insert(data_path='rotation_quaternion', frame=frame,
                                  group='URDF ankle-roll servo | terrain normal')
        root_rotation = Euler((0.0, 0.0, yaw), 'XYZ').to_quaternion()
        ankle_offset_arm = Quaternion(ankle_roll[side]['axis_arm'], roll_angle) @ (
            base_toe[side] - ankle_tail[side])
        target.location = desired_toe - root_rotation @ ankle_offset_arm
        poles[side].location = pole_position(side, frame)
        # Insert the current keys BEFORE measuring: otherwise Blender evaluates
        # the prior-frame F-curve and the solver sees a stale target transform.
        target.keyframe_insert(data_path='location', frame=frame, group='World-space planted-foot IK')
        poles[side].keyframe_insert(data_path='location', frame=frame, group='Moving knee pole target')
        bpy.context.view_layer.update()
        # Correct the initial geometric toe-to-ankle offset against the actual
        # evaluated, skinned foot; ankle-pitch IK can change that vector slightly.
        for _ in range(3):
            toe_error = desired_toe - posed_toe_world(side)
            if toe_error.xy.length < 0.0005:
                break
            target.location.x += toe_error.x
            target.location.y += toe_error.y
            target.keyframe_insert(data_path='location', frame=frame,
                                   group='World-space planted-foot IK')
            bpy.context.view_layer.update()
        if planted:
            gap, iterations, correction, converged, diagnostic = solve_stance_height(
                target, side, frame, contact_clearance_target)
            contact_solver_iterations.append(iterations)
            contact_samples.append(diagnostic)
            if not converged:
                contact_solver_failures.append(diagnostic)
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
        toe_position_errors.append((desired_toe - posed_toe_world(side)).xy.length)
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
if rig.animation_data and rig.animation_data.action:
    for curve in rig.animation_data.action.fcurves:
        if any(f'pose.bones["{side}_foot_link"].rotation_quaternion' in curve.data_path for side in ('l', 'r')):
            for key in curve.keyframe_points:
                key.interpolation = 'LINEAR'

rig['foot_ik_version'] = 'URDF four-link ankle-pivot IK plus quaternion-keyed, locally constrained ankle-roll servo; measured terrain-planted sole meshes'
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
    'IK_endpoint_bones': {side: side + '_ankle_link' for side in ('l', 'r')},
    'IK_chain_lengths': {side: IKs[side].chain_count for side in ('l', 'r')},
    'ankle_roll_limits_rad': {side: list(ankle_roll[side]['limits']) for side in ('l', 'r')},
    'ankle_roll_angle_ranges_rad': {side: [min(ankle_roll_samples[side]), max(ankle_roll_samples[side])]
                                    for side in ('l', 'r')},
    'ankle_roll_limit_constraints': {side: ankle_roll_constraints[side].name for side in ('l', 'r')},
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
    'max_horizontal_toe_target_error_m': max(toe_position_errors),
    'contact_clearance_target_m': contact_clearance_target,
    'contact_clearance_tolerance_m': contact_clearance_tolerance,
    'lowest_sole_sample': min(clearance_samples, key=lambda item: item['clearance_m']),
    'highest_sole_sample': max(clearance_samples, key=lambda item: item['clearance_m']),
    'lowest_planted_sample': min(contact_samples, key=lambda item: item['clearance_m']),
    'highest_planted_sample': max(contact_samples, key=lambda item: item['clearance_m']),
    'largest_ik_endpoint_error': max(target_error_samples, key=lambda item: item['error_m']),
    'stance_contact_solver': {'method': 'bidirectional coarse scan + local refinement on evaluated sole-mesh clearance',
                              'tolerance_m': contact_clearance_tolerance,
                              'max_evaluations': max(contact_solver_iterations, default=0),
                              'max_abs_target_correction_m': max((abs(v['target_z_correction_m']) for v in contact_samples), default=0.0),
                              'failed_samples': contact_solver_failures[:12],
                              'failure_count': len(contact_solver_failures)},
    'frames': scene.frame_end,
    'visual_review_completed': False,
}
(ROOT / 'output' / 'robot_ik_audit.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
print('URDF-LIMITED IK WALK DIAGNOSTIC', json.dumps(summary, ensure_ascii=False), flush=True)
assert len(IKs) == 2 and all(c.chain_count == 4 and c.pole_target for c in IKs.values())
assert all(len([c for c in rig.pose.bones[side + '_foot_link'].constraints
                if c.type == 'LIMIT_ROTATION']) >= 1 for side in ('l', 'r'))
assert all(ankle_roll[side]['limits'][0] <= min(ankle_roll_samples[side])
           and max(ankle_roll_samples[side]) <= ankle_roll[side]['limits'][1]
           for side in ('l', 'r'))
assert min(clearances) >= -0.003 and max(clearances) < 0.15, summary['foot_clearance_range_m']
assert not contact_solver_failures, summary['stance_contact_solver']
assert min(contact_errors) >= -0.001 and max(contact_errors) <= 0.006, summary['planted_foot_clearance_range_m']
assert max(target_errors) < 0.001, summary['largest_ik_endpoint_error']
assert max(toe_position_errors) < 0.01, summary['max_horizontal_toe_target_error_m']
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
    'urdf_ankle_roll_limit_constraints': sum(c.type == 'LIMIT_ROTATION' and c.name.startswith('URDF ankle-roll limit')
                                             for pb in rig.pose.bones for c in pb.constraints),
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
    'terminal_ankle_roll_locally_constrained': True,
    'ankle_roll_servo_keyed_every_frame': all(len(ankle_roll_samples[side]) == scene.frame_end for side in ('l', 'r')),
    'pole_targets_baked_every_frame': True,
    'two_sole_meshes_measured_each_frame': len(clearances) == 288,
    'ik_position_targets_only': all(not c.use_rotation for c in IKs.values()),
    'all_frame_foot_clearance_range_m': [min(clearances), max(clearances)],
    'stance_only_foot_clearance_range_m': [min(contact_errors), max(contact_errors)],
    'max_ik_endpoint_error_m': max(target_errors),
    'max_horizontal_toe_target_error_m': max(toe_position_errors),
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
