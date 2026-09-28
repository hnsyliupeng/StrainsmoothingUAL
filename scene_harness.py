"""Scene-specific pre-render contract harness; never grants visual approval.

Run after build_scene.py -> upgrade_animation.py -> animate_flight.py ->
refine_walk.py. It verifies the saved .blend itself, including real NEWTON
settings, bounded IK, near-zero sole contact, emissions, framing, and the
submerged-scatter exclusion. The workflow still requires actual EEVEE PNGs and
an image review after this script passes.
"""
import json
from pathlib import Path

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Vector

ROOT = Path(__file__).resolve().parent
BLEND = ROOT / 'output' / 'Twilight_Wilderness_Robot.blend'
AUDIT = ROOT / 'output' / 'robot_ik_audit.json'
OUT = ROOT / 'output' / 'scene_contract_audit.json'

if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != BLEND.resolve():
    bpy.ops.wm.open_mainfile(filepath=str(BLEND))
scene = bpy.context.scene

points = bpy.data.objects['Baked fireflies | 144-frame physical particle cache']
insect = bpy.data.objects['Firefly | original CC0 insect mesh | source']
rig = bpy.data.objects['Robot armature | 144-frame scanning and step pose']
emitter = bpy.data.objects['Simulation emitter | hidden after full flight bake']
particle_system = emitter.particle_systems[0] if emitter.particle_systems else None
settings = particle_system.settings if particle_system else None
firefly_collection = bpy.data.collections['Fireflies']
force_fields = [o for o in firefly_collection.objects if getattr(o, 'field', None)]
wind = [o for o in force_fields if o.field.type == 'WIND']
turbulence = [o for o in force_fields if o.field.type == 'TURBULENCE']
lights = [o for o in firefly_collection.objects if o.type == 'LIGHT']
water = bpy.data.objects['Lake | radial waterline inside excavated basin']
terrain = bpy.data.objects['Terrain | 2.5m relief and excavated lake basin']

checks = {}
details = {}

def check(name, passed, detail=None):
    checks[name] = bool(passed)
    if detail is not None:
        details[name] = detail

resolution = (
    round(scene.render.resolution_x * scene.render.resolution_percentage / 100),
    round(scene.render.resolution_y * scene.render.resolution_percentage / 100),
)
check('eevee_exactly_640x480', scene.render.engine == 'BLENDER_EEVEE_NEXT' and resolution == (640, 480),
      {'engine': scene.render.engine, 'resolution': resolution})
check('144_frame_range', scene.frame_start == 1 and scene.frame_end == 144,
      [scene.frame_start, scene.frame_end])

# Landscape: geometry and render framing are structural evidence only.
scatterers = [o for o in bpy.data.collections['Environment'].objects
              if any(mod.type == 'NODES' for mod in o.modifiers)]
check('fifteen_editable_gn_scatterers', len(scatterers) == 15, len(scatterers))
check('broad_analytic_lake_mesh',
      len(water.data.polygons) >= 4000 and water.get('shoreline_radial_samples') == 256,
      {'faces': len(water.data.polygons), 'radial_samples': water.get('shoreline_radial_samples')})
lake_xy = tuple(float(v) for v in water.get('lake_center_xy', (0, 0)))
water_center = Vector((lake_xy[0], lake_xy[1], float(water.get('water_level_z', 0))))
water_center_projection = world_to_camera_view(scene, scene.camera, water_center)
check('lake_center_inside_overview',
      0.03 <= water_center_projection.x <= 0.97 and 0.03 <= water_center_projection.y <= 0.97,
      [round(water_center_projection.x, 4), round(water_center_projection.y, 4)])
check('no_manual_foreground_rocks',
      not any(o.name.startswith('Foreground mossy rock') for o in bpy.data.objects))

# Check actual GN instance roots against the shared analytic waterline profile.
from terrain_profile import height, LAKE_LEVEL
scene.frame_set(75)
bpy.context.view_layer.update()
depsgraph = bpy.context.evaluated_depsgraph_get()
scatter_names = {o.name for o in scatterers}
scatter_instances = [i for i in depsgraph.object_instances
                     if i.is_instance and i.parent.original.name in scatter_names]
submerged_roots = []
for instance in scatter_instances:
    p = instance.matrix_world.translation
    if height(p.x, p.y) <= LAKE_LEVEL + 0.005:
        submerged_roots.append([round(p.x, 3), round(p.y, 3), round(p.z, 3)])
check('no_gn_instances_on_submerged_terrain', len(submerged_roots) == 0,
      {'tested_instances': len(scatter_instances), 'submerged_roots': submerged_roots[:8],
       'submerged_count': len(submerged_roots)})

# Physics: inspect the retained Blender emitter and force fields, not metadata alone.
check('retained_particle_emitter_is_newton_brownian',
      settings is not None and settings.physics_type == 'NEWTON'
      and settings.count >= 100 and settings.brownian_factor >= 0.5,
      None if settings is None else {
          'physics_type': settings.physics_type, 'count': settings.count,
          'brownian_factor': settings.brownian_factor, 'damping': settings.damping,
          'seed': particle_system.seed,
      })
check('directional_wind_and_turbulence_present', len(wind) == 1 and len(turbulence) == 1,
      {'wind': len(wind), 'turbulence': len(turbulence),
       'wind_strength': wind[0].field.strength if wind else None,
       'turbulence_strength': turbulence[0].field.strength if turbulence else None})
shape_keys = points.data.shape_keys.key_blocks if points.data.shape_keys else []
start_key = shape_keys.get('Blender NEWTON bake | frame 001') if shape_keys else None
end_key = shape_keys.get('Blender NEWTON bake | frame 144') if shape_keys else None
bake_travel = 0.0
if start_key and end_key and len(start_key.data) == len(end_key.data) == len(points.data.vertices):
    start_centroid = sum((v.co for v in start_key.data), Vector((0.0, 0.0, 0.0))) / len(start_key.data)
    end_centroid = sum((v.co for v in end_key.data), Vector((0.0, 0.0, 0.0))) / len(end_key.data)
    bake_travel = (end_centroid - start_centroid).length
check('full_144_sample_moving_geometry_bake',
      len(points.data.vertices) >= 80 and len(shape_keys) >= 145 and bake_travel > 2.0
      and points.get('simulation_frames') == 144,
      {'points': len(points.data.vertices), 'shape_keys': len(shape_keys),
       'centroid_travel_m': round(bake_travel, 4),
       'simulation_frames': points.get('simulation_frames')})

# Bioluminescence: chromatic abdomen first; the small transparent aura is secondary.
abdomen_bsdf = insect.data.materials[1].node_tree.nodes.get('Principled BSDF')
emission_strength = float(abdomen_bsdf.inputs['Emission Strength'].default_value)
emission_color = tuple(float(v) for v in abdomen_bsdf.inputs['Emission Color'].default_value[:3])
aura = bpy.data.objects['Firefly | faint 1.1cm secondary halo prototype']
aura_material = aura.data.materials[0]
aura_bsdf = aura_material.node_tree.nodes.get('Principled BSDF')
aura_alpha = float(aura_bsdf.inputs['Alpha'].default_value)
aura_emission = float(aura_bsdf.inputs['Emission Strength'].default_value)
aura_dims = [max(v.co[k] for v in aura.data.vertices) - min(v.co[k] for v in aura.data.vertices)
             for k in range(3)]
check('chromatic_emissive_abdomen_not_white_clipped',
      1.5 <= emission_strength <= 2.2 and emission_color[1] > emission_color[0] * 1.7
      and emission_color[1] > emission_color[2] * 3.0,
      {'strength': emission_strength, 'emission_color_rgb': emission_color})
check('small_faint_translucent_aura',
      max(aura_dims) <= 0.012 and aura_alpha <= 0.005 and aura_emission <= 0.05,
      {'dimensions_m': aura_dims, 'alpha': aura_alpha, 'emission_strength': aura_emission})
node_group = points.modifiers[0].node_group if points.modifiers else None
instance_nodes = [n for n in node_group.nodes if n.bl_idname == 'GeometryNodeInstanceOnPoints'] if node_group else []
check('physical_points_instance_insect_and_aura', len(instance_nodes) == 2,
      [n.label for n in instance_nodes])
light_energy_keys = [
    float(key.co.y)
    for obj in lights if obj.animation_data and obj.animation_data.action
    for curve in obj.animation_data.action.fcurves if curve.data_path == 'energy'
    for key in curve.keyframe_points
]
check('animated_flight_lights_and_eevee_bloom', len(lights) >= 18
      and all(o.animation_data is not None and o.data.energy > 0 for o in lights)
      and light_energy_keys and min(light_energy_keys) >= 0.40 and max(light_energy_keys) <= 0.70
      and scene.use_nodes and scene.render.use_compositing
      and any(n.bl_idname == 'CompositorNodeGlare' and n.glare_type == 'FOG_GLOW'
              for n in scene.node_tree.nodes),
      {'flight_lights': len(lights), 'flight_light_energy_range':
       [min(light_energy_keys), max(light_energy_keys)] if light_energy_keys else [],
       'glare_nodes': sum(
          n.bl_idname == 'CompositorNodeGlare' and n.glare_type == 'FOG_GLOW'
          for n in scene.node_tree.nodes)})

# Robot: real URDF links, both constrained IK chains, keyed controls, close-to-zero plants.
ik_pairs = [(pb, c) for pb in rig.pose.bones for c in pb.constraints if c.type == 'IK']
ik_constraints = [c for _, c in ik_pairs]
check('toe_endpoint_ik_is_owned_by_both_foot_bones',
      {pb.name for pb, _ in ik_pairs} == {'l_foot_link', 'r_foot_link'})
check('two_five_link_urdf_ik_chains_with_poles', len(ik_constraints) == 2
      and all(c.chain_count == 5 and c.target is not None and c.pole_target is not None
              and not c.use_rotation and not c.use_stretch for c in ik_constraints),
      [{'name': c.name, 'chain_count': c.chain_count,
        'target': c.target.name if c.target else None,
        'pole': c.pole_target.name if c.pole_target else None,
        'position_only_target': not c.use_rotation} for c in ik_constraints])
leg_joints_ok = True
for side in ('l', 'r'):
    for name in (side + '_hip_roll_link', side + '_knee_link',
                 side + '_ankle_pitch_joint', side + '_ankle_link', side + '_foot_link'):
        pb = rig.pose.bones[name]
        lock_count = sum((pb.lock_ik_x, pb.lock_ik_y, pb.lock_ik_z))
        limits_enabled = any((pb.use_ik_limit_x, pb.use_ik_limit_y, pb.use_ik_limit_z))
        leg_joints_ok &= lock_count >= 2 and limits_enabled
check('urdf_axis_locks_and_limits_on_both_legs', leg_joints_ok)
controls = bpy.data.collections['Robot Controls | URDF foot IK targets and poles']
keyed_controls = [o for o in controls.objects if o.animation_data and o.animation_data.action]
control_key_counts = {o.name: min((len(fc.keyframe_points) for fc in o.animation_data.action.fcurves
                                   if fc.data_path == 'location'), default=0)
                      for o in keyed_controls}
check('world_space_foot_targets_and_poles_are_keyed_all_frames',
      len(keyed_controls) == 4 and all(count >= 144 for count in control_key_counts.values()),
      control_key_counts)
check('ik_targets_solve_reachable_position_without_rotation_overconstraint',
      len(ik_constraints) == 2 and all(not c.use_rotation for c in ik_constraints))
robot_links = [o for o in bpy.data.collections['Robot'].objects if o.type == 'MESH']
check('18_authored_rigged_urdf_mesh_links', len(robot_links) == 18 and len(rig.data.bones) == 18
      and all(o.parent == rig and any(m.type == 'ARMATURE' and m.object == rig for m in o.modifiers)
              for o in robot_links),
      {'mesh_links': len(robot_links), 'bones': len(rig.data.bones)})
robot_camera = bpy.data.objects['Camera | robot rig inspection']
root_ground_errors = []
for frame in range(scene.frame_start, scene.frame_end + 1):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    root = rig.matrix_world.translation.copy()
    root_ground_errors.append(abs(root.z - height(root.x, root.y)))
scene.frame_set(1)
bpy.context.view_layer.update()
root_start = rig.matrix_world.translation.copy()
robot_cam_start = robot_camera.matrix_world.translation.copy()
scene.frame_set(144)
bpy.context.view_layer.update()
root_end = rig.matrix_world.translation.copy()
robot_cam_end = robot_camera.matrix_world.translation.copy()
check('measured_robot_root_and_closeup_camera_travel',
      (root_end - root_start).length > 2.0 and (robot_cam_end - robot_cam_start).length > 2.0,
      {'root_distance_m': round((root_end - root_start).length, 4),
       'camera_distance_m': round((robot_cam_end - robot_cam_start).length, 4)})
check('root_tracks_analytic_ground_all_144_frames',
      max(root_ground_errors) < 0.002,
      {'max_abs_z_error_m': round(max(root_ground_errors), 6), 'samples': len(root_ground_errors)})

ik_report = json.loads(AUDIT.read_text())
stance = ik_report['planted_foot_clearance_range_m']
check('planted_sole_clearance_within_6mm', stance[0] >= -0.001 and stance[1] <= 0.006,
      {'min_m': stance[0], 'max_m': stance[1],
       'target_m': ik_report.get('contact_clearance_target_m')})
solver = ik_report.get('stance_contact_solver', {})
check('stance_contact_solver_converged_with_small_corrections',
      solver.get('failure_count') == 0 and solver.get('max_abs_target_correction_m', 1.0) <= 0.12,
      solver)
check('ik_target_endpoint_error_below_1mm', ik_report['max_ik_target_error_m'] < 0.001,
      ik_report['max_ik_target_error_m'])

result = {
    'harness': 'Blender scene structural contract v1',
    'blender_version': bpy.app.version_string,
    'candidate': str(BLEND.relative_to(ROOT)),
    'kind': 'automated data/kinematic checks; NOT visual approval',
    'checks': checks,
    'passed': all(checks.values()),
    'passed_count': sum(checks.values()),
    'check_count': len(checks),
    'details': details,
    'visual_review_required': True,
    'full_visual_review_completed': False,
}
OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(result, ensure_ascii=False, indent=2))
if not result['passed']:
    failed = [name for name, ok in checks.items() if not ok]
    raise RuntimeError('Scene contract failed: ' + ', '.join(failed))
