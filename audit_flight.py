"""Measure the actual evaluated geometry and spatial chase in the final blend.

Data-only review: passing checks do NOT imply an EEVEE render or visual approval.
"""
import bpy
import math
import json
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view
from terrain_profile import height
ROOT=Path(__file__).resolve().parent
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'output/Twilight_Wilderness_Robot.blend'))
s=bpy.context.scene
rig=bpy.data.objects['Robot armature | 144-frame scanning and step pose']
body=bpy.data.objects['URDF | base_link']
points=bpy.data.objects['Baked fireflies | 144-frame physical particle cache']
fly=bpy.data.objects['Firefly | original CC0 insect mesh | source']
land=bpy.data.objects['Terrain | 2.5m relief and excavated lake basin']
water=bpy.data.objects['Lake | radial waterline inside excavated basin']
frames={}
for f in (1,13,37,75,110,144):
    s.frame_set(f)
    instance_positions=[i.matrix_world.translation.copy() for i in bpy.context.evaluated_depsgraph_get().object_instances
               if i.is_instance and i.parent.original==points]
    centroid=sum(instance_positions,Vector())/len(instance_positions)
    b=body.evaluated_get(bpy.context.evaluated_depsgraph_get())
    frames[f]={
        'baked_insect_and_halo_instances':len(instance_positions),
        'flight_centroid_xyz_m':[round(t,3) for t in centroid],
        'rig_xy_m':[round(t,3) for t in rig.matrix_world.translation.xy],
        'visible_body_xy_m':[round(t,3) for t in b.matrix_world.translation.xy],
        'distance_body_to_flying_centroid_xy_m':round((b.matrix_world.translation.xy-centroid.xy).length,3),
    }
start=Vector(frames[1]['visible_body_xy_m']);end=Vector(frames[144]['visible_body_xy_m'])
fly_start=Vector(frames[1]['flight_centroid_xyz_m']);fly_end=Vector(frames[144]['flight_centroid_xyz_m'])
# Direction must agree with the moving insects, not just a fixed target.
travel=(end-start).normalized()
flight=(fly_end.xy-fly_start.xy).normalized()
foot_clearance=[]
for f in range(1,145):
    s.frame_set(f)
    for side in ('l','r'):
        o=bpy.data.objects['URDF | '+side+'_foot_link']
        e=o.evaluated_get(bpy.context.evaluated_depsgraph_get());mesh=e.to_mesh()
        try:
            low=min((lambda v:v.z-height(v.x,v.y))(e.matrix_world@p.co) for p in mesh.vertices)
        finally:e.to_mesh_clear()
        foot_clearance.append(low)
pine=bpy.data.objects['Source | pine_sapling_small_b']
pine_dim=[max(v.co[k] for v in pine.data.vertices)-min(v.co[k] for v in pine.data.vertices) for k in range(3)]
fly_dim=[max(v.co[k] for v in fly.data.vertices)-min(v.co[k] for v in fly.data.vertices) for k in range(3)]
project=world_to_camera_view(s,s.camera,Vector((3.7,3.8,-.38)))
inspection=bpy.data.objects['Camera | firefly anatomical inspection']
hero=inspection['tracked_particle_index']
ik_constraints=[c for pb in rig.pose.bones for c in pb.constraints if c.type=='IK']
ik_audit=json.loads((ROOT/'output'/'robot_ik_audit.json').read_text())
emitter=bpy.data.objects.get('Simulation emitter | hidden after full flight bake')
particle_system=emitter.particle_systems[0] if emitter and emitter.particle_systems else None
particle_settings=particle_system.settings if particle_system else None
field_objects=[o for o in bpy.data.collections['Fireflies'].objects if getattr(o,'field',None)]
wind_fields=[o for o in field_objects if o.field.type=='WIND']
turbulence_fields=[o for o in field_objects if o.field.type=='TURBULENCE']
abdomen_node=fly.data.materials[1].node_tree.nodes.get('Principled BSDF')
emission_strength=float(abdomen_node.inputs['Emission Strength'].default_value)
emission_color=tuple(float(c) for c in abdomen_node.inputs['Emission Color'].default_value[:3])
point_lights=[o for o in bpy.data.collections['Fireflies'].objects if o.type=='LIGHT']
glow_nodes=[n for n in s.node_tree.nodes if n.bl_idname=='CompositorNodeGlare' and n.glare_type=='FOG_GLOW']
water_projection=[world_to_camera_view(s,s.camera,water.matrix_world@v.co) for v in water.data.vertices]
water_visible_fraction=sum(0.0<=p.x<=1.0 and 0.0<=p.y<=1.0 for p in water_projection)/max(1,len(water_projection))
hero_pixels={}
for f in (1,75,144):
    s.frame_set(f)
    keys=points.data.shape_keys.key_blocks
    p=keys[0].data[hero].co.copy()
    for key in keys[1:]:p+=(key.data[hero].co-keys[0].data[hero].co)*key.value
    projection=world_to_camera_view(s,inspection,p)
    hero_pixels[f]=[round(projection.x*640),round((1-projection.y)*480)]
checks={
 'terrain_broad_relief_over_2m':land['height_range_m']>2,
 'smooth_256_sample_analytic_shoreline':water.get('shoreline_radial_samples',0)==256 and len(water.data.polygons)>4000 and all(abs(v.co.z-water['water_level_z'])<.0001 for v in water.data.vertices),
 'lake_center_in_preview_framing':0<project.x<1 and 0<project.y<1,
 'most_of_lake_surface_in_overview_frame':water_visible_fraction>=.70,
 'up_axis_fixed_pine':pine_dim[2]>pine_dim[1]*1.3,
 'insect_wings_body_head_preserved':len(fly.data.vertices)>900 and len({p.material_index for p in fly.data.polygons})==5 and fly_dim[1]>fly_dim[2]*1.3,
 'chromatic_bioluminescent_abdomen':emission_strength>=1.5 and emission_color[1]>emission_color[0]*1.7 and emission_color[1]>emission_color[2]*3.0,
 'emissive_insect_and_aura_instances_at_every_sample':all(data['baked_insect_and_halo_instances']==2*len(points.data.vertices) for data in frames.values()),
 'actual_newton_system_and_brownian_motion':particle_settings is not None and particle_settings.physics_type=='NEWTON' and particle_settings.count>=100 and particle_settings.brownian_factor>=.5,
 'directional_wind_and_turbulence_fields':len(wind_fields)==1 and len(turbulence_fields)==1 and wind_fields[0].field.strength>=5 and turbulence_fields[0].field.strength>=.5,
 'genuine_newton_particle_simulation_baked_all_frames':'NEWTON' in points.get('physics','') and points.get('simulation_frames',0)==144 and len(points.data.shape_keys.key_blocks)>=140,
 'baked_swarm_changes_world_position_gt_2m':(fly_end-fly_start).length>2,
 'real_robot_visual_body_moves_gt_2m':(end-start).length>2,
 'visible_robot_tracks_physical_flock_direction':travel.dot(flight)>.96,
 'robot_keeps_ahead_flight_within_1_7m':all(data['distance_body_to_flying_centroid_xy_m']<1.7 for data in frames.values()),
 'both_soles_measured_each_frame_with_small_clearance':min(foot_clearance)>=-.003 and max(foot_clearance)<.075,
 'stance_sole_contact_within_6mm':ik_audit['planted_foot_clearance_range_m'][0]>=-.001 and ik_audit['planted_foot_clearance_range_m'][1]<=.006,
 'two_real_urdf_limited_ik_chains_and_poles':len(ik_constraints)==2 and all(c.chain_count==4 and c.pole_target for c in ik_constraints),
 'urdf_axis_locks_applied_to_both_legs':all(any(rig.pose.bones[name].lock_ik_x or rig.pose.bones[name].lock_ik_y or rig.pose.bones[name].lock_ik_z for name in side_bones) for side_bones in (( 'l_hip_roll_link','l_knee_link','l_ankle_pitch_joint','l_ankle_link'),('r_hip_roll_link','r_knee_link','r_ankle_pitch_joint','r_ankle_link'))),
 'original_articulated_links_and_rig_kept':len(rig.data.bones)==18 and all(o.parent==rig for o in bpy.data.collections['Robot'].objects if o.type=='MESH'),
 '18_animated_real_firefly_lights_and_compositor_glow':len(point_lights)>=18 and all(o.animation_data and o.data.energy>0 for o in point_lights) and bool(glow_nodes) and s.render.use_compositing,
 'animated_insect_closeup_tracks_one_baked_particle':all(200<x<440 and 120<y<360 for x,y in hero_pixels.values()),
 'render_is_eevee_640x480':s.render.engine=='BLENDER_EEVEE_NEXT' and (s.render.resolution_x,s.render.resolution_y)==(640,480),
}
report={'kind':'actual evaluated geometric motion; NOT a visual render review',
        'checks':checks,'frames':frames,'flight_world_distance_m':round((fly_end-fly_start).length,3),
        'robot_visible_world_distance_m':round((end-start).length,3),
        'flight_robot_direction_dot':round(travel.dot(flight),3),
        'foot_clearance_range_m':[round(min(foot_clearance),3),round(max(foot_clearance),3)],
        'planted_sole_clearance_range_m':ik_audit['planted_foot_clearance_range_m'],
        'max_ik_endpoint_error_m':ik_audit['max_ik_target_error_m'],
        'urdf_limited_ik_chain_count':len(ik_constraints),
        'physics_baked_firefly_count':len(points.data.vertices),
        'physical_flock_path_distance_m':points.get('particle_path_distance_sum_m'),
        'unresolved_walk_contact':ik_audit['planted_foot_clearance_range_m'][0]<-.001 or ik_audit['planted_foot_clearance_range_m'][1]>.006,
        'lake_face_count':len(water.data.polygons),'terrain_height_range_m':land['height_range_m'],
        'lake_preview_pixel':[round(project.x*640),round((1-project.y)*480)],
        'lake_surface_visible_fraction':round(water_visible_fraction,4),
        'tracked_firefly_pixel_by_frame':hero_pixels,
        'abdomen_emission_strength':emission_strength,'abdomen_emission_color_rgb':list(emission_color),
        'newton_particle_system':None if particle_settings is None else {'physics_type':particle_settings.physics_type,'count':particle_settings.count,'brownian_factor':particle_settings.brownian_factor,'damping':particle_settings.damping},
        'wind_field_count':len(wind_fields),'turbulence_field_count':len(turbulence_fields),
        'preview_png_available':(ROOT/'output/preview_640x480.png').exists(),
        'full_visual_review_completed':False}
(ROOT/'output/flight_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
if not all(checks.values()):raise RuntimeError('Flight and world-space audit failed')
