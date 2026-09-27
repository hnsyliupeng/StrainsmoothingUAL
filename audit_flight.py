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
points=bpy.data.objects['Baked fireflies | frozen instances']
fly=bpy.data.objects['Firefly | original CC0 insect mesh | source']
land=bpy.data.objects['Terrain | 2.5m relief and excavated lake basin']
water=bpy.data.objects['Lake | water inside excavated basin']
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
 'actual_lake_water_mesh_above_submerged_terrain':len(water.data.polygons)>20 and all(abs(v.co.z-water['water_level_z'])<.0001 for v in water.data.vertices),
 'lake_center_in_preview_framing':0<project.x<1 and 0<project.y<1,
 'up_axis_fixed_pine':pine_dim[2]>pine_dim[1]*1.3,
 'insect_wings_body_head_preserved':len(fly.data.vertices)>900 and len({p.material_index for p in fly.data.polygons})==5 and fly_dim[1]>fly_dim[2]*1.3,
 'abdomen_emission':fly.data.materials[1].node_tree.nodes.get('Principled BSDF').inputs['Emission Strength'].default_value>=18,
 'actually_instanced_insect_and_halo_at_every_sample':all(data['baked_insect_and_halo_instances']==184 for data in frames.values()),
 'baked_mesh_has_animated_shape_keys':points.data.shape_keys is not None and len(points.data.shape_keys.key_blocks)>15,
 'baked_swarm_changes_world_position_gt_2m':(fly_end-fly_start).length>2,
 'real_robot_visual_body_moves_gt_2m':(end-start).length>2,
 'visible_robot_tracks_fly_direction':travel.dot(flight)>.96,
 'robot_keeps_ahead_flight_within_1_7m':all(data['distance_body_to_flying_centroid_xy_m']<1.7 for data in frames.values()),
 'every_frame_both_feet_near_terrain':min(foot_clearance)>=-.005 and max(foot_clearance)<.035,
 'original_articulated_links_and_rig_kept':len(rig.data.bones)==18 and all(o.parent==rig for o in bpy.data.collections['Robot'].objects if o.type=='MESH'),
 'lake_and_glow_compositor_saved':any(n.bl_idname=='CompositorNodeGlare' for n in s.node_tree.nodes),
 'animated_insect_closeup_actually_tracks_one_baked_particle':all(240<x<400 and 160<y<320 for x,y in hero_pixels.values()),
 'render_is_eevee_640x480':s.render.engine=='BLENDER_EEVEE_NEXT' and (s.render.resolution_x,s.render.resolution_y)==(640,480),
}
report={'kind':'actual evaluated geometric motion; NOT a visual render review',
        'checks':checks,'frames':frames,'flight_world_distance_m':round((fly_end-fly_start).length,3),
        'robot_visible_world_distance_m':round((end-start).length,3),
        'flight_robot_direction_dot':round(travel.dot(flight),3),
        'foot_clearance_range_m':[round(min(foot_clearance),3),round(max(foot_clearance),3)],
        'unresolved_walk_contact':max(foot_clearance)>.04,
        'lake_face_count':len(water.data.polygons),'terrain_height_range_m':land['height_range_m'],
        'lake_preview_pixel':[round(project.x*640),round((1-project.y)*480)],
        'tracked_firefly_pixel_by_frame':hero_pixels,
        'preview_png_available':(ROOT/'output/preview_640x480.png').exists(),
        'full_visual_review_completed':False}
(ROOT/'output/flight_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
if not all(checks.values()):raise RuntimeError('Flight and world-space audit failed')
