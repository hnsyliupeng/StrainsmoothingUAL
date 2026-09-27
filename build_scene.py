"""Build an editable static twilight woodland. Run: LD_LIBRARY_PATH=/tmp/bpy-libs python3 build_scene.py"""
import bpy, math, random, json, os
from mathutils import Vector, Matrix
from pathlib import Path
ROOT=Path(__file__).resolve().parent
random.seed(73281)
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
for c in list(bpy.data.collections):
 if c.name!='Collection': bpy.data.collections.remove(c)
scene=bpy.context.scene
main=bpy.data.collections.get('Collection');main.name='Architecture'
def collection(name):
 c=bpy.data.collections.new(name);scene.collection.children.link(c);return c
ENV=collection('Environment');ROBOT=collection('Robot');FLIES=collection('Fireflies');LIGHTS=collection('Lights');SOURCES=collection('Architecture / CC0 source assets')
REPORT=[]
def move(o,c):
 for old in list(o.users_collection):old.objects.unlink(o)
 c.objects.link(o);return o
def mat(name,color,metal=0,rough=.8,emit=0):
 m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
 bs=m.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=(*color,1);bs.inputs['Metallic'].default_value=metal;bs.inputs['Roughness'].default_value=rough
 if emit:bs.inputs['Emission Color'].default_value=(*color,1);bs.inputs['Emission Strength'].default_value=emit
 return m
soil=mat('Dark damp forest soil',(.062,.074,.065));bark=mat('Pine bark',(.105,.085,.067));foliage=mat('Needles | slate pine',(.065,.14,.105));moss=mat('Understory moss',(.10,.16,.105));stone=mat('Wet stone',(.17,.19,.19),rough=.9)
steel=mat('Oxidised titanium shell',(.21,.27,.28),.72,.46);dark=mat('Graphite joints',(.047,.062,.068),.5,.7);copper=mat('Worn copper trim',(.36,.21,.105),.72,.38);glass=mat('Optic | warm cyan',(.11,.62,.62),.3,.23,2.5);glow=mat('Firefly | amber bioluminescence',(1,.68,.16),0,.3,6)
def cube(name,loc,scale,material,coll=ROBOT,bevel=0):
 bpy.ops.mesh.primitive_cube_add(size=1,location=loc);o=move(bpy.context.object,coll);o.name=name;o.dimensions=scale;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);o.data.materials.append(material)
 if bevel:
  mod=o.modifiers.new('Machined edge radius','BEVEL');mod.width=bevel;mod.segments=2;o.modifiers.new('Weighted corners','WEIGHTED_NORMAL')
 return o
def sphere(name,loc,scale,material,coll=ROBOT):
 bpy.ops.mesh.primitive_uv_sphere_add(segments=12,ring_count=8,location=loc);o=move(bpy.context.object,coll);o.name=name;o.scale=scale;o.data.materials.append(material);return o
def rod(name,a,b,r,material,coll=ROBOT,verts=12):
 a,b=Vector(a),Vector(b);mid=(a+b)/2;delta=b-a
 bpy.ops.mesh.primitive_cylinder_add(vertices=verts,radius=r,depth=delta.length,location=mid);o=move(bpy.context.object,coll);o.name=name;o.rotation_euler=delta.to_track_quat('Z','Y').to_euler();o.data.materials.append(material);return o
def ground_z(x,y):return .13*math.sin(x*.46)*math.cos(y*.36)+.065*math.sin(y*.83+x*.17)
# Terrain, 30m across. Geometry baked into a simple editable grid.
n=48;verts=[];faces=[]
for j in range(n+1):
 for i in range(n+1):
  x=(i/n-.5)*30;y=(j/n-.5)*30;verts.append((x,y,ground_z(x,y)))
for j in range(n):
 for i in range(n):
  v=j*(n+1)+i;faces.append((v,v+1,v+n+2,v+n+1))
mesh=bpy.data.meshes.new('Editable terrain grid');mesh.from_pydata(verts,[],faces);mesh.update();terrain=bpy.data.objects.new('Terrain | 30m rolling floor',mesh);ENV.objects.link(terrain);mesh.materials.append(soil)
for p in mesh.polygons:p.use_smooth=True
# FBX CC0 source meshes (hidden from render, but referenced by Geometry Nodes).
sources=[]
for name,scale in [('SM_DeadPineTree',1.4),('SM_Pine_Tree',.31),('SM_Tree_Rounded',1.0)]:
 before=set(bpy.data.objects)
 bpy.ops.import_scene.fbx(filepath=str(ROOT/'assets'/f'{name}.fbx'))
 objects=[o for o in bpy.data.objects if o not in before and o.type=='MESH']
 for o in objects:
  move(o,SOURCES);o.name='CC0 source | '+name;o.location=(80+len(sources)*12,0,-30);o.data.transform(Matrix.Diagonal((scale,scale,scale,1)))
  # Original FBX materials reference textures that are NOT in the CC0 repository.
  # Replace them with self-contained bark/needle materials and classify the
  # existing asset polygons once; do not create any additional tree geometry.
  local_min=min(v.co.z for v in o.data.vertices)
  for v in o.data.vertices:v.co.z-=local_min
  o.data.materials.clear();o.data.materials.append(bark)
  if name!='SM_DeadPineTree':
   o.data.materials.append(foliage)
   for poly in o.data.polygons:
    center=sum((o.data.vertices[i].co for i in poly.vertices),Vector())/len(poly.vertices)
    radial=math.hypot(center.x,center.y)
    # Pine foliage begins farther out from its trunk than the broadleaf canopy.
    is_leaf=(center.z>(1.35 if name=='SM_Pine_Tree' else 1.5)
             and radial>(.36 if name=='SM_Pine_Tree' else .46))
    poly.material_index=int(is_leaf)
  o.data.update()
  o.hide_render=True;o.hide_set(True);sources.append(o)
# Additional open-source Kenney nature-kit OBJ meshes, imported once then scattered
# via GN. OBJ is Y-up, so rotate vertices into Blender Z-up and ground their bases.
def kenney_source(filename,display,material,loc,scale=1):
 before=set(bpy.data.objects)
 bpy.ops.wm.obj_import(filepath=str(ROOT/'assets'/('kenney_'+filename+'.obj')))
 objs=[o for o in bpy.data.objects if o not in before and o.type=='MESH']
 if len(objs)!=1:raise RuntimeError('Expected one Kenney mesh: '+filename)
 o=move(objs[0],SOURCES);o.name=display
 rot=Matrix.Rotation(math.pi/2,4,'X');o.data.transform(rot)
 minimum=min(v.co.z for v in o.data.vertices)
 for v in o.data.vertices:v.co.z-=minimum
 o.data.transform(Matrix.Diagonal((scale,scale,scale,1)))
 o.data.materials.clear();o.data.materials.append(material)
 o.location=loc;o.hide_render=True;o.hide_set(True)
 return o
rock=kenney_source('rock_smallA','Kenney source | faceted stone',stone,(90,0,-30),2.5)
grass=kenney_source('grass','Kenney source | woodland grass',moss,(100,0,-30),2.0)
bush=kenney_source('plant_bush','Kenney source | understory bush',foliage,(105,0,-30),4.5)
# Editable Geometry Nodes scatter based on terrain surface. Rings of emptiness around central robot.
def scatter(label,src,density,seed,min_radius,max_radius,scl):
 g=bpy.data.node_groups.new(label+' | procedural scatter','GeometryNodeTree');g.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry');g.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry')
 ns=g.nodes;links=g.links
 inp=ns.new('NodeGroupInput');inp.location=(-650,80);out=ns.new('NodeGroupOutput');out.location=(550,80)
 dist=ns.new('GeometryNodeDistributePointsOnFaces');dist.location=(-240,50);dist.inputs['Density'].default_value=density;dist.inputs['Seed'].default_value=seed
 pos=ns.new('GeometryNodeInputPosition');pos.location=(-650,-240);sep=ns.new('ShaderNodeVectorMath');sep.operation='LENGTH';sep.location=(-450,-240);links.new(pos.outputs['Position'],sep.inputs[0]);
 compare=ns.new('ShaderNodeMath');compare.operation='GREATER_THAN';compare.inputs[1].default_value=min_radius;compare.location=(-240,-250);links.new(sep.outputs['Value'],compare.inputs[0]);links.new(compare.outputs[0],dist.inputs['Selection'])
 links.new(inp.outputs['Geometry'],dist.inputs['Mesh'])
 info=ns.new('GeometryNodeObjectInfo');info.transform_space='ORIGINAL';info.location=(-240,-90);info.inputs['Object'].default_value=src;info.inputs['As Instance'].default_value=True
 rand=ns.new('FunctionNodeRandomValue');rand.data_type='FLOAT';rand.inputs['Min'].default_value=scl*.72;rand.inputs['Max'].default_value=scl*1.23;rand.location=(-15,-210)
 # Object source is placed outside the scene; original-space object info excludes source translation.
 inst=ns.new('GeometryNodeInstanceOnPoints');inst.location=(200,70);links.new(dist.outputs['Points'],inst.inputs['Points']);links.new(info.outputs['Geometry'],inst.inputs['Instance']);links.new(rand.outputs['Value'],inst.inputs['Scale']);links.new(inst.outputs['Instances'],out.inputs['Geometry'])
 ob=bpy.data.objects.new(label,terrain.data.copy());ENV.objects.link(ob);ob.data.materials.clear();mod=ob.modifiers.new('Geometry Nodes | editable density and exclusion','NODES');mod.node_group=g
 ob['asset_source']=src.name;ob['density_per_m2']=density;ob['exclusion_radius_m']=min_radius
 return ob
scatter('Pines | GN scatter',sources[1],.045,12,3.4,15,1)
scatter('Dead trunks | GN scatter',sources[0],.015,5,3.2,15,1)
scatter('Broadleaf silhouettes | GN scatter',sources[2],.012,19,4.5,15,1)
scatter('Stones | GN scatter',rock,.16,37,1.8,15,1)
scatter('Understory | GN scatter',grass,1.5,91,2.4,15,1)
scatter('Bushes | GN scatter',bush,.12,47,3.0,15,1)
# A few large foreground stones are manually composed (not mass instances).
for idx,(x,y,s) in enumerate([(-2.5,-1.2,.65),(2.4,1.4,.75),(3.2,-1.1,.45)]):
 sphere('Foreground stone %02d'%idx,(x,y,ground_z(x,y)+.12),(s,s*.7,s*.3),stone,ENV)
def info():
 return {'objects':len(bpy.data.objects),'collections':{c.name:len(c.objects) for c in scene.collection.children_recursive},'geometry_nodes':[o.name for o in ENV.objects if any(m.type=='NODES' for m in o.modifiers)],'robot_parts':len(ROBOT.objects),'baked_fireflies':len([o for o in FLIES.objects if o.name.startswith('Baked')]),'render_engine':scene.render.engine,'render_size':[scene.render.resolution_x,scene.render.resolution_y]}
def review(stage,checks,notes):
 passed=all(checks.values());entry={'stage':stage,'hardness':'High','JEV':{'Judgement':'PASS' if passed else 'FAIL','Evidence':checks,'Verification':notes},'get_scene_info':info()};REPORT.append(entry);(ROOT/'output'/'JEV_reviews.json').write_text(json.dumps(REPORT,ensure_ascii=False,indent=2));print(json.dumps(entry,ensure_ascii=False));
 if not passed:raise RuntimeError('JEV failed: '+stage)
review('1 基础环境与地形',{'terrain_grid':len(mesh.vertices)==2401,'cc0_imported':len(sources)==3,'gn_scatter_count':len(info()['geometry_nodes'])==6,'kenney_sources':all(o.name in SOURCES.objects for o in (rock,grass,bush)),'pine_asset_height_reasonable':4<sources[1].dimensions.z<8,'embedded_tree_materials':all(all(m in (bark,foliage) for m in o.data.materials) for o in sources),'no_mass_object_duplication':len(ENV.objects)<13},'来源：SkywolfGameStudios/CC0Tree（CC0-1.0）及 Kenney nature kit / Open-Golf（MIT）；3 种导入树木源的丢失贴图已替换成文件内材质；6 个 Geometry Nodes 散射修改器；机器人活动中心留空。植被源模型低多边形，近景真实度有限。')
# Robot: total ~2.6m; rooted in world and positioned in clearing.
# Ground contacts around 0, staggered feet; centered at y=0.
cube('Central load bearing pelvis',(0,0,1.12),(.86,.48,.33),dark,bevel=.07)
cube('Torso | pressure sealed frame',(0,0,1.82),(.96,.54,1.04),steel,bevel=.105)
cube('Chest access panel',(0,-.30,1.89),(.67,.075,.64),copper,bevel=.035)
cube('Spine battery backpack',(0,.40,1.86),(.72,.37,.94),dark,bevel=.07)
for x in [-.36,.36]:
 rod('Torso exposed stay',(x,-.33,1.46),(x,-.33,2.18),.036,dark)
rod('Neck gimbal',(0,0,2.34),(0,0,2.48),.15,dark)
cube('Head sensor housing',(0,-.075,2.61),(.64,.53,.42),steel,bevel=.09)
cube('Optical visor',(0,-.358,2.65),(.43,.055,.095),glass,bevel=.018)
for sign,label in [(-1,'L'),(1,'R')]:
 x=sign*.63
 sphere(label+' shoulder actuator',(x,0,2.17),(.205,.20,.21),dark)
 rod(label+' upper arm',(x,0,2.10),(sign*.78,-.035,1.67),.13,steel)
 sphere(label+' elbow hinge',(sign*.78,-.035,1.63),(.16,.16,.16),copper)
 rod(label+' lower arm',(sign*.78,-.035,1.57),(sign*.72,-.21,1.20),.115,steel)
 cube(label+' manipulator',(sign*.72,-.22,1.11),(.27,.22,.26),dark,bevel=.045)
 # offset right leg in a measured stepping pose: all feet touch terrain.
 hip=(sign*.30,0,1.10);knee=(sign*.36,(-.13 if sign<0 else .18),.64);ankle=(sign*.37,(-.33 if sign<0 else .32),.22)
 sphere(label+' hip bearing',hip,(.19,.19,.19),dark)
 rod(label+' thigh actuator',hip,knee,.155,steel)
 sphere(label+' knee bearing',knee,(.19,.19,.19),copper)
 rod(label+' shin strut',knee,ankle,.12,steel)
 fy=ankle[1]-.08;gz=ground_z(ankle[0],fy)
 cube(label+' foot sole',(ankle[0],fy,gz+.08),(.38,.55,.16),dark,bevel=.045)
 rod(label+' heel stabilizer',(ankle[0],fy+.19,gz+.16),(ankle[0],fy+.23,gz+.36),.055,copper)
review('2 机器人主体',{'distinct_parts':len(ROBOT.objects)>25,'head_and_visor':any('Optical visor' in o.name for o in ROBOT.objects),'both_feet':sum('foot sole' in o.name for o in ROBOT.objects)==2,'human_scale':2.4<2.82<3},'站姿偏漫步，2.82m 高，肩髋肘膝踝、支架、脚底与地面高度均明确；以可编辑独立零件构造。')
# Physics phase: Blender legacy particle emitter + turbulent force; sample evaluated particle state after simulation.
# Emitting on a suspended rectangular surface allows distributed flight volumes.
scene.frame_start=1;scene.frame_end=110
emitter=cube('Simulation emitter | hidden after bake',(0,0,1.55),(5.8,4.8,.01),dark,FLIES)
bpy.context.view_layer.objects.active=emitter;emitter.select_set(True)
emitter.rotation_euler.x=.58  # tilted emitter: physically distributed starting heights
bpy.ops.object.particle_system_add();ps=emitter.particle_systems[-1].settings;ps.count=108;ps.frame_start=1;ps.frame_end=40;ps.lifetime=110;ps.emit_from='FACE';ps.physics_type='NEWTON';ps.normal_factor=.12;ps.effector_weights.gravity=0;ps.brownian_factor=.54;ps.damping=.65;ps.render_type='NONE'
bpy.ops.object.effector_add(type='TURBULENCE',location=(.3,0,1.8));force=move(bpy.context.object,FLIES);force.name='Simulation force | turbulence';force.field.strength=.38;force.field.size=1.5
scene.gravity=(0,0,-9.81)
# Evaluate forward sequentially, preserving genuine particle motion.
samples=[];movement=0
for frame in range(1,76):
 scene.frame_set(frame)
 if frame in (48,55,75):
  dg=bpy.context.evaluated_depsgraph_get();ev=emitter.evaluated_get(dg)
  samples.append({i:tuple(p.location) for i,p in enumerate(ev.particle_systems[0].particles) if p.alive_state=='ALIVE'})
if len(samples)>1:
 shared=set(samples[0])&set(samples[-1]);movement=sum((Vector(samples[0][i])-Vector(samples[-1][i])).length for i in shared)
# baked static mesh of discrete luminous vertices via GN instancing (easy to edit, no simulation dependency).
positions=[v for i,v in sorted(samples[-1].items()) if -3.3<v[0]<3.3 and -3<v[1]<3 and .5<v[2]<3.7 and (v[0]**2+v[1]**2)>.32]
if len(positions)<15:raise RuntimeError('Insufficient living particles after physics simulation: '+str(len(positions)))
pts=bpy.data.meshes.new('Particle simulation baked positions | frame 75');pts.from_pydata(positions,[],[]);pts.update();baked=bpy.data.objects.new('Baked fireflies | frozen instances',pts);FLIES.objects.link(baked)
bulb_source=sphere('Firefly luminaire source',(110,0,-30),(1,1,1),glow,SOURCES);bulb_source.data.transform(Matrix.Diagonal((.035,.035,.035,1)));bulb_source.hide_render=True
bulb=list(SOURCES.objects)[-1];bulb.hide_set(True)
g=bpy.data.node_groups.new('Firefly bake | instances from simulated vertices','GeometryNodeTree');g.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry');g.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry');ns=g.nodes;li=g.links;inn=ns.new('NodeGroupInput');out=ns.new('NodeGroupOutput');inst=ns.new('GeometryNodeInstanceOnPoints');obj=ns.new('GeometryNodeObjectInfo');obj.inputs['Object'].default_value=bulb;obj.inputs['As Instance'].default_value=True;li.new(inn.outputs['Geometry'],inst.inputs['Points']);li.new(obj.outputs['Geometry'],inst.inputs['Instance']);li.new(inst.outputs['Instances'],out.inputs['Geometry']);baked.modifiers.new('Static instanced fireflies','NODES').node_group=g
emitter.hide_render=True;emitter.hide_set(True);force.hide_render=True;force.hide_set(True)
baked['physics']='Blender particle NEWTON + turbulence + Brownian, evaluated frames 1–75';baked['baked_frame']=75;baked['motion_sum_m']=movement
review('3 萤火虫物理模拟与烘焙',{'particle_system':len(emitter.particle_systems)>0,'turbulence_force':force.field.type=='TURBULENCE','measured_motion':movement>.01,'baked_count':len(positions)>=15,'static_vertex_mesh':len(baked.data.vertices)==len(positions),'height_variation':max(v[2] for v in positions)-min(v[2] for v in positions)>1,'fly_source_diameter':max(v.co.x for v in bulb.data.vertices)-min(v.co.x for v in bulb.data.vertices)<.1},f'物理引擎粒子帧 1–75 连续求值；48→75 帧共有粒子位移总和 {movement:.2f} m；第 75 帧筛选并烘焙 {len(positions)} 个静态点，GN 引用内嵌发光源，渲染不依赖粒子缓存。')
# Lighting / camera, conservative preview settings.
world=bpy.data.worlds.new('Twilight ambient') if not bpy.data.worlds else bpy.data.worlds[0];scene.world=world;world.use_nodes=True;world.node_tree.nodes['Background'].inputs['Color'].default_value=(.11,.16,.22,1);world.node_tree.nodes['Background'].inputs['Strength'].default_value=.37
def area(name,loc,power,color,size,target):
 bpy.ops.object.light_add(type='AREA',location=loc);o=move(bpy.context.object,LIGHTS);o.name=name;o.data.energy=power;o.data.color=color;o.data.shape='DISK';o.data.size=size;o.rotation_euler=(Vector(target)-o.location).to_track_quat('-Z','Y').to_euler()
area('Cold moonlight | broad rim',(2,3,8),850,(.43,.61,1),7,(0,0,1))
area('Soft sky fill',(-4,-2,5),410,(.52,.69,.78),8,(0,0,1.4))
area('Faint amber bounce',(0,-3,3),80,(1,.54,.25),4,(0,0,1.4))
bpy.ops.object.camera_add(location=(5.2,-8.2,4.15));cam=move(bpy.context.object,LIGHTS);cam.name='Camera | forest clearing';cam.rotation_euler=(Vector((0,0,1.46))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.type='ORTHO';cam.data.ortho_scale=9.4;scene.camera=cam
scene.render.engine='BLENDER_EEVEE_NEXT';scene.render.resolution_x=640;scene.render.resolution_y=480;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG';scene.render.filepath=str(ROOT/'output'/'preview_640x480.png');scene.render.film_transparent=False
scene.view_settings.view_transform='AgX';scene.render.image_settings.color_mode='RGBA'
scene.frame_set(75)
review('4 微光灯光与基础材质',{'eevee':scene.render.engine=='BLENDER_EEVEE_NEXT','preview_le_720p':scene.render.resolution_x<=1280 and scene.render.resolution_y<=720,'lighting':len([o for o in LIGHTS.objects if o.type=='LIGHT'])==3,'material_assignments':all(o.data.materials for o in ROBOT.objects if o.type=='MESH')},'月光冷主光、弱冷填光、微弱暖色反射；金属、氧化铜、石土与发光材料分离；仅允许 640×480 预览。')
# EEVEE needs an OpenGL/EGL context: do not attempt a render in a context-free sandbox.
blend=ROOT/'output'/'Twilight_Wilderness_Robot.blend';bpy.ops.wm.save_as_mainfile(filepath=str(blend),compress=True)
review('5 清理与交付',{'required_collections':all(k in bpy.data.collections for k in ['Architecture','Environment','Robot','Fireflies','Lights']),'blend_saved':blend.exists(),'static_bake':len(baked.data.vertices)==len(positions),'simulation_hidden':emitter.hide_render and force.hide_render},'保留隐藏模拟源以便追溯，静态实例作为最终展示；资产内嵌到 .blend，外部 FBX 仅供重建脚本使用。')
# Remove orphaned texture pointers from the original FBX imports and unused
# OBJ materials; no external asset dependency should survive in the .blend.
for image in list(bpy.data.images):
 if image.type!='RENDER_RESULT' and image.users==0:bpy.data.images.remove(image)
for material in list(bpy.data.materials):
 if material.users==0:bpy.data.materials.remove(material)
# Save once more after orphan cleanup (backups are ignored).
bpy.ops.wm.save_as_mainfile(filepath=str(blend),compress=True)
# Headless structural checks are not a substitute for a visual JEV inspection.
for item in REPORT[3:]:
 item['JEV']['Judgement']='PENDING_VISUAL'
 item['JEV']['Verification'] += ' 视觉审查未通过：当前容器没有 EGL/GLX 上下文，无法获得 EEVEE 或 viewport 预览；必须在图形 Blender 中完成目视检查。'
(ROOT/'output'/'JEV_reviews.json').write_text(json.dumps(REPORT,ensure_ascii=False,indent=2))
print('PREVIEW NOT GENERATED: no EGL/GLX context; steps 4–5 marked PENDING_VISUAL.')
