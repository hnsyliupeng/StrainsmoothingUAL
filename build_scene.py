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
ENV=collection('Environment');ROBOT=collection('Robot');FLIES=collection('Fireflies');LIGHTS=collection('Lights');SOURCES=collection('Architecture / open-source asset prototypes')
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
# Actual CC0 Poly Haven Forest Floor photography-derived surface maps (not noise).
forest_color=bpy.data.images.load(str(ROOT/'assets/polyhaven/Textures/ForestLitter_Color.png'));forest_color.pack()
forest_normal=bpy.data.images.load(str(ROOT/'assets/polyhaven/Textures/ForestLitter_Normal.png'));forest_normal.colorspace_settings.name='Non-Color';forest_normal.pack()
forest_mask=bpy.data.images.load(str(ROOT/'assets/polyhaven/Textures/ForestLitter_Mask.png'));forest_mask.colorspace_settings.name='Non-Color';forest_mask.pack()
soil_nodes=soil.node_tree.nodes;soil_links=soil.node_tree.links
uv=soil_nodes.new('ShaderNodeTexCoord');mapping=soil_nodes.new('ShaderNodeVectorMath');mapping.operation='SCALE';mapping.inputs[3].default_value=14
soil_links.new(uv.outputs['Generated'],mapping.inputs[0])
color_tex=soil_nodes.new('ShaderNodeTexImage');color_tex.image=forest_color
soil_links.new(mapping.outputs['Vector'],color_tex.inputs['Vector'])
soil_links.new(color_tex.outputs['Color'],soil_nodes.get('Principled BSDF').inputs['Base Color'])
normal_tex=soil_nodes.new('ShaderNodeTexImage');normal_tex.image=forest_normal
soil_links.new(mapping.outputs['Vector'],normal_tex.inputs['Vector'])
normal_map=soil_nodes.new('ShaderNodeNormalMap');normal_map.inputs['Strength'].default_value=.65
soil_links.new(normal_tex.outputs['Color'],normal_map.inputs['Color']);soil_links.new(normal_map.outputs['Normal'],soil_nodes.get('Principled BSDF').inputs['Normal'])
steel=mat('Oxidised titanium shell',(.16,.19,.18),.64,.54);dark=mat('Graphite joints',(.043,.054,.056),.48,.71);copper=mat('Worn copper trim',(.19,.16,.115),.57,.62);glass=mat('Optic | warm cyan',(.10,.39,.37),.22,.28,1.4);glow=mat('Firefly | amber bioluminescence',(1,.68,.16),0,.3,4.0)
seal=mat('Black elastomer gasket',(.026,.029,.028),.04,.83);alloy=mat('Machined alloy',(.31,.33,.31),.8,.34);lens=mat('Sensor glass',(.032,.11,.11),.35,.18,.35)
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
# Screen-tested source assets retain the actual CC0 UV base maps and normal maps.
# No former Proton Scatter vegetation is used in this candidate.
from forest_assets import import_asset, WOOD, FOL, FERN, SHRUB, PINE_BARK, PINE_TWIG, MOSS_ROCK

def bf(filename,lod=1,scale=1):
 return import_asset('bfjord',filename,lod,[WOOD if filename in ('MatureOak_A','MatureOak_B','SilverBirch_A','SilverBirch_B','TallMeadowGrass_A') else FOL],SOURCES,filename,scale)
def ph(filename,materials,lod=1,scale=1):
 return import_asset('polyhaven',filename,lod,materials,SOURCES,filename,scale)
oak=bf('MatureOak_A');oak2=bf('MatureOak_B');birch=bf('SilverBirch_A');birch2=bf('SilverBirch_B')
log=bf('FallenHollowLog_A');grass=bf('TallMeadowGrass_A',scale=.45)
shortgrass=bf('CoastalGrass_A',scale=.4);sorrel=bf('WoodSorrel_A')
rose=bf('RoseThicket_A',scale=.68)
fern=ph('fern_02_b',[FERN]);fern2=ph('fern_02_c',[FERN]);shrub=ph('shrub_03_a',[SHRUB],scale=2.0)
pine=ph('pine_sapling_small_b',[PINE_BARK,PINE_TWIG],scale=3.8)
rock=ph('rock_moss_set_01_rock01',[MOSS_ROCK],scale=.52)
rock2=ph('rock_moss_set_01_rock03',[MOSS_ROCK],scale=.65)
sources=[oak,oak2,birch,birch2,pine,grass,shortgrass,fern,fern2,shrub,rose,sorrel,rock,rock2,log]
# Editable Geometry Nodes scatter based on terrain surface. Rings of emptiness around central robot.
def scatter(label,src,density,seed,min_radius,max_radius,scl,front_clear=False):
 g=bpy.data.node_groups.new(label+' | procedural scatter','GeometryNodeTree');g.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry');g.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry')
 ns=g.nodes;links=g.links
 inp=ns.new('NodeGroupInput');inp.location=(-850,120);out=ns.new('NodeGroupOutput');out.location=(550,80)
 dist=ns.new('GeometryNodeDistributePointsOnFaces');dist.location=(-240,50);dist.inputs['Density'].default_value=density;dist.inputs['Seed'].default_value=seed
 pos=ns.new('GeometryNodeInputPosition');pos.location=(-850,-240)
 # EXCLUSION IS EVALUATED IN 3D WORLD XY, not local terrain Z. The earlier
 # circular mask allowed a foreground trunk directly in the line of sight.
 sep=ns.new('ShaderNodeSeparateXYZ');sep.location=(-630,-250);links.new(pos.outputs['Position'],sep.inputs['Vector'])
 radius=ns.new('ShaderNodeVectorMath');radius.operation='LENGTH';radius.location=(-440,-240)
 xy=ns.new('ShaderNodeCombineXYZ');xy.location=(-640,-420);links.new(sep.outputs['X'],xy.inputs['X']);links.new(sep.outputs['Y'],xy.inputs['Y']);links.new(xy.outputs['Vector'],radius.inputs[0])
 outside=ns.new('ShaderNodeMath');outside.operation='GREATER_THAN';outside.inputs[1].default_value=min_radius;outside.location=(-240,-260);links.new(radius.outputs['Value'],outside.inputs[0])
 selection=outside.outputs[0]
 if label.startswith('Flowering'):
  inside=ns.new('ShaderNodeMath');inside.operation='LESS_THAN';inside.inputs[1].default_value=max_radius;links.new(radius.outputs['Value'],inside.inputs[0])
  annulus=ns.new('ShaderNodeMath');annulus.operation='MULTIPLY';links.new(outside.outputs[0],annulus.inputs[0]);links.new(inside.outputs[0],annulus.inputs[1]);selection=annulus.outputs[0]
 if front_clear:
  # Central camera corridor: y=-8 to +5, x=-3.8 to +3.8.
  # Preserve trees at far left/right and at the back, never center foreground.
  abx=ns.new('ShaderNodeMath');abx.operation='ABSOLUTE';abx.location=(-420,-460);links.new(sep.outputs['X'],abx.inputs[0])
  xinside=ns.new('ShaderNodeMath');xinside.operation='LESS_THAN';xinside.inputs[1].default_value=3.8;xinside.location=(-230,-460);links.new(abx.outputs[0],xinside.inputs[0])
  ylow=ns.new('ShaderNodeMath');ylow.operation='GREATER_THAN';ylow.inputs[1].default_value=-8;ylow.location=(-230,-590);links.new(sep.outputs['Y'],ylow.inputs[0])
  yhigh=ns.new('ShaderNodeMath');yhigh.operation='LESS_THAN';yhigh.inputs[1].default_value=5;yhigh.location=(-230,-710);links.new(sep.outputs['Y'],yhigh.inputs[0])
  corridor=ns.new('ShaderNodeMath');corridor.operation='MULTIPLY';corridor.location=(-10,-520);links.new(xinside.outputs[0],corridor.inputs[0]);links.new(ylow.outputs[0],corridor.inputs[1])
  between=ns.new('ShaderNodeMath');between.operation='MULTIPLY';between.location=(155,-520);links.new(corridor.outputs[0],between.inputs[0]);links.new(yhigh.outputs[0],between.inputs[1])
  not_center=ns.new('ShaderNodeMath');not_center.operation='SUBTRACT';not_center.inputs[0].default_value=1;not_center.location=(300,-520);links.new(between.outputs[0],not_center.inputs[1])
  mask=ns.new('ShaderNodeMath');mask.operation='MULTIPLY';mask.location=(350,-180);links.new(outside.outputs[0],mask.inputs[0]);links.new(not_center.outputs[0],mask.inputs[1]);selection=mask.outputs[0]
 links.new(inp.outputs['Geometry'],dist.inputs['Mesh']);links.new(selection,dist.inputs['Selection'])
 info=ns.new('GeometryNodeObjectInfo');info.transform_space='ORIGINAL';info.location=(-240,-90);info.inputs['Object'].default_value=src;info.inputs['As Instance'].default_value=True
 rand=ns.new('FunctionNodeRandomValue');rand.data_type='FLOAT';rand.inputs['Min'].default_value=scl*.72;rand.inputs['Max'].default_value=scl*1.23;rand.location=(-15,-210)
 inst=ns.new('GeometryNodeInstanceOnPoints');inst.location=(200,70);links.new(dist.outputs['Points'],inst.inputs['Points']);links.new(info.outputs['Geometry'],inst.inputs['Instance']);links.new(rand.outputs['Value'],inst.inputs['Scale']);links.new(inst.outputs['Instances'],out.inputs['Geometry'])
 ob=bpy.data.objects.new(label,terrain.data.copy());ENV.objects.link(ob);ob.data.materials.clear();mod=ob.modifiers.new('Geometry Nodes | editable density and exclusion','NODES');mod.node_group=g
 ob['asset_source']=src.name;ob['density_per_m2']=density;ob['exclusion_radius_m']=min_radius;ob['camera_corridor_mask']=front_clear
 return ob
scatter('Oaks A | GN scatter',oak,.013,12,5.8,15,1,True)
scatter('Oaks B | GN scatter',oak2,.008,42,6.5,15,.92,True)
scatter('Birches A | GN scatter',birch,.012,19,6,15,.84,True)
scatter('Birches B | GN scatter',birch2,.007,26,6,15,.88,True)
scatter('Pine saplings | GN scatter',pine,.026,5,5.5,15,1,True)
scatter('Moss rocks A | GN scatter',rock,.047,37,2.6,15,1)
scatter('Moss rocks B | GN scatter',rock2,.048,38,3,15,1)
scatter('Meadow grass | GN scatter',grass,.39,91,2.8,15,1)
scatter('Coastal grass | GN scatter',shortgrass,.33,81,2.5,15,1)
scatter('Ferns A | GN scatter',fern,.18,93,2.5,15,1)
scatter('Ferns B | GN scatter',fern2,.14,96,2.8,15,1)
scatter('Shrubs | GN scatter',shrub,.19,47,3.8,15,1)
scatter('Sorrel ground cover | GN scatter',sorrel,.19,34,2.1,15,1)
scatter('Flowering rose plants | GN scatter',rose,.085,68,1.85,8.5,1)
scatter('Fallen log | GN scatter',log,.008,35,5.5,15,1)
# Sparse foreground moss rocks are the same authored asset, not mesh primitives.
for idx,(x,y,sc) in enumerate([(-2.9,-1.0,.65),(2.7,1.7,.65)]):
 obj=bpy.data.objects.new('Foreground mossy rock %02d'%idx,rock.data);ENV.objects.link(obj);obj.location=(x,y,ground_z(x,y));obj.scale=(sc,sc,sc)
def info():
 return {'objects':len(bpy.data.objects),'collections':{c.name:len(c.objects) for c in scene.collection.children_recursive},'geometry_nodes':[o.name for o in ENV.objects if any(m.type=='NODES' for m in o.modifiers)],'robot_parts':len(ROBOT.objects),'baked_fireflies':len([o for o in FLIES.objects if o.name.startswith('Baked')]),'render_engine':scene.render.engine,'render_size':[scene.render.resolution_x,scene.render.resolution_y]}
def review(stage,checks,notes):
 passed=all(checks.values());entry={'stage':stage,'hardness':'High','JEV':{'Judgement':'DATA_ONLY' if passed else 'FAIL','Evidence':checks,'Verification':notes},'get_scene_info':info()};REPORT.append(entry);(ROOT/'output'/'JEV_reviews.json').write_text(json.dumps(REPORT,ensure_ascii=False,indent=2));print(json.dumps(entry,ensure_ascii=False));
 if not passed:raise RuntimeError('JEV failed: '+stage)
review('1 基础环境与地形',{'terrain_grid':len(mesh.vertices)==2401,'cc0_source_meshes':len(sources)==15 and all(x.data.uv_layers for x in sources),'gn_scatter_count':len(info()['geometry_nodes'])==15,'real_color_maps':forest_color.packed_file is not None and all(n.image and n.image.packed_file for m in [WOOD,FOL,FERN,SHRUB,PINE_BARK,PINE_TWIG,MOSS_ROCK] for n in m.node_tree.nodes if n.type=='TEX_IMAGE'),'original_foliage_only':all('assets/' in x['asset_file'] for x in sources)},'15 个 CC0 FBX 源资产；树木、蕨类、草、灌木、苔岩及真实开花玫瑰；15 组 GN 散射。贴图为源资产本来图像，非程序化颜色模拟。几何审查通过；视觉审查必须核对新场景的真实 EEVEE 预览。')
# Robot is now an imported measured real-servo-axis URDF mesh assembly, not
# 99 generated boxes/bolts. Its 20 authored STL visual links are editable.
from robot_asset import build_robot
urdf_robot = build_robot(ROBOT, steel, alloy)
assert urdf_robot['joints'] >= 17 and len(urdf_robot['objects']) >= 18
review('2 机器人主体', {
    'authored_urdf_visual_meshes':len(urdf_robot['objects'])>=18,
    'real_joint_hierarchy':urdf_robot['joints']>=17,
    'imported_detail_vertices':urdf_robot['vertices']>40000,
    'no_generated_box_body':all(o.name.startswith('URDF |') for o in ROBOT.objects),
}, '机械主体来自 MIT 授权 UBTECH Alpha 1S 数字孪生 17关节 URDF（人工测量舵机轴），20 个导入视觉网格；仍需低分辨率目视检查站姿、尺寸与地面接触。')
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
for screen in bpy.data.screens:
 for area_view in screen.areas:
  if area_view.type=='VIEW_3D':
   area_view.spaces.active.shading.type='MATERIAL'
   area_view.spaces.active.overlay.show_overlays=False
   area_view.spaces.active.region_3d.view_perspective='CAMERA'
review('4 微光灯光与基础材质',{'eevee':scene.render.engine=='BLENDER_EEVEE_NEXT','preview_le_720p':scene.render.resolution_x<=1280 and scene.render.resolution_y<=720,'default_viewport_material_mode':all(a.spaces.active.shading.type=='MATERIAL' for sc in bpy.data.screens for a in sc.areas if a.type=='VIEW_3D'),'lighting':len([o for o in LIGHTS.objects if o.type=='LIGHT'])==3,'material_assignments':all(o.data.materials for o in ROBOT.objects if o.type=='MESH')},'月光冷主光、弱冷填光、微弱暖色反射；金属、氧化铜、石土与发光材料分离；仅允许 640×480 预览。')
# EEVEE needs an OpenGL/EGL context: do not attempt a render in a context-free sandbox.
blend=ROOT/'output'/'Twilight_Wilderness_Robot.blend';bpy.ops.wm.save_as_mainfile(filepath=str(blend),compress=True)
review('5 清理与交付',{'required_collections':all(k in bpy.data.collections for k in ['Architecture','Environment','Robot','Fireflies','Lights']),'blend_saved':blend.exists(),'static_bake':len(baked.data.vertices)==len(positions),'simulation_hidden':emitter.hide_render and force.hide_render},'保留隐藏模拟源以便追溯，静态实例作为最终展示；资产内嵌到 .blend，外部 FBX 仅供重建脚本使用。')
# Remove orphaned imported data; every active image is packed into .blend.
for image in list(bpy.data.images):
 if image.type!='RENDER_RESULT' and image.users==0:bpy.data.images.remove(image)
for material in list(bpy.data.materials):
 if material.users==0:bpy.data.materials.remove(material)
# Save once more after orphan cleanup (backups are ignored).
bpy.ops.wm.save_as_mainfile(filepath=str(blend),compress=True)
# Both user screenshots invalidate earlier visual acceptance.
# Status is FAIL_VISUAL_HISTORY for the known broken old scene and
# PENDING_VISUAL for this new candidate; do not conflate automated counts with PASS.
for item in REPORT:
 item['JEV']['Judgement']='PENDING_VISUAL'
 item['JEV']['Verification'] += ' 自动几何断言不是视觉验收；用户截图否定旧版场景。本次候选版本未生成 EEVEE 低分辨率预览，不能声称通过。'
 item['JEV']['Evidence']['new_visual_evidence_available']=False
(ROOT/'output'/'JEV_reviews.json').write_text(json.dumps(REPORT,ensure_ascii=False,indent=2))
print('PREVIEW NOT GENERATED: no EGL/GLX context; all candidate stages PENDING_VISUAL.')
