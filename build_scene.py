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
# Subtle microstructure; physically plausible surface breakup without mesh noise.
ns=soil.node_tree.nodes;ln=soil.node_tree.links;noise=ns.new('ShaderNodeTexNoise');noise.inputs['Scale'].default_value=15;noise.inputs['Detail'].default_value=3;bump=ns.new('ShaderNodeBump');bump.inputs['Strength'].default_value=.12;bump.inputs['Distance'].default_value=.065;ln.new(noise.outputs['Fac'],bump.inputs['Height']);ln.new(bump.outputs['Normal'],ns.get('Principled BSDF').inputs['Normal'])
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
# Open-source MIT vegetation and rock prototypes imported ONCE; geometry nodes
# scatter linked instances of these full UV-textured models. No synthetic cones.
def textured(name, texture, rough=.87, shade=(.64,.76,.66)):
 m=mat(name,(.16,.2,.14),rough=rough);n=m.node_tree.nodes;l=m.node_tree.links
 im=bpy.data.images.load(str(ROOT/'assets'/'proton_scatter'/texture),check_existing=True)
 im.pack() # the .blend is portable without external image paths
 tex=n.new('ShaderNodeTexImage');tex.image=im;tex.interpolation='Linear'
 tint=n.new('ShaderNodeMixRGB');tint.blend_type='MULTIPLY';tint.inputs[0].default_value=1;tint.inputs[2].default_value=(*shade,1)
 l.new(tex.outputs['Color'],tint.inputs[1]);l.new(tint.outputs['Color'],n.get('Principled BSDF').inputs['Base Color'])
 if im.channels==4:
  l.new(tex.outputs['Alpha'],n.get('Principled BSDF').inputs['Alpha'])
  m.surface_render_method='DITHERED';m.use_transparency_overlap=False
 # Material Preview and Solid(color=MATERIAL) both get legible fallback color.
 m.diffuse_color=(*shade,1)
 return m
needles=textured('Pine | photographic alpha branch','t_pine_branch.png',shade=(.47,.63,.48))
leaves=textured('Broadleaf | photographic alpha','t_leaves_1.png',shade=(.54,.65,.52))
grassmat=textured('Grass | alpha blade','t_grass.png',shade=(.5,.62,.48))
bushmat=textured('Bush | alpha foliage','t_bush.png',shade=(.52,.63,.51))
barkmat=textured('Bark | photographic','t_tree_bark.png',shade=(.60,.58,.49))
rockmat=textured('Stone | photographic','t_rock.jpg',shade=(.27,.29,.28));rockmat.diffuse_color=(.20,.23,.22,1)
def asset_source(file,name,scale,material_by_piece):
 before=set(bpy.data.objects);bpy.ops.import_scene.gltf(filepath=str(ROOT/'assets'/'proton_scatter'/(file+'.glb')))
 imported=[o for o in bpy.data.objects if o not in before and o.type=='MESH']
 if not imported:raise RuntimeError('Missing imported mesh: '+file)
 # Keep UV islands and individual meshes separately inside a collection instanced
 # through Geometry Nodes; join for predictable coordinate alignment.
 for o in imported:
  bpy.context.view_layer.objects.active=o
  o.select_set(True)
  bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM') if o.parent else None
  bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
  o.data.materials.clear();o.data.materials.append(material_by_piece.get(o.name, list(material_by_piece.values())[0]))
  o.select_set(False)
 bpy.ops.object.select_all(action='DESELECT')
 for o in imported:o.select_set(True)
 bpy.context.view_layer.objects.active=imported[0]
 if len(imported)>1:bpy.ops.object.join()
 o=move(bpy.context.object,SOURCES);o.name=name
 o.data.transform(Matrix.Diagonal((scale,scale,scale,1)))
 minimum=min(v.co.z for v in o.data.vertices)
 for v in o.data.vertices:v.co.z-=minimum
 o.location=(80+len(SOURCES.objects)*9,0,-30)
 o.hide_render=True;o.hide_set(True)
 return o
pine=asset_source('pine_tree','Source | textured pine',1.8,{'Leaves':needles,'Trunk':barkmat})
broad=asset_source('tree','Source | textured deciduous',.96,{'Leaves1':leaves,'Trunk':barkmat})
# The deadwood is a separate original open-source pine trunk mesh, not a mass
# of coded trees. Extract from the pine prototype once, scatter that one mesh.
trunk_idx=next((i for i,m in enumerate(pine.data.materials) if m==barkmat),None)
if trunk_idx is None:raise RuntimeError('Pine bark material lost during import')
import bmesh
bm=bmesh.new();bm.from_mesh(pine.data);bm.faces.ensure_lookup_table()
for face in list(bm.faces):
 if face.material_index!=trunk_idx:bm.faces.remove(face)
deadmesh=bpy.data.meshes.new('Imported pine trunk subset');bm.to_mesh(deadmesh);bm.free()
for polygon in deadmesh.polygons:polygon.material_index=0
# bmesh removal can leave unused foliage vertices/edges; remove them so deadwood
# never appears as a black wireframe in solid/selection viewport.
bm=bmesh.new();bm.from_mesh(deadmesh)
for edge in list(bm.edges):
 if not edge.link_faces:bm.edges.remove(edge)
for vert in list(bm.verts):
 if not vert.link_faces:bm.verts.remove(vert)
bm.to_mesh(deadmesh);bm.free();deadmesh.update()
dead=bpy.data.objects.new('Source | standing dead pine',deadmesh);SOURCES.objects.link(dead);deadmesh.materials.append(barkmat);dead.location=(150,0,-30);dead.hide_render=True;dead.hide_set(True)
rock=asset_source('large_rock','Source | textured natural rock',.23,{'LargeRock':rockmat})
grass=asset_source('grass','Source | textured ground grass',.58,{'Plane.011':grassmat})
bush=asset_source('bush','Source | textured understory bush',.85,{'Bush':bushmat})
# Original LGPL flower model: a small redbud bloom source used as a controlled,
# visibly flowering understory accent, never copied thousands of times.
flowerpink=mat('Redbud blossom | muted dusky pink',(.44,.23,.31),rough=.8)
before=set(bpy.data.objects);bpy.ops.wm.obj_import(filepath=str(ROOT/'assets'/'proton_scatter'/'RedbudFlower.obj'))
flowers=[o for o in bpy.data.objects if o not in before and o.type=='MESH']
assert len(flowers)==1
flower=move(flowers[0],SOURCES);flower.name='Source | redbud blossom'
flower.data.materials.clear();flower.data.materials.append(flowerpink)
flower.data.transform(Matrix.Diagonal((.18,.18,.18,1)))
minimum=min(v.co.z for v in flower.data.vertices)
for v in flower.data.vertices:v.co.z-=minimum
flower.location=(160,0,-30);flower.hide_render=True;flower.hide_set(True)
sources=[pine,dead,broad]
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
scatter('Pines | GN scatter',pine,.032,12,5.8,15,1,True)
scatter('Dead trunks | GN scatter',dead,.006,5,5,15,1,True)
scatter('Deciduous | GN scatter',broad,.009,19,6,15,1,True)
scatter('Stones | GN scatter',rock,.085,37,2.6,15,1)
scatter('Understory | GN scatter',grass,1.1,91,2.8,15,1)
scatter('Bushes | GN scatter',bush,.07,47,4.2,15,1)
scatter('Flowering understory | GN scatter',flower,.014,68,3.0,15,1)
# A few large foreground stones are manually composed (not mass instances).
# Foreground stones come from the same imported rock source, not a primitive.
for idx,(x,y,sc) in enumerate([(-2.9,-1.0,.7),(2.7,1.7,.6),(3.5,-1.1,.45)]):
 obj=bpy.data.objects.new('Foreground stone %02d'%idx,rock.data);ENV.objects.link(obj);obj.location=(x,y,ground_z(x,y));obj.scale=(sc,sc,sc)
def info():
 return {'objects':len(bpy.data.objects),'collections':{c.name:len(c.objects) for c in scene.collection.children_recursive},'geometry_nodes':[o.name for o in ENV.objects if any(m.type=='NODES' for m in o.modifiers)],'robot_parts':len(ROBOT.objects),'baked_fireflies':len([o for o in FLIES.objects if o.name.startswith('Baked')]),'render_engine':scene.render.engine,'render_size':[scene.render.resolution_x,scene.render.resolution_y]}
def review(stage,checks,notes):
 passed=all(checks.values());entry={'stage':stage,'hardness':'High','JEV':{'Judgement':'PASS' if passed else 'FAIL','Evidence':checks,'Verification':notes},'get_scene_info':info()};REPORT.append(entry);(ROOT/'output'/'JEV_reviews.json').write_text(json.dumps(REPORT,ensure_ascii=False,indent=2));print(json.dumps(entry,ensure_ascii=False));
 if not passed:raise RuntimeError('JEV failed: '+stage)
review('1 基础环境与地形',{'terrain_grid':len(mesh.vertices)==2401,'open_source_textured_sources':len(sources)==3 and all(x.data.uv_layers for x in (pine,broad,grass,bush)),'gn_scatter_count':len(info()['geometry_nodes'])==7,'pine_has_needles_and_bark':len(pine.data.materials)>=2 and len(pine.data.polygons)>1200,'no_missing_texture':all(i.packed_file for i in bpy.data.images if i.source=='FILE'),'no_mass_object_duplication':len(ENV.objects)<14},'用户提供截图显示旧环境大块多边形树冠/草/石，原步骤1审查作废。现用 HungryProton/scatter MIT UV 树木、针叶 alpha 纹理、地被及岩石源资产，7 GN 散射（包含小规模真实花模型）；当前仅数据审查，缺少真实 EEVEE 画面故视觉审查仍待完成。')
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
# Additional fabricated mechanics: circumferential bearings and bolts, paired
# hydraulic struts, layered plates and visible wire routing. These are robot
# structural components (not forest repetition), individually editable.
def bolt(name,xyz,r=.026):
 bpy.ops.mesh.primitive_cylinder_add(vertices=6,radius=r,depth=.016,location=xyz,rotation=(math.pi/2,0,0))
 o=move(bpy.context.object,ROBOT);o.name=name;o.data.materials.append(alloy)
def cable(name,coords,radius=.013,material=seal):
 curve=bpy.data.curves.new(name,'CURVE');curve.dimensions='3D';curve.resolution_u=16;curve.bevel_depth=radius;curve.bevel_resolution=3
 sp=curve.splines.new('BEZIER');sp.bezier_points.add(len(coords)-1)
 for p,co in zip(sp.bezier_points,coords):p.co=co;p.handle_left_type='AUTO';p.handle_right_type='AUTO'
 obj=bpy.data.objects.new(name,curve);ROBOT.objects.link(obj);curve.materials.append(material);return obj
# Continuous load path: shoulder gearbox -> keyed upper arm -> elbow bearing
# -> rod and pneumatic line -> wrist. No unsupported decorative pieces.
for sign,label in [(-1,'L'),(1,'R')]:
 sx=sign*.62;ex=sign*.78
 for z in [2.15,1.63,.65,1.08]:
  x= sx if z>2 else ex if z>1.3 else sign*.36 if z<.8 else sign*.3
  y=0 if z>2 else -.035 if z>1.3 else (-.13 if sign<0 else .18) if z<.8 else 0
  rod(label+' bearing axle %0.2f'%z,(x-.17,y,z),(x+.17,y,z),.064,alloy,verts=20)
  rod(label+' bearing endcap %0.2f'%z,(x+sign*.145,y,z),(x+sign*.185,y,z),.104,dark,verts=20)
  bolt(label+' bearing fastener %0.2f'%z,(x+sign*.187,y,z),.032)
 cube(label+' upper arm side shield',(sign*.72,-.15,1.89),(.23,.11,.40),steel,bevel=.032)
 cube(label+' forearm outer frame',(sign*.76,-.14,1.39),(.21,.14,.33),steel,bevel=.028)
 cable(label+' shoulder to elbow data harness',[(sx,-.20,2.20),(sign*.72,-.23,2.0),(ex,-.22,1.73),(ex,-.12,1.60)],.014,seal)
 rod(label+' leg piston sleeve',(sign*.31,-.22,.98),(sign*.34,-.29,.72),.061,dark)
 rod(label+' leg piston bright rod',(sign*.34,-.29,.74),(sign*.35,-.27,.58),.036,alloy)
 cube(label+' shin face armour',(sign*.38,-.24 if sign<0 else .22,.43),(.235,.12,.39),steel,bevel=.035)
 cable(label+' hip-to-ankle cable',[(sign*.32,.16,1.07),(sign*.38,.25,.85),(sign*.43,.12,.66),(sign*.43,-.08 if sign<0 else .43,.32)],.013,dark)
 foot=next(o for o in ROBOT.objects if o.name==label+' foot sole');fz=foot.location.z
 cube(label+' ankle gimbal',(foot.location.x,foot.location.y+.09,fz+.16),(.22,.22,.19),alloy,bevel=.035)
 cube(label+' articulated toe cap',(foot.location.x,foot.location.y-.20,fz+.06),(.37,.19,.12),steel,bevel=.025)
 for k in [-1,1]:
  bolt(label+' toe fastener '+str(k),(foot.location.x+k*.12,foot.location.y-.25,fz+.11),.019)
# Body: layered chassis, external reinforcing ribs, cooled battery, optical
# cluster and removable access hatches. Keep proportions and face direction.
cube('Chest panel gasket',(0,-.338,1.89),(.78,.028,.74),seal,bevel=.015)
cube('Weathered service hatch',(0,-.36,1.90),(.65,.035,.58),steel,bevel=.03)
for xx in (-.24,.24):
 for zz in (1.68,2.12):bolt('Service hatch fastener %+.2f %.2f'%(xx,zz),(xx,-.388,zz),.021)
cube('Offset serial recess',(.02,-.395,1.98),(.26,.012,.055),copper,bevel=.009)
for x in (-.41,.41):
 cube('Ribbed torso side rail %+.2f'%x,(x,-.31,1.86),(.075,.10,.83),alloy,bevel=.024)
 cable('Spine battery umbilical %+.2f'%x,[(x,.52,2.18),(x,.61,1.94),(x,.59,1.72),(x,.35,1.57)],.024,seal)
for z in (1.57,1.77,1.97,2.17):
 cube('Battery cooling vane %.2f'%z,(0,.618,z),(.63,.055,.045),alloy,bevel=.012)
rod('Optic barrel L',(-.17,-.385,2.65),(-.17,-.43,2.65),.075,dark)
rod('Optic barrel R',(.17,-.385,2.65),(.17,-.43,2.65),.075,dark)
sphere('Optic aperture L',(-.17,-.44,2.65),(.047,.019,.047),lens)
sphere('Optic aperture R',(.17,-.44,2.65),(.047,.019,.047),lens)
cube('Top ranging module',(0,-.08,2.88),(.24,.3,.095),dark,bevel=.028)
for x in (-.16,.16):
 bolt('Head back service fastener %+.2f'%x,(x,.196,2.71),.024)
# Custom labels make structural validation refer to actual components.
review('2 机器人主体',{'multi_part_actuation':len(ROBOT.objects)>95,'head_and_optics':all(n in ROBOT.objects for n in ('Optic aperture L','Optic aperture R','Top ranging module')),'bilateral_pistons':all(n in ROBOT.objects for n in ('L leg piston sleeve','R leg piston sleeve')),'power_data_harnesses':sum(o.type=='CURVE' for o in ROBOT.objects)>=6,'both_feet':sum('foot sole' in o.name for o in ROBOT.objects)==2},'用户截图表明旧31件方块机器人欠缺机械结构：旧步骤2视觉验收失败。本候选版补齐外露轴承、连杆活塞、线束、护板、足部可动趾、检修舱、双目传感器及腰背电池；仍需真实预览确认尺度和干涉。')
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
# A user-provided screenshot invalidated the previous visual acceptance.
# All new candidate steps require a fresh real EEVEE/viewport visual review.
for item in REPORT:
 item['JEV']['Judgement']='PENDING_VISUAL'
 item['JEV']['Verification'] += ' 此处 PASS 曾仅代表自动几何数据断言：由于旧版用户截图已经证明仅凭这些断言会错判，候选版本仍未获得新图像/视口审查，不能视为通过。'
 item['JEV']['Evidence']['new_visual_evidence_available']=False
(ROOT/'output'/'JEV_reviews.json').write_text(json.dumps(REPORT,ensure_ascii=False,indent=2))
print('PREVIEW NOT GENERATED: no EGL/GLX context; all candidate stages PENDING_VISUAL.')
