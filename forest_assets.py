"""Import UV-preserving CC0 source meshes and their actual texture maps.
Original foliage: BFjord OriginalFoliage CC0; supplemental models: Poly Haven CC0.
No base-color remapping or generated mesh foliage is performed here.
"""
import bpy
from pathlib import Path
from mathutils import Matrix
ROOT=Path(__file__).resolve().parent

def atlas_material(name,folder,stem,cutout=False,mask=None):
    m=bpy.data.materials.get('CC0 | '+name)
    if m:return m
    m=bpy.data.materials.new('CC0 | '+name);m.use_nodes=True
    nodes=m.node_tree.nodes;nodes.clear();links=m.node_tree.links
    output=nodes.new('ShaderNodeOutputMaterial');output.location=(510,60)
    bs=nodes.new('ShaderNodeBsdfPrincipled');bs.location=(220,60)
    bs.inputs['Roughness'].default_value=.83
    links.new(bs.outputs['BSDF'],output.inputs['Surface'])
    color=nodes.new('ShaderNodeTexImage');color.location=(-480,210)
    color.image=bpy.data.images.load(str(ROOT/'assets'/folder/'Textures'/(stem+('' if folder=='bfjord' else '_BaseMap')+'.png')),check_existing=True)
    color.image.pack();links.new(color.outputs['Color'],bs.inputs['Base Color'])
    normalpath=ROOT/'assets'/folder/'Textures'/(('FoliageNormal' if stem=='FoliageAtlas' else 'WoodlandNormal' if stem=='WoodlandAtlas' else 'Woodland06Normal' if stem=='Woodland06Atlas' else stem+'_Normal')+'.png')
    if normalpath.exists():
        im=bpy.data.images.load(str(normalpath),check_existing=True);im.colorspace_settings.name='Non-Color';im.pack()
        tex=nodes.new('ShaderNodeTexImage');tex.location=(-480,-90);tex.image=im
        nm=nodes.new('ShaderNodeNormalMap');nm.location=(-40,-70);nm.inputs['Strength'].default_value=.55
        links.new(tex.outputs['Color'],nm.inputs['Color']);links.new(nm.outputs['Normal'],bs.inputs['Normal'])
    if cutout:
        # Poly Haven RGBA leaf atlases carry real coverage masks, not tinted cards.
        alpha=nodes.new('ShaderNodeMath');alpha.operation='GREATER_THAN';alpha.inputs[1].default_value=.36
        links.new(color.outputs['Alpha'],alpha.inputs[0]);links.new(alpha.outputs[0],bs.inputs['Alpha'])
        m.surface_render_method='DITHERED';m.use_transparency_overlap=False
    if mask:
        im=bpy.data.images.load(str(ROOT/'assets'/folder/'Textures'/mask),check_existing=True);im.colorspace_settings.name='Non-Color';im.pack()
        t=nodes.new('ShaderNodeTexImage');t.location=(-480,-330);t.image=im
        # Atlas mask red channel stores opacity for foliage geometry; saturate valid coverage.
        sep=nodes.new('ShaderNodeSeparateColor');links.new(t.outputs['Color'],sep.inputs['Color'])
        op=nodes.new('ShaderNodeMath');op.operation='GREATER_THAN';op.inputs[1].default_value=.06
        links.new(sep.outputs['Red'],op.inputs[0]);links.new(op.outputs[0],bs.inputs['Alpha'])
        m.surface_render_method='DITHERED';m.use_transparency_overlap=False
    return m

WOOD=atlas_material('BFjord woodland atlas','bfjord','WoodlandAtlas')
WOOD06=atlas_material('BFjord woodland 06 atlas / original B variants','bfjord','Woodland06Atlas')
FOL=atlas_material('BFjord original foliage atlas','bfjord','FoliageAtlas')
FERN=atlas_material('Poly Haven fern_02','polyhaven','fern_02',cutout=True)
SHRUB=atlas_material('Poly Haven shrub_03','polyhaven','shrub_03',cutout=True)
PINE_BARK=atlas_material('Poly Haven pine bark','polyhaven','pine_sapling_small_bark')
PINE_TWIG=atlas_material('Poly Haven pine needles','polyhaven','pine_sapling_small_twig',cutout=True)
MOSS_ROCK=atlas_material('Poly Haven mossy rock','polyhaven','rock_moss_set_01')

def import_asset(folder,filename,lod,materials,collection,name,scale=1):
    before=set(bpy.data.objects)
    bpy.ops.import_scene.fbx(filepath=str(ROOT/'assets'/folder/'Models'/(filename+'.fbx')))
    new=[o for o in bpy.data.objects if o not in before]
    options=[o for o in new if o.type=='MESH' and o.name.endswith('_LOD'+str(lod))]
    assert len(options)==1,(filename,lod,[(o.name,o.type) for o in new])
    obj=options[0]
    # FBX LOD meshes are independent source representations; avoid retaining extra LODs.
    for o in new:
        if o!=obj:bpy.data.objects.remove(o,do_unlink=True)
    for c in list(obj.users_collection):c.objects.unlink(obj)
    collection.objects.link(obj);obj.name='Source | '+name
    obj.parent=None;obj.matrix_world.identity()
    obj.data=obj.data.copy()
    # Blender resets EVERY polygon.material_index to 0 when all slots are
    # cleared. Preserve the FBX face-group indices, especially pine bark/twigs.
    old_indices=[face.material_index for face in obj.data.polygons]
    if len(materials)!=len(obj.data.materials):
        raise ValueError(f'{filename}: expected {len(obj.data.materials)} material slots, got {len(materials)}')
    for i,material in enumerate(materials):obj.data.materials[i]=material
    assert all(p.material_index==old_indices[i] for i,p in enumerate(obj.data.polygons)), filename
    if len(materials)>1:
        assert all(any(p.material_index==i for p in obj.data.polygons) for i in range(len(materials))), filename
    obj.data.transform(Matrix.Diagonal((scale,scale,scale,1)))
    bottom=min(v.co.z for v in obj.data.vertices)
    for v in obj.data.vertices:v.co.z-=bottom
    obj.location=(100+len(collection.objects)*9,0,-30)
    obj.hide_render=True;obj.hide_set(True)
    obj['license']='CC0 1.0';obj['asset_file']=str((ROOT/'assets'/folder/'Models'/(filename+'.fbx')).relative_to(ROOT))
    return obj
