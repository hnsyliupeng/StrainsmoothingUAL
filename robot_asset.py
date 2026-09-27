"""Import an actual 17-joint Alpha 1S URDF visual assembly, not box primitives.
Source: andresjjn/alpha1s-ros2-twin, MIT license. Visual STL meshes retained
as individually editable objects, with joint ancestry recorded as custom props.
"""
from pathlib import Path
from xml.etree import ElementTree as ET
import math
import bpy
import xacro
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parent
ASSET = ROOT / 'assets' / 'alpha1s'


def xyz(text):
    return tuple(float(v) for v in (text or '0 0 0').split())


def origin(node):
    if node is None:
        return Matrix.Identity(4)
    a, b, c = xyz(node.get('rpy'))
    trans = Matrix.Translation(Vector(xyz(node.get('xyz'))))
    return trans @ Matrix.Rotation(c, 4, 'Z') @ Matrix.Rotation(b, 4, 'Y') @ Matrix.Rotation(a, 4, 'X')


def build_robot(collection, material_body, material_joint):
    path = ASSET / 'alpha1s.urdf.xacro'
    assert path.exists(), 'real articulated asset unavailable'
    urdf = ET.fromstring(xacro.process_file(str(path)).toxml())
    links = {item.get('name'): item for item in urdf.findall('link')}
    joints = {}
    for item in urdf.findall('joint'):
        child = item.find('child').get('link')
        joints[child] = item
    cache = {}
    # A measured servo-axis pose; mild asymmetry rather than assembling pieces
    # out of relationship. Preserve the URDF joint axes and limit ranges.
    pose = {
        'l_shoulder_joint': -0.10, 'r_shoulder_joint': 0.10,
        'l_arm_joint': 0.95, 'r_arm_joint': -0.95,
        'l_elbow_joint': 0.45, 'r_elbow_joint': -0.45,
        'l_hip_pitch_joint': 0.10, 'r_hip_pitch_joint': -0.06,
        'l_knee_joint': -0.12, 'r_knee_joint': 0.08,
        'l_ankle_pitch_joint_joint': 0.04,
        'r_ankle_pitch_joint_joint': -0.03,
    }

    def link_transform(name):
        if name in cache:
            return cache[name]
        if name not in joints:
            cache[name] = Matrix.Identity(4)
            return cache[name]
        j = joints[name]
        parent = j.find('parent').get('link')
        mat = link_transform(parent) @ origin(j.find('origin'))
        if j.get('type') == 'revolute':
            axis = Vector(xyz(j.find('axis').get('xyz')))
            angle = pose.get(j.get('name'), 0.0)
            mat = mat @ Matrix.Rotation(angle, 4, axis)
        cache[name] = mat
        return mat

    objects = []
    for name, link in links.items():
        visual = link.find('visual')
        if visual is None:
            continue
        mesh = visual.find('geometry/mesh')
        if mesh is None:
            continue
        stl = ASSET / 'visual' / (name + '.stl')
        if not stl.exists():
            raise RuntimeError('Missing authentic robot link mesh: ' + str(stl))
        before = set(bpy.data.objects)
        bpy.ops.wm.stl_import(filepath=str(stl))
        imported = [o for o in bpy.data.objects if o not in before and o.type == 'MESH']
        if len(imported) != 1:
            raise RuntimeError('Bad robot link asset: ' + name)
        obj = imported[0]
        for c in list(obj.users_collection):
            c.objects.unlink(obj)
        collection.objects.link(obj)
        obj.name = 'URDF | ' + name
        obj.matrix_world = link_transform(name) @ origin(visual.find('origin'))
        obj.data.materials.clear()
        obj.data.materials.append(material_joint if any(
            bit in name for bit in ('ankle', 'hip', 'shoulder', 'knee', 'forearm')) else material_body)
        obj['asset_source'] = 'andresjjn/alpha1s-ros2-twin, MIT'
        obj['urdf_link'] = name
        obj['parent_joint'] = joints[name].get('name') if name in joints else 'world'
        obj['geometry_type'] = 'imported visual STL; not procedural box'
        objects.append(obj)
    # Scale complete measured assembly, not individual segments. Put the lowest
    # imported vertex on the measured ground surface in the robot clearing.
    scale = 5.1
    assembly = Matrix.Diagonal((scale, scale, scale, 1))
    for obj in objects:
        obj.matrix_world = assembly @ obj.matrix_world
    zmin = min((obj.matrix_world @ Vector(corner)).z for obj in objects for corner in obj.bound_box)
    root = Matrix.Translation(Vector((0, 0, -zmin)))
    for obj in objects:
        obj.matrix_world = root @ obj.matrix_world
    vertices = sum(len(o.data.vertices) for o in objects)
    return {'objects': objects, 'joints': len(urdf.findall('joint')), 'vertices': vertices,
            'source': 'andresjjn/alpha1s-ros2-twin (MIT)', 'scale': scale}
