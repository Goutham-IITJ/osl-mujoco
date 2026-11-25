"""Build the upright, distal-fixed knee fixture without changing full-leg models.

Reuses CAD segmentation and mass reconstruction from build_mjcf. The fixed
housing visuals use its existing knee-module grouping. No hardware control.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from build_mjcf import OslModel, KNEE_POS, fmt, mat_to_quat

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'models/osl_v2_knee_fixture.xml'


def build():
    source = ROOT / 'models/osl_v2_bench.xml'
    tree = ET.parse(source)
    root = tree.getroot()
    root.set('model', 'osl_v2_knee_fixture_provisional')
    cad = OslModel()
    old_world = root.find('worldbody')
    old_upper = old_world.find("body[@name='knee_prox']")
    old_lower = old_upper.find("body[@name='shank']")
    joint = deepcopy(old_lower.find("joint[@name='knee']"))
    # Reversing parent and child reverses the axis for the SAME relative angle.
    joint.set('axis', '0 1 0')
    root.remove(old_world)
    for tag in ('actuator', 'sensor', 'keyframe', 'contact', 'equality'):
        item = root.find(tag)
        if item is not None:
            root.remove(item)
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 0 -1')
    fixed = ET.SubElement(world, 'body', name='knee_housing_fixed')
    meshes = {m.get('file'): m.get('name') for m in root.find('asset').findall('mesh')}
    fixed_links = dict(cad.modules['shank'])['knee_module']
    for link in fixed_links:
        p, rotation = cad.geom_pose(link)
        part = cad.cad.links[link]
        ET.SubElement(fixed, 'geom', name='fixed_' + link, attrib={
            'class': 'visual', 'mesh': meshes[part.mesh_file],
            'pos': fmt(p), 'quat': fmt(mat_to_quat(rotation)),
            'rgba': fmt(part.rgba[:3]) + ' 1'})
    # No fixed-body inertia is needed: the fixture is rigidly anchored to world.
    upper = ET.SubElement(fixed, 'body', name='knee_prox', pos=fmt(KNEE_POS))
    upper.append(joint)
    for child in old_upper:
        if child.tag in ('inertial', 'geom', 'site'):
            item = deepcopy(child)
            item.set('pos', fmt(np.fromstring(item.get('pos', '0 0 0'), sep=' ') - KNEE_POS))
            upper.append(item)
    actuator = ET.SubElement(root, 'actuator')
    ET.SubElement(actuator, 'motor', name='knee_torque', joint='knee', gear='1')
    sensor = ET.SubElement(root, 'sensor')
    ET.SubElement(sensor, 'jointpos', name='knee_q', joint='knee')
    ET.SubElement(sensor, 'jointvel', name='knee_qd', joint='knee')
    used = {g.get('mesh') for g in world.iter('geom') if g.get('mesh')}
    for mesh in list(root.find('asset').findall('mesh')):
        if mesh.get('name') not in used:
            root.find('asset').remove(mesh)
    ET.indent(tree, space='  ')
    tree.write(OUTPUT, encoding='unicode')
    manifest = {
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'fixed': 'distal knee housing; lower plate attached to desk',
        'moving': 'CAD knee_prox cluster, including upper thigh attachment/output pulley',
        'orientation': 'upright, CAD +Z proximal; not inverted',
        'axis': '+Y, reversed from distal-moving model to preserve relative knee angle',
        'moving_mass_kg': float(cad.inertia['knee_prox']['mass']),
        'fixed_visual_links': fixed_links,
        'unverified': [
            'Physical moving mass/inertia and exact CAD-to-hardware component match',
            'Fixed housing visuals use CAD module grouping; clamps/table/cables omitted',
            'Joint armature 0.01, damping 0.3, frictionloss 0.4 inherited, not measured',
            'Standalone torque-input plant; current drivetrain/live runtime not selected by this model',
            'Encoder polarity must be confirmed against observed rotation',
        ],
    }
    OUTPUT.with_suffix('.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(OUTPUT)
    return OUTPUT


if __name__ == '__main__':
    build()
