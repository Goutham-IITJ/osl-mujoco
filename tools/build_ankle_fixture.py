"""Provisional ankle-only fixture: fixed housing, distal output rotating.

Two explicit loads can be previewed pending lab mounting confirmation. Does
not select a plant for any production experiment or change existing models.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from build_mjcf import OslModel, ANKLE_POS, AXIS_CLUSTER_RADIUS, fmt, mat_to_quat

ROOT=Path(__file__).resolve().parents[1]


def build(load='output-only'):
    if load not in ('output-only','with-foot'):
        raise ValueError('Unknown load')
    source=ROOT/'models/osl_v2_bench.xml'
    tree=ET.parse(source);root=tree.getroot()
    cad=OslModel()
    moving=set(cad.kept['foot']+cad.culled['foot'])
    if load=='output-only':
        # Reuse the original CAD segmentation's ankle-concentric cluster rule.
        # This avoids retaining an artificial fraction of the complete foot inertia.
        moving=set()
        for members in cad.cad.rigid_clusters().values():
            mesh=[n for n in members if cad.cad.links[n].mesh_file]
            volume=0.;center=np.zeros(3)
            for name in mesh:
                mp=cad.cad.mass_props(cad.cad.links[name].mesh_file)
                p,r=cad.geom_pose(name)
                volume+=mp.volume;center+=mp.volume*(p+r@mp.centroid)
            if volume:
                center/=volume
                if np.hypot(center[0]-ANKLE_POS[0],center[2]-ANKLE_POS[2])<AXIS_CLUSTER_RADIUS:
                    moving.update(n for n in mesh if cad.seg_of_link[n]=='foot')
        if not moving:raise ValueError('No rotating output clusters found')
        cad.kept['foot']=[n for n in cad.kept['foot'] if n in moving]
        cad.culled['foot']=[n for n in cad.culled['foot'] if n in moving]
        cad.inertia['foot']=cad._segment_inertia('foot')
    old_world=root.find('worldbody')
    old_foot=old_world.find(".//body[@name='foot']")
    joint=deepcopy(old_foot.find('joint'))
    root.remove(old_world)
    for tag in ('actuator','sensor','keyframe','contact','equality'):
        node=root.find(tag)
        if node is not None:root.remove(node)
    root.set('model','osl_v2_ankle_fixture_'+load+'_provisional')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 -1 2',dir='0 0 -1')
    # Translate original ankle axis to z=0; preserve CAD +Z proximal, -Z distal.
    fixed=ET.SubElement(world,'body',name='ankle_housing_fixed',pos=fmt(-ANKLE_POS))
    mesh_names={m.get('file'):m.get('name') for m in root.find('asset').findall('mesh')}
    fixed_links=dict(cad.modules['shank'])['ankle_module']
    def geom(parent,name):
        part=cad.cad.links[name];p,r=cad.geom_pose(name)
        ET.SubElement(parent,'geom',name=name,attrib={'class':'visual',
            'mesh':mesh_names[part.mesh_file],'pos':fmt(p),'quat':fmt(mat_to_quat(r)),
            'rgba':fmt(part.rgba[:3])+' 1'})
    for name in fixed_links:geom(fixed,name)
    output=ET.SubElement(fixed,'body',name='ankle_output',pos=fmt(ANKLE_POS))
    output.append(joint)
    output.append(ET.fromstring(cad._inertial('foot',ANKLE_POS)))
    for name in cad.kept['foot']:
        geom(output,name)
        output[-1].set('pos',fmt(np.fromstring(output[-1].get('pos'),sep=' ')-ANKLE_POS))
    act=ET.SubElement(root,'actuator')
    ET.SubElement(act,'motor',name='ankle_torque',joint='ankle',gear='1')
    sensor=ET.SubElement(root,'sensor')
    for kind,name in [('jointpos','ankle_q'),('jointvel','ankle_qd')]:
        ET.SubElement(sensor,kind,name=name,joint='ankle')
    used={g.get('mesh') for g in world.iter('geom') if g.get('mesh')}
    for node in list(root.find('asset').findall('mesh')):
        if node.get('name') not in used:root.find('asset').remove(node)
    path=ROOT/'models'/('osl_v2_ankle_fixture_'+load.replace('-','_')+'.xml')
    ET.indent(tree,space='  ');tree.write(path,encoding='unicode')
    path.with_suffix('.json').write_text(json.dumps(dict(
        provisional=True,load=load,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        fixed='CAD ankle housing/module, assumed proximal clamp',
        orientation='CAD +Z proximal / -Z distal; ankle hinge translated to world origin; no inversion',
        moving_links=sorted(moving),moving_mass_kg=float(cad.inertia['foot']['mass']),
        moving_selection='Complete foot segment' if load=='with-foot' else 'Only ankle-concentric rigid output clusters; excludes blade and distal adapter clusters',
        unresolved=['Clamp and moving side require hardware confirmation','Attached output load and encoder zero/polarity unknown',
                    'Armature/damping/friction inherited, not measured','No controller/drivetrain comparison run for this fixture'],
    ),indent=2)+'\n')
    print(path)
    return path


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--load',choices=('output-only','with-foot'),default='output-only')
    build(p.parse_args().load)
