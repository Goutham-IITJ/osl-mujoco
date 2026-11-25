"""Cross-check mass arithmetic without editing the CAD or MJCF files."""

import argparse
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from build_mjcf import OslModel, ROOT, URDF, MOTOR_MASS
from oslcad import density_for, mesh_mass_props


def verify_cuboid():
    size = np.array([.1, .2, .3])
    offset = np.array([.23, -.17, .42])
    vertices = np.array([
        [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
        [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
    ]) * size + offset
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    triangles = vertices[[indices for a, b, c, d in faces for indices in ((a, b, c), (a, c, d))]]
    volume = float(np.prod(size))
    inertia = np.diag(volume / 12 * ((size ** 2).sum() - size ** 2))
    for test in (triangles, triangles[:, ::-1]):
        props = mesh_mass_props(test)
        assert props.closed
        np.testing.assert_allclose(props.volume, volume, rtol=1e-12)
        np.testing.assert_allclose(props.centroid, offset + size / 2, atol=1e-12)
        np.testing.assert_allclose(props.inertia, inertia, atol=1e-12)
    return {"size_m": size.tolist(), "volume_m3": volume, "global_reversed_winding_passed": True}


def surface_volume(triangles):
    """Integrate face centroid · outward area vector / 3 about a nearby origin."""
    shifted = triangles - triangles[0, 0]
    normal_twice_area = np.cross(shifted[:, 1] - shifted[:, 0], shifted[:, 2] - shifted[:, 0])
    return abs(math.fsum(np.einsum("ij,ij->i", shifted.mean(axis=1), normal_twice_area)) / 6)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", required=True, type=Path)
    out = parser.parse_args().outdir
    out.mkdir(parents=True, exist_ok=False)
    box = verify_cuboid()
    assembly = OslModel()
    cad = assembly.cad
    mesh_names = {cad.links[part].mesh_file for part in assembly.seg_of_link}
    volumes = {}
    for name in sorted(mesh_names):
        volume = surface_volume(cad.triangles(name))
        reference = cad.mass_props(name).volume
        np.testing.assert_allclose(volume, reference, rtol=1e-7, atol=1e-15, err_msg=name)
        volumes[name] = volume
    parts = []
    by_segment = defaultdict(list)
    by_rule = defaultdict(list)
    for part, segment in assembly.seg_of_link.items():
        mesh = cad.links[part].mesh_file
        density, rule = density_for(part)
        volume = volumes[mesh]
        mass = density * volume
        by_segment[segment].append(mass)
        by_rule[rule].append(mass)
        parts.append(dict(part=part, segment=segment, mesh=mesh, volume_m3=volume,
                          density_kg_m3=density, density_rule=rule, mass_kg=mass))
    model_path = Path(ROOT) / "models" / "osl_v2_bench.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    segments = {}
    for segment, masses in by_segment.items():
        reconstructed = math.fsum(masses)
        compiled = float(model.body(segment).mass[0])
        np.testing.assert_allclose(reconstructed, compiled, atol=1e-10, rtol=0)
        segments[segment] = dict(surface_reconstructed_kg=reconstructed,
                                 compiled_kg=compiled, difference_kg=reconstructed - compiled)
    urdf = ET.parse(URDF)
    unit_scale = all(tuple(map(float, mesh.get("scale", "1 1 1").split())) == (1., 1., 1.)
                     for mesh in urdf.findall(".//visual/geometry/mesh"))
    raw_mass = math.fsum(float(node.attrib["value"]) for node in urdf.findall("./link/inertial/mass"))
    report = dict(
        urdf_raw_mass_kg=raw_mass, unit_visual_mesh_scale=unit_scale,
        cad_mesh_count=len(volumes), part_instances=len(parts),
        max_mesh_volume_difference_m3=max(abs(v - cad.mass_props(n).volume) for n, v in volumes.items()),
        assigned_motor_mass_per_motor_kg=MOTOR_MASS, cuboid_check=box, segments=segments,
        model_total_kg=float(mujoco.mj_getTotalmass(model)),
        density_rule_totals_kg={rule: math.fsum(masses) for rule, masses in by_rule.items()},
        mjcf_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
        limitation="Same source meshes, segmentation and assigned densities. Arithmetic check only; physical component inventory and weights still needed.",
    )
    with (out / "Part_Mass_Inventory.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=parts[0])
        writer.writeheader()
        writer.writerows(sorted(parts, key=lambda row: row["mass_kg"], reverse=True))
    (out / "Mass_Check.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
