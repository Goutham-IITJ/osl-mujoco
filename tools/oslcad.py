"""
oslcad.py -- dependency-light reader / analyser for the Onshape-exported
OSL V2 URDF (osl_v2_0_assembly).

Nothing here needs MuJoCo, trimesh, scipy or ROS: only numpy + stdlib.
It exists so that every downstream step (mass properties, joint-axis
identification, MJCF generation, offscreen rendering) works off ONE
verified geometric model of the CAD export.

Frame convention of the export (verified in tools/verify_frames.py):
  * +Z is proximal (towards the socket), -Z distal (towards the foot)
  * the knee / ankle flexion axis is world  Y
  * the sagittal plane is therefore the world XZ plane
"""

from __future__ import annotations

import collections
import math
import os
import re
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

# --------------------------------------------------------------------------
# small maths helpers
# --------------------------------------------------------------------------


def rpy_to_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw -> rotation matrix  Rz(yaw)Ry(pitch)Rx(roll)."""
    ca, sa = math.cos(roll), math.sin(roll)
    cb, sb = math.cos(pitch), math.sin(pitch)
    cc, sc = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa],
            [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa],
            [-sb, cb * sa, cb * ca],
        ]
    )


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> MuJoCo quaternion (w, x, y, z)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def rot_y(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# --------------------------------------------------------------------------
# STL reading + exact mass properties of a closed triangle soup
# --------------------------------------------------------------------------


def read_stl(path: str) -> np.ndarray:
    """Return an (ntri, 3, 3) float64 array of triangle vertices."""
    with open(path, "rb") as fh:
        data = fh.read()
    # ASCII STLs start with "solid" AND contain "facet" early on. Binary STLs
    # may also start with "solid" in their 80-byte header, hence the 2nd test.
    if data[:5] == b"solid" and b"facet" in data[:600]:
        vals = [
            float(x)
            for m in re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
            for x in m
        ]
        return np.asarray(vals, dtype=np.float64).reshape(-1, 3, 3)
    ntri = struct.unpack("<I", data[80:84])[0]
    raw = np.frombuffer(data, dtype=np.uint8, count=ntri * 50, offset=84)
    raw = raw.reshape(ntri, 50)
    tri = np.frombuffer(raw[:, 12:48].tobytes(), dtype="<f4")
    return tri.reshape(ntri, 3, 3).astype(np.float64)


@dataclass
class MassProps:
    """Mass properties of a unit-density closed triangle mesh."""

    volume: float  # m^3   (signed volume, made positive)
    centroid: np.ndarray  # (3,) in mesh-local coordinates
    inertia: np.ndarray  # (3,3) about the centroid, for unit density
    ntri: int
    closed: bool  # every edge traversed exactly twice in opposite senses


def mesh_mass_props(tri: np.ndarray) -> MassProps:
    """
    Exact volume / centroid / inertia of a closed triangle mesh by the
    divergence theorem (tetrahedra from the origin).  Signs handle
    concavities and internal voids correctly provided the mesh is closed
    and consistently oriented.
    """
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    # 6 * signed volume of each tetrahedron (origin, a, b, c)
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = v6.sum() / 6.0
    sgn = 1.0 if vol >= 0.0 else -1.0
    vol *= sgn
    v6 = v6 * sgn

    if vol <= 0.0:
        return MassProps(0.0, np.zeros(3), np.zeros((3, 3)), len(tri), False)

    centroid = ((a + b + c) / 4.0 * v6[:, None]).sum(0) / (6.0 * vol)

    # Inertia of a tetrahedron (origin,a,b,c) about the origin, unit density.
    # Standard covariance formulation: C = det(J)/120 * (sum outer products)
    P = np.stack([a, b, c], axis=1)  # (n,3,3) rows = the 3 vertices
    S = P.sum(axis=1)  # (n,3)
    # sum_i sum_j (v_i . v_j) style covariance per tet
    cov = (
        np.einsum("nij,nik->njk", P, P) + np.einsum("nj,nk->njk", S, S)
    ) * (v6 / 120.0)[:, None, None]
    C = cov.sum(0)
    trace = np.trace(C)
    I_origin = np.eye(3) * trace - C
    # shift to centroid (unit density -> mass == volume)
    d = centroid
    I_c = I_origin - vol * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    I_c = 0.5 * (I_c + I_c.T)

    # closedness: every undirected edge should appear exactly twice
    q = np.round(tri.reshape(-1, 3), 9)
    _, idx = np.unique(q, axis=0, return_inverse=True)
    f = idx.reshape(-1, 3)
    e = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    e.sort(axis=1)
    _, cnt = np.unique(e, axis=0, return_counts=True)
    closed = bool(np.all(cnt == 2))

    return MassProps(vol, centroid, I_c, len(tri), closed)


# --------------------------------------------------------------------------
# the URDF itself
# --------------------------------------------------------------------------


@dataclass
class Link:
    name: str
    mesh_file: str | None  # basename as written on disk (correct case)
    urdf_mass: float
    rgba: tuple


@dataclass
class Joint:
    name: str
    jtype: str
    parent: str
    child: str
    xyz: np.ndarray
    rot: np.ndarray
    axis: np.ndarray


class OslCad:
    """Parsed + forward-kinematically resolved view of the Onshape export."""

    def __init__(self, urdf_path: str, mesh_dir: str):
        self.urdf_path = urdf_path
        self.mesh_dir = mesh_dir
        # case-insensitive lookup: the URDF references different casing than
        # the files on disk, which breaks on Linux but not Windows.
        self._ondisk = {f.lower(): f for f in os.listdir(mesh_dir)}

        root = ET.parse(urdf_path).getroot()
        self.links: dict[str, Link] = {}
        for el in root.findall("link"):
            name = el.get("name")
            mel = el.find("visual/geometry/mesh")
            mesh = None
            if mel is not None:
                base = os.path.basename(mel.get("filename"))
                mesh = self._ondisk.get(base.lower(), base)
            mel2 = el.find("inertial/mass")
            mass = float(mel2.get("value")) if mel2 is not None else 0.0
            col = el.find("visual/material/color")
            rgba = (
                tuple(float(v) for v in col.get("rgba").split())
                if col is not None
                else (0.7, 0.7, 0.7, 1.0)
            )
            self.links[name] = Link(name, mesh, mass, rgba)

        self.joints: dict[str, Joint] = {}
        self.parent_of: dict[str, str] = {}
        self.joint_to_child: dict[str, Joint] = {}
        self.children: dict[str, list] = collections.defaultdict(list)
        for el in root.findall("joint"):
            o = el.find("origin")
            xyz = (
                np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
                if o is not None
                else np.zeros(3)
            )
            rot = (
                rpy_to_mat(*[float(v) for v in o.get("rpy", "0 0 0").split()])
                if o is not None
                else np.eye(3)
            )
            ax = el.find("axis")
            axis = (
                np.array([float(v) for v in ax.get("xyz").split()])
                if ax is not None
                else np.array([0.0, 0.0, 1.0])
            )
            j = Joint(
                el.get("name"),
                el.get("type"),
                el.find("parent").get("link"),
                el.find("child").get("link"),
                xyz,
                rot,
                axis,
            )
            self.joints[j.name] = j
            self.parent_of[j.child] = j.parent
            self.joint_to_child[j.child] = j
            self.children[j.parent].append(j.child)

        self.root_link = next(n for n in self.links if n not in self.parent_of)
        self.pose = self._forward_kinematics()
        self._mp_cache: dict[str, MassProps] = {}

    # -- kinematics ---------------------------------------------------------

    def _forward_kinematics(self) -> dict[str, tuple]:
        """World pose of every link at zero joint displacement (the CAD pose)."""
        pose = {self.root_link: (np.zeros(3), np.eye(3))}
        stack = [self.root_link]
        while stack:
            n = stack.pop()
            p, R = pose[n]
            for c in self.children[n]:
                j = self.joint_to_child[c]
                pose[c] = (p + R @ j.xyz, R @ j.rot)
                stack.append(c)
        return pose

    def chain_to(self, link: str) -> list:
        out = []
        while link in self.parent_of:
            out.append(self.joint_to_child[link])
            link = self.parent_of[link]
        return list(reversed(out))

    # -- geometry ----------------------------------------------------------

    def mass_props(self, mesh_file: str) -> MassProps:
        if mesh_file not in self._mp_cache:
            tri = read_stl(os.path.join(self.mesh_dir, mesh_file))
            self._mp_cache[mesh_file] = mesh_mass_props(tri)
        return self._mp_cache[mesh_file]

    def triangles(self, mesh_file: str) -> np.ndarray:
        return read_stl(os.path.join(self.mesh_dir, mesh_file))

    def mesh_links(self) -> list:
        return [n for n, l in self.links.items() if l.mesh_file]

    def world_triangles(self, link: str, apply_pose: bool = True) -> np.ndarray:
        """Triangles of `link` placed by the LINK POSE ONLY -- usually wrong.

        DO NOT USE for anything that has to be geometrically correct.  207 of
        the 682 links in this export carry a non-identity `<visual><origin>`
        (largest: a 206 mm translation on the Variflex foot), and the true mesh
        pose is `link pose o visual origin`.  This method knows nothing about
        the visual origin, so it reproduces the interpenetrating-modules
        assembly that cost an earlier session hours.

        Use `build_mjcf.OslModel.world_tris(link)` instead; it composes both.
        This method survives only so `tools/verify_visorigin.py` can render the
        wrong answer next to the right one as evidence.
        """
        tri = self.triangles(self.links[link].mesh_file)
        if not apply_pose:
            return tri
        p, R = self.pose[link]
        return tri @ R.T + p

    def world_centroid(self, link: str) -> np.ndarray:
        """Mesh centroid placed by the LINK POSE ONLY -- see world_triangles.

        Same trap, same fix: prefer the centroid of
        `build_mjcf.OslModel.geom_pose(link)`, which folds in the visual origin
        and agrees with the export's own declared `<inertial><origin>` to a
        0.002 mm p90 (this method is off by 158.5 mm at worst).
        """
        mp = self.mass_props(self.links[link].mesh_file)
        p, R = self.pose[link]
        return p + R @ mp.centroid

    # -- topology ----------------------------------------------------------

    def rigid_clusters(self) -> dict[str, list]:
        """Contract every `fixed` joint; returns representative -> members."""
        uf = {n: n for n in self.links}

        def find(x):
            while uf[x] != x:
                uf[x] = uf[uf[x]]
                x = uf[x]
            return x

        for j in self.joints.values():
            if j.jtype == "fixed":
                a, b = find(j.parent), find(j.child)
                if a != b:
                    uf[b] = a
        out = collections.defaultdict(list)
        for n in self.links:
            out[find(n)].append(n)
        return dict(out)

    def movable_joints(self) -> list:
        return [j for j in self.joints.values() if j.jtype != "fixed"]

    def is_phantom(self, link: str) -> bool:
        """
        Onshape emits mesh-less placeholder links to encode mates it cannot
        express as one URDF joint (`parallel_*`, `planar_*`, `cylindrical_*`,
        `*_loop_closure`).  They carry no geometry and no mass.
        """
        return self.links[link].mesh_file is None


# --------------------------------------------------------------------------
# density model
# --------------------------------------------------------------------------

# Onshape exported this assembly with no materials assigned: the URDF's total
# mass is 14.8 g for a leg that really weighs several kg, and 52 links have
# singular inertia tensors.  All of that data is therefore discarded and mass
# is recovered from mesh volume x an assigned density instead.
#
# (substring matched against the lower-cased link name, first hit wins)
DENSITY_RULES: list = [
    # --- fasteners / bearings / shafts / magnets : steel -------------------
    ("6656k11", 7850.0, "bearing steel"),
    ("5972k82", 7850.0, "bearing steel"),
    ("ball_bearing", 7850.0, "bearing steel"),
    ("angularcontactbearing", 7850.0, "bearing steel"),
    ("dowel_pin", 7850.0, "steel"),
    ("steelshaft", 7850.0, "steel"),
    ("countersunkmagnets", 7500.0, "NdFeB magnet"),
    ("_hex_", 7850.0, "stainless fastener"),
    ("socket_head", 7850.0, "stainless fastener"),
    ("button_head", 7850.0, "stainless fastener"),
    ("washer", 7850.0, "stainless fastener"),
    ("phillips", 7850.0, "stainless fastener"),
    ("mcmaster", 7850.0, "stainless fastener"),
    ("93395a", 7850.0, "stainless fastener"),
    ("92290a", 7850.0, "stainless fastener"),
    ("92125a", 7850.0, "stainless fastener"),
    ("91294a", 7850.0, "stainless fastener"),
    ("91290a", 7850.0, "stainless fastener"),
    ("91595a", 7850.0, "steel"),
    ("95966a", 7850.0, "stainless fastener"),
    ("wtm_", 7850.0, "steel"),
    ("loadcell", 7850.0, "steel load cell"),
    ("sri_m3564f", 7850.0, "steel load cell"),
    # --- aluminium spacers ------------------------------------------------
    ("94669a", 2700.0, "aluminium spacer"),
    # --- polymer / electronics -------------------------------------------
    ("pcb__rpi", 1900.0, "FR-4 PCB"),
    ("rpi4modelb", 1500.0, "electronics"),
    ("connector", 1500.0, "electronics"),
    ("pin_header", 1500.0, "electronics"),
    ("slot__", 1500.0, "electronics"),
    ("ethernet", 1500.0, "electronics"),
    ("usb", 1500.0, "electronics"),
    ("hdmi", 1500.0, "electronics"),
    ("cpu", 1500.0, "electronics"),
    ("sdram", 1500.0, "electronics"),
    ("belt", 1250.0, "rubber/HNBR belt"),
    ("electronicscover", 1250.0, "3D-printed cover"),
    # NOTE: rules are ordered and the first hit wins, so a specific key must
    # precede a more general one that also matches.  The two
    # `*batteryattachmentdephy` parts are machined aluminium brackets that bolt
    # the BA30 pack to the housing -- they are not battery cells, and without
    # this line they would be caught by the "battery" rule below.
    ("batteryattachment", 2700.0, "aluminium battery bracket"),
    ("ba30_", 1900.0, "Dephy BA30 battery"),
    ("battery", 1900.0, "battery"),
    ("variflexfoot", 1550.0, "carbon-fibre foot"),
    # --- default structural aluminium ------------------------------------
    ("", 2700.0, "aluminium (default)"),
]


def density_for(link_name: str) -> tuple:
    n = link_name.lower()
    for key, rho, label in DENSITY_RULES:
        if key == "" or key in n:
            return rho, label
    return 2700.0, "aluminium (default)"
