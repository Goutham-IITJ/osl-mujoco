"""
softrender.py -- tiny dependency-free triangle rasteriser (numpy + zlib only).

This exists purely so the geometry can be *looked at* without MuJoCo, OpenGL
or a display.  It is a verification tool, not part of the simulation.
"""

from __future__ import annotations

import struct
import zlib

import numpy as np


def write_png(path: str, rgb: np.ndarray) -> None:
    """rgb: (H, W, 3) uint8 -> 8-bit RGB PNG."""
    h, w, _ = rgb.shape
    raw = b"".join(
        b"\x00" + rgb[y].tobytes() for y in range(h)
    )  # filter byte 0 per scanline

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )
    with open(path, "wb") as fh:
        fh.write(png)


def look_at(eye: np.ndarray, target: np.ndarray, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """World -> camera rotation (rows are the camera basis in world coords)."""
    f = np.asarray(target, float) - np.asarray(eye, float)
    f = f / np.linalg.norm(f)
    up = np.asarray(up, float)
    if abs(float(f @ up)) > 0.999:
        up = np.array([0.0, 1.0, 0.0])
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.stack([r, u, f])


def render(
    tris: np.ndarray,
    colors: np.ndarray,
    eye,
    target,
    width: int = 700,
    height: int = 900,
    fov_scale: float = 1.0,
    bg=(250, 250, 252),
    up=(0.0, 0.0, 1.0),
    light=(0.4, -0.75, 0.55),
    samples_per_px: float = 3.0,
    max_samples: int = 24_000_000,
    seed: int = 0,
) -> np.ndarray:
    """
    Orthographic z-buffered flat-shaded renderer, fully vectorised.

    Rather than looping over triangles (hopeless at ~1.7 M of them in pure
    Python) each triangle is stratified-sampled in barycentric space at a
    density proportional to its projected area, and the resulting point
    cloud is resolved with an argsort-based z-buffer.  With >=2 samples per
    pixel of projected area the result is indistinguishable from scanline
    rasterisation at this image size.

    tris   : (n, 3, 3) world-space triangle vertices
    colors : (n, 3) float in 0..1 base colour per triangle
    """
    rng = np.random.default_rng(seed)
    eye = np.asarray(eye, float)
    target = np.asarray(target, float)
    R = look_at(eye, target, up)

    # ---- to camera space: x right, y up, z into the screen ----
    V = ((tris.reshape(-1, 3) - eye) @ R.T).reshape(-1, 3, 3)

    ext = max(
        V[:, :, 0].max() - V[:, :, 0].min(), V[:, :, 1].max() - V[:, :, 1].min()
    )
    ext = ext if ext > 0 else 1.0
    scale = min(width, height) / (ext * 1.06) * fov_scale
    cx = 0.5 * (V[:, :, 0].max() + V[:, :, 0].min())
    cy = 0.5 * (V[:, :, 1].max() + V[:, :, 1].min())

    sx = (V[:, :, 0] - cx) * scale + width * 0.5
    sy = height * 0.5 - (V[:, :, 1] - cy) * scale
    sz = V[:, :, 2]

    # ---- flat shading from the world-space normal ----
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    ln[ln == 0] = 1.0
    nrm = nrm / ln
    L = np.asarray(light, float)
    L = L / np.linalg.norm(L)
    shade = (0.30 + 0.70 * np.abs(nrm @ L))[:, None]
    face = np.abs(nrm @ R[2])[:, None]
    rgb_tri = np.clip(colors * shade * (0.72 + 0.28 * face), 0.0, 1.0)

    # ---- how many samples does each triangle need? ----
    area = 0.5 * np.abs(
        (sx[:, 1] - sx[:, 0]) * (sy[:, 2] - sy[:, 0])
        - (sx[:, 2] - sx[:, 0]) * (sy[:, 1] - sy[:, 0])
    )
    dens = samples_per_px
    want = np.ceil(area * dens).astype(np.int64) + 1
    total = int(want.sum())
    if total > max_samples:  # scale back uniformly if the budget is blown
        want = np.maximum(1, (want * (max_samples / total)).astype(np.int64))
        total = int(want.sum())

    tid = np.repeat(np.arange(len(tris), dtype=np.int64), want)
    # uniform barycentric sampling of a triangle
    r1 = rng.random(total)
    r2 = rng.random(total)
    s = np.sqrt(r1)
    b0 = 1.0 - s
    b1 = s * (1.0 - r2)
    b2 = s * r2

    px = b0 * sx[tid, 0] + b1 * sx[tid, 1] + b2 * sx[tid, 2]
    py = b0 * sy[tid, 0] + b1 * sy[tid, 1] + b2 * sy[tid, 2]
    pz = b0 * sz[tid, 0] + b1 * sz[tid, 1] + b2 * sz[tid, 2]

    ix = np.floor(px).astype(np.int64)
    iy = np.floor(py).astype(np.int64)
    ok = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix, iy, pz, tid = ix[ok], iy[ok], pz[ok], tid[ok]

    img = np.empty((height * width, 3), np.float64)
    img[:] = np.asarray(bg, float) / 255.0
    if len(ix) == 0:
        return (np.clip(img.reshape(height, width, 3), 0, 1) * 255).astype(np.uint8)

    flat = iy * width + ix
    # z-buffer: sort by (pixel, depth) then keep the first hit per pixel
    order = np.lexsort((pz, flat))
    flat_s = flat[order]
    first = np.ones(len(flat_s), bool)
    first[1:] = flat_s[1:] != flat_s[:-1]
    win = order[first]
    img[flat[win]] = rgb_tri[tid[win]]

    return (np.clip(img.reshape(height, width, 3), 0, 1) * 255).astype(np.uint8)


class ChunkRenderer:
    """
    Memory-bounded version of `render` for very large scenes.

    The OSL V2 export instantiates roughly five million triangles once mesh
    reuse is expanded, which will not fit in memory as one array alongside
    its sample buffers.  Feed it in blocks instead: the z-buffer and colour
    buffer persist across `add()` calls, so the result is identical to
    rendering everything at once.

    Usage:
        r = ChunkRenderer(eye, target, bbox_min, bbox_max, 520, 760)
        for tris, cols in blocks:
            r.add(tris, cols)
        img = r.image()
    """

    def __init__(
        self,
        eye,
        target,
        bbox_min,
        bbox_max,
        width: int = 700,
        height: int = 900,
        fov_scale: float = 1.0,
        bg=(250, 250, 252),
        up=(0.0, 0.0, 1.0),
        light=(0.4, -0.75, 0.55),
        samples_per_px: float = 2.5,
        seed: int = 0,
    ):
        self.eye = np.asarray(eye, float)
        self.R = look_at(self.eye, np.asarray(target, float), up)
        self.w, self.h = int(width), int(height)
        self.bg = np.asarray(bg, float) / 255.0
        L = np.asarray(light, float)
        self.light = L / np.linalg.norm(L)
        self.dens = samples_per_px
        self.rng = np.random.default_rng(seed)

        # camera fit from the 8 bbox corners, so no pre-pass over geometry
        lo = np.asarray(bbox_min, float)
        hi = np.asarray(bbox_max, float)
        corners = np.array(
            [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        )
        Vc = (corners - self.eye) @ self.R.T
        ext = max(
            Vc[:, 0].max() - Vc[:, 0].min(), Vc[:, 1].max() - Vc[:, 1].min()
        )
        ext = ext if ext > 0 else 1.0
        self.scale = min(self.w, self.h) / (ext * 1.06) * fov_scale
        self.cx = 0.5 * (Vc[:, 0].max() + Vc[:, 0].min())
        self.cy = 0.5 * (Vc[:, 1].max() + Vc[:, 1].min())

        self.zbuf = np.full(self.h * self.w, np.inf)
        self.cbuf = np.tile(self.bg, (self.h * self.w, 1))

    def add(self, tris: np.ndarray, colors: np.ndarray) -> None:
        if len(tris) == 0:
            return
        tris = np.ascontiguousarray(tris, dtype=np.float64)
        V = ((tris.reshape(-1, 3) - self.eye) @ self.R.T).reshape(-1, 3, 3)
        sx = (V[:, :, 0] - self.cx) * self.scale + self.w * 0.5
        sy = self.h * 0.5 - (V[:, :, 1] - self.cy) * self.scale
        sz = V[:, :, 2]

        nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        ln = np.linalg.norm(nrm, axis=1, keepdims=True)
        ln[ln == 0] = 1.0
        nrm /= ln
        shade = (0.30 + 0.70 * np.abs(nrm @ self.light))[:, None]
        face = np.abs(nrm @ self.R[2])[:, None]
        rgb = np.clip(np.asarray(colors, float) * shade * (0.72 + 0.28 * face), 0.0, 1.0)

        area = 0.5 * np.abs(
            (sx[:, 1] - sx[:, 0]) * (sy[:, 2] - sy[:, 0])
            - (sx[:, 2] - sx[:, 0]) * (sy[:, 1] - sy[:, 0])
        )
        want = np.ceil(area * self.dens).astype(np.int64) + 1
        tid = np.repeat(np.arange(len(tris), dtype=np.int64), want)
        n = len(tid)
        s = np.sqrt(self.rng.random(n))
        r2 = self.rng.random(n)
        b0, b1, b2 = 1.0 - s, s * (1.0 - r2), s * r2

        ix = np.floor(b0 * sx[tid, 0] + b1 * sx[tid, 1] + b2 * sx[tid, 2]).astype(np.int64)
        iy = np.floor(b0 * sy[tid, 0] + b1 * sy[tid, 1] + b2 * sy[tid, 2]).astype(np.int64)
        pz = b0 * sz[tid, 0] + b1 * sz[tid, 1] + b2 * sz[tid, 2]

        ok = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        if not ok.any():
            return
        ix, iy, pz, tid = ix[ok], iy[ok], pz[ok], tid[ok]
        flat = iy * self.w + ix

        # keep the nearest sample per pixel within this chunk ...
        order = np.lexsort((pz, flat))
        fs = flat[order]
        first = np.ones(len(fs), bool)
        first[1:] = fs[1:] != fs[:-1]
        win = order[first]
        pix, zz, tt = flat[win], pz[win], tid[win]
        # ... then merge against the persistent buffer
        better = zz < self.zbuf[pix]
        self.zbuf[pix[better]] = zz[better]
        self.cbuf[pix[better]] = rgb[tt[better]]

    def image(self) -> np.ndarray:
        return (np.clip(self.cbuf.reshape(self.h, self.w, 3), 0, 1) * 255).astype(np.uint8)


def hstack_images(imgs: list, gap: int = 12, bg=(250, 250, 252)) -> np.ndarray:
    h = max(i.shape[0] for i in imgs)
    tot = sum(i.shape[1] for i in imgs) + gap * (len(imgs) - 1)
    out = np.zeros((h, tot, 3), np.uint8)
    out[:] = np.asarray(bg, np.uint8)
    x = 0
    for i in imgs:
        out[: i.shape[0], x : x + i.shape[1]] = i
        x += i.shape[1] + gap
    return out
