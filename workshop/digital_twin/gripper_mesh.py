"""Load all requested URDF gripper visuals once as explicitly NOMINAL CAD."""
import hashlib
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

LINKS = ("gripper_link", "moving_jaw_so101_v1_link")
MAX_TRIANGLES = 200000


def load_gripper_visuals(description, resolve_package=None):
    if len(description) > 2_000_000:
        raise ValueError("robot description exceeds the bounded CAD input")
    try:
        root = ET.fromstring(description)
    except ET.ParseError as error:
        raise ValueError("invalid robot description XML") from error
    result, provenance, total = [], [], 0
    for link_name in LINKS:
        link = root.find(f"link[@name='{link_name}']")
        if link is None or not link.findall("visual"):
            raise ValueError(f"requested gripper visual link missing: {link_name}")
        for i, visual in enumerate(link.findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is None or len(result) >= 16:
                raise ValueError("all gripper visuals must be bounded STL meshes")
            filename = mesh.get("filename", "")
            if filename.startswith("package://"):
                package, separator, relative = filename[10:].partition("/")
                if not separator or resolve_package is None:
                    raise ValueError("package mesh requires a package resolver")
                filename = str(Path(resolve_package(package)) / relative)
            path = Path(filename)
            if not path.is_absolute() or path.suffix.lower() != ".stl" or path.stat().st_size > 84 + MAX_TRIANGLES * 50:
                raise ValueError("gripper mesh must be a bounded absolute STL path")
            raw = path.read_bytes()
            if len(raw) < 84:
                raise ValueError("truncated binary STL")
            count = struct.unpack_from("<I", raw, 80)[0]
            total += count
            if not 0 < count or total > MAX_TRIANGLES or len(raw) != 84 + count * 50:
                raise ValueError("binary STL triangle count/layout exceeds bound")
            records = np.frombuffer(raw, dtype=np.dtype([
                ("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]), offset=84, count=count)
            scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            origin = visual.find("origin")
            xyz = np.fromstring("0 0 0" if origin is None else origin.get("xyz", "0 0 0"), sep=" ")
            rpy = np.fromstring("0 0 0" if origin is None else origin.get("rpy", "0 0 0"), sep=" ")
            if any(v.shape != (3,) or not np.isfinite(v).all() for v in (scale, xyz, rpy)) or np.any(scale <= 0):
                raise ValueError("finite positive mesh scales and finite URDF origins required")
            vertices = records["vertices"] * scale
            vertices = vertices @ Rotation.from_euler("xyz", rpy).as_matrix().T + xyz
            if not np.isfinite(vertices).all():
                raise ValueError("nonfinite gripper vertices")
            # Baked per-face diagnostic shading; these are synthetic gray CAD
            # colors, never measured RGB/texture. Vectorized, once per load.
            normals = np.cross(vertices[:, 1]-vertices[:, 0], vertices[:, 2]-vertices[:, 0])
            lengths = np.linalg.norm(normals, axis=1)
            normals /= np.maximum(lengths[:, None], 1e-15)
            shade = .45 + .55 * np.abs(normals @ (np.array([1., -1., 2.])/np.sqrt(6.)))
            gray = np.asarray(np.rint(shade * (210 if link_name == LINKS[0] else 240)), np.uint8)
            colors = np.repeat(np.repeat(gray[:, None], 3, axis=1), 3, axis=0)
            name = f"{link_name}/visual_{i}"
            result.append({"name": name, "link": link_name,
                "points": np.ascontiguousarray(vertices.reshape(-1, 3), dtype=np.float32),
                "colors": colors})
            provenance.append({"name": name, "link": link_name, "path": str(path),
                "sha256": hashlib.sha256(raw).hexdigest(), "triangles": count,
                "origin_xyz_m": xyz.tolist(), "origin_rpy_rad": rpy.tolist(), "scale": scale.tolist()})
    return result, {"qualified": False, "layer": "nominal_cad", "triangles": total,
        "description_sha256": hashlib.sha256(description.encode()).hexdigest(), "visuals": provenance}
