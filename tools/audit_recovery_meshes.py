#!/usr/bin/env python3
"""Read-only offline mesh inventory; never a collision or material certificate.

Exact coordinate welding changes indices only. No faces are repaired, removed,
filled or simplified. Outward-rounded AABBs enumerate possible intersecting
triangle pairs; no narrow-phase intersection proof is performed. Watertightness
and consistent winding alone cannot authorize material carving or recovery.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from rtree import index


def count_candidate_pairs(vertices, faces, *, timeout_s=30., clock=time.monotonic):
    """Count broadphase candidates, including shared simplices, without proof.

    The elapsed check bounds enumeration after index construction returns. It
    cannot interrupt a blocking mesh loader or native Rtree operation.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("candidate enumeration timeout must be positive and finite")
    started = clock()
    vertices, faces = np.asarray(vertices), np.asarray(faces)
    triangles = vertices[faces]
    bounds = np.concatenate((np.nextafter(triangles.min(axis=1), -np.inf),
                             np.nextafter(triangles.max(axis=1), np.inf)), axis=1)
    properties = index.Property()
    properties.dimension = 3
    tree = index.Index(((int(i), tuple(b), None) for i, b in enumerate(bounds)), properties=properties)
    built = clock()
    counts = dict.fromkeys(("unshared", "shared_vertex", "shared_edge", "duplicate"), 0)
    rows = 0
    try:
        for i, bound in enumerate(bounds):
            if clock() - started >= timeout_s:
                break
            candidates = np.array([j for j in tree.intersection(tuple(bound)) if j > i], dtype=np.int64)
            if len(candidates):
                shared = (faces[candidates, None, :] == faces[i][None, :, None]).any(axis=2).sum(axis=1)
                for number, key in enumerate(counts):
                    counts[key] += int(np.sum(shared == number))
            rows += 1
    finally:
        tree.close()
    ended = clock()
    return {"complete": rows == len(faces), "rows_checked": rows,
            "candidate_pairs": sum(counts.values()), "pair_types": counts,
            "index_build_s": max(0., built-started),
            "enumeration_s": max(0., ended-built), "elapsed_s": max(0., ended-started),
            "intersection_proof": "not_performed"}


def audit_mesh(path, *, timeout_s=30.):
    """Preserve all original triangles and report unqualified topology facts."""
    started = time.monotonic()
    path = Path(path).resolve()
    source = path.read_bytes()
    mesh = trimesh.load(io.BytesIO(source), file_type=path.suffix.lstrip("."),
                        force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh) or not 0 < len(mesh.faces) <= 250_000:
        raise ValueError("audit requires a nonempty triangle mesh with at most 250000 faces")
    if not np.isfinite(mesh.vertices).all():
        raise ValueError("mesh vertices must be finite")
    vertices, inverse = np.unique(mesh.vertices, axis=0, return_inverse=True)
    faces = inverse[mesh.faces]
    triangles_preserved = bool(np.array_equal(mesh.vertices[mesh.faces], vertices[faces]))
    if not triangles_preserved:
        raise ValueError("exact welding changed triangle coordinates")
    welded = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    triangles = vertices[faces]
    nondegenerate = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                           triangles[:, 2]-triangles[:, 0]), axis=1) > 0
    components = []
    # repair=False is essential: split defaults can fill small holes.
    for component in welded.split(only_watertight=False, repair=False):
        tri = component.vertices[component.faces]
        volume = float(np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum()/6)
        components.append({"faces": len(component.faces), "watertight": bool(component.is_watertight),
            "winding_consistent": bool(component.is_winding_consistent),
            "signed_volume_source_units_cubed": volume, "bounds_source_units": component.bounds.tolist()})
    warnings = ["watertight_winding_not_intersection_proof"]
    if not nondegenerate.all():
        warnings.append("degenerate_triangles")
    if not welded.is_watertight:
        warnings.append("mesh_not_watertight")
    if not welded.is_winding_consistent:
        warnings.append("inconsistent_winding")
    if any(component["faces"] < 4 for component in components):
        warnings.append("component_has_too_few_faces_to_bound_a_solid")
    if any(component["signed_volume_source_units_cubed"] == 0 for component in components):
        warnings.append("zero_volume_component")
    signs = {math.copysign(1, component["signed_volume_source_units_cubed"]) for component in components
             if component["signed_volume_source_units_cubed"] != 0}
    if len(signs) > 1:
        warnings.append("opposing_shell_winding_possible_cavity")
    if len(components) > 1:
        warnings.append("shell_nesting_and_intersections_unproven_do_not_union_fill")
    pairs = count_candidate_pairs(vertices, faces, timeout_s=timeout_s)
    if not pairs["complete"]:
        warnings.append("candidate_enumeration_incomplete")
    return {"path": str(path), "sha256": hashlib.sha256(source).hexdigest(),
            "coordinate_units": "source mesh units; URDF instance scales recorded separately",
            "faces": len(faces), "source_vertices": len(mesh.vertices),
            "exact_weld": {"vertices": len(vertices), "triangle_positions_preserved": triangles_preserved,
                "faces_repaired_or_removed": 0},
            "topology": {"watertight": bool(welded.is_watertight),
                "winding_consistent": bool(welded.is_winding_consistent),
                "all_faces_nondegenerate": bool(nondegenerate.all())},
            "components": components, "warnings": warnings,
            "candidate_enumeration": pairs, "elapsed_s": time.monotonic()-started,
            "intersection_proof": "not_performed", "shell_nesting_proof": "not_performed",
            "material_interior_qualified": False}


def _mesh_path(uri, package_paths, directory):
    if uri.startswith("package://"):
        package, relative = uri[len("package://"):].split("/", 1)
        if package not in package_paths:
            raise ValueError("explicit package root required: " + package)
        root = Path(package_paths[package]).resolve()
        path = (root / relative).resolve()
        if not path.is_relative_to(root):
            raise ValueError("collision mesh escapes explicit package root")
        return path
    if uri.startswith("file://"):
        return Path(uri[len("file://"):]).resolve()
    if "://" in uri:
        raise ValueError("unsupported mesh URI: " + uri)
    path = Path(uri)
    if not path.is_absolute():
        if directory is None:
            raise ValueError("relative mesh URI requires an explicit source directory")
        path = Path(directory) / path
    return path.resolve()


def audit_urdf(xml, *, package_paths, directory=None, timeout_s=30.):
    root = ET.fromstring(xml)
    if root.tag != "robot":
        raise ValueError("effective URDF robot required")
    entries = {}
    other_geometry = []
    instances = 0
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            instances += 1
            geometry = collision.find("geometry")
            if geometry is None or len(geometry) != 1:
                raise ValueError("exactly one collision geometry per instance required")
            shape = geometry[0]
            if shape.tag != "mesh":
                other_geometry.append({"link": link.get("name"), "type": shape.tag})
                continue
            uri = shape.get("filename", "")
            scale = [float(v) for v in shape.get("scale", "1 1 1").split()]
            if len(scale) != 3 or any(not math.isfinite(v) or v == 0 for v in scale):
                raise ValueError("finite nonzero mesh scale required")
            path = _mesh_path(uri, package_paths, directory)
            entries.setdefault(path, []).append({"link": link.get("name"), "uri": uri, "scale": scale})
    meshes = []
    for path, occurrences in sorted(entries.items()):
        row = audit_mesh(path, timeout_s=timeout_s)
        row["instances"] = occurrences
        meshes.append(row)
    return {"schema": "so101-recovery-mesh-audit-v1", "robot_name": root.get("name"),
            "urdf_sha256": hashlib.sha256(xml.encode()).hexdigest(), "collision_instances": instances,
            "meshes": meshes, "other_geometry_not_audited": other_geometry,
            "qualified": False, "hardware_access": False, "actuator_commands": 0,
            "gate": {"recovery_allowed": False, "blockers": [
                "triangle_self_intersections_not_proven_absent",
                "shell_nesting_and_material_parity_not_proven",
                "conservative_full_cell_boundary_distance_not_proven",
                "physical_registration_not_evaluated",
                "physical_encoder_to_urdf_jaw_angle_not_evaluated",
                "qualified_live_scene_provider_not_evaluated"]},
            "versions": {"python": sys.version.split()[0], "numpy": np.__version__,
                         "trimesh": trimesh.__version__}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--urdf", type=Path, help="saved effective URDF XML")
    source.add_argument("--ros-parameters", type=Path, help="saved ROS YAML parameters; no ROS connection")
    parser.add_argument("--node", default="/follower/robot_state_publisher")
    parser.add_argument("--package-root", action="append", default=[], metavar="NAME=PATH")
    parser.add_argument("--pair-timeout-s", type=float, default=30., help="enumeration budget per mesh")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        packages = {}
        for binding in args.package_root:
            name, path = binding.split("=", 1)
            if not name or name in packages:
                raise ValueError("package roots must have unique nonempty names")
            packages[name] = Path(path).resolve()
        source_path = (args.urdf or args.ros_parameters).resolve()
        if args.ros_parameters:
            import yaml
            xml = yaml.safe_load(source_path.read_text())[args.node]["ros__parameters"]["robot_description"]
        else:
            xml = source_path.read_text()
        report = audit_urdf(xml, package_paths=packages, directory=source_path.parent,
                            timeout_s=args.pair_timeout_s)
        report["source"] = {"path": str(source_path), "node": args.node if args.ros_parameters else None,
                            "package_paths": {k: str(v) for k, v in packages.items()}}
        report["reproduce_argv"] = [str(Path(__file__).resolve()), *(argv if argv is not None else sys.argv[1:])]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + ".tmp")
        temporary.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
        temporary.replace(args.output)
        print(json.dumps({"output": str(args.output.resolve()), "meshes": len(report["meshes"]),
                          "recovery_allowed": False}))
        return 0
    except (OSError, ValueError, TypeError, KeyError, ET.ParseError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
