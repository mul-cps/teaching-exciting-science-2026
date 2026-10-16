"""The offline mesh inventory must never become a recovery clearance proof."""
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

trimesh = pytest.importorskip("trimesh")
pytest.importorskip("rtree")

from tools.audit_recovery_meshes import audit_mesh, audit_urdf, count_candidate_pairs


def test_exact_welding_keeps_every_triangle_and_does_not_qualify_material(tmp_path):
    path = tmp_path / "cube.stl"
    trimesh.creation.box().export(path)
    before = path.read_bytes()
    result = audit_mesh(path)
    assert result["sha256"] == hashlib.sha256(before).hexdigest()
    assert result["exact_weld"]["triangle_positions_preserved"]
    assert result["topology"]["watertight"]
    assert result["topology"]["winding_consistent"]
    assert result["intersection_proof"] == "not_performed"
    assert result["material_interior_qualified"] is False
    assert path.read_bytes() == before


def test_mesh_geometry_comes_from_the_exact_bytes_that_were_hashed(tmp_path, monkeypatch):
    path = tmp_path / "changing.stl"
    trimesh.creation.box().export(path)
    original = path.read_bytes()
    replacement = trimesh.creation.icosphere(subdivisions=1).export(file_type="stl")
    read_bytes = Path.read_bytes
    def replace_after_read(value):
        data = read_bytes(value)
        if value == path:
            path.write_bytes(replacement)
        return data
    monkeypatch.setattr(Path, "read_bytes", replace_after_read)
    result = audit_mesh(path)
    assert result["sha256"] == hashlib.sha256(original).hexdigest()
    assert result["faces"] == 12


def test_hollow_nested_shells_warn_instead_of_union_filling_a_cavity(tmp_path):
    outer = trimesh.creation.box(extents=[4, 4, 4])
    inner = trimesh.creation.box(extents=[2, 2, 2])
    inner.invert()
    path = tmp_path / "hollow.stl"
    trimesh.util.concatenate([outer, inner]).export(path)
    result = audit_mesh(path)
    assert len(result["components"]) == 2
    assert sorted(c["signed_volume_source_units_cubed"] for c in result["components"]) == [-8., 64.]
    assert "opposing_shell_winding_possible_cavity" in result["warnings"]
    assert result["material_interior_qualified"] is False


def test_two_face_membrane_is_not_repaired_or_accepted_as_material(tmp_path):
    mesh = trimesh.Trimesh(vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                           faces=[[0, 1, 2], [2, 1, 0]], process=False)
    path = tmp_path / "membrane.stl"
    mesh.export(path)
    result = audit_mesh(path)
    assert result["faces"] == 2
    assert result["components"][0]["faces"] == 2
    assert result["components"][0]["signed_volume_source_units_cubed"] == 0.
    assert "component_has_too_few_faces_to_bound_a_solid" in result["warnings"]
    assert "zero_volume_component" in result["warnings"]
    assert result["material_interior_qualified"] is False


def test_open_component_is_not_silently_repaired_during_split(tmp_path):
    box = trimesh.creation.box()
    box.update_faces(np.arange(len(box.faces)-1))
    path = tmp_path / "open.stl"
    box.export(path)
    result = audit_mesh(path)
    assert result["faces"] == 11
    assert result["components"][0]["faces"] == 11
    assert result["components"][0]["watertight"] is False
    assert "mesh_not_watertight" in result["warnings"]
    assert result["material_interior_qualified"] is False


def test_broadphase_counts_shared_edge_and_unshared_candidate_without_proving_intersection():
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, -1, 0]], float)
    shared = count_candidate_pairs(vertices, np.array([[0, 1, 2], [1, 0, 3]]))
    assert shared["complete"]
    assert shared["candidate_pairs"] == 1
    assert shared["pair_types"]["shared_edge"] == 1
    assert shared["intersection_proof"] == "not_performed"
    # Coincident coordinates with separate indices are intentionally reported
    # as unshared by this low-level enumerator; audit_mesh welds them first.
    vertices = np.concatenate([vertices[:3], vertices[:3]])
    unshared = count_candidate_pairs(vertices, np.array([[0, 1, 2], [3, 4, 5]]))
    assert unshared["candidate_pairs"] == 1
    assert unshared["pair_types"]["unshared"] == 1


def test_candidate_enumeration_deadline_is_incomplete_and_cannot_qualify():
    ticks = iter([0., 0., 2.])
    mesh = trimesh.creation.box()
    result = count_candidate_pairs(mesh.vertices, mesh.faces, timeout_s=1.,
                                   clock=lambda: next(ticks, 2.))
    assert not result["complete"]
    assert result["rows_checked"] == 0
    assert result["intersection_proof"] == "not_performed"


def test_urdf_package_resolution_and_cli_keep_the_global_gate_blocked(tmp_path):
    package = tmp_path / "cad"
    package.mkdir()
    trimesh.creation.box().export(package / "cube.stl")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text('<robot name="audit"><link name="base_link"><collision><geometry>'
        '<mesh filename="package://cad/cube.stl" scale=".001 .001 .001"/>'
        '</geometry></collision></link></robot>')
    result = audit_urdf(urdf.read_text(), package_paths={"cad": package}, directory=tmp_path)
    assert result["collision_instances"] == 1
    assert len(result["meshes"]) == 1
    mesh = result["meshes"][0]
    assert mesh["instances"][0]["scale"] == [.001, .001, .001]
    assert mesh["components"][0]["signed_volume_source_units_cubed"] == 1.
    assert "signed_volume_m3" not in mesh["components"][0]
    assert result["qualified"] is False
    assert result["hardware_access"] is False
    assert result["gate"]["recovery_allowed"] is False
    assert "physical_encoder_to_urdf_jaw_angle_not_evaluated" in result["gate"]["blockers"]
    output = tmp_path / "audit.json"
    script = Path(__file__).resolve().parents[1] / "tools/audit_recovery_meshes.py"
    command = [sys.executable, str(script), "--urdf", str(urdf),
               "--package-root", f"cad={package}", "--output", str(output)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    saved = json.loads(output.read_text())
    assert saved["urdf_sha256"] == hashlib.sha256(urdf.read_bytes()).hexdigest()
    assert saved["gate"]["recovery_allowed"] is False
    urdf.write_text(urdf.read_text().replace("cube.stl", "../outside.stl"))
    with pytest.raises(ValueError, match="escapes"):
        audit_urdf(urdf.read_text(), package_paths={"cad": package}, directory=tmp_path)
