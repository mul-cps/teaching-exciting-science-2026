import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_objects'))
from so101_station_objects.colored_object_validation import (
    split_depth_coherent_components, validate_colored_component)

INTR = {'fx': 200., 'fy': 200., 'ppx': 49.5, 'ppy': 49.5}


def inputs():
    rgb = np.full((100, 100, 3), 100, np.uint8)
    mask = np.zeros((100, 100), bool)
    mask[35:65, 30:70] = True
    rgb[mask] = [0, 0, 255]
    depth = np.ones((100, 100), np.float32)
    depth[mask] = .5
    return rgb, depth, mask


def test_coherent_surface_is_not_a_claim_of_semantic_brick_identity():
    rgb, depth, mask = inputs()
    result = validate_colored_component(rgb, depth, mask, INTR)
    assert result.accepted
    assert result.metrics['semantic_identity'] == 'unqualified_colored_surface'
    assert result.metrics['workspace_gate_applied'] is False


def test_mostly_invalid_glass_color_cannot_become_a_floating_brick_after_cropping():
    rgb, depth, mask = inputs()
    depth[mask] = 0
    depth[43:55, 40:60] = .5
    pieces = split_depth_coherent_components(mask, depth)
    assert len(pieces) == 1
    result = validate_colored_component(rgb, depth, pieces[0].mask, INTR, support_mask=pieces[0].support_mask)
    assert not result.accepted
    assert result.reason == 'locally_unsupported_color_depth'


def test_real_foreground_survives_thin_same_color_bridge_to_large_invalid_background():
    rgb, depth, mask = inputs()
    mask[5:25, 5:85] = True
    mask[24:36, 49] = True
    rgb[mask] = [0, 0, 255]
    depth[5:25, 5:85] = 0
    depth[24:35, 49] = 0
    accepted = []
    for piece in split_depth_coherent_components(mask, depth):
        result = validate_colored_component(rgb, depth, piece.mask, INTR, support_mask=piece.support_mask)
        if result.accepted:
            accepted.append(piece)
    assert len(accepted) == 1
    assert accepted[0].mask[50, 50]


def test_touching_same_color_bricks_are_not_rejected_by_parent_size():
    rgb, depth, mask = inputs()
    mask[35:65, 5:95] = True
    depth[mask] = .5
    rgb[mask] = [0, 0, 255]
    pieces = split_depth_coherent_components(mask, depth)
    assert len(pieces) == 1
    assert validate_colored_component(rgb, depth, pieces[0].mask, INTR, support_mask=pieces[0].support_mask).accepted


def test_depth_discontinuities_are_split_before_contour_fill():
    rgb, depth, mask = inputs()
    depth[35:65, 50:70] = .9
    assert not validate_colored_component(rgb, depth, mask, INTR).accepted
    pieces = split_depth_coherent_components(mask, depth)
    assert len(pieces) == 2
    assert not any(piece.mask[50, 49] or piece.mask[50, 50] for piece in pieces)


def test_tilted_and_held_surfaces_use_camera_geometry_without_table_height_gate():
    rgb, depth, mask = inputs()
    y, x = np.indices(depth.shape)
    depth[mask] = (.3+.001*x+.0005*y)[mask]
    result = validate_colored_component(rgb, depth, mask, INTR,
                                         distortion=[.02, -.01, .001, 0, 0])
    assert result.accepted
    assert result.metrics['support_plane_gate_applied'] is False


def test_invalid_input_and_explicit_local_provenance_are_required():
    rgb, depth, mask = inputs()
    with pytest.raises(ValueError):
        validate_colored_component(rgb, depth.astype(np.uint16), mask, INTR)
    with pytest.raises(ValueError):
        validate_colored_component(rgb, depth, mask, INTR, support_mask=np.zeros_like(mask))


def test_noisy_or_holed_real_colored_surface_is_explicitly_unknown_below_ninety_percent():
    rgb, depth, mask = inputs()
    depth[35:40, 30:70] = 0
    result = validate_colored_component(rgb, depth, mask, INTR)
    assert not result.accepted
    assert result.reason == 'locally_unsupported_color_depth'
    assert result.metrics['supported_fraction'] < .90


def test_full_workspace_camera_translation_and_tilt_do_not_create_an_roi_gate():
    rgb, depth, mask = inputs()
    depth[mask] = .37
    translated_intrinsics = dict(INTR, ppx=-400., ppy=500.)
    result = validate_colored_component(rgb, depth, mask, translated_intrinsics)
    assert result.accepted
    assert abs(result.metrics['centroid_camera_m'][0]) > .5


def test_two_visible_brick_faces_are_not_rejected_for_global_nonplanarity():
    rgb, depth, mask = inputs()
    x = np.indices(depth.shape)[1]
    depth[mask] = (.45+.002*np.abs(x-50))[mask]
    result = validate_colored_component(rgb, depth, mask, INTR)
    assert result.accepted
    assert result.metrics['plane_residual_p95_m'] > .005


def test_local_computation_crop_preserves_global_calibrated_geometry():
    rgb, depth, mask = inputs()
    reference = validate_colored_component(rgb, depth, mask, INTR)
    shifted_rgb = np.roll(rgb, 10, axis=1)
    shifted_depth = np.roll(depth, 10, axis=1)
    shifted_mask = np.roll(mask, 10, axis=1)
    shifted = validate_colored_component(shifted_rgb, shifted_depth, shifted_mask,
        dict(INTR, ppx=INTR['ppx']+10))
    assert shifted.accepted == reference.accepted
    assert shifted.metrics['centroid_camera_m'] == pytest.approx(reference.metrics['centroid_camera_m'])
    assert shifted.metrics['visible_extents_m'] == pytest.approx(reference.metrics['visible_extents_m'])


def test_splitter_local_crop_restores_exact_global_mask_coordinates():
    rgb, depth, mask = inputs()
    reference = split_depth_coherent_components(mask, depth)
    shifted = split_depth_coherent_components(np.roll(mask, 12, axis=1), np.roll(depth, 12, axis=1))
    assert len(reference) == len(shifted) == 1
    assert shifted[0].mask.shape == mask.shape
    assert np.array_equal(np.roll(reference[0].mask, 12, axis=1), shifted[0].mask)
    assert np.array_equal(np.roll(reference[0].support_mask, 12, axis=1), shifted[0].support_mask)
    assert shifted[0].parent_pixels == reference[0].parent_pixels
    assert shifted[0].depth_cut_pixels == reference[0].depth_cut_pixels


def test_empty_splitter_input_still_validates_the_depth_gate_parameters():
    _, depth, mask = inputs()
    mask[:] = False
    assert split_depth_coherent_components(mask, depth) == ()
    with pytest.raises(ValueError):
        split_depth_coherent_components(mask, depth, max_neighbor_jump_m=-1.)


def test_final_child_silhouette_rejects_concave_floating_patch():
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    contour = np.array([[0,0],[30,0],[30,3],[3,3],[3,30],[0,30]], np.float32)
    result = validate_brick_silhouette(contour)
    assert not result.accepted
    assert result.reason == 'silhouette_inconsistent_with_convex_brick'
    assert result.metrics['min_rectangle_fill'] < .45


@pytest.mark.parametrize('angle', [0,15,30,45,60,75,89,90])
def test_projected_solid_brick_silhouettes_allow_oblique_and_edge_on_views(angle):
    import cv2
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    corners = np.array([[x,y,z]for x in (-.02,.02)for y in (-.012,.012)for z in (-.006,.006)])
    radians = np.deg2rad(angle)
    rotation = np.array([[np.cos(radians),0,np.sin(radians)],[0,1,0],[-np.sin(radians),0,np.cos(radians)]])
    points = corners@rotation.T + [0,0,.6]
    pixels = points[:,:2]/points[:,2,None]*450 + [320,240]
    contour = cv2.convexHull(pixels.astype(np.float32))
    assert validate_brick_silhouette(contour).accepted


def test_corner_occlusion_and_final_children_of_touching_group_are_retained():
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    corner_occluded = np.array([[8,0],[40,0],[40,30],[0,30],[0,8]], np.float32)
    assert validate_brick_silhouette(corner_occluded).accepted
    # The helper's contract is final children, never a connected parent group.
    children = [np.array([[0,0],[20,0],[20,20],[0,20]],np.float32),
                np.array([[20,20],[40,20],[40,40],[20,40]],np.float32)]
    assert all(validate_brick_silhouette(child).accepted for child in children)


def test_foreground_thin_bridge_component_retains_brick_silhouette():
    import cv2
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    rgb, depth, mask = inputs()
    mask[5:25, 5:85] = True
    mask[24:36, 49] = True
    depth[5:25, 5:85] = 0
    depth[24:35, 49] = 0
    pieces = split_depth_coherent_components(mask, depth)
    foreground = next(piece for piece in pieces if piece.mask[50,50])
    contours, _ = cv2.findContours(foreground.mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    assert validate_brick_silhouette(max(contours,key=cv2.contourArea)).accepted


def test_shape_degenerate_unknown_and_threshold_are_explicit():
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    assert not validate_brick_silhouette(np.array([[0,0],[1,1],[2,2]],np.float32)).accepted
    with pytest.raises(ValueError):
        validate_brick_silhouette(np.eye(3), min_rect_fill=-1)


def _capture_evidence_file(name):
    import os
    evidence = Path(os.environ.get('SO101_COLORED_CAPTURE_EVIDENCE',
        str(Path.home()/'.local/share/exciting-science/station/evidence/2026-10-16-sorting')))/name
    if not evidence.is_file():
        pytest.skip('local real-capture evidence unavailable; synthetic geometry tests still run')
    return evidence


def test_real_final_child_capture_contours_retain_all_development_bricks():
    import json
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    evidence = json.loads(_capture_evidence_file('final-child-silhouette-evidence.json').read_text())
    rows = evidence['contours']
    real = [row for row in rows if row['manual_table_brick']]
    assert len(real) == 320
    for row in real:
        assert validate_brick_silhouette(np.asarray(row['contour'],np.float32)).accepted
    assert evidence['summaries']['tracking-ghost-diagnosis']['real'] == 112
    assert evidence['summaries']['tracking-yellow-diagnosis']['real'] == 112


def test_recorded_fresh_live_shape_metrics_separate_chair_from_real_bricks():
    import json
    reports = json.loads(_capture_evidence_file('tracking-depth-optimized-status.json').read_text())
    # Manual static-capture labels: eight table-brick track IDs present in the
    # first report; chair/conveyor components are explicitly outside that set.
    real_ids = {(obj['color'],obj['track_id'])for obj in reports[0]['objects']
                if (obj['color'] != 'yellow' or obj['surface_centroid_world'][0] > .08)
                and (obj['color'] != 'red' or obj['surface_centroid_world'][1] > -.03)}
    real = [obj['depth_validation']['metrics']['silhouette_rectangularity']
            for report in reports for obj in report['objects']
            if (obj['color'],obj['track_id'])in real_ids]
    chair = [obj['depth_validation']['metrics']['silhouette_rectangularity']
             for report in reports for obj in report['objects']
             if obj['color']=='yellow' and obj['surface_centroid_world'][0]<.08]
    assert len(real) == 334
    assert len(chair) == 29
    assert min(real) >= .45
    assert max(chair) < .45


def test_morphology_closing_cannot_manufacture_admissible_observed_shape():
    import cv2
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    observed = np.zeros((40,60),np.uint8)
    observed[10:30,10:50:3] = 1
    observed[10,10:50] = 1
    closed = cv2.morphologyEx(observed,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,_ = cv2.findContours(closed,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours,key=cv2.contourArea)
    assert validate_brick_silhouette(contour).accepted
    result = validate_brick_silhouette(contour,observed_mask=observed)
    assert not result.accepted
    assert result.metrics['min_rectangle_fill'] < .45
    assert result.metrics['shape_evidence'] == 'original_color_depth_pixels'


def test_observed_pixel_rectangle_corrects_small_dense_and_holed_surfaces():
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    observed = np.ones((2,2),np.uint8)
    contour = np.array([[0,0],[1,0],[1,1],[0,1]],np.float32)
    result = validate_brick_silhouette(contour,observed_mask=observed)
    assert result.accepted
    assert result.metrics['min_rectangle_fill'] == pytest.approx(1.)
    observed = np.ones((20,30),np.uint8)
    observed[3:17:3,3:27:3] = 0
    contour = np.array([[0,0],[29,0],[29,19],[0,19]],np.float32)
    result = validate_brick_silhouette(contour,observed_mask=observed)
    assert result.accepted
    assert .8 < result.metrics['min_rectangle_fill'] < 1.


@pytest.mark.parametrize('angle',[0,15,30,45,60,75,90])
def test_observed_projected_solid_face_retains_oblique_views(angle):
    import cv2
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    points = cv2.boxPoints(((50,50),(36,22),angle))
    mask = np.zeros((100,100),np.uint8)
    cv2.fillConvexPoly(mask,np.rint(points).astype(np.int32),1)
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    assert validate_brick_silhouette(contours[0],observed_mask=mask).accepted


def test_dense_pixels_cannot_hide_an_unmorphed_concave_boundary():
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    observed = np.zeros((10,10),np.uint8)
    observed[:3,:] = 1
    observed[:,:3] = 1
    reconstructed = np.array([[0,0],[9,0],[9,9],[0,9]],np.float32)
    result = validate_brick_silhouette(reconstructed,observed_mask=observed)
    assert not result.accepted
    assert result.metrics['observed_pixel_fill'] > .45
    assert result.metrics['observed_boundary_fill'] < .45


def test_exact_live_chair_frame005_original_support_is_rejected():
    import json
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    case = json.loads(_capture_evidence_file('chair-observed-regression.json').read_text())
    pixels = np.asarray(case['observed_pixels'],np.int64)
    extent = pixels.max(axis=0)+1
    mask = np.zeros((extent[1],extent[0]),np.uint8)
    mask[pixels[:,1],pixels[:,0]] = 1
    result = validate_brick_silhouette(np.asarray(case['contour'],np.float32),observed_mask=mask)
    assert not result.accepted
    assert result.metrics['observed_boundary_fill'] < .45


def test_all_recorded_real_children_retain_original_observed_pixel_evidence():
    import json
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    evidence = json.loads(_capture_evidence_file('observed-child-silhouette-evidence.json').read_text())
    rows = [row for row in evidence['contours']if row['manual_table_brick']]
    assert len(rows) == 432
    for row in rows:
        pixels = np.asarray(row['observed_pixels'],np.int64)
        extent = pixels.max(axis=0)+1
        mask = np.zeros((extent[1],extent[0]),np.uint8)
        mask[pixels[:,1],pixels[:,0]] = 1
        assert validate_brick_silhouette(np.asarray(row['contour'],np.float32),observed_mask=mask).accepted
    assert not any(row['capture']=='tracking-shape-live'and row['color']=='yellow'
                   and row['pixel'][1]<290 for row in evidence['contours'])


def test_observed_corner_occlusion_and_touching_children_survive_without_parent_gate():
    import cv2
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    corner = np.array([[8,0],[40,0],[40,30],[0,30],[0,8]],np.int32)
    observed = np.zeros((45,45),np.uint8)
    cv2.fillPoly(observed,[corner],1)
    assert validate_brick_silhouette(corner,observed_mask=observed).accepted
    children = [np.array([[0,0],[20,0],[20,20],[0,20]],np.int32),
                np.array([[20,20],[40,20],[40,40],[20,40]],np.int32)]
    parent = np.zeros((45,45),np.uint8)
    cv2.fillPoly(parent,children,1)
    for child in children:
        selected = np.zeros_like(parent)
        cv2.fillPoly(selected,[child],1)
        assert validate_brick_silhouette(child,observed_mask=parent&selected).accepted


def test_observed_pixels_cannot_relax_existing_reconstructed_contour_gate():
    import json
    from so101_station_objects.colored_object_validation import validate_brick_silhouette
    case = json.loads(_capture_evidence_file('blue-observed-regression.json').read_text())
    points = np.asarray(case['observed_pixels'],np.int64)
    extent = points.max(axis=0)+1
    mask = np.zeros((extent[1],extent[0]),np.uint8)
    mask[points[:,1],points[:,0]] = 1
    contour = np.asarray(case['contour'],np.float32)
    assert not validate_brick_silhouette(contour).accepted
    result = validate_brick_silhouette(contour,observed_mask=mask)
    assert not result.accepted
    assert result.metrics['reconstructed_min_rectangle_fill'] < .45
