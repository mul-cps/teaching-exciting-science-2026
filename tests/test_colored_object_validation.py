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
