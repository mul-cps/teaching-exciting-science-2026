"""Metadata-only consumer readiness; original leases and newest evidence rule."""
import gc
import weakref

import pytest
from test_aligned_cohort import gate, feed
from test_derived_registration import endpoints


def prepared():
    from so101_station_perception.admission_proof import AdmissionProof
    core, value = gate()
    images = feed(core, value)
    token = core.take(now_ns=10_120_000_000, now_mono=100.12)
    graph = endpoints(value)
    matrix = [[float(i == j) for j in range(4)] for i in range(4)]
    for i in range(3):
        matrix[i][3] = value['calibration']['translation_m'][i]
    core.check(token, now_ns=10_120_000_000, now_mono=100.12,
               lookup=graph.__getitem__, color_from_depth=matrix)
    proof = AdmissionProof.from_token(token)
    return core, value, images, token, graph, matrix, proof


def test_saved_token_is_revoked_when_newest_lineage_was_cleared():
    core, value = gate()
    feed(core, value)
    token = core.take(now_ns=10_120_000_000, now_mono=100.12)
    core.ledger.entries.clear()
    graph = endpoints(value)
    matrix = [[float(i == j) for j in range(4)] for i in range(4)]
    for i in range(3):
        matrix[i][3] = value['calibration']['translation_m'][i]
    with pytest.raises(ValueError, match='newest|latest'):
        core.check(token, now_ns=10_120_000_000, now_mono=100.12,
                   lookup=graph.__getitem__, color_from_depth=matrix)


def test_proof_retains_no_image_payload_and_checks_actual_graph():
    core, value, images, token, graph, matrix, proof = prepared()
    reference = weakref.ref(images['color'].data)
    for records in core.streams.values():
        records.clear()
    del images, token
    gc.collect()
    assert reference() is None
    assert proof.check(core, now_ns=10_120_000_000, now_mono=100.12,
                       lookup=graph.__getitem__, color_from_depth=matrix)
    graph[value['outputs']['aligned_topic']][0].endpoint_gid = b'\xee' * 16
    with pytest.raises(ValueError):
        proof.check(core, now_ns=10_120_000_000, now_mono=100.12,
                    lookup=graph.__getitem__, color_from_depth=matrix)


@pytest.mark.parametrize('ns,mono', [(10_400_000_000,100.4), (10_119_000_000,100.119)])
def test_status_cannot_renew_original_leases_or_move_clock_back(ns, mono):
    core, _, _, _, graph, matrix, proof = prepared()
    with pytest.raises(ValueError):
        proof.check(core, now_ns=ns, now_mono=mono,
                    lookup=graph.__getitem__, color_from_depth=matrix)


def test_reset_and_invalid_latest_revoke_metadata_proof():
    core, _, _, _, graph, matrix, proof = prepared()
    core.ledger.entries.clear()
    with pytest.raises(ValueError, match='latest|newest'):
        proof.check(core, now_ns=10_120_000_000, now_mono=100.12,
                    lookup=graph.__getitem__, color_from_depth=matrix)
