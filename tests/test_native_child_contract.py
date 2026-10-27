"""Child source/layout/ownership contract; no device/driver execution."""
from pathlib import Path
import subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'ros2/src/so101_station_vulkan'


def test_child_source_declares_alias_safe_and_guarded_contract():
    header=(PACKAGE/'include/so101_station_vulkan/children.hpp').read_text()
    source=(PACKAGE/'src/children.cpp').read_text()
    shader=(PACKAGE/'shaders/children.slang').read_text()
    assert 'ParentChildStatus' in header
    assert 'source_frame()' in source and 'configuration()' in source
    assert 'belongs_to(s.source_token)' in source
    submit=source.split('ChildLease ChildPreparation::submit(',1)[1].split('bool ChildPreparation::ready(',1)[0]
    for forbidden in ('vkWait','vkMapMemory','vkAllocate','.create(','new ','std::vector','make_shared'):
        assert forbidden not in submit
    assert 'vkCmdDispatchIndirect' in source
    assert 'VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT' in source
    assert 'VK_ACCESS_INDIRECT_COMMAND_READ_BIT' in source
    assert 'VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT' in source
    assert '.workspace.mapped' not in source and '.packed.mapped' not in source
    assert 'for(unsigned round=0;round<32;round++)' in source
    for forbidden in ('double','uint64_t'):
        assert forbidden not in shader
    assert 'child_validate' in shader and 'child_fill_validate' in shader


def test_child_cache_tracks_compiler_mode_and_postbuild_identity():
    cmake=(PACKAGE/'cmake/children.cmake').read_text()
    for text in ('CMAKE_CONFIGURE_DEPENDS','COMPILER_HASH','mode=offline','mode=compiler','PENDING_PROVENANCE','copy_if_different'):
        assert text in cmake
