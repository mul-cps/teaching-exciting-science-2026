"""Source-bound lifetime/resource guards for the immutable early parent map."""
from pathlib import Path
import re

SOURCE = Path(__file__).resolve().parents[1] / "ros2/src/so101_station_vulkan/shaders/children.slang"

def body(source, name):
    match = re.search(r"\b(?:void|uint|bool) " + name + r"\([^)]*\)\{", source)
    assert match is not None, f"missing function {name}"
    start = match.end()
    depth = 1
    end = start
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end - 1]

def check(source):
    assert "return wl(8*p.count+i);" in body(source, "graphParent")
    init = body(source, "child_init")
    assert "uint par=sourceSelectedParent(i);" in init
    assert "ws(8*p.count+i,par);" in init
    assert "graphParent(" not in init  # Neighbours are not yet published.
    assert len(re.findall(r"ws\(8\*p.count\+i,", source)) == 1
    for name in ("candidate", "childEdge", "child_hook", "child_jump", "validateChildEdge", "child_validate", "child_reduce"):
        text = body(source, name)
        assert "graphParent(" in text
        assert "sourceSelectedParent(" not in text
    gather = body(source, "child_gather")
    guard = "if(par==0||par>hl(3)||area==0){fault(64);return;}"
    assert "uint par=graphParent(i),area=" in gather
    assert guard in gather
    assert gather.index(guard) < gather.index("uint parent=par-1;")
    assert "sourceSelectedParent(global)" in body(source, "child_packed_init")
    late = source[source.index("groupshared uint digits"):]
    assert "graphParent(" not in late
    assert len(re.findall(r"vk::binding\(", source)) == 11
    assert source.count("GroupMemoryBarrierWithGroupSync()") == 14
    assert len(re.findall(r"\[numthreads", source)) == 21

def test_parent_cache_lifetime_and_resource_contract():
    check(SOURCE.read_text())

def test_guards_reject_lifetime_and_index_mutations():
    import pytest
    source = SOURCE.read_text()
    check(source)
    mutations = (
        ("ws(8*p.count+i,par);", "ws(8*p.count+i,0);"),
        ("sourceSelectedParent(i-1)", "graphParent(i-1)"),
        ("sourceSelectedParent(global)", "graphParent(global)"),
        ("if(par==0||par>hl(3)||area==0)", "if(par>hl(3)||area==0)"),
        ("// Status valid", "ws(8*p.count+i,par-1); // Status valid"),
    )
    for old, new in mutations:
        assert old in source
        with pytest.raises((AssertionError, AttributeError)):
            check(source.replace(old, new, 1))
