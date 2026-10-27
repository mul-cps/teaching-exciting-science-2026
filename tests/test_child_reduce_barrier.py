"""Guard collective control flow when one reducer lane has invalid input."""
from pathlib import Path
import re

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "ros2/src/so101_station_vulkan/shaders/children.slang"


def reducer(source):
    start = source.index("void child_reduce(")
    end = source.index("[numthreads", start)
    return source[start:end]


def check(source):
    text = reducer(source)
    # groupState publishes the same fault snapshot to every lane before this exit.
    uniform_exit = "groupState(k);if(groupFault!=0)return;"
    assert uniform_exit in text
    assert len(re.findall(r"\breturn\s*;", text)) == 1
    remainder = text.split(uniform_exit, 1)[1]
    assert remainder.count("GroupMemoryBarrierWithGroupSync();") == 2
    initialized, contribution, emission = remainder.split("GroupMemoryBarrierWithGroupSync();")
    assert "hashKeys[k]=sums[k]=0;" in initialized
    # Both failure arms skip the complete parent/hash/stat contribution block.
    guard = "if(par==0||root>i+1||graphParent(root-1)!=par){fault(4);}else if(par>hl(3)){fault(64);}else{"
    assert guard in contribution
    assert contribution.index(guard) < contribution.index("parentWord(par-1,4)")
    assert "hashSlot(root)" in contribution
    assert "InterlockedAdd(sums[slot],1,ignored)" in contribution
    assert "uint root=hashKeys[k];" in emission


def test_invalid_lane_joins_final_reduction_barrier():
    check(SOURCE.read_text())


def test_rejects_divergent_exit_and_unconditional_contribution():
    source = SOURCE.read_text()
    check(source)
    mutations = (
        ("{fault(4);}else if(par>hl(3))", "{fault(4);return;}else if(par>hl(3))"),
        ("{fault(4);}else if(par>hl(3))", "{fault(4);}if(par>hl(3))"),
        ("{fault(64);}else{\n  uint px=parentWord", "{fault(64);}{\n  uint px=parentWord"),
        ("if(par==0||root>i+1||graphParent(root-1)!=par)", "if(root>i+1||graphParent(root-1)!=par)"),
    )
    for old, new in mutations:
        assert old in reducer(source)
        # Restrict substitutions to the owned function.
        mutated = source.replace(reducer(source), reducer(source).replace(old, new, 1), 1)
        with pytest.raises(AssertionError):
            check(mutated)
