"""Prepare and decode a disposable, non-qualifying child_hook fault sample."""

import argparse
import hashlib
import json
from pathlib import Path


SOURCE_SHA256 = "8412aec1442aed563f99581a11bd3105db60b60947b5909ccf4e2e57ec0fe0ec"
HOOK_PREFIX = b"[numthreads(256,1,1)]\nvoid child_hook("
HELPER = b"""void hookFaultSample(uint i,uint root,uint par){
    uint payload=0x80000004u | ((i&255u)<<7) | ((root&255u)<<15) | ((par&63u)<<23) | (uint(par>63u)<<29) | (uint(root>255u)<<30);
    uint ignored;
    compact.InterlockedCompareExchange(28,0u,payload,ignored);
    fault(4);
}
"""


def prepare(source: Path, destination: Path) -> dict:
    """Write an exclusive derived-source directory after checking pinned bytes."""
    original = source.read_bytes()
    original_sha = hashlib.sha256(original).hexdigest()
    if original_sha != SOURCE_SHA256:
        raise ValueError("source does not match the pinned children.slang SHA256")
    insertion = original.index(HOOK_PREFIX)
    hook_end = original.index(b"\n", insertion + len(HOOK_PREFIX))
    hook = original[insertion:hook_end]
    if hook.count(b"fault(4)") != 1:
        raise ValueError("expected exactly one child_hook fault(4) call")
    patched_hook = hook.replace(b"fault(4)", b"hookFaultSample(i,root,par)", 1)
    derived = original[:insertion] + HELPER + patched_hook + original[hook_end:]
    manifest = {
        "original_source_sha256": original_sha,
        "derived_source_sha256": hashlib.sha256(derived).hexdigest(),
        "scope": "child_hook_fault_sample_only",
        "qualifying": False,
        "required_fixture_sha256": "e85eb99b89f9173a5ffd50f85638d8b5542dd482f90100d116012da9c368c3dd",
        "required_width": 7,
        "required_height": 5,
        "required_dispatch_groups": 1,
        "limitations": [
            "The sample is not the first wall-clock fault or round.",
            "The 8-bit i capture is only valid for this 7x5 fixture in one 256-lane dispatch group.",
            "Any earlier nonzero fault prevents CAS capture; an absent marker does not exclude a hook failure.",
            "No rootParent capture is performed.",
            "The current host qualifier may perform its existing 560B raw copy only if the unmarked fault is exactly 4.",
        ],
    }
    destination.mkdir(exist_ok=False)
    (destination / "children.slang").write_bytes(derived)
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def decode(word: int) -> dict:
    """Decode a uint32 without interpreting unmarked words as samples."""
    if isinstance(word, bool) or not isinstance(word, int):
        raise TypeError("word must be an integer uint32")
    if not 0 <= word < 2**32:
        raise ValueError("word must be in the uint32 range")
    result = {
        "raw_word": word,
        "low_fault_bits": word & 127,
        "captured": bool(word & 0x80000000),
    }
    if result["captured"]:
        result.update(
            i=(word >> 7) & 255,
            root_mod_256=(word >> 15) & 255,
            parent_mod_64=(word >> 23) & 63,
            root_overflow=bool(word & (1 << 30)),
            parent_overflow=bool(word & (1 << 29)),
        )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prepare_parser = commands.add_parser("prepare")
    prepare_parser.add_argument("source", type=Path)
    prepare_parser.add_argument("destination", type=Path)
    decode_parser = commands.add_parser("decode")
    decode_parser.add_argument("word", type=lambda value: int(value, 0))
    args = parser.parse_args()
    if args.command == "prepare":
        result = prepare(args.source, args.destination)
    else:
        result = decode(args.word)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
