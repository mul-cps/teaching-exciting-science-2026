"""Historical performance evidence binds its actual source, independently of later edits."""
import hashlib
import importlib.util
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'docs/performance/evidence/fwmini4-quality-profiles-20261027/selfcheck.py'
spec = importlib.util.spec_from_file_location('profile_evidence_selfcheck', SCRIPT)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


@pytest.fixture
def source_repository(tmp_path):
    subprocess.run(['git', 'init', '-q', str(tmp_path)], check=True)
    source = tmp_path / 'source.cpp'
    source.write_bytes(b'original measured source\n')
    subprocess.run(['git', 'add', 'source.cpp'], cwd=tmp_path, check=True)
    subprocess.run(['git', '-c', 'user.name=Evidence Test', '-c', 'user.email=evidence@example.invalid',
                    'commit', '-qm', 'Freeze measured source'], cwd=tmp_path, check=True)
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=tmp_path, text=True).strip()
    return tmp_path, revision, {'source.cpp': hashlib.sha256(source.read_bytes()).hexdigest()}


def test_later_working_tree_edits_do_not_invalidate_measured_revision(source_repository):
    root, revision, pins = source_repository
    (root / 'source.cpp').write_bytes(b'next implementation, not yet measured\n')
    assert hashlib.sha256((root / 'source.cpp').read_bytes()).hexdigest() != pins['source.cpp']
    checker.verify_source_revision(root, revision, pins)


def test_changed_expected_source_is_still_rejected(source_repository):
    root, revision, pins = source_repository
    pins['source.cpp'] = hashlib.sha256(b'a different implementation\n').hexdigest()
    with pytest.raises(ValueError, match='source revision hash mismatch'):
        checker.verify_source_revision(root, revision, pins)


@pytest.mark.parametrize('revision', ['HEAD', 'main', 'abc123', '0' * 39])
def test_mutable_or_short_source_revisions_are_rejected(source_repository, revision):
    root, _, pins = source_repository
    with pytest.raises(ValueError, match='full immutable Git commit ID'):
        checker.verify_source_revision(root, revision, pins)


def test_missing_source_is_not_silently_ignored(source_repository):
    root, revision, pins = source_repository
    with pytest.raises(subprocess.CalledProcessError):
        checker.verify_source_revision(root, revision, {'absent.cpp': pins['source.cpp']})
