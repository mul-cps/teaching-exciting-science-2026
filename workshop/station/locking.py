"""Shared process ownership for activation and commissioning entry points."""
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path


def state_dir():
    return Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')) / 'exciting-science'


def station_lock_path():
    return state_dir() / 'station.lock'


@contextmanager
def station_lock(path):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('another process owns the station lock') from error
        try:
            yield
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)
