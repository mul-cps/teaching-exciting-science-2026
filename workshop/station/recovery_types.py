"""Shared immutable recovery policy, identical for CLI and imported owners."""
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraRecoveryPolicy:
    """Trusted in-process identities; never populated from a motion command."""
    model_sha256: str
    calibration_sha256: str
    camera_serial: str
    arm_calibration_sha256: str



# Existing bounded camera-reentry policy, shared by planning and observation.
CAMERA_RECOVERY_MAX_DISPLACEMENT_COUNTS = 32
CAMERA_RECOVERY_INWARD_COUNTS = 2


OWNER_SOURCE_MODULES = ('arm', 'arm_session', 'recovery_runtime', 'recovery_types', 'recovery_runtime_config', 'camera_recovery', 'idle_telemetry')


def owner_source_sha256(station_dir):
    """Hash exact authority sources once per startup/generation; never a tick."""
    import hashlib
    from pathlib import Path
    root = Path(station_dir)
    return {name: hashlib.sha256((root/(name+'.py')).read_bytes()).hexdigest()
            for name in OWNER_SOURCE_MODULES}
