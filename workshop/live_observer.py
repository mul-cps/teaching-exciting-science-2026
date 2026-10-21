"""Read-only classroom client. Standard Python; ROS stays in a bounded subprocess."""
import json
import math
from pathlib import Path
import subprocess

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


def demo_samples():
    """Invented teaching data, never presented as recorded or measured hardware."""
    return {'mode': 'DEMO — invented data, no hardware', 'samples': [
        {'elapsed_s': i * .2, 'leader': {'state': 'fresh', 'degrees': {j: 10 + k * 4 + i for k, j in enumerate(JOINTS)}},
         'follower': {'state': 'fresh', 'degrees': {j: 30 + k * 4 + i * .9 for k, j in enumerate(JOINTS)}},
         'teleop': {'state': 'fresh', 'value': {'state': 'active', 'teleop_mode': 'relative', 'reason': 'invented_example'}}}
        for i in range(16)]}


def observe(*, live=False, duration=3):
    """Take a new sample each call. No cached live data and no automatic demo fallback."""
    if not live:
        return demo_samples()
    if not math.isfinite(duration) or not .5 <= duration <= 5:
        raise ValueError('duration must be 0.5 to 5 seconds')
    root = Path(__file__).resolve().parents[1]
    command = [str(root / 'tools/live_observer.sh'), str(duration)]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=duration + 20, check=True)
        report = json.loads(result.stdout)
        if report.get('mode') != 'LIVE — ROS topic observations':
            raise ValueError('unexpected observer response')
        return report
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        detail = getattr(error, 'stderr', None) or str(error)
        return {'mode': 'LIVE unavailable — no measurements', 'samples': [], 'error': detail[:500]}


def plot_samples(report, joint='shoulder_pan'):
    """Plot only fresh samples of a selected body joint, never a gripper proxy."""
    import matplotlib.pyplot as plt
    if joint not in JOINTS:
        raise ValueError('choose one of JOINTS')
    fig, ax = plt.subplots(figsize=(9, 3))
    for label in ('leader', 'follower'):
        points = [(s['elapsed_s'], s[label]['degrees'][joint]) for s in report['samples']
                  if s.get(label, {}).get('state') == 'fresh' and joint in s[label].get('degrees', {})]
        if points:
            ax.plot(*zip(*points), marker='.', label=label)
    ax.set(xlabel='Seconds in this observation', ylabel='Joint angle in degrees', title=report['mode'])
    if ax.lines:
        ax.legend()
    else:
        ax.text(.5, .5, 'No fresh joint samples. Ask instructor; do not infer a pose.', ha='center', transform=ax.transAxes)
    ax.grid(alpha=.2)
    plt.show()
