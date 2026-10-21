"""Run this before class. Standard Python only; no internet or devices needed."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    if sys.version_info < (3, 11):
        print("Use Python 3.11 or newer.")
        return 1
    root = Path(__file__).resolve().parents[1]
    env = dict(os.environ, PYTHONPATH=str(root), PYTHONNOUSERSITE="1")
    programs = [(str(root / "exercises" / name),) for name in
                ("01_python.py", "02_robot.py", "03_vision.py", "04_sort.py")]
    programs.append(("-m", "workshop"))
    with tempfile.TemporaryDirectory(prefix="sorting-check-") as directory:
        programs.append((str(root / "exercises/05_visual_sort.py"), "--output", str(Path(directory) / "sorting-result.html")))
        for program in programs:
            result = subprocess.run([sys.executable, "-S", *program], cwd=directory,
                                    env=env, text=True, capture_output=True, timeout=10)
            print(("PASS" if result.returncode == 0 else "FAIL"), " ".join(program))
            if result.returncode:
                print(result.stdout + result.stderr)
                return 1
    print("READY: six offline student entrypoints passed, without installed packages.")
    print("This checks this computer only. Hardware and the other laptops need separate checks.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
