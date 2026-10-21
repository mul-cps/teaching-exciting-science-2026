"""Launch the prepared classroom notebook without installing anything."""
import importlib.util
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
if importlib.util.find_spec("jupyterlab") is None:
    raise SystemExit("JupyterLab is missing from this Python. Ask the instructor for the prepared environment, or use the plain Python scripts in exercises/.")
subprocess.run([sys.executable, "-m", "jupyterlab", "--ServerApp.root_dir=" + str(root), str(root / "notebooks/Robot_Candy_Factory_Student.ipynb")], check=True, cwd=root)
