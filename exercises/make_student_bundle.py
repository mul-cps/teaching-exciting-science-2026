"""Create a small offline ZIP: unzip, then run exercises/check_setup.py."""
import argparse
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("student-workshop.zip"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    files = sorted((root / "exercises").glob("*.py"))
    files += [root / "workshop" / name for name in ("__init__.py", "__main__.py", "factory.py")]
    with ZipFile(args.output, "w", ZIP_DEFLATED) as archive:
        for path in files:
            archive.write(path, path.relative_to(root))
        archive.writestr("START-HERE.txt", "Autonomous Robot Sorting — offline student edition\n"
            "Requires Python 3.11 or newer. No installation, internet, ROS or hardware required.\n"
            "Unzip this folder, open a terminal in it, and run:\n"
            "  python exercises/check_setup.py\n"
            "  python exercises/01_python.py\n"
            "Continue with 02_robot.py, 03_vision.py and 04_sort.py.\n"
            "For animation run: python exercises/05_visual_sort.py\n"
            "Then open sorting-result.html in a browser. Edit rules in the Python file.\n"
            "If python is unavailable try python3 (Linux/macOS) or py (Windows).\n"
            "This bundle contains simulation only; the optional --camera mode is not included.\n")
    print("Created:", args.output.resolve())


if __name__ == "__main__":
    main()
