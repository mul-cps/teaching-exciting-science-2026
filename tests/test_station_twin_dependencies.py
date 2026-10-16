from importlib.metadata import PackageNotFoundError, requires
from pathlib import Path
import tomllib

import pytest

packaging = pytest.importorskip("packaging.requirements")
version_types = pytest.importorskip("packaging.version")


def test_station_installer_selects_pinned_passive_twin_extra():
    root = Path(__file__).resolve().parents[1]
    project = tomllib.loads((root / "pyproject.toml").read_text())
    extra = project["project"]["optional-dependencies"]["digital-twin"]
    twin_numpy = next(packaging.Requirement(item) for item in extra
                      if packaging.Requirement(item).name.lower() == "numpy")
    assert twin_numpy.specifier == "==2.2.6"
    assert '"$root[station,digital-twin]"' in (root / "tools/install_station.sh").read_text()

    constraints = []
    for distribution in ("lerobot", "cmeel-boost"):
        try:
            distribution_requirements = requires(distribution) or []
        except PackageNotFoundError:
            continue
        for text in distribution_requirements:
            requirement = packaging.Requirement(text)
            if (requirement.name.lower() == "numpy"
                    and (requirement.marker is None or requirement.marker.evaluate())):
                constraints.append((distribution, requirement.specifier))
    if not constraints:
        pytest.skip("station control distributions are absent from this Python environment")
    selected = version_types.Version("2.2.6")
    for distribution, specifier in constraints:
        assert selected in specifier, f"digital-twin NumPy pin conflicts with {distribution}: {specifier}"
