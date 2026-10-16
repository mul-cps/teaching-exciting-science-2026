#!/usr/bin/env bash
# Rootless station environment. This installs packages, never activates devices.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
install_dir=${STATION_INSTALL_DIR:-"$HOME/.local/share/exciting-science/station"}
python_bin=${STATION_PYTHON:-python3.13}
[[ $(uname -s) == Linux ]] || { echo 'Station hardware requires Linux.' >&2; exit 1; }
"$python_bin" -c 'import sys; assert sys.version_info[:2] in ((3,12),(3,13)), "Use Python 3.12 or 3.13"'
mkdir -p -- "$install_dir"
venv="$install_dir/venv"
if command -v uv >/dev/null 2>&1; then
    [[ -x "$venv/bin/python" ]] || uv venv --python "$python_bin" "$venv"
    pip_install() { uv pip install --python "$venv/bin/python" "$@"; }
    pip_check() { uv pip check --python "$venv/bin/python"; }
else
    [[ -x "$venv/bin/python" ]] || "$python_bin" -m venv "$venv"
    pip_install() { "$venv/bin/python" -m pip install "$@"; }
    pip_check() { "$venv/bin/python" -m pip check; }
fi
"$venv/bin/python" -c 'import sys; assert sys.version_info[:2] in ((3,12),(3,13)), "Existing venv needs Python 3.12 or 3.13"'
pip_install --index-url https://download.pytorch.org/whl/cpu 'torch==2.11.0+cpu' 'torchvision==0.26.0+cpu'
pip_install --index-url https://pypi.org/simple 'torch==2.11.0+cpu' 'torchvision==0.26.0+cpu' "$root[station,digital-twin]"
pip_check
cd -- "$install_dir"
"$venv/bin/python" - <<'PY'
import torch, torchvision, usb.backend.libusb1
from importlib.metadata import version
from importlib.resources import files
import mujoco, viser
assert torch.__version__ == '2.11.0+cpu' and torchvision.__version__ == '0.26.0+cpu'
assert torch.version.cuda is None and torch.version.hip is None
assert version('lerobot') == '0.6.1' and version('pyusb') == '1.3.1'
assert version('mujoco') == '3.14.0' and version('viser') == '1.1.0'
assert usb.backend.libusb1.get_backend() is not None, 'Install system libusb via your administrator'
for name in ('ev3_agent.py', 'station.example.json'):
    assert files('workshop.station').joinpath('resources', name).is_file()
print('Verified CPU packages, passive-twin viewer, resources and system libusb')
PY
config="$install_dir/station.json"
if [[ ! -e "$config" ]]; then
    "$venv/bin/python" - "$config" <<'PY'
import sys
from importlib.resources import files
from pathlib import Path
with Path(sys.argv[1]).open('x') as stream:
    stream.write(files('workshop.station').joinpath('resources/station.example.json').read_text())
PY
fi
# Launcher paths are shell-quoted; preserve user arguments verbatim.
printf '#!/usr/bin/env bash\nexec %q -m workshop.station --config %q "$@"\n' "$venv/bin/python" "$config" > "$install_dir/station"
chmod +x "$install_dir/station"
(cd "$install_dir" && ./station --simulate && ./station --simulate --workflow arm_unloads_sorter && ./station --detect)
printf '\nInstalled launcher: %s\nConfiguration (preserved on rerun): %s\nRead-only detection: %q --detect\nSimulation: %q --simulate\nCommission pose: %q -m workshop.station.commission --config %q --capture-pose NAME --output POSE.json --update-config %q\nAfter full commissioning: %q --activate\n' "$install_dir/station" "$config" "$install_dir/station" "$install_dir/station" "$venv/bin/python" "$config" "$config" "$install_dir/station"
