#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd -- "$root"
exec "${STATION_TWIN_PYTHON:-python3}" -m workshop.digital_twin "$@"
