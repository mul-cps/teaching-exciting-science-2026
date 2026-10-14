#!/usr/bin/env bash
# Install the persistent native API on the identified lab EV3; no motion on start.
set -euo pipefail
ev3_api_host=${1:-192.168.0.1}
ev3_api_key=${2:-${XDG_DATA_HOME:-$HOME/.local/share}/exciting-science/station/ssh/id_ev3}
ev3_api_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ev3_api_source="$ev3_api_root/tools/ev3_native/live_api.py"
[[ -f "$ev3_api_source" && -f "$ev3_api_key" ]] || { echo 'API source or EV3 SSH key missing' >&2; exit 1; }
ev3_api_auth=(-o BatchMode=yes -o StrictHostKeyChecking=yes -o IdentitiesOnly=yes -i "$ev3_api_key")
ev3_api_remote="robot@$ev3_api_host"
ev3_api_id=$(ssh "${ev3_api_auth[@]}" "$ev3_api_remote" 'cat /etc/machine-id')
[[ "$ev3_api_id" == 0a8ffb6d314b403ab9cb5dc4ba3f599d ]] || { echo 'Unexpected EV3 machine identity' >&2; exit 1; }
ssh "${ev3_api_auth[@]}" "$ev3_api_remote" 'command -v brickrun >/dev/null && command -v pybricks-micropython >/dev/null && command -v flock >/dev/null && mkdir -p /home/robot/cps-station/ev3_native /home/robot/.config/systemd/user'
scp "${ev3_api_auth[@]}" "$ev3_api_source" "$ev3_api_remote:/home/robot/cps-station/ev3_native/live_api.py"
ev3_api_unit=$(mktemp)
trap 'rm -f -- "$ev3_api_unit"' EXIT
cat > "$ev3_api_unit" <<'UNIT'
[Unit]
Description=CPS EV3 persistent native command API

[Service]
Type=simple
WorkingDirectory=%h/cps-station/ev3_native
ExecStart=/usr/bin/flock -n %h/cps-station/ev3-live-api.lock /usr/bin/brickrun -r -- /usr/bin/pybricks-micropython %h/cps-station/ev3_native/live_api.py
Restart=on-failure
RestartSec=2
KillSignal=SIGINT
TimeoutStopSec=5

[Install]
WantedBy=default.target
UNIT
scp "${ev3_api_auth[@]}" "$ev3_api_unit" "$ev3_api_remote:/home/robot/.config/systemd/user/cps-ev3-api.service"
ssh "${ev3_api_auth[@]}" "$ev3_api_remote" 'systemctl --user daemon-reload && systemctl --user enable cps-ev3-api.service && systemctl --user restart cps-ev3-api.service && systemctl --user is-active cps-ev3-api.service && loginctl show-user robot -p Linger'
printf 'Forward the API with: ssh -N -T -o ExitOnForwardFailure=yes -o BatchMode=yes -o StrictHostKeyChecking=yes -o IdentitiesOnly=yes -i %q -L 127.0.0.1:18766:127.0.0.1:8766 %q\n' "$ev3_api_key" "$ev3_api_remote"
printf 'Then: python -m workshop.station.ev3_live_client --port 18766 status\n'
