#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
systemctl --user restart @@UNIT@@-core.service
deadline=$((SECONDS + 20))
while (( SECONDS < deadline )); do
  [[ "$(systemctl --user is-active @@UNIT@@-core.service 2>/dev/null || true)" == active ]] || { sleep 0.2; continue; }
  ss -H -lun 'sport = :15300' | grep -q . && exit 0
  sleep 0.2
done
exit 1
