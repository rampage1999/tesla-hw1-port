#!/bin/bash
# Tesla HW1 port installer for comma devices (comma 3/3X/4, works with Chestnut companion compute)
# Installs the sunnypilot+Tesla HW1 port to /data/tesla-hw1-port WITHOUT touching the active fork.
#   usage:  bash install-tesla-port.sh          # install
#           bash install-tesla-port.sh --force  # reinstall (removes existing target only)
set -euo pipefail

REPO=https://github.com/rampage1999/tesla-hw1-port
TAG=v1
TARGET=${TARGET:-/data/tesla-hw1-port}
B=/data/tesla-port-bundles
mkdir -p "$B"

if [ -d "$TARGET" ]; then
  if [ "${1:-}" = "--force" ]; then sudo rm -rf "$TARGET"; else echo "ERROR: $TARGET exists. Re-run with --force to replace it (active fork is never touched)."; exit 1; fi
fi

echo "== downloading metarepo bundle"
curl -fL --retry 3 -o "$B/sunnypilot-tesla-hw1-port.bundle" \
  "$REPO/releases/download/$TAG/sunnypilot-tesla-hw1-port.bundle"

echo "== laying down repo (bundle-only metarepo; submodules from GitHub)"
mkdir -p "$TARGET" && cd "$TARGET"
git init -q -b master .
git remote add origin "$REPO"
git -c fetch.recurseSubmodules=false fetch --force --update-head-ok "$B/sunnypilot-tesla-hw1-port.bundle" port-final:tesla-hw1-port
git checkout -q -f tesla-hw1-port

echo "== submodules (panda/msgq/opendbc/neural-network-data etc. resolve publicly from GitHub)"
for sm in msgq_repo panda rednose_repo teleoprtc_repo tinygrad_repo openpilot/sunnypilot/neural_network_data opendbc_repo; do
  echo "-- $sm"
  git -c fetch.recurseSubmodules=false submodule update --init -f --single-branch "$sm" 2>&1 | tail -1 || true
done

# safety net: if submodule init missed the port commit (custom submodule SHA), fetch it directly
OD=$(git ls-tree HEAD opendbc_repo | awk '{print $3}')
if [ "$(cd opendbc_repo && git rev-parse HEAD 2>/dev/null)" != "$OD" ]; then
  echo "-- opendbc_repo: fetching port commit $OD directly"
  (cd opendbc_repo && git fetch --force origin "$OD") && git -c fetch.recurseSubmodules=false submodule update --init -f opendbc_repo
fi

echo "== verify"
G=$(git ls-tree HEAD opendbc_repo | awk '{print $3}')
C=$(cd opendbc_repo && git rev-parse HEAD)
[ "$G" = "$C" ] || { echo "FATAL: gitlink mismatch $G != $C"; exit 1; }
test -f opendbc_repo/opendbc/safety/modes/tesla_legacy.h || { echo "FATAL: safety mode missing"; exit 1; }
echo "GITLINK_OK ($G)"

echo "== launch script"
printf '#!/usr/bin/env bash\ncd /data/tesla-hw1-port || exit 1\nexec ./launch_openpilot.sh\n' > /data/continue-tesla-port.sh
chmod +x /data/continue-tesla-port.sh

cat <<'EOF'

INSTALL COMPLETE. Nothing is running yet — your current setup is untouched.

Activate (device must be OFFROAD):
  sudo cp /data/continue.sh /data/continue.sh.bak-$(date +%s)
  sudo cp /data/continue-tesla-port.sh /data/continue.sh
  sudo reboot

Rollback:
  sudo cp /data/continue.sh.bak-* /data/continue.sh   # newest backup
  sudo reboot
EOF
