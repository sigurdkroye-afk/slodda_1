#!/usr/bin/env bash
# Kopierer Python-noder og launch/config-filer fra src/ til install/
# Kjøres etter git pull på Pi (unngår colcon rebuild).
set -e
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
SHARE="$REPO/install/slodda_bringup/share/slodda_bringup"
PYSITE="$REPO/install/slodda_bringup/lib/python3.12/site-packages/slodda_bringup"

echo "Kopierer launch-filer..."
sudo cp "$REPO/src/slodda_bringup/launch/"*.py "$SHARE/launch/"

echo "Kopierer config-filer..."
sudo cp "$REPO/src/slodda_bringup/config/"*.yaml "$SHARE/config/"
[ -d "$REPO/src/slodda_bringup/config/"*.xml ] 2>/dev/null && \
  sudo cp "$REPO/src/slodda_bringup/config/"*.xml "$SHARE/config/" || true

echo "Kopierer Python-noder..."
sudo cp "$REPO/src/slodda_bringup/slodda_bringup/"*.py "$PYSITE/"

echo "Ferdig."
