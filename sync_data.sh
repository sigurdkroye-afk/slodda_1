#!/bin/bash
# Syncs data files (yaml/launch/xml/rviz) from src to install after config changes.
# No rebuild needed. Run this instead of colcon when only editing config/launch files.
set -e

PKG=slodda_bringup
SHARE="install/${PKG}/share/${PKG}"

cp -v src/${PKG}/launch/*.py       "${SHARE}/launch/"
cp -v src/${PKG}/config/*.yaml     "${SHARE}/config/"
cp -v src/${PKG}/config/*.rviz     "${SHARE}/config/"
cp -v src/${PKG}/behavior_trees/*.xml "${SHARE}/behavior_trees/"
cp -v src/${PKG}/scripts/*.py      "${SHARE}/scripts/" 2>/dev/null || true

echo "Sync done."
