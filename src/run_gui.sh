#!/bin/bash
cd ~/atlas_ws/src/atlas-scanner/src
# Preserve display vars that sudo may clear
_DISPLAY="$DISPLAY"
_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
sudo ~/atlas_ws/src/atlas-scanner/src/setup_camera_permissions.sh
export DISPLAY="$_DISPLAY"
export XAUTHORITY="$_XAUTHORITY"
export SKIP_SUDO_CHECK=1
python3 fusion_gui.py "$@"
