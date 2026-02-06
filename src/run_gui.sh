#!/bin/bash
cd ~/atlas_ws/src/atlas-scanner/src
sudo ~/atlas_ws/src/atlas-scanner/src/setup_camera_permissions.sh
export SKIP_SUDO_CHECK=1
python3 fusion_gui.py
