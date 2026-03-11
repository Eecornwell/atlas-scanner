#!/bin/bash

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Launcher script for the ATLAS GUI. Sets up the display environment and starts fusion_gui.py.
cd ~/atlas_ws/src/atlas-scanner/src
export SKIP_SUDO_CHECK=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$(dirname "$0")/config/fastdds_atlas.xml"
source ~/atlas_ws/install/setup.bash
python3 fusion_gui.py "$@"
