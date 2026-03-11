#!/bin/bash

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Installs a persistent udev rule for the Insta360 camera and sets device permissions. Only requires elevated privileges on first-time setup when the rule is not yet present.

echo "Setting up Insta360 camera permissions..."

# Add udev rule for Insta360 camera (only if not already present)
# If already root (e.g. called via pkexec), run commands directly; otherwise use sudo
_sudo() { [ "$(id -u)" = "0" ] && "$@" || sudo "$@"; }

RULE='SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", SYMLINK+="insta", MODE="0777"'
if ! grep -q "$RULE" /etc/udev/rules.d/99-insta.rules 2>/dev/null; then
    echo "$RULE" | _sudo tee /etc/udev/rules.d/99-insta.rules > /dev/null
    _sudo udevadm control --reload-rules
    _sudo udevadm trigger
    sleep 1
fi

echo "Waiting for camera device..."
for i in {1..15}; do
    if [ -e /dev/insta ]; then
        _sudo chmod 777 /dev/insta
        echo "✓ Camera permissions set successfully"
        ls -la /dev/insta
        exit 0
    else
        echo "Waiting for /dev/insta... (attempt $i/15)"
        sleep 1
    fi
done

echo "✗ Camera device not found. Please check:"
echo "1. Camera is connected via USB"
echo "2. Camera is in the correct mode"
echo "3. Run 'lsusb | grep -i insta' to verify detection"