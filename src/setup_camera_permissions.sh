#!/bin/bash

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Installs persistent udev rules for Insta360 cameras and sets device
# permissions. Supports multiple cameras: creates /dev/insta (first camera, legacy
# compat) plus /dev/insta0, /dev/insta1, /dev/insta2 indexed by USB bus path.

echo "Setting up Insta360 camera permissions..."

_sudo() { [ "$(id -u)" = "0" ] && "$@" || sudo "$@"; }

# Multi-camera udev rule: creates /dev/insta symlink for the first camera found
# and /dev/instaN for each camera indexed by kernel device order.
RULE_FILE="/etc/udev/rules.d/99-insta.rules"
# Per-serial symlinks ensure /dev/insta_<SERIAL> always resolves to the right
# device node regardless of USB enumeration order. /dev/insta is kept as a
# legacy compat link pointing to the first camera by serial (cam_0 = X5).
RULES='SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", MODE="0777"
SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", SYMLINK+="insta_%s{serial}", MODE="0777"
SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", RUN+="/bin/sh -c '\''test -e /dev/insta || ln -sf /dev/%k /dev/insta'\''"
SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", ATTR{serial}=="IAHEA26019RESN", SYMLINK+="insta", MODE="0777"'

if ! diff -q <(echo "$RULES") "$RULE_FILE" > /dev/null 2>&1; then
    echo "$RULES" | _sudo tee "$RULE_FILE" > /dev/null
    _sudo udevadm control --reload-rules
    _sudo udevadm trigger
    sleep 1
fi

echo "Waiting for camera device(s)..."
FOUND=0
for i in {1..15}; do
    # Count Insta360 USB devices by vendor ID
    FOUND=$(lsusb -d 2e1a: 2>/dev/null | wc -l)
    if [ "$FOUND" -gt 0 ]; then
        break
    fi
    echo "Waiting for Insta360 camera(s)... (attempt $i/15)"
    sleep 1
done

if [ "$FOUND" -eq 0 ]; then
    echo "✗ No Insta360 camera found. Please check:"
    echo "  1. Camera(s) connected via USB"
    echo "  2. Camera(s) in Android USB mode"
    echo "  3. Run 'lsusb -d 2e1a:' to verify detection"
    exit 1
fi

# Set permissions on all Insta360 USB device nodes
for dev in /dev/bus/usb/*/; do
    for devfile in "$dev"*; do
        [ -f "$devfile" ] || continue
        # Check if this is an Insta360 device
        _vid=$(cat "/sys/bus/usb/devices/$(udevadm info -q name -n "$devfile" 2>/dev/null)/idVendor" 2>/dev/null)
        if [ "$_vid" = "2e1a" ]; then
            _sudo chmod 777 "$devfile" 2>/dev/null
        fi
    done
done

echo "✓ Found $FOUND Insta360 camera(s)"
lsusb -d 2e1a: 2>/dev/null | while read -r line; do
    echo "  $line"
done
