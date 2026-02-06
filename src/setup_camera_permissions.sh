#!/bin/bash

echo "Setting up Insta360 camera permissions..."

# Add udev rule for Insta360 camera (only if not already present)
RULE='SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", SYMLINK+="insta", MODE="0777"'
if ! grep -q "$RULE" /etc/udev/rules.d/99-insta.rules 2>/dev/null; then
    echo "$RULE" | sudo tee /etc/udev/rules.d/99-insta.rules > /dev/null
    # Reload udev rules only if we added a new rule
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    sleep 1
fi

# Wait for device and set permissions
echo "Waiting for camera device..."
for i in {1..15}; do
    if [ -e /dev/insta ]; then
        sudo chmod 777 /dev/insta
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