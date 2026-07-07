#!/bin/bash
# Install and start PTP master services for Livox MID360 sync.
# Run once: sudo ./install_ptp.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG_SRC="$SCRIPT_DIR/config/ptp4l_livox.cfg"

sudo mkdir -p /etc/linuxptp
sudo cp "$SCRIPT_DIR/config/ptp4l_livox.cfg"      /etc/linuxptp/ptp4l_livox.cfg
sudo cp "$SCRIPT_DIR/config/ts2phc_livox.cfg"     /etc/linuxptp/ts2phc_livox.cfg
sudo cp "$SCRIPT_DIR/config/phc_step_to_sys.sh"   /etc/linuxptp/phc_step_to_sys.sh
sudo cp "$SCRIPT_DIR/config/phc_sync.py"          /etc/linuxptp/phc_sync.py
sudo chmod +x /etc/linuxptp/phc_step_to_sys.sh
sudo cp "$SCRIPT_DIR/config/ptp4l-livox.service"  /etc/systemd/system/
sudo cp "$SCRIPT_DIR/config/phc2sys-livox.service" /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl restart ptp4l-livox.service
sudo systemctl restart phc2sys-livox.service

echo ""
echo "Waiting 8s for PTP to converge..."
sleep 8
echo ""
echo "=== ptp4l status ==="
sudo journalctl -u ptp4l-livox.service -n 20 --no-pager
echo ""
echo "=== phc2sys status ==="
sudo journalctl -u phc2sys-livox.service -n 10 --no-pager
echo ""
echo "=== PHC vs system clock offset ==="
sudo phc_ctl /dev/ptp0 get 2>/dev/null && date +%s.%N
