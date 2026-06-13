#!/usr/bin/env bash
# Real-hardware CAN-FD bring-up for OpenArm 2.0 (reference; needs a SocketCAN
# adapter). The software demo does NOT use this — see `make can` instead.
# Mirrors docs.openarm.dev.
set -euo pipefail

for IFACE in can0 can1; do
  echo "configuring ${IFACE} (CAN-FD 1M/5M)"
  sudo ip link set "${IFACE}" down || true
  sudo ip link set "${IFACE}" type can bitrate 1000000 dbitrate 5000000 fd on
  sudo ip link set "${IFACE}" up
  ip link show "${IFACE}"
done

# Or use the OpenArm helper:
#   openarm-can-cli -i can0 can_configure
#   openarm-can-cli -i can1 can_configure

# Zero-position calibration (one arm at a time; the arm WILL move):
#   openarm-can-zero-position-calibration --canport can0 --arm-side right_arm
#   openarm-can-zero-position-calibration --canport can1 --arm-side left_arm

# Verify telemetry:
#   openarm-can-cli -i can0 monitor
