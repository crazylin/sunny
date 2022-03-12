#!/usr/bin/env bash
set -e

# load uvc extensions for USB3 33 cameras
/usr/bin/tcam-uvc-extension-loader --device=/dev/video0 -f /usr/share/theimagingsource/tiscamera/uvc-extension/usb37.json

# setup ros2 environment
source "/ws/install/setup.bash"
exec "$@"
