#!/usr/bin/env bash
set -e

dev=$(lsusb | grep 'The Imaging Source')
vid=${dev:23:4}
pid=${dev:28:2}

# load uvc extensions for USB3 33 cameras
if [ ${vid} = 199e ] && [ ${pid} = 94 ] && [ -e /dev/video0 ]
then
    tcam-uvc-extension-loader --device=/dev/video0 -f /usr/share/theimagingsource/tiscamera/uvc-extension/usb37.json
fi

# setup ros2 environment
source "/workspace/sunny/install/setup.bash"
exec "$@"
