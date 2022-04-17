# camera_tis (c++)

The imaging souce camera library (tiscamera), warpped in ROS2.

## Package, node name

camera_tis::camera_tis_node

## Publisher

/camera_tis_node/image

## Usage information

The imaging source camera did not provide debian install file under ubuntu 20.04.  
Instead, it is configurated, built and packaged by another repository.  
For more details, see [github: ros-tis](https://github.com/zhuoqiw/ros-tis)  
And [dockerhub: ros-tis](https://hub.docker.com/repository/docker/zhuoqiw/ros-tis/general)

To successfully utilize the camera hardware:  
- Make sure the camera is connected with USB 3.0 by using `lsusb` etc. (host)
- Check the UDev rule is installed correctly under `/et/udev/rules.d`. (host)
- Check `tcam-uvc-extension-loader` is installed under `/usr/bin`. (host)
- Check `libtcam-uvc-extension.so` is installed under `/usr/lib`. (host)
- Check `uvc-extension` is installed under `/usr/share/theimagingsource/tiscamera`. (host)
- Make sure the `/dev/video*` is mapped into container via --device option. (host, container)
- Make sure the user is in group: video if you plan to use it as non root. (container)
- Make sure the debian install file runs successfully.