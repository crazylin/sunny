# sunny

## Development system requirements

- Ubuntu 20.04 desktop
- Git
- Docker CE/EE 18.06+ and Docker Compose 1.21+.
- Visual Studio Code
- Visual Studio Code extension: Remote - Containers (ms-vscode-remote.remote-containers)

## Embedded system requirements

- Raspberry Pi 4 Model B (4GB or 8GB)
- Micro-SD card (16GB)
- MobaXterm (SSH)
- VMware (Player or Workstation)
- [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
- [Ubuntu Server 20.04.4 LTS ARM64](https://ubuntu.com/download/raspberry-pi/thank-you?version=20.04.4&architecture=server-arm64+raspi)
- Docker CE/EE 18.06+ and Docker Compose 1.21+.

## GUI application requirements

- Ubuntu 20.04 desktop
- Docker CE/EE 18.06+ and Docker Compose 1.21+.

## Prepare development system

1. Clone this repository itself: `git clone https://github.com/zhuoqiw/sunny.git`
1. CD into local repository and start vscode: `code .`
1. Install Remote-Containers extension
1. In Menu->View->Command Palette, find Remote-Container: Open Folder in Container...
1. Follow the instructions

## Prepare embedded system

1. Install ubuntu to a microSD card.
1. SSH into ubuntu. (user: ubuntu, password: ubuntu)
1. [Install Docker Engine on Ubuntu.](https://docs.docker.com/engine/install/ubuntu/)
1. [Install docker-compose.](https://docs.docker.com/compose/install/)
1. Pull deploy image: `docker pull zhuoqiw/sunny-tis:deploy`
1. Pull tiscamera image: `docker pull zhuoqiw/ros-tis:0.14.0`
1. Install udev rules for tiscamera:
    ```bash
    docker run --name tis zhuoqiw/ros-tis:0.14.0
    sudo docker cp tis:setup/etc /
    sudo docker cp tis:setup/usr /
    sudo udevadm control --reload-rules
    docker rm tis
    ```
1. Reboot
1. Download [docker-compose.yml](https://github.com/zhuoqiw/sunny/blob/main/docker-compose.yml)
1. Boot up system: `docker-compose up`

## Prepare GUI application

1. Pull desktop image: `docker pull zhuoqiw/sunny-tis:desktop`
1. Start X Authentication: `xhost +local:root`
1. Start the container: 
    `docker run --rm --network=host --env=DISPLAY -v /tmp/.X11-unix zhuoqiw/sunny-tis:desktop`
1. Stop X Authentication: `xhost -local:root`

## System optimization

- Turn off apt-daily service (/etc/apt/apt.conf.d/20auto-upgrades)
- [Remove snap](https://linuxhint.com/turn-off-snap-ubuntu/)
- [Remove cloud-init](https://blog.rylander.io/2020/12/23/how-to-remove-cloud-init-from-ubuntu-server-20-04/) sudo touch /etc/cloud/cloud-init.disabled

## Name convention

| Context      | Syntax | Example |
| :---------- | :---------- | :---------- |
| ROS2 package name      | abc_def       | intra_process_demo |
| ROS2 node name      | abc_def_node       | camera_mvs_node |
| ROS2 node header(c++)      | abc_def.hpp      | camera_mvs.hpp |
| ROS2 node source(c++)      | abc_def.cpp      | camera_mvs.cpp |
| ROS2 node namespace(c++)      | abc_def      | namespace camera_mvs |
| ROS2 node class(c++)      | AbcDef      | class CameraMvs |
| ROS2 node header include guard(c++)      | ABC_DEF__ABC_DEF_HPP_      | PACKAGE_NAME__CLASS_NAME_HPP_ |
| ROS2 service name      | abc_def       | set_parameters, get_parameters |
| ROS2 service type   | AbcDef        | SetParameters, GetParameters |
| ROS2 srv file name   | AbcDef.srv        | SetParameters.srv, GetParameters.srv |
| ROS2 topic name      | abc_def       | parameter_events |
| ROS2 topic type   | AbcDef        | ParameterEvent |
| ROS2 msg file name   | AbcDef.msg        | ParameterEvent.msg |
| ROS2 launch file name      | main.launch.py       |  |
| Var in srv or msg | abc_def | new_parameters |

## Coding style

```cpp
// derived.hpp
namespace derived_space
{

class Derived : public Base
{
}

}  // namespace derived_space

// derived.cpp
namespace derived_space
{

Derived::Derived()
: Base()
{
}

}  // namespace derived_space

if (true) {
  // Todo
} else {
  // Todo
}

if (
  a ||
  b ||
  c)
{
  // Todo
} else {
  // Todo
}
```
