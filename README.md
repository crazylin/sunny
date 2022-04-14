# sunny

## Development requirements

- Ubuntu 20.04 desktop
- Docker CE/EE 18.06+ and Docker Compose 1.21+.
- Visual Studio Code
- Visual Studio Code extension: Remote - Containers (ms-vscode-remote.remote-containers)

## System requirements

- Raspberry Pi 4 Model B (4GB or 8GB)
- Micro-SD card (16GB)
- MobaXterm (SSH)
- VMware (Player or Workstation)
- [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
- [Ubuntu Server 20.04.4 LTS ARM64](https://ubuntu.com/download/raspberry-pi/thank-you?version=20.04.4&architecture=server-arm64+raspi)
- docker, docker-compose

## Prepare system

1. Install ubuntu to a microSD card.
1. SSH into ubuntu. (user: ubuntu, password: ubuntu)
1. [Install Docker Engine on Ubuntu.](https://docs.docker.com/engine/install/ubuntu/)
1. [Install docker-compose.](https://docs.docker.com/compose/install/)
1. Use docker pull image: zhuoqiw/sunny-tis:latest

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
