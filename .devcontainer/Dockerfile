# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.231.5/containers/cpp/.devcontainer/base.Dockerfile

# [Choice] Debian / Ubuntu version (use Debian 11, Ubuntu 18.04/21.04 on local arm64/Apple Silicon): debian-11, debian-10, ubuntu-21.04, ubuntu-20.04, ubuntu-18.04
# ARG VARIANT="bullseye"
# FROM mcr.microsoft.com/vscode/devcontainers/cpp:0-${VARIANT}

# [Optional] Install CMake version different from what base image has already installed. 
# CMake reinstall choices: none, 3.21.5, 3.22.2, or versions from https://cmake.org/download/
# ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="none"

# Optionally install the cmake for vcpkg
# COPY ./reinstall-cmake.sh /tmp/
# RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
#         chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
#     fi \
#     && rm -f /tmp/reinstall-cmake.sh

# [Optional] Uncomment this section to install additional vcpkg ports.
# RUN su vscode -c "${VCPKG_ROOT}/vcpkg install <your-port-name-here>"

# [Optional] Uncomment this section to install additional packages.
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>

# Build enviroments
FROM zhuoqiw/ros-opencv:4.5.5 AS opencv

FROM zhuoqiw/ros-tis:0.14.0 AS tiscamera

FROM ros:galactic AS runtime

COPY --from=tiscamera /tiscamera.deb /tiscamera.deb
COPY --from=opencv /opt/opencv /opt/opencv

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    gpiod \
    libmodbus5 \
    /tiscamera.deb \
    && rm -rf /var/lib/apt/lists/* /tiscamera.deb

RUN echo "/opt/opencv/lib" >> /etc/ld.so.conf.d/opencv.conf \
    && ldconfig

RUN echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc

FROM runtime AS dev

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    libgpiod-dev \
    libmodbus-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgirepository1.0-dev \
    && rm -rf /var/lib/apt/lists/*

FROM dev AS CI

COPY ./src /workspace/sunny/src

WORKDIR /workspace/sunny

RUN /ros_entrypoint.sh colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to pipeline_tis \
    && rm -rf build log

FROM runtime AS deploy

COPY --from=CI /workspace/sunny/install /workspace/sunny/install
COPY ./ros_entrypoint.sh /ros_entrypoint.sh

RUN sed -i '$c source /workspace/sunny/install/setup.bash' /root/.bashrc
