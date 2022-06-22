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

# ROS code name: galactic, humble
ARG TAG

# Ubuntu code name: focal, jammy
ARG CODENAME

# Build enviroments
FROM zhuoqiw/ros-opencv:${CODENAME}-4.5.5 AS opencv

FROM zhuoqiw/ros-tis:${CODENAME}-1.0.0 AS tiscamera

FROM ros:${TAG} AS runtime

COPY --from=tiscamera /tiscamera.deb /tiscamera.deb
COPY --from=opencv /opt/opencv /opt/opencv
COPY ./tools/ros2_numpy /usr/lib/python3/dist-packages/ros2_numpy

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    nano \
    gpiod \
    libmodbus5 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    /tiscamera.deb \
    && rm -rf /var/lib/apt/lists/* /tiscamera.deb

RUN echo "/opt/opencv/lib" >> /etc/ld.so.conf.d/opencv.conf \
    && ldconfig

RUN echo "source /opt/ros/${TAG}/setup.bash" >> /root/.bashrc

FROM runtime AS dev

ARG TAG

# Create a non-root user
RUN groupadd --gid 1000 ros \
    && useradd -s /bin/bash --uid 1000 --gid 1000 -m ros \
    && usermod -a -G video ros \
    && echo "source /opt/ros/${TAG}/setup.bash" >> /home/ros/.bashrc

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    gdb \
    python3-tk \
    python3-pil \
    python3-matplotlib \
    libgpiod-dev \
    libmodbus-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgirepository1.0-dev \
    && rm -rf /var/lib/apt/lists/*

FROM dev AS build

COPY ./src /workspace/sunny/src

WORKDIR /workspace/sunny

RUN /ros_entrypoint.sh colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to pipeline_tis \
    && rm -rf build log

FROM runtime AS deploy

COPY --from=build /workspace/sunny/install /workspace/sunny/install

RUN sed -i '$c source /workspace/sunny/install/setup.bash' /root/.bashrc

RUN sed -i '/source/c source /workspace/sunny/install/setup.bash' /ros_entrypoint.sh

FROM ros:${TAG} AS gui

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    python3-tk \
    python3-pil \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

COPY ./tools/seam_tracking_gui /workspace/sunny/tools/seam_tracking_gui
COPY ./tools/ros2_numpy /usr/lib/python3/dist-packages/ros2_numpy

RUN sed -i '/source/a umask 0' /ros_entrypoint.sh

CMD [ "python3", "/workspace/sunny/tools/seam_tracking_gui/main.py" ]
