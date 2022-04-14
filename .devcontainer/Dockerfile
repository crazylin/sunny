FROM zhuoqiw/sunny-tis:dev

ARG USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME

RUN echo "source /opt/ros/galactic/setup.bash" >> /home/$USERNAME/.bashrc
