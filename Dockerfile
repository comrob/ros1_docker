FROM osrf/ros:noetic-desktop-full

# 1. Install basics
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-pip \
    git \
    nano \
    vim \
    bash-completion \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 2. Install conversion libs
RUN pip3 install rosbags mcap-ros2-support zstandard

# 3. Create user
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 4. Environment setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/$USERNAME/catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc

# This ensures the script is always available inside the container at a known path.
COPY --chown=$USERNAME:$USERNAME convert.py /home/$USERNAME/convert.py

# 5. Entrypoint
USER $USERNAME
WORKDIR /home/$USERNAME/catkin_ws