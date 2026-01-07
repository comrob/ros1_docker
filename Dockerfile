FROM osrf/ros:noetic-desktop-full

# 1. Install basics, dev tools, and pip
# We include 'sudo' because we are creating a non-root user.
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-pip \
    git \
    nano \
    vim \
    bash-completion \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 2. Install the conversion libraries
# These are required for your python script to work.
RUN pip3 install rosbags mcap-ros2-support zstandard

# 3. Create the user 'dev' with UID 1000
# This matches the user on most host Linux systems to avoid permission issues.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Enable password-less sudo for convenience
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 4. Auto-source ROS in the .bashrc
# This ensures that when you 'docker exec' into the container, ROS is ready.
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc
# Optional: Source your workspace if it exists
RUN echo "source /home/$USERNAME/catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc

# 5. Set entrypoint context
USER $USERNAME
WORKDIR /home/$USERNAME/catkin_ws