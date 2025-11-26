FROM osrf/ros:jazzy-desktop-full-noble

ARG USERNAME=bval4
ARG USER_UID=1001
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get install -y tmux vim python3-colcon-ed

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y vim python3-colcon-ed tree tmux neovim curl wget gnupg lsb-release
RUN apt-get install -y ros-jazzy-rviz2 ros-jazzy-turtle-tf2-py ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-turtlesim ros-jazzy-teleop-twist-keyboard ros-jazzy-rqt-robot-steering ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-interfaces ros-jazzy-ros-gz-bridge ros-jazzy-rviz-imu-plugin

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update
RUN apt-get install -y libgz-sensors10-lidar libgz-sensors10-gpu-lidar libgz-sensors10-imu

SHELL ["/usr/bin/bash", "-c"]

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$USERNAME/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/jazzy/" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/ros2_ws/install/local_setup.bash" >> /home/$USERNAME/.bashrc

RUN chown -R $USER_UID:$USER_GID /home/$USERNAME

WORKDIR /home/$USERNAME/workbench

USER $USERNAME
