FROM ros:jazzy-ros-base AS base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
  apt-get install -y curl ros-jazzy-rviz2

RUN apt-get update && \
  apt-get install -y ros-jazzy-navigation2

RUN apt-get update && \
  apt-get install -y ros-jazzy-nav2-bringup

RUN apt-get update && \
  apt-get install -y ros-jazzy-rmw-cyclonedds-cpp

RUN apt-get update && \
  apt-get install -y ros-jazzy-cartographer-ros

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && \
  apt-get install -y gz-harmonic && \
  rm -rf /var/lib/apt/lists/*

#===========================================================
FROM base AS dev
# Install your package here 
RUN apt-get update && \
  apt-get -y install ros-jazzy-xacro ros-jazzy-ros-gz-bridge \
  ros-jazzy-joint-state-publisher ros-jazzy-ros-gz-sim 

ENV WS=gazebo_ws
ENV WORKSPACE=/workspaces/${WS}
WORKDIR /workspaces

COPY --chown=root:root --chmod=700 . /workspaces/gazebo_ws
COPY --chown=root:root ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR ${WORKSPACE}
RUN cat .bashconfig >> ~/.bashrc
RUN ./build.sh