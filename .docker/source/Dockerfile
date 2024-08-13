ARG ROS_DISTRO="humble"
FROM osrf/ros:${ROS_DISTRO}-desktop
ENV OVERLAY_WS /root/overlay_ws
WORKDIR $OVERLAY_WS/src

# Copy source files
COPY . crane_plus

# Install depnedencies
RUN apt-get update && apt-get -y dist-upgrade && rosdep update && \
    rosdep install -iy --from-paths . && \
    # https://github.com/ros-planning/moveit2/issues/1094
    apt install -y ros-${ROS_DISTRO}-backward-ros && \
    rm -rf /var/lib/apt/lists/

# Build packages
ARG COLCON_OPTION=""
RUN cd $OVERLAY_WS && \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        colcon build --symlink-install $COLCON_OPTION

# Download gazebo models by sparse checkout
RUN mkdir -p /root/.gazebo/models && \
    cd  /root/.gazebo/models && \
    git init . && \
    git config core.sparsecheckout true && \
    echo "ground_plane" >> .git/info/sparse-checkout && \
    echo "sun" >> .git/info/sparse-checkout && \
    echo "table" >> .git/info/sparse-checkout && \
    git remote add origin https://github.com/osrf/gazebo_models && \
    git pull origin master && \
    rm -rf .git

# Download ignition model
RUN ign fuel download -v 4 -u https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/sun && \
    ign fuel download -v 4 -u https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/ground%20plane && \
    ign fuel download -v 4 -u https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/wood%20cube%205cm && \
    ign fuel download -v 4 -u https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/table

# Edit entrypoint to source overlay setup file
WORKDIR $OVERLAY_WS
RUN sed --in-place --expression \
    '$i if [ -e $OVERLAY_WS/install/setup.bash ]; then\n\tsource "$OVERLAY_WS/install/setup.bash" \nfi' \
    /ros_entrypoint.sh
