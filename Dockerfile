FROM ros:kinetic

MAINTAINER robottime "robottime@yeah.net"

# install build tools
RUN apt-get update && apt-get install -y \
      python-catkin-tools python-pip \
    && rm -rf /var/lib/apt/lists/*

# clone ros package repo
ENV ROS_WS /root/catkin_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
COPY differential_robot_driver $ROS_WS/src/differential_robot_driver

# install ros package dependencies
RUN apt-get update && \
		rosdep update && \
    rosdep install -y \
      --from-paths \
        src/differential_robot_driver \
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*

RUN pip install pyserial

# build ros package source
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build \
      differential_robot_driver

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

# run ros package launch file
CMD ["rosrun", "differential_robot_driver", "differential_robot_driver.py"]
