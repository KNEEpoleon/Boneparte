#!/bin/bash

# Source existing bashrc
source /opt/ros/humble/setup.bash
source ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "entrypoint"
echo ${PWD}
# Check if the workspace is built
# if [ ! -d "/ros_ws/build" ]; then
#     echo "Building the ROS workspace"
#     cd /ros_ws && colcon build
# fi
if [ ! -d "/ros_ws/build" ]; then
    echo "Building the ROS workspace"
    colcon build
fi
# Source the workspace
source /ros_ws/install/setup.bash

# Run the entrypoint
exec "$@"