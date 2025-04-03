## Commands to run the node to publish the drill site locations just once:

#### after 'running' the docker container, rm install, then do a colcon build and then 
#### source install/setup.bash
#### To attach a new terminal, run docker/shell.sh

1) First fire up the realsense node: ros2 launch parasight rs_launch.py

2) Fire up the host node: ros2 run parasight host

3) Then it's about sending a drill command and viewing the topic: ros2 topic pub --once /trigger_host_ui std_msgs/msg/Empty '{}' && ros2 topic echo /surgical_drill_pose