Connects with the robot
```bash
ros2 launch lbr_bringup hardware.launch.py model:=med7
```
Launches RViz Visualization
```bash
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=hardware rviz:=true
```
  
Drill motion execution with arduino trigger
```bash
ros2 launch surgical_robot_planner drill_motion_executor.launch.py model:=med7
```

Sample Topic publish
```bash
ros2 topic pub /surgical_drill_pose geometry_msgs/msg/PoseArray "header:
  frame_id: 'lbr/link_0'
poses:
- position: {x: -0.400, y: 0.277, z: 0.285}
  orientation: {x: -0.000, y: 0.977, z: -0.215, w: 0.001}" --once
```

Select pose to drill
```bash
ros2 service call /select_pose surgical_robot_planner/srv/SelectPose "{index: 0}"
```
### REREGISTRATION

Start reregistration (removes obstacles, blocks drilling)
```bash
ros2 topic pub --once /registration std_msgs/msg/String "{data: 'reregistering'}"
```

Publish new poses (after reregistration)
```bash
ros2 topic pub /surgical_drill_pose geometry_msgs/msg/PoseArray "header:
  frame_id: 'lbr/link_0'
poses:
- position: { -0.400, y: 0.277, z: 0.385}
  orientation: {x: -0.000, y: 0.977, z: -0.215, w: 0.001}" --once
```

Complete reregistration (re-adds obstacles at new poses)
```bash
ros2 topic pub --once /registration std_msgs/msg/String "{data: 'complete'}"
```

Select index again and continue

### OLD COMMANDS

Runs joint config based control (Hardcoded)
```bash
ros2 run joint_config_moveit_cpp moveit_joint_config.py```
```

Runs EE pose based control (Hardcoded)
```bash
ros2 launch lbr_moveit_cpp hello_moveit.launch.py mode:=hardware model:=med7
```  
Runs EE pose based control (Subscription)
```bash
ros2 launch surgical_robot_planner move_to_pose.launch.py mode:=hardware model:=med7
```   