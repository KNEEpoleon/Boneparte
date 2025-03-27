Connects with the robot
```bash
ros2 launch lbr_bringup hardware.launch.py model:=med7
```
Launches Moveit and RViz
```bash
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=hardware rviz:=true
```

Runs joint config based control (Hardcoded)
```bash
ros2 run joint_config_moveit_cpp moveit_joint_config.py```
```

Runs EE pose based control (Hardcoded)
```bash
ros2 launch lbr_moveit_cpp hello_moveit.launch.py mode:=hardware model:=med7
```     