Connects with the robot
```ros2 launch lbr_bringup hardware.launch.py model:=med7```

Launches Moveit and RViz
```ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=hardware rviz:=true```

Runs joint config based control (Hardcoded)
```ros2 run kuka_med_moveit_cpp moveit_point_to_point.py```

Runs EE pose based control (Hardcoded)
```ros2 launch lbr_moveit_cpp hello_moveit.launch.py mode:=hardware model:=med7```     