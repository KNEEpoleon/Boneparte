## COMMANDS TO RUN FOR FVD Encore:

**LBR bringup hardware**
```bash
ros2 launch lbr_bringup hardware.launch.py model:=med7
```     

**Launch the planning-hardware nodes**
> Launches RVIZ, Planner, Arduino Serial and AVP TCP connection:
```bash
ros2 launch launch_subsystems planning_hardware.launch.py
```

**Launch the planning-perception nodes**
> Launches the transforms package and the parasight package
```bash
ros2 launch launch_subsystems planning_perception.launch.py
```  

**Launch the second AVP node**
```bash
ros2 run tcp_server_pkg avp_tcp_server 
```
___


## TERMINAL PUBLISH AND SERVICES
### Hard reset to `bring_manipulator`

```bash
ros2 topic pub --once /reset_mission std_msgs/msg/Empty '{}'
```

### When in the awaiting surgeon state

> proceed mission to trigger `bring_manipulator` (boneparte_home) and `auto_reposition`
```bash
ros2 topic pub --once /proceed_mission std_msgs/msg/Empty
```

> Send the `manipulator away`
```bash
ros2 service call /robot_command surgical_robot_planner/srv/RobotCommand "{command: 'away'}"
```

> bring the `manipulator home`
```bash
ros2 service call /robot_command surgical_robot_planner/srv/RobotCommand "{command: 'home'}"
```

## Workstation workflow
> publish the annotate command:
```bash
ros2 topic pub --once /trigger_host_ui std_msgs/msg/Empty '{}'
```

> send the service to drill to index 0:
Change index to any of {0,1} for drill sites on the Femur and {2,3} for drill sites on the Tibia.
```bash
ros2 service call /select_pose surgical_robot_planner/srv/SelectPose "{index: 0}"
```

> publish the arduino start and stop command:
```bash
ros2 topic pub --once /drill_commands std_msgs/msg/String "{data: 'start'}"
```












## TO BUILD THE DOCKER CONTAINER
> CREATE docker image
```bash
./docker/boneparte.sh build
```

> BUILD and RUN the docker container from an image
```bash
./docker/boneparte.sh run
```

> ATTACH a docker shell
```bash
./docker/shell.sh
```     

## ONCE INSIDE THE DOCKER CONTAINER

#### HARDWARE and PLANNING
**LBR bringup**

```bash
ros2 launch lbr_bringup hardware.launch.py model:=med7
```     

**RIVZ2**
```bash
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=hardware rviz:=true
```

**DRILL motion trigger**
```bash
ros2 launch surgical_robot_planner drill_motion_executor.launch.py model:=med7
```

#### SERIAL COMMS

**Open arduino serial comms**
```bash
 ros2 run serialcomms write_to_serial 
```

#### INTEGRATION

**Publish handeye and transform drill poses and pointcloud from the camera frame to lbr_link_00**
```bash
 ros2 launch cam2base cam_to_base_launch.py
```
#### PERCEPTION

**Run the perception node to segment and register the bones**
```bash
 ros2 launch parasight segment_rs_launch.py
```
___
