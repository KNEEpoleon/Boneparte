# Configuration file for ROS2 and custom aliases
# Gets exported to .bashrc within the container

# Source ROS2 Humble setup file
source /opt/ros/humble/setup.bash

# Alias to build the workspace and source the new setup file
alias bone_install='colcon build && source install/setup.bash'
alias rossrc='source install/setup.bash'

# Zsh like experience
bind 'set completion-ignore-case on'
bind 'set show-all-if-ambiguous on'
bind 'TAB:menu-complete'

# Add Conditional Source of ROS2 workspace setup file
if [ -f "/ros_ws/install/setup.bash" ]; then
    source /ros_ws/install/setup.bash
fi
