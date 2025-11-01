# Quick Start Guide - ArucoTransform

**Goal**: Get drill sites visualized in Vision Pro in 30 minutes

---

## 🚀 Fast Track (30 minutes)

### 1. Print Marker (5 min)
```bash
cd src/vision_pro/ArucoTransform
xdg-open aruco_marker_id0_15cm.png
# Print at 15cm x 15cm - MEASURE WITH RULER!
# Mount on foam board
```

### 2. Build App (5 min)
```bash
open ArucoTransform.xcodeproj
# In Xcode: Select Team → Build (⌘B)
```

### 3. Calibrate (5 min)
```bash
# IMPORTANT: For calibration, robot needs to be positioned so camera sees marker
# Option A: Use hardware mode and move robot
ros2 launch lbr_bringup bringup.launch.py model:=med7 mode:=position &
# Move robot so RealSense camera can see ArUco marker

# Option B: Use mock mode if camera can be positioned separately
ros2 launch lbr_bringup bringup.launch.py model:=med7 mode:=mock &

# Start camera and TF
ros2 launch parasight rs_launch.py &
ros2 run cam2base hand2eye_tf_publisher &

# Calibrate (one-time)
ros2 run cam2base aruco_calibration_node
# Point camera at marker → Copy transform values

# Update aruco_tf_publisher.cpp with values
colcon build --packages-select cam2base
source install/setup.bash

# After calibration: Marker is fixed, you can use mock mode for testing
```

### 4. Test (15 min)
```bash
# Start full pipeline
# NOTE: After calibration, mock mode is fine - marker is fixed in room
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock &
ros2 launch parasight rs_launch.py &
ros2 run cam2base hand2eye_tf_publisher &
ros2 run cam2base aruco_tf_publisher &
ros2 run cam2base aruco_drill_pose_publisher &
ros2 launch tcp_server_pkg avp_server_launch.py &
ros2 run parasight host &

# Generate drill poses
ros2 topic pub --once /trigger_host_ui std_msgs/msg/Empty '{}'

# On Vision Pro:
# 1. Launch ArucoTransform
# 2. Enter server IP
# 3. Connect
# 4. Show drill sites
# 5. Look at marker (fixed in room, not on robot!)
# 6. See red spheres! 🎉
```

---

## 📋 Pre-flight Checklist

- [ ] ArUco marker printed (15cm x 15cm)
- [ ] Marker mounted and placed
- [ ] Xcode project builds
- [ ] Vision Pro connected/paired
- [ ] ROS 2 workspace built (`colcon build`)
- [ ] Network: Vision Pro + PC same WiFi
- [ ] Know your PC IP: `hostname -I`

---

## 🐛 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| **ArUco not detected** | Check lighting, marker size (15cm), hold steady |
| **No drill sites** | Run `ros2 topic pub --once /trigger_host_ui std_msgs/msg/Empty '{}'` |
| **Can't connect** | Verify server IP, check `ros2 node list \| grep avp` |
| **Sites misaligned** | Re-run calibration, update aruco_tf_publisher.cpp |

---

## 📚 Full Documentation

- **README.md** - Complete overview and features
- **SETUP_GUIDE.md** - Detailed step-by-step instructions
- **IMPLEMENTATION_COMPLETE.md** - What's been built
- **WHATS_LEFT.md** - Optional improvements
- **QUICK_START.md** - This file

---

## 🎯 Success = Red Spheres at Drill Locations!

When you see red spheres with blue drilling axes appearing over the physical bone location and staying stable as you move around, **you're done!**

Everything else is optional improvements.

---

**Need help?** Check SETUP_GUIDE.md Part 5 (Troubleshooting)

