# ArUco Calibration Steps - Docker Container

## Prerequisites Check

```bash
# 1. Source ROS environment
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash  # Adjust path if needed

# 2. Verify camera topic exists
ros2 topic list | grep camera
# Should see: /camera/camera/color/image_rect_raw
```

## Step-by-Step Calibration

### Terminal 1: Start Robot State Publisher (if not already running)
```bash
# Check if robot state is publishing
ros2 topic echo /tf_static --once | grep lbr_link_0

# If not running, start robot (adjust launch file as needed)
# ros2 launch lbr_bringup bringup.launch.py model:=med7 mode:=position
```

### Terminal 2: Start RealSense Camera
```bash
ros2 launch parasight rs_launch.py
# Wait for: "RealSense camera started" or similar message
# Verify images are publishing:
ros2 topic hz /camera/camera/color/image_rect_raw
# Should see ~30 Hz
```

### Terminal 3: Start Hand-Eye TF Publisher
```bash
ros2 run cam2base hand2eye_tf_publisher
# Should see: "Hand-eye transform published" or similar
# Verify TF exists:
ros2 run tf2_ros tf2_echo camera_frame lbr_link_0
# Should show transform values (press Ctrl+C after seeing output)
```

### Terminal 4: Run ArUco Calibration Node
```bash
ros2 run cam2base aruco_calibration_node
```

**What happens:**
- Node subscribes to `/camera/camera/color/image_rect_raw`
- Looks for ArUco marker (ID=0, 15cm)
- Once detected, computes transform
- Prints calibration results to console

**Expected output:**
```
============================================================
ARUCO CALIBRATION COMPLETE!
============================================================

Transform: lbr_link_0 → aruco_marker

--- TRANSLATION (meters) ---
  X: -0.007416
  Y: +0.509019
  Z: +0.363366

--- ROTATION (quaternion) ---
  x: +0.786208
  y: -0.252765
  z: -0.556702
  w: +0.089835

--- ROTATION (Euler angles, degrees) ---
  Roll  (X): +XX.XX°
  Pitch (Y): +XX.XX°
  Yaw   (Z): +XX.XX°

============================================================
COPY THIS INTO aruco_tf_publisher.cpp:
============================================================

// Translation
transform.transform.translation.x = -0.007416;
transform.transform.translation.y = 0.509019;
transform.transform.translation.z = 0.363366;

// Rotation (quaternion)
tf2::Quaternion q;
q.setX(0.786208);
q.setY(-0.252765);
q.setZ(-0.556702);
q.setW(0.089835);

transform.transform.rotation.x = q.x();
transform.transform.rotation.y = q.y();
transform.transform.rotation.z = q.z();
transform.transform.rotation.w = q.w();
```

### Step 5: Copy the Values

Copy the translation (x, y, z) and quaternion (x, y, z, w) values from the output.

### Step 6: Update aruco_tf_publisher.cpp

```bash
# Edit the file
nano src/integration/cam2base/src/aruco_tf_publisher.cpp
# or
vim src/integration/cam2base/src/aruco_tf_publisher.cpp
```

Update lines 32-41 with your new values:
```cpp
// Translation - REPLACE THESE VALUES
transform.transform.translation.x = YOUR_X_VALUE;
transform.transform.translation.y = YOUR_Y_VALUE;
transform.transform.translation.z = YOUR_Z_VALUE;

// Rotation (quaternion) - REPLACE THESE VALUES
tf2::Quaternion q;
q.setX(YOUR_QX_VALUE);
q.setY(YOUR_QY_VALUE);
q.setZ(YOUR_QZ_VALUE);
q.setW(YOUR_QW_VALUE);
```

### Step 7: Rebuild Package

```bash
# Build only the cam2base package
colcon build --packages-select cam2base

# Source the updated workspace
source install/setup.bash
```

### Step 8: Verify Calibration

```bash
# Start the ArUco TF publisher (with new values)
ros2 run cam2base aruco_tf_publisher

# In another terminal, verify the transform is published
ros2 run tf2_ros tf2_echo lbr_link_0 aruco_marker
# Should show your calibrated transform values
```

## Troubleshooting

### "No ArUco marker detected"
- Ensure marker is well-lit
- Check marker is exactly 15cm x 15cm
- Verify camera can see marker clearly
- Check marker ID is 0 (not a different ID)

### "Failed to get camera → robot_base transform"
- Verify hand2eye_tf_publisher is running
- Check TF topic: `ros2 topic echo /tf_static`
- Ensure robot state publisher is running

### "Camera images not publishing"
- Check RealSense camera is connected
- Verify camera permissions
- Restart camera node: `ros2 launch parasight rs_launch.py`

### "Calibration values seem wrong"
- Ensure robot is steady during detection
- Verify marker is flat (not warped)
- Check marker distance from camera (0.5-2 meters ideal)
- Re-run calibration multiple times and average values

