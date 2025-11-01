# ArucoTransform Setup & Testing Guide

Complete step-by-step guide to set up and test the ArUco-based drill site visualization system.

## Prerequisites Checklist

### Hardware
- [ ] Apple Vision Pro (physical device or simulator)
- [ ] Mac with Xcode 15+ and visionOS SDK
- [ ] PC/workstation running ROS 2 (Ubuntu 22.04 recommended)
- [ ] RealSense D435 camera
- [ ] KUKA LBR Med 7 robot arm (or mock/simulation)
- [ ] 15cm ArUco marker (printed from `aruco_marker_id0_15cm.png`)

### Software
- [ ] ROS 2 Humble or later
- [ ] Apple Developer account (standard, no enterprise needed)
- [ ] Network connectivity between Vision Pro and ROS server

### Files Required
- [ ] `/src/vision_pro/ArucoTransform/` - Complete app directory
- [ ] `/src/integration/cam2base/` - TF publishers and calibration nodes
- [ ] `/src/vision_pro/tcp_server_pkg/` - AVP TCP server
- [ ] `/src/perception/parasight/` - Perception pipeline

---

## Part 1: Print and Mount ArUco Marker

### Step 1: Print Marker

```bash
cd src/vision_pro/ArucoTransform

# Marker already generated:
ls -lh aruco_marker_id0_15cm.png
```

**Printing Instructions:**
1. Open `aruco_marker_id0_15cm.png` in image viewer
2. Print settings:
   - **Paper**: A4 or Letter
   - **Scale**: Fit to page
   - **Quality**: Best/High
   - **Color**: Black & White
3. **Measure carefully**: Black square MUST be exactly **15cm x 15cm**
   - Use a ruler to verify
   - If not 15cm, adjust print scale and reprint

### Step 2: Mount Marker

1. Cut out the marker leaving white border
2. Mount on rigid surface:
   - Foam board (recommended)
   - Acrylic sheet
   - Cardboard (stiff)
3. Ensure marker is **perfectly flat** (no warping)
4. Place in operating room where it's visible to:
   - RealSense camera
   - Vision Pro user
5. **Keep marker stationary** - do not move after calibration!

---

## Part 2: ROS System Setup

### Step 1: Build ROS Packages

```bash
cd /home/kneepolean/Boneparte

# Build all packages
colcon build

# Or build specific packages:
colcon build --packages-select cam2base parasight tcp_server_pkg surgical_robot_planner lbr_bringup

# Source the workspace
source install/setup.bash
```

### Step 2: Run ArUco Calibration (ONE TIME)

This establishes the `robot_base → aruco_marker` transform.

**IMPORTANT**: The ArUco marker is a **fixed reference point** in the room. For calibration, you need the RealSense camera (attached to the robot) to see the marker. Choose one:

**Option A: Hardware Mode (Recommended for Calibration)**
```bash
# Terminal 1: Start robot in hardware mode
ros2 launch lbr_bringup bringup.launch.py model:=med7 mode:=position

# Move robot to position where RealSense camera can see ArUco marker
# Use MoveIt or manual jogging to position camera
# Ensure marker is well-lit and clearly visible

# Terminal 2: Start RealSense camera
ros2 launch parasight rs_launch.py

# Terminal 3: Start hand-eye TF publisher (camera → robot_base)
ros2 run cam2base hand2eye_tf_publisher

# Terminal 4: Run calibration node
ros2 run cam2base aruco_calibration_node

# Hold robot steady, marker should be visible in camera view
# Calibration output will print transform values
```

**Option B: Mock Mode (If Camera Can Be Positioned Separately)**
```bash
# Terminal 1: Start robot in mock mode
ros2 launch lbr_bringup bringup.launch.py model:=med7 mode:=mock

# Terminal 2: Start RealSense camera
ros2 launch parasight rs_launch.py

# Terminal 3: Start hand-eye TF publisher
ros2 run cam2base hand2eye_tf_publisher

# Terminal 4: Run calibration node
ros2 run cam2base aruco_calibration_node

# If camera is attached to robot: Manually position robot so camera sees marker
# If camera is detached: Position camera manually to see marker
# Note: Hand-eye calibration assumes camera is at its normal position on robot
```

**After Calibration**: The marker stays **fixed** in the room. Vision Pro will detect it regardless of robot state (mock/hardware).

**Expected Output:**
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
...
```

**Copy values into `src/integration/cam2base/src/aruco_tf_publisher.cpp`:**
```cpp
transform.transform.translation.x = -0.007416;  // Your values here
transform.transform.translation.y = 0.509019;
transform.transform.translation.z = 0.363366;

tf2::Quaternion q;
q.setX(0.786208);
q.setY(-0.252765);
q.setZ(-0.556702);
q.setW(0.089835);
```

**Rebuild:**
```bash
colcon build --packages-select cam2base
source install/setup.bash
```

### Step 3: Verify TF Tree

```bash
# Check all transforms are publishing
ros2 run tf2_tools view_frames

# Should show:
# lbr_link_0 → camera_frame (hand2eye_tf_publisher)
# lbr_link_0 → aruco_marker (aruco_tf_publisher)

# Open frames.pdf to verify
evince frames.pdf
```

**Note**: After calibration, you can switch to mock mode for testing. The ArUco marker transform is **static** and doesn't change.

---

## Part 3: Vision Pro App Setup

### Step 1: Open Xcode Project

```bash
cd src/vision_pro/ArucoTransform
open ArucoTransform.xcodeproj
```

### Step 2: Configure Signing

1. Select **ArucoTransform** target
2. Go to **Signing & Capabilities** tab
3. Select your **Team** (Apple Developer account)
4. Change **Bundle Identifier** if needed:
   - Default: `com.boneparte.ArucoTransform`
   - Change to: `com.yourteam.ArucoTransform`

### Step 3: Build and Deploy

**For Physical Device:**
1. Connect Vision Pro via USB-C
2. Trust computer on Vision Pro
3. Select **Vision Pro** as destination in Xcode
4. Click **Run** (⌘R)
5. App installs and launches

**For Simulator:**
1. Select **Vision Pro Simulator** as destination
2. Click **Run** (⌘R)
3. Note: Camera/ArUco detection won't work in simulator

---

## Part 4: Complete System Test

### Step 1: Start All ROS Nodes

Create launch script: `scripts/launch_boneparte_avp.sh`

```bash
#!/bin/bash

# Launch all components for AVP drill site visualization

# Kill existing nodes
pkill -f ros2

# Wait
sleep 2

# Terminal splits (use tmux or separate terminals)

# Session 1: Robot & MoveIt
# NOTE: You can use mock mode AFTER calibration is complete
# The ArUco marker is fixed, so mock mode is fine for testing visualization
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock rviz:=true &
sleep 5

# Session 2: RealSense
ros2 launch parasight rs_launch.py &
sleep 3

# Session 3: Perception
ros2 run parasight host &
sleep 2

# Session 4: Hand-eye TF
ros2 run cam2base hand2eye_tf_publisher &
sleep 1

# Session 5: ArUco TF  
ros2 run cam2base aruco_tf_publisher &
sleep 1

# Session 6: Drill pose transformer
ros2 run cam2base aruco_drill_pose_publisher &
sleep 1

# Session 7: AVP TCP server
ros2 launch tcp_server_pkg avp_server_launch.py &

echo "All nodes started. Check with: ros2 node list"
```

Make executable and run:
```bash
chmod +x scripts/launch_boneparte_avp.sh
./scripts/launch_boneparte_avp.sh
```

### Step 2: Verify ROS Pipeline

```bash
# Check all nodes are running
ros2 node list

# Expected nodes:
# - /aruco_tf_publisher
# - /hand2eye_tf_publisher
# - /aruco_drill_pose_publisher
# - /avp_tcp_server
# - /parasight_host
# - /move_group
# - /realsense2_camera_node

# Check topics
ros2 topic list

# Important topics:
# - /surgical_drill_pose (PoseArray in lbr_link_0)
# - /aruco_drill_poses (PoseArray in aruco_marker)
# - /tf_static

# Monitor drill poses
ros2 topic echo /aruco_drill_poses
```

### Step 3: Generate Drill Poses

```bash
# Trigger perception to compute drill sites
ros2 topic pub --once /trigger_host_ui std_msgs/msg/Empty '{}'

# Wait for segmentation UI to appear
# Annotate bone regions in the UI
# Drill poses will be published automatically
```

### Step 4: Connect Vision Pro

1. **Launch App** on Vision Pro
2. **Enter Server IP**:
   - Find ROS server IP: `hostname -I`
   - Enter in app (e.g., `192.168.1.100`)
3. **Click "Connect"**
   - Status should turn green: "Connected to 192.168.1.100:5001"

### Step 5: Enable Visualization

1. **Click "Show Drill Sites"**
   - App enters immersive mixed reality mode
2. **Localize ArUco Marker**:
   - Look at the physical ArUco marker
   - Hold Vision Pro steady
   - Wait for green checkmark: "ArUco marker detected & localized"
3. **View Drill Sites**:
   - Red spheres appear at drill locations
   - Blue axis shows drilling direction
   - Walk around - sites stay locked to physical space

---

## Part 5: Troubleshooting

### Issue: ArUco Marker Not Detected

**Symptoms:**
- "Looking for ArUco marker..." message persists
- No green checkmark

**Solutions:**
1. **Check Lighting**:
   - Marker must be well-lit
   - Avoid shadows, reflections
   - Try different lighting angle

2. **Verify Marker Size**:
   - Measure printed marker: must be exactly 15cm x 15cm
   - If wrong size, reprint with correct scale

3. **Check Marker Quality**:
   - Ensure marker is flat (no warping)
   - Check for print quality (sharp edges)
   - Verify marker is not damaged

4. **Hold Steady**:
   - Keep Vision Pro still for 2-3 seconds
   - Position 0.5-1 meter from marker
   - Face marker directly (not at angle)

### Issue: No Drill Sites Visible

**Symptoms:**
- ArUco detected but no red spheres appear
- "Drill sites: 0" in UI

**Solutions:**
1. **Check ROS Pipeline**:
   ```bash
   # Verify drill poses are published
   ros2 topic echo /aruco_drill_poses
   
   # Should see PoseArray messages
   ```

2. **Trigger Perception**:
   ```bash
   # Generate drill poses
   ros2 topic pub --once /trigger_host_ui std_msgs/msg/Empty '{}'
   ```

3. **Check TCP Connection**:
   ```bash
   # Verify AVP server is running
   ros2 node info /avp_tcp_server
   
   # Check for errors
   ros2 run tcp_server_pkg avp_tcp_server
   ```

4. **Network Connectivity**:
   - Ping Vision Pro from ROS server
   - Check firewall allows port 5001
   - Ensure both devices on same network

### Issue: Drill Sites Misaligned

**Symptoms:**
- Red spheres appear but not at correct location
- Sites drift or float incorrectly

**Solutions:**
1. **Re-run Calibration**:
   ```bash
   ros2 run cam2base aruco_calibration_node
   ```
   - Update `aruco_tf_publisher.cpp` with new values
   - Rebuild and restart

2. **Check Marker Moved**:
   - Verify ArUco marker hasn't been moved
   - If moved, re-calibrate

3. **Verify TF Tree**:
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```
   - Check all transforms are correct

4. **Re-detect Marker**:
   - Exit and re-enter immersive space
   - Detect ArUco marker again

### Issue: Connection Failed

**Symptoms:**
- "Connection failed" or "Disconnected" status
- Cannot connect to ROS server

**Solutions:**
1. **Verify Server IP**:
   ```bash
   # On ROS server
   hostname -I
   
   # Should match IP entered in Vision Pro app
   ```

2. **Check Server Running**:
   ```bash
   ros2 node list | grep avp_tcp_server
   ```

3. **Test Port**:
   ```bash
   # On another machine, test connection
   telnet <server_ip> 5001
   ```

4. **Firewall**:
   ```bash
   # Allow port 5001 on ROS server
   sudo ufw allow 5001/tcp
   ```

5. **Network**:
   - Ensure Vision Pro and server on same network
   - Check WiFi connection on Vision Pro

---

## Part 6: Validation Tests

### Test 1: Static Transform Accuracy

1. Place a known object at drill site location
2. Verify red sphere aligns with object in Vision Pro
3. Measure distance (should be < 5mm error)

### Test 2: Real-Time Updates

1. Generate new drill poses:
   ```bash
   ros2 topic pub --once /trigger_host_ui std_msgs/msg/Empty '{}'
   ```
2. Annotate different bone region
3. Verify drill sites update in Vision Pro within 1-2 seconds

### Test 3: World Tracking Stability

1. Enable visualization
2. Walk around operating room (2-3 meters)
3. Return to original position
4. Verify drill sites haven't drifted (< 1cm)

### Test 4: Multiple Drill Sites

1. Generate plan with 4+ drill sites
2. Verify all sites render correctly
3. Check axis orientations are correct

---

## Part 7: Production Deployment

### Optimization 1: OpenCV Integration

For robust ArUco detection, integrate OpenCV (see `ArucoDetector.swift` comments):

1. Add OpenCV via CocoaPods:
   ```ruby
   # Podfile
   platform :visionos, '1.0'
   target 'ArucoTransform' do
     pod 'OpenCV', '~> 4.0'
   end
   ```

2. Create Objective-C++ bridge: `ArucoDetectorBridge.mm`
3. Use `cv::aruco::detectMarkers()` and `cv::aruco::estimatePoseSingleMarkers()`

### Optimization 2: Persistent World Anchors

Save ArUco anchor across app sessions:
- Use `WorldAnchor` persistence API
- Store anchor ID in UserDefaults
- Re-query on app launch

### Optimization 3: Performance Tuning

- Reduce TCP update rate if network bandwidth limited
- Adjust sphere/axis rendering detail based on distance
- Implement LOD (Level of Detail) for many drill sites

---

## Appendix: System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         ROS 2 SYSTEM                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐ │
│  │  RealSense   │──────▶│ Perception   │──────▶│   MoveIt     │ │
│  │    Camera    │      │   Pipeline   │      │  Planning    │ │
│  └──────────────┘      └──────────────┘      └──────────────┘ │
│         │                      │                               │
│         │ (camera_frame)       │ (/surgical_drill_pose)        │
│         ▼                      ▼                               │
│  ┌──────────────┐      ┌──────────────┐                       │
│  │  hand2eye_tf │      │ aruco_drill  │                       │
│  │  _publisher  │      │ _pose_pub    │                       │
│  └──────────────┘      └──────────────┘                       │
│         │                      │                               │
│         │ (lbr_link_0)        │ (aruco_marker frame)          │
│         ▼                      ▼                               │
│  ┌──────────────┐      ┌──────────────┐                       │
│  │  aruco_tf    │      │   AVP TCP    │                       │
│  │  _publisher  │      │    Server    │                       │
│  └──────────────┘      └──────┬───────┘                       │
│                               │ TCP port 5001                  │
└───────────────────────────────┼────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    APPLE VISION PRO                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐ │
│  │  TCP Client  │──────▶│  App Model   │──────▶│    UI View   │ │
│  └──────────────┘      └──────────────┘      └──────────────┘ │
│                                                                 │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐ │
│  │    ARKit     │──────▶│    World     │──────▶│   Drill Site │ │
│  │  Tracking    │      │   Anchor Mgr │      │   Renderer   │ │
│  └──────────────┘      └──────────────┘      └──────────────┘ │
│         │                      │                      │         │
│         │ (Device pose)        │ (aruco→world)       │         │
│         ▼                      ▼                      ▼         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │           RealityKit Immersive Space                    │   │
│  │  (Red spheres + Blue axes at drill locations)           │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Quick Reference Commands

```bash
# Build workspace
colcon build && source install/setup.bash

# Run calibration
ros2 run cam2base aruco_calibration_node

# Start full system (use tmux/screen for multiple terminals)
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock &
ros2 launch parasight rs_launch.py &
ros2 run parasight host &
ros2 run cam2base hand2eye_tf_publisher &
ros2 run cam2base aruco_tf_publisher &
ros2 run cam2base aruco_drill_pose_publisher &
ros2 launch tcp_server_pkg avp_server_launch.py &

# Check system health
ros2 node list
ros2 topic list
ros2 topic echo /aruco_drill_poses

# Generate drill poses
ros2 topic pub --once /trigger_host_ui std_msgs/msg/Empty '{}'

# View TF tree
ros2 run tf2_tools view_frames && evince frames.pdf
```

---

**Support**: For issues, check ROS logs with `ros2 topic echo /rosout` and Vision Pro console in Xcode (Window → Devices and Simulators).

**Version**: 1.0  
**Last Updated**: 2025-11-01  
**Project**: BONE.P.A.R.T.E. - CMU MRSD Capstone

