# ArucoTransform - Apple Vision Pro Drill Site Visualization

ArUco-based spatial localization and drill site visualization for surgical robotics.

## Overview

**ArucoTransform** is a Vision Pro application that:
1. Detects an ArUco marker (ID=0, 15cm) to establish spatial localization
2. Connects to ROS server via TCP to receive drill site poses
3. Visualizes drill sites as red spheres with orientation axes in mixed reality
4. Updates visualization in real-time as the user moves around the operating room

## Features

**ArUco Marker Detection** - One-time localization using ArUco marker (DICT_6X6_250, ID=0)  
**TCP Communication** - Receives drill poses from ROS server on port 5001  
**World Tracking** - Maintains stable coordinate system using ARKit  
**Mixed Reality Rendering** - Red spheres (1cm) with 3-axis vectors (5cm)  
**Real-time Updates** - 30 Hz refresh rate for smooth visualization  
**Transform Chain** - Bone → Camera → Robot Base → ArUco → AVP World

## Requirements

- **Hardware**: Apple Vision Pro
- **Software**: visionOS 1.0 or later
- **Developer Account**: Apple Developer account (standard, no enterprise needed)
- **Network**: Local network access to ROS server
- **Physical**: 15cm ArUco marker (DICT_6X6_250, ID=0)

## Transformation Pipeline

```
┌─────────────────┐
│  Bone Point     │
│  Cloud          │  camera_frame
└────────┬────────┘
         │ hand-eye calibration (TF)
         ▼
┌─────────────────┐
│  RealSense      │
│  Camera         │  camera_frame
└────────┬────────┘
         │ hand2eye_tf_publisher
         ▼
┌─────────────────┐
│  Robot Base     │
│  (Kuka LBR)     │  lbr_link_0
└────────┬────────┘
         │ aruco_tf_publisher (static)
         ▼
┌─────────────────┐
│  ArUco Marker   │
│  (Physical)     │  aruco_marker
└────────┬────────┘
         │ Vision Pro detection (ONE TIME)
         ▼
┌─────────────────┐
│  AVP World      │
│  Anchor         │  world space
└────────┬────────┘
         │ ARKit world tracking (REAL-TIME)
         ▼
┌─────────────────┐
│  AVP Device     │
│  (Current)      │  device transform
└─────────────────┘
```

## Setup Instructions

### 1. Generate ArUco Marker

Generate a 15cm ArUco marker (ID=0, DICT_6X6_250):

```bash
# Using Python + OpenCV
python3 << EOF
import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
marker_id = 0
marker_size = 700  # pixels (print at 15cm physical size)

marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
cv2.imwrite('aruco_marker_id0.png', marker_image)
print("ArUco marker generated: aruco_marker_id0.png")
EOF

# Print at exactly 15cm x 15cm size
```

**Important**: The marker MUST be exactly 15cm x 15cm to match calibration parameters.

### 2. Install ArUco Marker

1. Print the marker on rigid material (foam board, acrylic, etc.)
2. Place marker in a **fixed location** visible to both:
   - RealSense camera (for calibration)
   - Vision Pro (for localization)
3. Ensure marker is flat and stable (no warping or movement)
4. **CRITICAL**: The marker is a **fixed reference point** in the room. It does NOT move. After calibration, it stays in the same physical location.

**Important**: For calibration, the robot needs to be positioned so the RealSense camera can see the marker. After calibration, you can use mock mode - the marker stays fixed.

### 3. Build Xcode Project

```bash
cd src/vision_pro/ArucoTransform
open ArucoTransform.xcodeproj
```

In Xcode:
1. Select your Apple Developer Team in Signing & Capabilities
2. Change Bundle Identifier if needed (e.g., `com.yourteam.ArucoTransform`)
3. Connect Vision Pro or select simulator
4. Build and run (⌘R)

### 4. Configure Server IP

In the app UI:
1. Enter your ROS server IP address (default: `192.168.1.100`)
2. Port is fixed at `5001` (AVP TCP server)
3. Click "Connect"

## Usage Workflow

### Complete System Startup

```bash
# Terminal 1: Robot & MoveIt
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock rviz:=true

# Terminal 2: RealSense Camera
ros2 launch parasight rs_launch.py

# Terminal 3: Perception (bone segmentation & registration)
ros2 run parasight host

# Terminal 4: Hand-eye TF (camera → robot base)
ros2 run cam2base hand2eye_tf_publisher

# Terminal 5: ArUco TF (robot base → aruco marker)
ros2 run cam2base aruco_tf_publisher

# Terminal 6: Drill pose transformer (lbr_link_0 → aruco_marker)
ros2 run cam2base aruco_drill_pose_publisher

# Terminal 7: AVP TCP server
ros2 launch tcp_server_pkg avp_server_launch.py
```

### On Vision Pro

1. **Launch App**: Open "ArUco Transform" from Home
2. **Connect**: 
   - Enter server IP
   - Click "Connect"
   - Wait for green "Connected" status
3. **Enable Visualization**:
   - Click "Show Drill Sites"
   - App enters immersive mode
4. **Localize**:
   - Point Vision Pro at the 15cm ArUco marker
   - Hold steady for 1-2 seconds
   - Green checkmark appears: "ArUco marker detected & localized"
5. **View Drill Sites**:
   - Red spheres appear at drill locations
   - Blue axis shows drilling direction (Z-axis)
   - Walk around - visualization stays locked to physical space

### Troubleshooting

**ArUco Not Detected:**
- Ensure marker is well-lit (not in shadow)
- Hold Vision Pro steady while detecting
- Verify marker is exactly 15cm x 15cm
- Check marker is not warped or damaged

**No Drill Sites Visible:**
- Verify ROS pipeline is running
- Check TCP connection status (green)
- Ensure drill poses are being published: `ros2 topic echo /aruco_drill_poses`
- Verify ArUco transform is published: `ros2 topic echo /tf_static`

**Drill Sites Misaligned:**
- Re-run ArUco calibration: `ros2 run cam2base aruco_calibration_node`
- Update transform in `aruco_tf_publisher.cpp`
- Rebuild: `colcon build --packages-select cam2base`

**Connection Failed:**
- Verify Vision Pro and PC are on same network
- Check firewall allows port 5001
- Ping server IP from another device
- Verify AVP TCP server is running: `ros2 node list | grep avp`

## Architecture

### Core Components

**ArucoDetector.swift**
- Detects ArUco markers in camera frames
- Estimates 6-DOF pose using PnP algorithm
- Returns camera → marker transform

**WorldAnchorManager.swift**
- Manages world space anchors
- Stores aruco → world transform
- Provides coordinate transformation utilities

**TCPClient.swift**
- Connects to ROS server on port 5001
- Parses drill pose messages (POSES|x,y,z,qx,qy,qz,qw|...)
- Publishes to AppModel for rendering

**DrillSiteRenderer.swift**
- Creates RealityKit entities for each drill site
- Red sphere (1cm radius) at drill location
- 3-axis coordinate frame:
  - X-axis: Red (5cm)
  - Y-axis: Green (5cm)
  - Z-axis: Blue (10cm) - drilling direction
- Transforms from ArUco frame to world frame

### Data Flow

```
ROS Topics:
  /surgical_drill_pose (lbr_link_0)
       ↓
  aruco_drill_pose_publisher
       ↓
  /aruco_drill_poses (aruco_marker)
       ↓
  avp_tcp_server (port 5001)
       ↓
Vision Pro:
  TCPClient → AppModel → DrillSiteRenderer
       ↓
  RealityKit rendering in world space
```

## Technical Notes

### ArUco Detection

The current implementation uses a **simplified ArUco detector** based on Vision framework's rectangle detection. For production use, integrate OpenCV:

```swift
// Add OpenCV via CocoaPods or SPM
// Create Objective-C++ bridge: ArucoDetectorBridge.mm

#import <opencv2/opencv.hpp>
#import <opencv2/aruco.hpp>

cv::Ptr<cv::aruco::Dictionary> dictionary = 
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

cv::aruco::detectMarkers(image, dictionary, corners, ids);
cv::aruco::estimatePoseSingleMarkers(corners, 0.15, cameraMatrix, 
    distCoeffs, rvecs, tvecs);
```

See comments in `ArucoDetector.swift` for full implementation guide.

### Coordinate Frames

**ROS Convention** (Right-handed):
- X: Forward
- Y: Left  
- Z: Up

**ARKit/RealityKit Convention** (Right-handed):
- X: Right
- Y: Up
- Z: Backward (toward user)

Transforms are automatically handled in the ArUco detection and rendering pipeline.

### Performance

- **ArUco Detection**: One-time only (1-2 seconds)
- **World Tracking**: Native ARKit (60 Hz)
- **TCP Updates**: ~30 Hz (as fast as ROS publishes)
- **Rendering**: 60 Hz (RealityKit)

### Network Protocol

**TCP Message Format:**
```
POSES|x1,y1,z1,qx1,qy1,qz1,qw1|x2,y2,z2,qx2,qy2,qz2,qw2|...\n
```

**Example:**
```
POSES|0.123,0.456,0.789,0.0,0.0,0.707,0.707|0.234,0.567,0.890,0.0,0.0,0.707,0.707\n
```

All positions in meters, orientations as quaternions (x, y, z, w).

## Limitations

### Without Enterprise License

**Available:**
- World tracking (WorldTrackingProvider)
- Camera access
- World anchors
- AR overlays
- TCP networking

**Not Available:**
- Object tracking (ObjectTrackingProvider)
- Scene reconstruction
- Plane detection
- Enterprise app distribution

For this use case, standard license is **sufficient**.

### Known Issues

1. **First-time ArUco detection** may take 2-3 seconds in low light
2. **Transform drift** over long sessions (>1 hour) - re-detect marker if needed
3. **Simplified ArUco detector** less robust than OpenCV - consider upgrading

## Future Enhancements

- [ ] Integrate OpenCV for robust ArUco detection
- [ ] Add marker re-detection during runtime (handle marker movement)
- [ ] Persist world anchor across app restarts
- [ ] Add bone point cloud visualization (not just drill sites)
- [ ] Multi-marker support for improved accuracy
- [ ] Hand gesture controls (tap to hide/show drill sites)
- [ ] Voice commands integration
- [ ] Drill progress visualization (real-time feedback)

## License

Part of the BONE.P.A.R.T.E. surgical robotics project.

## Support

For issues or questions:
1. Check ROS topics are publishing: `ros2 topic list`
2. Verify TF tree: `ros2 run tf2_tools view_frames`
3. Monitor AVP TCP server: `ros2 node info /avp_tcp_server`

---

**Built for**: CMU MRSD Capstone Project  
**Sponsor**: Smith + Nephew  
**Platform**: Apple Vision Pro + ROS 2

