# ParaSight State Machine Documentation

## 🎯 Overview
The ParaSight Host implements a surgical workflow state machine with 9 states managing the complete flow from initialization to drilling completion.

## 📊 State Diagram

```
                                    ┌─────────────┐
                                    │   START     │
                                    └──────┬──────┘
                                           │ (auto)
                                           ▼
                    ┌──────────────────────────────────────────┐
                    │                                          │
                    │      ┌─────────────────────────┐         │
                    │      │ AWAIT_SURGEON_INPUT     │◄────────┼─── (hard_reset from any state)
                    │      │   (Main Idle State)     │         │
                    │      └──────────┬──┬───┬───────┘         │
                    │                 │  │   │                 │
                    │    ┌────────────┘  │   └─────────────┐   │
                    │    │               │                 │   │
                    │    │ (proceed)     │ (annotate)      │   │
                    │    │               │               (complete)
                    │    ▼               ▼                 │   │
                    │  ┌──────────┐  ┌─────────────┐      │   │
                    │  │  BRING   │  │  SEGMENT &  │      │   │
                    │  │   MANI.  │  │  REGISTER   │      │   │
                    │  └────┬─────┘  └──────┬──────┘      │   │
                    │       │               │             │   │
                    │       │ (complete)    │ (complete)  │   │
                    │       ▼               ▼             │   │
                    │  ┌──────────┐  ┌─────────────┐     │   │
                    │  │  AUTO    │  │   UPDATE    │     │   │
                    │  │REPOSITION│  │    RVIZ     │     │   │
                    │  └────┬─────┘  └──────┬──────┘     │   │
                    │       │               │            │   │
                    └───────┘               │ (complete) │   │
                                            ▼            │   │
                                     ┌─────────────┐    │   │
                                     │   READY TO  │    │   │
                                     │    DRILL    │    │   │
                                     └──────┬──┬───┘    │   │
                                            │  │        │   │
                                  (reset)───┘  │        │   │
                                               │        │   │
                                      (drill_cmd)       │   │
                                               │        │   │
                                               ▼        │   │
                                        ┌──────────┐   │   │
                                        │  DRILL   │   │   │
                                        └────┬─────┘   │   │
                                             │         │   │
                                     (complete)        │   │
                                             └─────────┘   │
                                                           │
                                                           ▼
                                                    ┌──────────┐
                                                    │ FINISHED │
                                                    └──────────┘
```

## 🔄 State Descriptions

### 1. **START** (Initial State)
- **Purpose**: System initialization
- **Entry**: System boot
- **Exit**: Auto-transitions to `await_surgeon_input`
- **Duration**: < 1 second

### 2. **AWAIT_SURGEON_INPUT** (Main Hub)
- **Purpose**: Main idle state waiting for surgeon commands
- **Entry**: 
  - After system boot
  - After completing any major workflow
  - After hard reset
- **Available Triggers**:
  - `annotate` → Start segmentation workflow
  - `proceed_mission` → Start manipulator positioning
  - `complete_mission` → End surgery
- **Duration**: Indefinite (waits for input)

### 3. **BRING_MANIPULATOR**
- **Purpose**: Move robot arm from far home to near home position
- **Entry**: Surgeon triggers "Proceed Mission"
- **Exit**: `complete_bring_manipulator` → auto_reposition
- **Status**: ⚠️ NOT IMPLEMENTED (auto-completes)
- **TODO**: 
  - Call robot motion service
  - Move from `far_home_pose` to `near_home_pose`

### 4. **AUTO_REPOSITION**
- **Purpose**: Auto-position camera using DINO bone detection
- **Entry**: After manipulator positioning completes
- **Exit**: `complete_auto_reposition` → await_surgeon_input
- **Status**: ⚠️ NOT IMPLEMENTED (auto-completes)
- **TODO**:
  - Call `DINOBoneExtractor.get_centroid(rgb_image)`
  - Compute optimal camera pose
  - Command robot to move camera

### 5. **SEGMENT_AND_REGISTER**
- **Purpose**: Segment bones using SAM and register to CT model
- **Entry**: Surgeon triggers "Start ParaSight"
- **Exit**: `complete_segment_and_register` → update_rviz
- **Status**: ✅ FULLY IMPLEMENTED
- **Process**:
  1. Display RGB image in OpenCV window
  2. Surgeon clicks points on femur/tibia
  3. SAM segments bone regions
  4. ICP registration to CT model
  5. Compute drill poses from plan YAML
  6. Publish point clouds and pose array

### 6. **UPDATE_RVIZ**
- **Purpose**: Update RViz visualization with registered models
- **Entry**: After segmentation/registration completes
- **Exit**: `complete_map_update` → ready_to_drill
- **Status**: ⚠️ NOT IMPLEMENTED (auto-completes)
- **TODO**:
  - Publish visualization markers for drill poses
  - Update TF frames for registered bones
  - Display fitness metrics

### 7. **READY_TO_DRILL**
- **Purpose**: Wait for specific drill hole command from surgeon
- **Entry**: After RViz update completes
- **Exit**: 
  - `start_drill` → drill (when `/lbr/plan_flag` received)
  - `reset_mission` → await_surgeon_input
- **Duration**: Indefinite (waits for drill command)

### 8. **DRILL**
- **Purpose**: Execute drilling motion for specific hole
- **Entry**: Surgeon sends drill command via `/lbr/plan_flag`
- **Exit**: `complete_drill` → await_surgeon_input
- **Status**: ⚠️ NOT IMPLEMENTED (auto-completes)
- **TODO**:
  - Extract pose from `last_drill_pose_array` using `current_drill_pin`
  - Send motion command to robot controller
  - Wait for motion completion

### 9. **FINISHED**
- **Purpose**: Surgery complete, system can shut down
- **Entry**: Surgeon triggers "Complete Mission"
- **Exit**: None (terminal state)

## 🎮 External Triggers (Topics)

### Input Topics
| Topic | Type | Trigger | Description |
|-------|------|---------|-------------|
| `/trigger_host_ui` | Empty | `annotate` | Start segmentation (CLI: "Start ParaSight") |
| `/hard_reset_host` | Empty | `hard_reset` | Reset to await_surgeon_input from any state |
| `/lbr/plan_flag` | Int32 | `start_drill` | Execute drill for specific pin number |

### Data Input Topics
| Topic | Type | Stored In | Description |
|-------|------|-----------|-------------|
| `/camera/color/image_rect_raw` | Image | `last_rgb_image` | RGB camera feed |
| `/camera/aligned_depth_to_color/image_raw` | Image | `last_depth_image` | Depth camera feed |
| `/camera/depth/color/points` | PointCloud2 | `last_cloud` | 3D point cloud |

### Output Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/processed_point_cloud` | PointCloud2 | Registered bone point clouds (colored) |
| `/drill_pose_camera_frame` | PoseArray | Computed drill hole poses |
| `/fitness_marker` | Marker | Registration quality visualization |

## 🔧 Parameters

### `selected_bones` (string)
- **Default**: `'both'`
- **Options**: `'femur'`, `'tibia'`, `'both'`
- **Purpose**: Configure which bones to segment and register
- **Modifiable**: Yes, at runtime via CLI "Toggle Bones"
- **Effect**: Controls which bones are processed in segmentation state

## 📝 Implementation Status

| State | Status | Priority |
|-------|--------|----------|
| start | ✅ Complete | - |
| await_surgeon_input | ✅ Complete | - |
| bring_manipulator | ⚠️ TODO | HIGH |
| auto_reposition | ⚠️ TODO | MEDIUM |
| segment_and_register | ✅ Complete | - |
| update_rviz | ⚠️ TODO | LOW |
| ready_to_drill | ✅ Complete | - |
| drill | ⚠️ TODO | HIGH |
| finished | ✅ Complete | - |

## 🚀 Next Steps for Implementation

### 1. **DRILL State** (Highest Priority)
```python
def on_enter_drill(self):
    # Get the drill pose for the requested pin
    if self.current_drill_pin is None or self.last_drill_pose_array is None:
        self.get_logger().error('No drill pin or pose array available!')
        self.hard_reset()
        return
    
    # Extract pose from array
    drill_pose = self.last_drill_pose_array.poses[self.current_drill_pin]
    
    # TODO: Send to robot controller
    # - Transform pose from camera_frame to base_frame
    # - Call motion planning service
    # - Execute trajectory
    # - Wait for completion
    
    self.complete_drill()
```

### 2. **BRING_MANIPULATOR State**
```python
def on_enter_bring_manipulator(self):
    # TODO: Call motion service
    # srv_client = self.create_client(MoveToPose, '/move_to_pose')
    # request = MoveToPose.Request()
    # request.target_pose = near_home_pose
    # future = srv_client.call_async(request)
    # future.add_done_callback(lambda f: self.complete_bring_manipulator())
```

### 3. **AUTO_REPOSITION State**
```python
def on_enter_auto_reposition(self):
    # TODO: Use DINO to find bone centroid
    # from parasight.dino_bone_extract import DINOBoneExtractor
    # extractor = DINOBoneExtractor()
    # centroid = extractor.get_centroid(self.last_rgb_image)
    # 
    # # Compute camera pose to center bones
    # target_pose = compute_camera_pose(centroid)
    # 
    # # Move robot
    # move_robot_to_pose(target_pose)
```

### 4. **UPDATE_RVIZ State**
```python
def on_enter_update_rviz(self):
    # TODO: Publish visualization markers
    # for i, pose in enumerate(self.last_drill_pose_array.poses):
    #     marker = create_drill_marker(pose, i)
    #     self.marker_publisher.publish(marker)
```

## 🔍 Debugging Tips

1. **Check current state**: All state changes log with `━━━ STATE: <name> ━━━`
2. **Use hard reset**: Send Empty message to `/hard_reset_host` from any state
3. **Toggle bones**: Use CLI "Toggle Bones" to cycle through femur/tibia/both
4. **Visualize**: All states use emoji icons for easy log scanning

## 📚 Related Files

- **Main Implementation**: `host.py`
- **CLI Client**: `cli_client.py` (surgeon interface)
- **Segmentation UI**: `segment_ui.py` (SAM integration)
- **Registration**: `registration.py` (ICP pipeline)
- **DINO Extractor**: `dino_bone_extract.py` (auto-positioning)
- **Plan Config**: `resource/plan_boneparte.yaml` (drill hole definitions)

