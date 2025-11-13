# FSM and DINO Test Commands

## State Machine Overview
States: `start` → `await_surgeon_input` → `bring_manipulator` → `auto_reposition` → `await_surgeon_input` → `segment_and_register` → `update_rviz` → `ready_to_drill` → `drill` → `finished`

---

## 1. Launch Host Node
```bash
ros2 run parasight host
```
**Expected:** Node starts in `await_surgeon_input` state

---

## 2. Test Camera Data
### Check RGB Image
```bash
ros2 topic echo /camera/color/image_rect_raw --once
```
**Expected:** Image message data

### Check Depth Image
```bash
ros2 topic echo /camera/aligned_depth_to_color/image_raw --once
```
**Expected:** Depth image data

### Check Point Cloud
```bash
ros2 topic echo /camera/depth/color/points --once
```
**Expected:** Point cloud data

---

## 3. Test Auto-Reposition (DINO)
### Trigger proceed_mission → bring_manipulator → auto_reposition
```bash
ros2 topic pub --once /proceed_mission std_msgs/msg/Empty
```
**Expected:**
- State transitions: `await_surgeon_input` → `bring_manipulator` → `auto_reposition` → `await_surgeon_input`
- Image saved to `/ros_ws/src/perception/auto_reposition/query_MM-DD_HH:MM:SS.png`
- DINO detection runs (~770ms)
- Bone centroid published to `/bone_centroid_camera_frame`
- Reposition vector published to `/error_recovery_direction`
- Console shows: "DINO forward pass", "MeanShift clustering", "✓ Bone detected", "Displacement vector"

---

## 4. Test Segmentation and Registration
```bash
ros2 topic pub --once /annotate std_msgs/msg/Empty
```
**Expected:**
- State transitions: `await_surgeon_input` → `segment_and_register` → `update_rviz` → `ready_to_drill`
- SAM UI opens for annotation
- Registration completes
- Drill poses published to `/drill_pose_camera_frame`
- Point cloud published to `/processed_point_cloud`

---

## 5. Test Reset from ready_to_drill
```bash
ros2 topic pub --once /reset_mission std_msgs/msg/Empty
```
**Expected:**
- State transitions: `ready_to_drill` → `await_surgeon_input`

---

## 6. Test Drilling
```bash
ros2 topic pub --once /start_drill std_msgs/msg/Empty
```
**Expected:**
- State transitions: `ready_to_drill` → `drill` → `await_surgeon_input`

---

## 7. Test Mission Complete
```bash
ros2 topic pub --once /complete_mission std_msgs/msg/Empty
```
**Expected:**
- State transitions: `await_surgeon_input` → `finished`
- Console shows: "Mission complete!"

---

## 8. Test Hard Reset
```bash
ros2 topic pub --once /hard_reset_host std_msgs/msg/Empty
```
**Expected:**
- State transitions: `<any_state>` → `start`

---

## 9. Monitor State Changes
```bash
ros2 topic echo /rosout | grep "State changed"
```
**Expected:** See all state transitions logged

---

## 10. Verify DINO Output Files
After running auto_reposition, check:
```bash
ls /ros_ws/src/perception/auto_reposition/
```
**Expected:**
- `query_MM-DD_HH:MM:SS/` directory
- `query_MM-DD_HH:MM:SS_detected_bone.json` or `query_MM-DD_HH:MM:SS_NO_DETECTION.txt`
- Optional: `DINO_PCA_Features_pca_vis.png`, `DINO_Clusters.png`, `query_MM-DD_HH:MM:SS_detection.png`

---

## 11. Inspect DINO Detection Result
```bash
cat /ros_ws/src/perception/auto_reposition/query_*/*_detected_bone.json | jq
```
**Expected JSON:**
```json
{
  "detected_cluster": {...},
  "confidence": 0.918,
  "2d_bone_centroid": {
    "u": 371.0,
    "v": 160.0,
    "convention": "RealSense (u=horizontal, v=vertical)"
  },
  "distance_from_image_center": 96.0
}
```

---

## 12. Full Workflow Test Sequence
```bash
# Start from await_surgeon_input
ros2 topic pub --once /proceed_mission std_msgs/msg/Empty
# Wait for auto_reposition to complete (returns to await_surgeon_input)

ros2 topic pub --once /annotate std_msgs/msg/Empty  
# Annotate in SAM UI, wait for ready_to_drill

ros2 topic pub --once /start_drill std_msgs/msg/Empty
# Wait for drill to complete (returns to await_surgeon_input)

ros2 topic pub --once /complete_mission std_msgs/msg/Empty
# Ends in finished state
```

**Expected:** Complete state flow from start to finish

---

## State Transition Summary
| Current State | Command | Next State |
|--------------|---------|-----------|
| `await_surgeon_input` | `/proceed_mission` | `bring_manipulator` → `auto_reposition` → `await_surgeon_input` |
| `await_surgeon_input` | `/annotate` | `segment_and_register` → `update_rviz` → `ready_to_drill` |
| `await_surgeon_input` | `/complete_mission` | `finished` |
| `ready_to_drill` | `/reset_mission` | `await_surgeon_input` |
| `ready_to_drill` | `/start_drill` | `drill` → `await_surgeon_input` |
| `<any_state>` | `/hard_reset_host` | `start` |

