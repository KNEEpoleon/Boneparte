# ParaSight State Machine Implementation Summary

## Files Created

1. **`host_state_machine.py`** - New simplified state machine implementation
2. **`STATE_MACHINE_DESIGN.md`** - Detailed design documentation
3. **`test_state_machine.sh`** - Test script for state transitions

## Key Changes from Original Design

### State Simplification

**Original (10 states):**
```
start → bring_manipulator → auto_reposition → doc_verify → annotate → 
register_and_update → ready_to_drill → drill → send_manipulator → finished
```

**Simplified (6 states):**
```
start → waiting ⟷ segmenting → registering → waiting ⟷ drilling → finished
```

### Merged States

1. **`doc_verify` + `ready_to_drill` → `waiting`**
   - Both are passive waiting states
   - Different transitions out based on command received
   
2. **Removed**: `bring_manipulator`, `auto_reposition`, `send_manipulator`
   - These are manipulation tasks, not perception tasks
   - Handled by manipulation node via topics/services

3. **Split**: `annotate` and `register_and_update`
   - `segmenting`: User interaction (blocking UI)
   - `registering`: Automatic processing

## Topic Interface

### Commands (Input)

| Topic | Type | Trigger |
|-------|------|---------|
| `/cmd/request_annotation` | Empty | Start segmentation UI |
| `/cmd/drill_mission` | Int32 | Start drilling (pin index) |
| `/cmd/end_surgery` | Empty | End workflow |
| `/manipulation/drill_complete` | Empty | Drilling finished |

### Status (Output)

| Topic | Type | Content |
|-------|------|---------|
| `/registration` | String | 'idle', 'registering', 'complete', 'failed' |
| `/surgical_drill_pose` | PoseArray | Computed drill poses |
| `/processed_point_cloud` | PointCloud2 | Registered bones |

## Placeholder Functions

The following functions need to be implemented with actual logic:

```python
def perform_registration(self, annotated_points, mask_points, masks):
    """TODO: Implement actual registration logic from existing host.py"""
    
def compute_drill_poses(self, transforms):
    """TODO: Implement actual pose computation from existing host.py"""
    
def publish_point_cloud(self, clouds):
    """TODO: Implement actual publishing from existing host.py"""

def rgb_image_callback(self, msg):
    """TODO: Convert ROS Image to OpenCV format"""

def pcd_callback(self, msg):
    """TODO: Convert ROS PointCloud2 to Open3D format"""
```

## Integration Steps

### Option 1: Side-by-Side Testing (Recommended)

Keep both versions and test the new one:

```bash
# Old version (if needed)
ros2 run parasight host.py

# New version (for testing)
ros2 run parasight host_state_machine.py
```

### Option 2: Gradual Migration

1. Copy functions from `host.py` to `host_state_machine.py`:
   - `perform_registration()` logic
   - `compute_drill_poses()` logic  
   - `publish_point_cloud()` logic
   - Image conversion utilities

2. Update `setup.py` to include new entry point

3. Test thoroughly

4. Replace when confident

### Option 3: Update Existing

Refactor existing `host.py` to use the new state machine structure.

## Testing

### Quick Test

```bash
# Terminal 1: Run state machine
ros2 run parasight host_state_machine.py

# Terminal 2: Run test script
./test_state_machine.sh
```

### Manual Test Commands

```bash
# Start workflow
ros2 topic pub /cmd/request_annotation std_msgs/msg/Empty "{}" --once

# Monitor status
ros2 topic echo /registration

# Request drilling
ros2 topic pub /cmd/drill_mission std_msgs/msg/Int32 "data: 0" --once

# Simulate drill complete
ros2 topic pub /manipulation/drill_complete std_msgs/msg/Empty "{}" --once

# End surgery
ros2 topic pub /cmd/end_surgery std_msgs/msg/Empty "{}" --once
```

## Answers to Your Questions

### Q: "is it right to implement those [planning nodes] in a state machine for this node?"

**A:** No. The perception node should only:
- Publish drill poses
- Publish registration status
- React to completion signals

The manipulation node should handle actual planning and execution.

### Q: "does it make sense to have this outside this node?"

**A:** Yes. External commands (AVP/TCP) should publish to ROS topics. This keeps perception decoupled from communication protocols.

### Q: "please suggest if it makes sense to merge this state"

**A:** Merged:
- ✅ `doc_verify` + `ready_to_drill` → `waiting` (both passive)
- ✅ Removed planning states (not perception's job)
- ✅ Kept `segmenting` + `registering` separate (different concerns)

## Next Steps

1. **Fill in placeholder functions** by copying from existing `host.py`
2. **Add to setup.py** for proper installation
3. **Test state transitions** using test script
4. **Integrate with manipulation stack** (update topics/services)
5. **Test full workflow** with real camera and robot
6. **Add error handling** and recovery mechanisms
7. **Add visualization** of current state in RViz

## Benefits of New Design

- ✅ **Simpler**: 6 states instead of 10
- ✅ **Clearer**: Each state has single responsibility
- ✅ **Decoupled**: Perception and manipulation separated
- ✅ **Testable**: Can test without robot hardware
- ✅ **Flexible**: Easy to add new states/transitions
- ✅ **Maintainable**: Clear documentation and structure

