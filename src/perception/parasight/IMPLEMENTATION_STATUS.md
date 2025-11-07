# ParaSight State Machine Implementation Status

## Overview
This document tracks the implementation status of the ParaSight surgical planning state machine.

---

## State Machine Structure

### States
1. `start` - Initial startup state
2. `await_surgeon_input` - Waiting for surgeon commands
3. `bring_manipulator` - Positioning the robot manipulator
4. `auto_reposition` - Automatic repositioning
5. `segment_and_register` - Segment bones and perform registration
6. `update_rviz` - Update visualization
7. `ready_to_drill` - Ready for drilling operation
8. `drill` - Execute drilling
9. `finished` - Mission complete

### Transitions
| Trigger | From State | To State |
|---------|-----------|----------|
| `proceed_mission` | await_surgeon_input | bring_manipulator |
| `complete_bring_manipulator` | bring_manipulator | auto_reposition |
| `complete_auto_reposition` | auto_reposition | await_surgeon_input |
| `annotate` | await_surgeon_input | segment_and_register |
| `complete_segment_and_register` | segment_and_register | update_rviz |
| `complete_map_update` | update_rviz | ready_to_drill |
| `reset_mission` | ready_to_drill | await_surgeon_input |
| `received_mission_pin` | ready_to_drill | drill |
| `complete_drill` | drill | await_surgeon_input |
| `complete_mission` | await_surgeon_input | finished |

---

## Implementation Status

### ✅ Fully Implemented Components

#### State Entry Callbacks
- ✅ `on_enter_segment_and_register()` - Triggers SAM segmentation and registration

#### ROS Callbacks
- ✅ `parameter_change_callback()` - Handles parameter updates
- ✅ `hard_reset_callback()` - Resets state machine to start
- ✅ `ui_trigger_callback()` - Handles UI triggers for annotation
- ✅ `rgb_image_callback()` - Caches RGB images
- ✅ `depth_image_callback()` - Caches depth images  
- ✅ `pcd_callback()` - Caches point clouds

#### Utility Methods
- ✅ `update_bones()` - Updates selected bones parameter
- ✅ `add_depth()` - Adds depth information to 2D points
- ✅ `pose_direction()` - Computes theta based on bone positions

#### Registration & Publishing
- ✅ `register_and_publish()` - Main registration pipeline
- ✅ `publish_point_cloud()` - Publishes combined point clouds
- ✅ `compute_plan()` - Computes drill poses from registration

#### Infrastructure
- ✅ State machine setup with `transitions` library
- ✅ All ROS publishers/subscribers configured
- ✅ TF buffer and listener setup
- ✅ Resource loading (bone meshes, plan config)
- ✅ Interface initialization (RegistrationPipeline, SegmentAnythingUI)

---

### 🔨 Needs Implementation

#### State Entry Callbacks
- ⚠️ `on_enter_start()` - TODO: Implement startup logic
- ⚠️ `on_enter_await_surgeon_input()` - TODO: Implement await logic
- ⚠️ `on_enter_bring_manipulator()` - TODO: Implement manipulator positioning
- ⚠️ `on_enter_auto_reposition()` - TODO: Implement auto-reposition logic
- ⚠️ `on_enter_update_rviz()` - TODO: Implement RViz update logic
- ⚠️ `on_enter_ready_to_drill()` - TODO: Implement ready_to_drill logic
- ⚠️ `on_enter_drill()` - TODO: Implement drilling logic
- ⚠️ `on_enter_finished()` - TODO: Implement cleanup/finish logic

#### State Exit Callbacks (Optional)
None defined yet - can be added as needed for cleanup

---

## Next Steps

### Recommended Implementation Order

1. **`on_enter_start()`**
   - Initialize system state
   - Check all subsystems are ready
   - Transition to `await_surgeon_input`

2. **`on_enter_await_surgeon_input()`**
   - Set up waiting state
   - Clear any active timers/processes
   - Ready to receive UI commands

3. **`on_enter_update_rviz()`**
   - Ensure point clouds are published
   - Ensure drill poses are published
   - Transition to `ready_to_drill`

4. **`on_enter_ready_to_drill()`**
   - Wait for mission pin confirmation
   - Display ready status
   - Set up drill callback

5. **`on_enter_bring_manipulator()` & `on_enter_auto_reposition()`**
   - Implement manipulator control
   - Call appropriate services/actions
   - Automatic transitions when complete

6. **`on_enter_drill()` & `on_enter_finished()`**
   - Execute drilling sequence
   - Cleanup and mission completion

---

## Key Design Patterns Used

### Entry/Exit Callbacks
- The `transitions` library automatically calls `on_enter_<state>()` when entering a state
- Use these for setup/initialization logic
- Can add `on_exit_<state>()` for cleanup if needed

### Event-Driven Transitions
- ROS callbacks check current state before acting
- Transitions triggered by calling `self.trigger('<trigger_name>')`
- Invalid transitions are automatically ignored by the library

### Publisher Management
- All publishers created once in `__init__`
- Publishing controlled by state logic (not by creating/destroying publishers)
- Callbacks check state before publishing

---

## Testing Strategy

### Unit Testing
1. Test each state entry callback independently
2. Test state transitions with mock data
3. Test that invalid transitions are rejected

### Integration Testing
1. Test full workflow from `start` → `finished`
2. Test error recovery and `hard_reset`
3. Test state transitions with real sensor data

### Manual Testing
1. Use `ui_trigger_callback` to trigger segmentation
2. Verify point clouds and drill poses are published
3. Verify state transitions in logs

---

## Notes

- The state machine uses the `transitions` library for automatic state management
- All state transitions are logged via `after_state_change()`
- The `hard_reset` callback can return to `start` from any state
- Data subscribers (RGB, depth, point cloud) are always active
- Publishing only happens in appropriate states via conditional logic

