# AVP-FSM Integration Fix Summary

## Problem Statement
The Apple Vision Pro (AVP) app was not properly coordinating with the ParaSight FSM state machine, causing annotation commands to only work at random points in the FSM cycle.

## Root Cause
1. AVP "Annotate" button was publishing to `/trigger_host_ui` (old/unused topic)
2. FSM expects `/annotate` topic for state transitions
3. Missing "Proceed Mission" button to properly start the surgical workflow
4. Segmentation rejection wasn't resetting FSM to correct state

## Changes Made

### 1. AVP ContentView.swift (`src/vision_pro/SVD_ROS_Comms/SVD_ROS_Comms/ContentView.swift`)

#### Added "Proceed Mission" Button
- **Location**: Above "Annotate" button in Control Panel
- **Command**: `proceed_mission`
- **Icon**: `play.circle.fill`
- **Description**: "Begin mission (home + auto-reposition)"
- **Function**: Moves robot home + triggers FSM transition through auto-reposition

#### Updated Button Titles
- "Home" button → "Bring the robot home (backup)" - clarifies it's a backup function
- Reordered Control Panel for logical workflow progression

#### Updated Command Map
```swift
private let commandMap: [String: String] = [
    "Proceed Mission": "proceed_mission",  // NEW
    "Annotate": "annotate",
    "robot_away": "robot_away",
    "robot_home": "robot_home",
    "Femur 1": "drill_femur_1",
    "Femur 2": "drill_femur_2",
    "Femur 3": "drill_femur_3",
    "Tibia 1": "drill_tibia_1",
    "Tibia 2": "drill_tibia_2"
]
```

### 2. TCP Server Node (`src/vision_pro/tcp_server_pkg/tcp_server_pkg/tcp_server_node.py`)

#### Fixed ROS2 Topic Publishers
**Before:**
```python
self.start_pub = self.create_publisher(Empty, '/trigger_host_ui', 10)  # WRONG TOPIC
self.stop_pub = self.create_publisher(Empty, '/hard_reset_host', 10)
```

**After:**
```python
self.annotate_pub = self.create_publisher(Empty, '/annotate', 10)          # FSM expects this
self.proceed_mission_pub = self.create_publisher(Empty, '/proceed_mission', 10)  # NEW
self.reset_mission_pub = self.create_publisher(Empty, '/reset_mission', 10)      # NEW
self.hard_reset_pub = self.create_publisher(Empty, '/hard_reset_host', 10)
```

#### Added `handle_proceed_mission()` Method
```python
def handle_proceed_mission(self):
    """Handle proceed_mission command - move robot home and advance FSM through auto-reposition"""
    self.get_logger().info('Proceed mission command received')
    
    # First, command robot to go home
    self.call_robot_command_service("home")
    
    # Then publish to FSM to start proceed_mission transition
    # FSM will: await_surgeon_input -> bring_manipulator -> auto_reposition -> await_surgeon_input
    self.proceed_mission_pub.publish(Empty())
    self.get_logger().info('Published to /proceed_mission - FSM starting bring_manipulator -> auto_reposition')
```

#### Fixed Annotate Flow
**Changed:**
```python
# OLD: self.start_pub.publish(Empty())  # Wrong topic!
# NEW:
self.annotate_pub.publish(Empty())
self.get_logger().info('Published to /annotate with AVP annotations')
```

#### Fixed Rejection Flow
**Enhanced to reset FSM:**
```python
def handle_reject_segmentation(self):
    """Handle reject command from AVP - reset FSM to await_surgeon_input for new annotations"""
    self.get_logger().info('Segmentation rejected by AVP')
    
    # Call reject service first to clean up ParaSight state
    if not self.reject_seg_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().error('Service /reject_segmentation not available')
    else:
        request = EmptySrv.Request()
        future = self.reject_seg_client.call_async(request)
        self.get_logger().info('Called reject_segmentation service')
    
    # Reset FSM back to await_surgeon_input state
    self.reset_mission_pub.publish(Empty())
    self.get_logger().info('Published to /reset_mission - FSM returning to await_surgeon_input')
```

### 3. ParaSight Host FSM (`src/perception/parasight/parasight/host.py`)

#### Extended `reset_mission` Transition
**Before (only worked from `ready_to_drill`):**
```python
self.machine.add_transition(trigger='reset_mission', source='ready_to_drill', dest='await_surgeon_input')
```

**After (works from multiple states):**
```python
self.machine.add_transition(trigger='reset_mission', 
                            source=['segment_and_register', 'update_rviz', 'ready_to_drill'], 
                            dest='await_surgeon_input')
```

#### Updated Reset Mission Callback
**Before:**
```python
def reset_mission_callback(self, msg):
    if self.state == 'ready_to_drill':  # ONLY works in this state!
        self.trigger('reset_mission')
    else:
        self.get_logger().warn(f'Not in ready_to_drill state')
```

**After:**
```python
def reset_mission_callback(self, msg):
    if self.state in ['segment_and_register', 'update_rviz', 'ready_to_drill']:
        self.get_logger().info('Triggering reset_mission transition...')
        self.trigger('reset_mission')
    else:
        self.get_logger().warn(f'Reset mission command received but not in valid state')
```

**Why this fix was needed:**
- When AVP rejects segmentation, FSM is in `segment_and_register` state (not `ready_to_drill`)
- Original code would reject the reset command with a warning
- Now reset works from any segmentation/drilling-related state

## Complete Surgical Workflow

### Correct FSM Cycle (After Fix)

1. **Initial State**: FSM starts in `await_surgeon_input`

2. **AVP: Press "Away"** (optional)
   - Robot moves away via service call
   - No FSM state change

3. **AVP: Press "Proceed Mission"** NEW
   - Robot moves home via service call
   - Publishes to `/proceed_mission` topic
   - FSM: `await_surgeon_input` → `bring_manipulator` → `auto_reposition` → `await_surgeon_input`
   - DINO vision system auto-detects bone position
   - Takes ~2.5 seconds

4. **AVP: Press "Annotate"** FIXED
   - Publishes to `/annotate` topic (was `/trigger_host_ui`)
   - FSM: `await_surgeon_input` → `segment_and_register`
   - AVP shows annotation window
   - Surgeon annotates femur and tibia points
   - AVP sends annotations back to ROS

5. **Segmentation Review**
   - ParaSight runs SAM segmentation
   - AVP receives segmented image
   - Surgeon reviews segmentation

6. **Accept/Reject**
   - **If ACCEPT**:
     - FSM: `segment_and_register` → `update_rviz` → `ready_to_drill`
     - Drill poses computed and visualized
   - **If REJECT**: FIXED
     - Publishes to `/reset_mission` topic
     - FSM: Returns to `await_surgeon_input`
     - Surgeon can press "Annotate" again

7. **AVP: Select Drill Site**
   - Choose femur hole 1/2/3 or tibia hole 1/2
   - Robot executes drilling operation
   - FSM: `ready_to_drill` → `drill` → `await_surgeon_input`

8. **Repeat** steps 6-7 for additional holes

### Backup Commands

- **"Home" button**: Emergency backup to just move robot home (no FSM interaction)
- **"Away" button**: Move robot away for safety (no FSM interaction)

## FSM State Machine Overview

```
┌─────────────────────────────────────────────────────────────┐
│  start  →  (auto-initialize)  →  await_surgeon_input        │
└─────────────────────────────────────────────────────────────┘
                         │
                         │ /proceed_mission (NEW)
                         ↓
                  ┌──────────────┐
                  │ bring_       │
                  │ manipulator  │
                  └──────────────┘
                         │
                         ↓
                  ┌──────────────┐
                  │ auto_        │  (DINO vision detection)
                  │ reposition   │
                  └──────────────┘
                         │
                         ↓
          ┌──────────────────────────────┐
          │   await_surgeon_input        │  ← Can annotate from here
          └──────────────────────────────┘
                         │
                         │ /annotate (FIXED)
                         ↓
                  ┌──────────────┐
                  │ segment_and_ │  (SAM segmentation)
                  │ register     │
                  └──────────────┘
                         │
                         ↓
                  ┌──────────────┐
                  │ update_rviz  │
                  └──────────────┘
                         │
                         ↓
                  ┌──────────────┐
                  │ ready_to_    │  ← Drill commands work here
                  │ drill        │
                  └──────────────┘
                         │
                         │ /start_drill (drill site selected)
                         ↓
                  ┌──────────────┐
                  │   drill      │
                  └──────────────┘
                         │
                         └───→ back to await_surgeon_input
```

## Testing Commands

```bash
# Test proceed_mission flow
ros2 topic pub --once /proceed_mission std_msgs/msg/Empty

# Test annotate flow
ros2 topic pub --once /annotate std_msgs/msg/Empty

# Test reset_mission (from ready_to_drill state)
ros2 topic pub --once /reset_mission std_msgs/msg/Empty

# Monitor FSM state
ros2 topic echo /parasight_state  # If published by FSM
```

## Files Modified

1. `src/vision_pro/SVD_ROS_Comms/SVD_ROS_Comms/ContentView.swift`
   - Added "Proceed Mission" button
   - Updated command map
   - Reordered UI for better workflow

2. `src/vision_pro/tcp_server_pkg/tcp_server_pkg/tcp_server_node.py`
   - Fixed topic publishers (annotate, proceed_mission, reset_mission)
   - Added `handle_proceed_mission()` method
   - Fixed `handle_reject_segmentation()` to reset FSM
   - Updated command handlers

3. `src/perception/parasight/parasight/host.py` **NEW**
   - Extended `reset_mission` transition to work from multiple states
   - Updated `reset_mission_callback()` to accept transitions from `segment_and_register`, `update_rviz`, and `ready_to_drill`
   - **Why**: Original implementation only allowed reset from `ready_to_drill`, but rejection happens during `segment_and_register`

## Expected Behavior After Fix

**Annotate button works from `await_surgeon_input` state**
- No longer random/broken
- Only works after auto-reposition completes

**Proceed Mission button starts full workflow**
- Robot goes home
- Auto-reposition runs
- Returns to await_surgeon_input ready for annotation

**Rejection properly resets workflow**
- Returns to await_surgeon_input
- Surgeon can re-annotate

**Clear workflow progression**
- Away → Proceed Mission → Annotate → Accept/Reject → Drill

## Notes

- **"Proceed Mission" can be pressed anytime** - User responsible for not pressing in wrong state
- **Home button remains as backup** - Direct robot control without FSM interaction
- **Sleep timing preserved** - FSM's 2.5 second sleep in `bring_manipulator` state maintained
- **No changes to FSM code** - All fixes in AVP app and TCP server

