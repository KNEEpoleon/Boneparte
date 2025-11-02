# ParaSight Host State Machine Design

## Simplified State Machine

### States

1. **`start`**: Initial state
2. **`auto_reposition`**: CV-based robot positioning using DINO features (ADDED BACK from original design)
3. **`waiting`**: Idle state, waiting for doctor commands (MERGED `doc_verify` + `ready_to_drill`)
4. **`segmenting`**: User annotates bone regions (MERGED `annotate` + first part of `register_and_update`)
5. **`registering`**: Automatic registration and obstacle update (second part of `register_and_update`)
6. **`drilling`**: Drilling in progress (manipulator working)
7. **`finished`**: Surgery complete

### Transitions

```
start --> auto_reposition [begin_surgery]
auto_reposition --> waiting [complete_auto_reposition]
waiting --> segmenting [request_annotation]
segmenting --> registering [complete_segmentation]
registering --> waiting [complete_registration]
waiting --> drilling [request_drill]
drilling --> waiting [complete_drill]
waiting --> finished [end_surgery]
```

## Design Decisions

### 1. **Added Back `auto_reposition` State**

**Rationale:**
- `auto_reposition` uses DINO features and computer vision pipeline
- This is perception-based functionality, not manipulation
- Determines optimal robot positioning based on scene analysis
- Belongs in perception stack, not manipulation stack

**Implementation:**
- Analyzes RGB/depth images and point clouds
- Extracts DINO features for scene understanding
- Computes optimal robot base position and orientation
- Publishes positioning commands to manipulation stack via `/cmd/robot_position`
- Waits for completion signal via `/manipulation/reposition_complete`

**Benefits:**
- Proper separation of concerns (CV in perception, motion in manipulation)
- Reusable computer vision pipeline
- Can be enhanced with additional scene analysis

### 2. **Merged `doc_verify` + `ready_to_drill` → `waiting`**

**Rationale:**
- Both states are passive waiting states
- Both wait for external commands
- The only difference is which command they're waiting for
- Can be handled by a single state with multiple possible transitions

**Benefits:**
- Simpler state machine
- Less code duplication
- Clearer intent: "system is ready, waiting for commands"

### 2. **Kept `segmenting` and `registering` Separate**

**Rationale:**
- `segmenting`: Requires user interaction (blocking UI)
- `registering`: Fully automatic, publishes to manipulation stack
- Different error handling requirements
- Clear separation of concerns

**Benefits:**
- Can add timeout/cancel to segmentation
- Registration can be retriggered independently
- Clear progress indication

### 3. **Manipulation Nodes NOT in State Machine**

**Rationale:**
- `bring_manipulator`, `send_manipulator`, `drill` are manipulation tasks
- Perception node should not control manipulator directly
- Use service calls or topic publishing instead

**Implementation:**
- Perception publishes commands: `/cmd/drill_mission`
- Manipulation subscribes and executes
- Manipulation publishes status: `/manipulation/drill_complete`
- Perception reacts to completion

**Benefits:**
- Clean separation of concerns
- Manipulation can be tested independently
- Easier to add safety checks in manipulation node

### 4. **External Commands via Topics**

Commands from AVP/TCP or CLI:
- `/cmd/request_annotation` (Empty) → triggers segmentation
- `/cmd/drill_mission` (Int32) → triggers drilling with pin index
- `/cmd/end_surgery` (Empty) → ends workflow

**Benefits:**
- Decoupled from external systems
- Easy to test with `ros2 topic pub`
- Can be called from AVP, CLI, or any other source

## Topic/Service Interface

### Published by Perception

| Topic | Type | Purpose |
|-------|------|---------|
| `/registration` | String | Registration status: 'idle', 'registering', 'complete', 'failed' |
| `/surgical_drill_pose` | PoseArray | Drill poses after registration |
| `/processed_point_cloud` | PointCloud2 | Registered bone point clouds |

### Subscribed by Perception

| Topic | Type | Purpose |
|-------|------|---------|
| `/cmd/request_annotation` | Empty | Doctor requests bone annotation |
| `/cmd/drill_mission` | Int32 | Doctor requests drilling (pin index) |
| `/cmd/end_surgery` | Empty | Doctor ends surgery |
| `/manipulation/drill_complete` | Empty | Manipulator signals drill complete |
| `/camera/color/image_rect_raw` | Image | RGB camera feed |
| `/camera/aligned_depth_to_color/image_raw` | Image | Depth camera feed |
| `/camera/depth/color/points` | PointCloud2 | Point cloud feed |

### Called by Manipulation (Not Perception!)

| Topic | Type | Purpose |
|-------|------|---------|
| `/cmd/drill_mission` | Int32 | Manipulation subscribes, receives pin index |
| `/select_pose` | Service | Alternative: manipulation uses existing service |

## Integration with Existing Code

### Option 1: Replace Existing host.py

```python
# In parasight/parasight/host.py
from parasight.host_state_machine import ParaSightHost
```

### Option 2: Keep Both (Recommended for Testing)

Keep existing `host.py` and new `host_state_machine.py` separate until tested:

```bash
# Run old version
ros2 run parasight host.py

# Run new version
ros2 run parasight host_state_machine.py
```

### Option 3: Gradual Migration

1. Copy functions from existing `host.py` to `host_state_machine.py`
2. Fill in placeholder functions
3. Test state transitions
4. Replace when ready

## Topic Interface

### Published Topics (by ParaSightHost)

- `/registration` (`std_msgs/String`): Registration status ("idle", "registering", "complete", "failed")
- `/surgical_drill_pose` (`geometry_msgs/PoseArray`): Computed drill poses after registration
- `/processed_point_cloud` (`sensor_msgs/PointCloud2`): Registered bone point clouds for visualization
- `/cmd/robot_position` (`geometry_msgs/PoseArray`): Robot positioning commands from auto_reposition

### Subscribed Topics (External Commands)

- `/cmd/request_annotation` (`std_msgs/Empty`): Trigger segmentation workflow
- `/cmd/drill_mission` (`std_msgs/Int32`): Request drilling at specific pin index
- `/cmd/end_surgery` (`std_msgs/Empty`): End surgery workflow

### Subscribed Topics (Manipulation Feedback)

- `/manipulation/drill_complete` (`std_msgs/Empty`): Drilling operation completed
- `/manipulation/reposition_complete` (`std_msgs/Empty`): Robot repositioning completed

### Sensor Data Topics

- `/camera/color/image_rect_raw` (`sensor_msgs/Image`): RGB camera feed
- `/camera/depth/image_rect_raw` (`sensor_msgs/Image`): Depth camera feed  
- `/camera/depth/color/points` (`sensor_msgs/PointCloud2`): Point cloud data

## Testing Workflow

### 1. Start the system

```bash
# Terminal 1: Start perception
ros2 run parasight host_state_machine.py

# Terminal 2: Start manipulation
ros2 launch surgical_robot_planner drill_motion_executor.launch.py model:=med7

# Terminal 3: Start robot & MoveIt
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock rviz:=true
```

### 2. Test auto-reposition (automatic after begin_surgery)

```bash
# Auto-reposition should start automatically
# Monitor robot positioning commands:
ros2 topic echo /cmd/robot_position

# Simulate reposition complete:
ros2 topic pub /manipulation/reposition_complete std_msgs/msg/Empty "{}" --once
```

Expected: State transitions from 'auto_reposition' to 'waiting'

### 3. Trigger annotation

```bash
ros2 topic pub /cmd/request_annotation std_msgs/msg/Empty "{}" --once
```

Expected: Segmentation UI opens, user annotates, registration happens automatically

### 4. Monitor registration

```bash
ros2 topic echo /registration
```

Expected: See 'registering' → 'complete' → 'idle'

### 5. Drill a pin

```bash
ros2 topic pub /cmd/drill_mission std_msgs/msg/Int32 "data: 0" --once
```

Expected: Manipulation receives command, drills, publishes completion

### 6. End surgery

```bash
ros2 topic pub /cmd/end_surgery std_msgs/msg/Empty "{}" --once
```

Expected: State transitions to 'finished'

## Future Enhancements

1. **Add timeout to segmentation**: Auto-cancel if user takes too long
2. **Add error recovery**: If registration fails, retry or request new annotation
3. **Add progress indicators**: Publish registration progress percentage
4. **Add safety checks**: Validate drill poses before publishing
5. **Add logging**: Record all state transitions and decisions
6. **Add RViz integration**: Visualize current state

## Questions Addressed

> "is it right to implement those [planning nodes] in a state machine for this node?"

**Answer:** No. The perception node should publish commands/requests and listen for completion signals. The manipulation node handles the actual planning and execution.

> "does it make sense to have this outside this node?"

**Answer:** Yes. External commands (from AVP/TCP) should publish to ROS topics that this node subscribes to. This keeps the perception node decoupled from the communication protocol.

> "please suggest if it makes sense to merge this state"

**Answer:** Yes, merging `doc_verify` and `ready_to_drill` into `waiting` makes sense. Also keeping `segmenting` and `registering` separate (not merged with `annotate` + `register_and_update`) provides better control.


