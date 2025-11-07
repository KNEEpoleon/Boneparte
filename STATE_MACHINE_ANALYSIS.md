# ParaSight State Machine Analysis

## Overview

This code uses the **`transitions`** Python library (specifically the `Machine` class) to implement a state machine. This is a **different approach** from manually implementing entry/exit callbacks - the library handles all that automatically!

## How the `transitions` Library Works

### 1. State Machine Setup

```python
self.machine = Machine(
    model=self,                          # The state machine operates on this object
    states=ParaSightHost.states,         # List of valid states
    initial='waiting',                   # Starting state
    after_state_change='publish_state'   # Callback after ANY state change
)
```

**What this does:**
- Creates a state machine that modifies `self` (the ParaSightHost node)
- Automatically adds a `self.state` attribute that tracks current state
- Calls `self.publish_state()` after every state transition

### 2. Defining Transitions

```python
self.machine.add_transition(
    trigger='all_systems_ready',  # Method name that will trigger this transition
    source='waiting',             # State you must be in
    dest='ready'                  # State you'll transition to
)
```

**What this creates:**
- Automatically generates a method: `self.all_systems_ready()` (the trigger)
- When called, if `self.state == 'waiting'`, it transitions to `'ready'`
- If you're not in `'waiting'`, the trigger is ignored (safe to call anytime)

**Special transition:**
```python
self.machine.add_transition(trigger='hard_reset', source='*', dest='waiting')
```
- The `'*'` means this transition works from **any state**
- Effectively a "panic button" that always returns to waiting

## State Machine Visualization

```
┌─────────────┐
│   waiting   │ ◄─────────────────────────────────────┐
└──────┬──────┘                                        │
       │ all_systems_ready()                           │
       ▼                                               │
┌─────────────┐                                        │
│    ready    │ ◄──────────────────────┐              │
└──────┬──────┘                        │              │
       │ start_parasight()        drill_complete()    │
       ▼                               │              │
┌─────────────┐                        │              │
│ user_input  │                        │              │
└──────┬──────┘                        │              │
       │ input_received()              │              │
       ▼                               │              │
┌──────────────────┐                   │              │
│  tracker_active  │ ──────────────────┘              │
└────┬────────┬────┘                                  │
     │        │ ready_to_drill()                      │
     │        ▼                                       │
     │   ┌─────────┐                                 │
     │   │ lock_in │                                 │
     │   └─────────┘                                 │
     │ tracking_lost()                               │
     ▼                                               │
┌──────────────┐                                     │
│system_paused │                                     │
└──────┬───────┘                                     │
       │ tracking_restored()                         │
       ▼                                             │
┌─────────────┐                                      │
│stabilizing  │                                      │
└──────┬──────┘                                      │
       │ stabilized()                                │
       └──────────────────────────────────────────►  │
                                                      │
         Any State: hard_reset() ─────────────────────┘
```

## Entry/Exit Callbacks with `transitions` Library

The library **automatically** looks for special methods:

### Naming Convention:
- `on_enter_<state_name>()` - called when entering a state
- `on_exit_<state_name>()` - called when leaving a state

### Example from the code:

```python
def on_enter_tracker_active(self):
    """Automatically called when entering 'tracker_active' state"""
    assert self.annotated_points is not None
    ros_points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in self.annotated_points]
    request = StartTracking.Request(points=ros_points, resume=False)
    
    # Start tracking service call
    self.start_tracking_client.wait_for_service()
    self.future = self.start_tracking_client.call_async(request)
    self.future.add_done_callback(partial(self.tracking_response_callback))
```

**What happens:**
1. User triggers `self.input_received()` (from `user_input` state)
2. State changes from `'user_input'` → `'tracker_active'`
3. Library automatically calls `self.on_enter_tracker_active()`
4. This method starts the tracking service

### You can also define exit callbacks:

```python
def on_exit_tracker_active(self):
    """Would be called automatically when leaving tracker_active state"""
    # Stop publishing, cleanup resources, etc.
    pass
```

## How Publishers Are Handled in This Code

### Current Approach: **All publishers always exist**

```python
# Publishers created once in __init__
self.pcd_publisher = self.create_publisher(PointCloud2, '/processed_point_cloud', 10)
self.pose_array_publisher = self.create_publisher(PoseArray, '/surgical_drill_pose', 10)
self.marker_publisher = self.create_publisher(Marker, '/fitness_marker', 10)
```

### Publishing is **state-aware** via conditionals:

```python
def tracked_points_callback(self, msg):
    # Only process if in the right state
    if not self.state == 'tracker_active':
        return
    
    # ... rest of the logic
    self.register_and_publish(annotated_points)
```

**Key insight:** Publishers exist all the time, but the **callback logic checks the state** before publishing.

### Where Publishing Happens:

| State | Publisher Used | Triggered By |
|-------|---------------|--------------|
| `tracker_active` | `pcd_publisher`, `pose_array_publisher` | `tracked_points_callback()` → `register_and_publish()` |
| Any (manual) | `pcd_publisher`, `pose_array_publisher` | `reg_request_callback()` → `register_and_publish()` |
| Any (manual) | `marker_publisher` | `publish_fitness_marker()` (currently deprecated) |

## Problem: Multiple Publishers Aren't Isolated by State

**Current behavior:**
- All publishers are always "active" (created and exist)
- Publishing is controlled by **conditional logic in callbacks**
- If you're in `'waiting'` state, `tracked_points_callback()` still receives messages (subscription is always active)

### Example of current state checking:

```python
def tracked_points_callback(self, msg):
    if not self.state == 'tracker_active':  # ← Manual state check
        return
    # ... publish logic
```

## How to Properly Isolate Publishers by State

### Option 1: Use Entry/Exit Callbacks to Control Timers

```python
def on_enter_tracker_active(self):
    """When entering tracker_active, start periodic publishing"""
    # Start tracking service
    self.start_tracking(...)
    
    # Create a timer to periodically check and publish
    self.tracking_timer = self.create_timer(0.1, self.tracking_publish_callback)

def on_exit_tracker_active(self):
    """When leaving tracker_active, stop publishing"""
    if hasattr(self, 'tracking_timer') and self.tracking_timer:
        self.tracking_timer.cancel()
        self.tracking_timer = None
    
    # Stop tracking service
    self.stop_tracking(...)

def tracking_publish_callback(self):
    """Only called while in tracker_active state (timer is active)"""
    # No need to check state - timer only exists in this state
    self.register_and_publish(...)
```

### Option 2: Keep Current Callback Approach (What You Have)

**Pros:**
- Simple - subscriptions always active
- Easy to debug - all callbacks always fire
- Lightweight - just early returns

**Cons:**
- Callbacks fire even when not needed
- Must remember state checks in every callback
- Can accidentally forget state check and publish in wrong state

### Option 3: Dynamically Create/Destroy Subscriptions (Advanced)

```python
def on_enter_tracker_active(self):
    """Create subscription only for this state"""
    self.tracked_points_subscription = self.create_subscription(
        TrackedPoints,
        '/tracked_points',
        self.tracked_points_callback,
        10
    )

def on_exit_tracker_active(self):
    """Destroy subscription when leaving state"""
    if self.tracked_points_subscription:
        self.destroy_subscription(self.tracked_points_subscription)
        self.tracked_points_subscription = None
```

**Pros:**
- True isolation - callback only fires in correct state
- No need for state checks in callbacks
- Most "pure" state machine approach

**Cons:**
- More complex
- Subscription creation/destruction overhead
- ROS subscriptions need time to establish

## Recommendations for Your Code

### Recommended Pattern: Entry/Exit with Timers

Since you're doing registration/processing that's computationally intensive:

```python
def __init__(self):
    # ... existing setup ...
    
    # Publishers - created once
    self.pcd_publisher = self.create_publisher(PointCloud2, '/processed_point_cloud', 10)
    self.pose_array_publisher = self.create_publisher(PoseArray, '/surgical_drill_pose', 10)
    
    # Data subscribers - always active (lightweight)
    self.rgb_image_subscription = self.create_subscription(...)
    self.depth_image_subscription = self.create_subscription(...)
    self.pcd_subscription = self.create_subscription(...)
    
    # Control subscribers - always active but check state
    self.ui_trigger_subscription = self.create_subscription(...)
    self.hard_reset_subscription = self.create_subscription(...)
    
    # State-specific timers - created/destroyed per state
    self.tracking_timer = None
    self.registration_timer = None

def on_enter_tracker_active(self):
    """Entry: Start tracking and periodic registration"""
    # Start tracking service
    self.start_tracking(...)
    
    # Create timer for periodic checking
    self.tracking_timer = self.create_timer(0.1, self.tracking_loop)

def on_exit_tracker_active(self):
    """Exit: Stop tracking and cancel timer"""
    # Cancel timer
    if self.tracking_timer:
        self.tracking_timer.cancel()
        self.tracking_timer = None
    
    # Stop tracking service
    self.stop_tracking(...)

def tracking_loop(self):
    """Periodic callback only active during tracker_active state"""
    # No state check needed - timer only exists in this state
    if self.need_to_register and self.annotated_points:
        self.register_and_publish(self.annotated_points)
```

## Key Differences from Manual Implementation

| Aspect | Manual Entry/Exit | `transitions` Library |
|--------|------------------|----------------------|
| State tracking | Manual `self.current_state` | Automatic `self.state` |
| Transitions | Manual `transition_to()` | Auto-generated trigger methods |
| Entry callbacks | Manual dispatch | Automatic via `on_enter_<state>()` |
| Exit callbacks | Manual dispatch | Automatic via `on_exit_<state>()` |
| Invalid transitions | Manual checking | Automatically ignored |
| Global transitions | Manual implementation | Built-in with `source='*'` |

## Summary

### How Your Code Handles Multiple Publishers:

1. **All publishers are created once** in `__init__`
2. **All subscriptions are always active** (always receiving messages)
3. **State is checked manually** in callbacks before publishing
4. **No automatic isolation** - you must remember to check state

### How to Improve:

1. **Add entry/exit callbacks** for states that publish
2. **Use timers** that are created/destroyed per state
3. **Keep data subscriptions always active** (RGB, depth, point cloud)
4. **Keep publishers always created** (lightweight)
5. **Only activate publishing timers** in appropriate states

This gives you:
- ✅ Automatic cleanup when leaving states
- ✅ No accidental publishing in wrong states
- ✅ Clear separation of state logic
- ✅ Efficient use of resources

