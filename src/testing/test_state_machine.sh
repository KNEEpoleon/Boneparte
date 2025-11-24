#!/bin/bash
# Test script for ParaSight Host State Machine

echo "========================================="
echo "ParaSight State Machine Test Script"
echo "========================================="
echo ""
echo "This script will test the state machine transitions."
echo "Make sure parasight host_state_machine.py is running!"
echo ""
echo "Press ENTER to continue..."
read

# Test 1: Auto-reposition (happens automatically after begin_surgery)
echo ""
echo "TEST 1: Auto-reposition should start automatically..."
echo "Expected: State should be in 'auto_reposition', computing robot position"
sleep 3

# Test 1b: Simulate reposition complete
echo ""
echo "TEST 1b: Simulating reposition complete..."
echo "Expected: State should transition to 'waiting'"
ros2 topic pub /manipulation/reposition_complete std_msgs/msg/Empty "{}" --once
sleep 2

# Test 2: Request annotation
echo ""
echo "TEST 2: Requesting annotation..."
echo "Expected: State should transition to 'segmenting'"
ros2 topic pub /cmd/request_annotation std_msgs/msg/Empty "{}" --once
sleep 2

# Test 3: Monitor registration status
echo ""
echo "TEST 3: Monitoring registration status..."
echo "Expected: Should see 'registering' → 'complete' → 'idle'"
timeout 5 ros2 topic echo /registration --once
sleep 2

# Test 4: Request drill mission
echo ""
echo "TEST 4: Requesting drill mission (pin 0)..."
echo "Expected: State should transition to 'drilling'"
ros2 topic pub /cmd/drill_mission std_msgs/msg/Int32 "data: 0" --once
sleep 2

# Test 5: Simulate drill complete
echo ""
echo "TEST 5: Simulating drill complete..."
echo "Expected: State should return to 'waiting'"
ros2 topic pub /manipulation/drill_complete std_msgs/msg/Empty "{}" --once
sleep 2

# Test 6: End surgery
echo ""
echo "TEST 6: Ending surgery..."
echo "Expected: State should transition to 'finished'"
ros2 topic pub /cmd/end_surgery std_msgs/msg/Empty "{}" --once
sleep 2

echo ""
echo "========================================="
echo "Test complete! Check the node logs."
echo "========================================="
