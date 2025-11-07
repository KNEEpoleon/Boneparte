# ParaSight Surgeon Workflow Guide

## 🎯 Quick Start

### Launch the System
```bash
# Terminal 1: Start ParaSight Host
ros2 run parasight host

# Terminal 2: Start CLI Client
ros2 run parasight cli_client
```

## 📋 Standard Surgical Workflow

### **Step 1: System Boot**
- System starts in `START` state
- Auto-transitions to `AWAIT_SURGEON_INPUT`
- You'll see: `⏸  Awaiting surgeon input...`

---

### **Step 2: Configure Bone Selection**
**When**: Before any segmentation

**CLI Command**: Select **"Toggle Bones"**

**Options**:
- `Both` - Process femur AND tibia (default)
- `Femur` - Process femur only
- `Tibia` - Process tibia only

**Note**: You can change this at any time before segmentation

---

### **Step 3: Optional - Position Manipulator**
**When**: If robot is at far home position

**CLI Command**: *Not yet implemented in CLI*

**What happens**:
1. System enters `BRING_MANIPULATOR` state
2. Robot moves from far home → near home
3. System enters `AUTO_REPOSITION` state
4. Camera auto-positions to center bones
5. Returns to `AWAIT_SURGEON_INPUT`

**Status**: ⚠️ Feature not yet implemented

---

### **Step 4: Segment and Register Bones**
**When**: Ready to register bone positions

**CLI Command**: Select **"Start ParaSight"**

**What happens**:
1. System enters `SEGMENT_AND_REGISTER` state
2. RGB image pops up in OpenCV window
3. **You click** on the femur (if selected)
4. **You click** on the tibia (if selected)
5. SAM segments the bone regions (you'll see colored masks)
6. Press **ENTER** to confirm or **ESC** to restart
7. System performs ICP registration to CT model
8. Computes drill hole poses from surgical plan
9. Publishes registered point clouds (red=femur, blue=tibia)

**Duration**: ~30-60 seconds (depends on registration complexity)

---

### **Step 5: Verify Registration**
**When**: After segmentation completes

**What to check**:
- System enters `UPDATE_RVIZ` state
- Registered point clouds appear in RViz
- Check registration quality (fitness score)
- Verify drill pose markers align with anatomy

**If good**: Continue to drilling
**If bad**: Select **"Reset ParaSight"** and repeat Step 4

---

### **Step 6: System Ready**
**When**: Registration verified

**State**: `READY_TO_DRILL`

**You'll see**: `✅ System ready to drill!`

**Available actions**:
- **Drill** - Execute drilling for specific hole
- **Reset ParaSight** - Go back to await input (e.g., to re-register)

---

### **Step 7: Execute Drilling**
**When**: Ready to drill specific hole

**CLI Command**: Select **"Drill"**

**What happens**:
1. System receives drill command on `/lbr/plan_flag`
2. Enters `DRILL` state
3. Extracts drill pose for requested pin number
4. Commands robot to execute drilling motion
5. Returns to `AWAIT_SURGEON_INPUT` when complete

**Status**: ⚠️ Currently auto-completes (not fully implemented)

**Note**: You can drill multiple holes by:
1. Select "Drill" → drills hole → returns to `AWAIT_SURGEON_INPUT`
2. Repeat as needed

---

### **Step 8: Complete Surgery**
**When**: All drilling complete

**CLI Command**: *Not yet in CLI*

**What happens**:
- System enters `FINISHED` state
- Can shut down or hard reset for next procedure

---

## 🔧 Emergency Controls

### **Hard Reset**
**CLI Command**: Select **"Reset ParaSight"**

**Effect**: Returns to `AWAIT_SURGEON_INPUT` from **any state**

**Use when**:
- Registration failed
- Wrong bone was selected
- Need to restart workflow
- System stuck in a state

---

## 🎨 Visual Feedback

The system uses emoji icons in logs for easy scanning:

| Icon | Meaning |
|------|---------|
| ✓ | Success / Completed |
| ⏸  | Waiting for input |
| 📥 | Received input |
| 🎯 | Processing (segmentation) |
| 🦾 | Robot motion |
| 📷 | Camera operation |
| 📊 | Visualization update |
| ✅ | Ready state |
| 🔩 | Drilling |
| 🏁 | Finished |
| ⚠️  | Warning / Not implemented |
| ❌ | Error |
| 🔄 | Reset |

---

## 📊 CLI Command Reference

| Command | State Required | Effect |
|---------|---------------|--------|
| **Start ParaSight** | await_surgeon_input | Begin segmentation workflow |
| **Reset ParaSight** | Any | Hard reset to await input |
| **Drill** | ready_to_drill | Execute drilling motion |
| **Toggle Bones** | Any | Cycle through femur/tibia/both |
| **Register** | Any | *Not connected to state machine* |
| **Register (+UI)** | Any | *Not connected to state machine* |
| **Exit** | Any | Shutdown CLI |

---

## ⚡ Typical Surgery Timeline

```
00:00 - System boot
00:05 - Configure bone selection (optional)
00:10 - Start ParaSight (segmentation)
00:15 - Click femur point
00:20 - Click tibia point
00:25 - Press ENTER to confirm
00:30 - Registration running...
01:00 - Registration complete
01:05 - Verify in RViz
01:10 - System ready to drill
01:15 - Execute drill (hole 1)
02:00 - Drill complete
02:05 - Execute drill (hole 2)
02:50 - All drilling complete
02:55 - Complete mission
```

**Total**: ~3 minutes per registration + drilling cycle

---

## 🐛 Troubleshooting

### "No RGB image available for segmentation!"
**Cause**: Camera not publishing images
**Fix**: Check camera is connected and topics are live

### "UI trigger ignored - not in await_surgeon_input state"
**Cause**: Tried to start segmentation from wrong state
**Fix**: Press "Reset ParaSight" first

### "Drill command ignored - not in ready_to_drill state"
**Cause**: Tried to drill before registration complete
**Fix**: Complete segmentation workflow first

### Registration looks wrong
**Cause**: Bad segmentation or initial point selection
**Fix**: Press "Reset ParaSight" and try again with better points

### System stuck
**Cause**: State machine in unexpected state
**Fix**: Press "Reset ParaSight" (works from any state)

---

## 📝 Notes

- **selected_bones parameter**: Changes what bones are segmented. Toggle before "Start ParaSight"
- **Multiple drill holes**: You can drill multiple times from ready_to_drill state
- **Re-registration**: If anatomy moves, press "Reset ParaSight" and re-register
- **Camera positioning**: DINO auto-positioning not yet implemented - manually position camera

---

## 🔮 Future Features (Not Yet Implemented)

- ⚠️ **Proceed Mission** - Automatic manipulator positioning
- ⚠️ **Auto-reposition** - DINO-based camera positioning
- ⚠️ **RViz markers** - Visual drill pose markers
- ⚠️ **Complete Mission** - Proper surgery completion workflow
- ⚠️ **Drill execution** - Full robot motion integration

---

## 📞 Support

For issues or questions:
1. Check logs for emoji indicators
2. Use "Reset ParaSight" if stuck
3. Verify camera topics are publishing
4. Check STATE_MACHINE.md for technical details

