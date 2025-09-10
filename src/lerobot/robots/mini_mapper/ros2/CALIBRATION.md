# Mini Mapper Odometry Calibration Guide

Complete guide to calibrate your Mini Mapper's odometry for accurate SLAM mapping and autonomous navigation.

## Why Calibrate?

Accurate odometry is **critical** for good SLAM performance:
- ✅ **Better loop closure** - robot knows when it returns to previous locations
- ✅ **Coherent maps** - prevents map fragments and overlays
- ✅ **Precise navigation** - autonomous waypoint following works correctly
- ✅ **Reduced drift** - robot position stays accurate over long missions

## Quick Start

**Easiest method:** Automatic lidar-based calibration

```bash
# 1. Start Nav2 SLAM system
./start_nav2_slam.sh

# 2. Position robot facing a wall (2+ meters away)
# 3. Run auto-calibration
cd ~/nav_ws && source install/setup.bash
export ROS_DOMAIN_ID=42
python3 ~/lerobot/src/lerobot/robots/mini_mapper/ros2/auto_calibrate.py

# 4. Follow prompts and get automatic scale factor
# 5. Update bridge code with recommended linear_scale value
```

## Calibration Methods

### Method 1: Automatic Lidar Calibration (Recommended)

**Requirements:**
- Flat wall or large obstacle
- 2+ meters clearance
- Nav2 SLAM running (lidar active)

**Setup:**
```
    Robot                Wall
      🤖  ←── 2+ meters ──→  ██
      ↑                     ██
   facing                   ██
    wall                    ██
```

**Process:**
1. **Lidar measures** initial distance to wall
2. **Robot drives** forward exactly 1 meter (commanded)
3. **Lidar measures** final distance  
4. **Actual movement** = initial_distance - final_distance
5. **Scale factor** = actual_movement / odometry_movement

**Example output:**
```
📐 Lidar measured movement: 0.985m
📐 Odometry measured movement: 1.534m  
🔧 Recommended odometry scale factor: 0.642
```

### Method 2: Manual Calibration with Hotkeys

Use your enhanced teleop with built-in calibration commands:

```bash
# On laptop
python -m lerobot.robots.mini_mapper.teleop_mini_mapper --robot-ip 100.64.227.10
```

**Calibration hotkeys:**
- **`1`** = Drive exactly 1 meter forward
- **`2`** = Rotate exactly 360° clockwise  
- **`3`** = Drive 0.5 meter forward (short test)
- **`4`** = Rotate 180° clockwise (half turn test)

**Linear calibration process:**
1. Position robot on known start line (tape measure)
2. Press **`1`** to drive 1 meter
3. Measure actual distance with tape measure
4. Calculate scale: `linear_scale = actual_distance / 1.0`

**Angular calibration process:**
1. Mark robot's initial orientation
2. Press **`2`** for full 360° rotation
3. Check if robot faces original direction
4. Adjust if needed: `angular_scale = 360° / actual_degrees`

### Method 3: Corner Calibration (Advanced)

**Perfect for both linear and angular calibration simultaneously**

**Setup - Position robot in room corner:**
```
    Wall A
    ██████████
    ██        
    ██        
    ██   🤖   ← Robot positioned ~1m from both walls
    ██    ↑     facing corner at 45° angle
    ██    │    
    ██    │    
    ██████████
         Wall B
```

**ASCII Art Setup Detail:**
```
Wall A (North wall)
██████████████████████████████████████████
██                                      ██
██                                      ██  
██                                      ██
██              🤖 ← Robot              ██ ← Wall B (East wall)
██             ↗                       ██
██        1m from each wall             ██
██                                      ██
██                                      ██
██████████████████████████████████████████
         Wall C (South wall)
```

**Corner calibration process:**

**Step 1: Linear calibration**
```
Before:           After 1m forward:
██████████        ██████████
██      ██        ██      ██
██  🤖  ██   →    ██      ██
██   ↑  ██        ██  🤖  ██
██      ██        ██   ↑  ██
██████████        ██████████
```

**Step 2: Angular calibration**
```
Before:           After 90° turn right:
██████████        ██████████  
██      ██        ██      ██
██  🤖  ██   →    ██  🤖→ ██
██   ↑  ██        ██      ██
██      ██        ██      ██
██████████        ██████████
```

**Advantages:**
- ✅ **Two reference walls** for cross-validation
- ✅ **Constrained space** prevents major drift
- ✅ **Both axes** can be calibrated in sequence
- ✅ **Easy to measure** with lidar or tape

## Applying Calibration Results

After getting your scale factors, update the bridge code:

**Edit:** `~/nav_ws/src/mini_mapper_nav/mini_mapper_nav/mini_mapper_bridge.py`

```python
# Odometry calibration factors (adjust based on real-world testing)
self.linear_scale = 0.642   # ← Update with your measured value
self.angular_scale = 1.0    # ← Update with your measured value
```

**Redeploy and restart:**
```bash
cd ~/lerobot/src/lerobot/robots/mini_mapper/ros2
./deploy_ros2_package.sh
cd ~/nav_ws && colcon build --packages-select mini_mapper_nav && source install/setup.bash

# Restart Nav2 SLAM to use new calibration
tmux kill-session -t mini_mapper_nav2_slam
./start_nav2_slam.sh
```

## Testing Calibration

**Linear test:**
1. Drive robot 2-3 meters in straight line
2. Check if odometry matches physical distance
3. Should be accurate within ±5cm

**Angular test:**  
1. Rotate robot 360° several times
2. Robot should face original direction
3. Should be accurate within ±5°

**SLAM test:**
1. Drive robot in a loop around room
2. Return to starting position
3. Map should show closed loop, not drift

## Calibration Tips

**🎯 Best Practices:**
- Calibrate on **flat, hard surfaces** (avoid carpet if possible)
- Use **slow speeds** for measurement accuracy
- **Multiple measurements** - take average of 3-5 tests
- **Different distances** - test both 0.5m and 1.0m movements
- **Fresh battery** - low battery affects motor speeds

**⚠️ Common Issues:**
- **Left/right drift** - Usually wheel diameter differences
- **Speed-dependent errors** - Calibrate at speeds you'll use for mapping
- **Surface dependency** - Carpet vs. hardwood may need different calibration
- **Motor wear** - Recalibrate periodically as motors wear

**🔧 Advanced Tuning:**
- **Wheelbase calibration** - If angular errors persist, check wheel separation distance in robot config
- **Encoder resolution** - Verify encoder counts per wheel revolution
- **Gear ratio** - Confirm motor gear ratios match configuration

## Troubleshooting

**Problem:** Odometry shows movement but robot didn't move
- **Cause:** Linear scale too high
- **Fix:** Reduce `linear_scale` value

**Problem:** Robot moves but odometry shows no movement  
- **Cause:** Linear scale too low
- **Fix:** Increase `linear_scale` value

**Problem:** Robot rotates in circles during straight driving
- **Cause:** Wheel diameter mismatch or angular scale error
- **Fix:** Check wheel mounting, calibrate angular scale

**Problem:** Map has good local features but poor loop closure
- **Cause:** Accumulated odometry drift
- **Fix:** Improve calibration accuracy, check for systematic errors

**Problem:** Calibration works in one room but not others
- **Cause:** Surface-dependent wheel slip
- **Fix:** Recalibrate on actual mapping surface

## File Locations

- **Auto-calibration script:** `~/lerobot/src/lerobot/robots/mini_mapper/ros2/auto_calibrate.py`
- **Enhanced teleop:** `teleop_mini_mapper.py` (hotkeys 1,2,3,4)
- **Bridge configuration:** `mini_mapper_nav/mini_mapper_bridge.py`
- **This guide:** `~/lerobot/src/lerobot/robots/mini_mapper/ros2/CALIBRATION.md`

---

**Remember:** Good calibration is the foundation of accurate autonomous navigation! Take time to get it right and your robot will perform much better. 🎯