# Control System Integration & Deployment Guide

## Quick Reference Files

```
ROS 2 Control Architecture:
├── Main Control
│   ├── scripts/control_node.py              ← LQR main loop
│   ├── sub_control/lqr_solver.py            ← Math engine
│   ├── scripts/actuator_node.py             ← Hardware interface
│   └── config/params.yaml                   ← Tuning knobs
│
├── Sensor Fusion (EKF)
│   ├── config/robot_localization.yaml       ← IMU + DVL + Depth
│   ├── config/robot_localization_imu_dvl.yaml
│   ├── config/robot_localization_imu_only.yaml
│   └── config/robot_localization_dvl_only.yaml
│
├── Hardware Drivers
│   ├── ../sub_hardware/scripts/sensor_node.py    ← Depth sensor
│   ├── ../sub_hardware/scripts/dvl_node.py       ← Velocity sensor
│   ├── ../sub_hardware/scripts/power_node.py     ← Battery monitor
│   ├── ../sub_hardware/scripts/gpio_node.py      ← Kill switch
│   └── ../sub_hardware/config/params.yaml        ← Hardware config
│
├── Testing Tools
│   ├── scripts/lqr_tuning.py                ← Real-time debug
│   ├── tools/mock_odom.py                   ← Simulator
│   ├── tools/control_action_client.py       ← Action test client
│   └── tools/publish_body_position_cmd.sh   ← CLI waypoint sender
│
└── Launch Files
    └── launch/standalone.launch.yaml        ← Quick start
```

---

## System Block Diagram

```
High-Level Mission (FlexBE)
        ↓
        ↓ [goal_pose]
        ↓
┌─────────────────────────────────────────────┐
│ CONTROL NODE (control_node.py) @ 25 Hz      │
│ ┌────────────────────────────────────────┐  │
│ │  State:                                │  │
│ │  x = [position, angle, velocity]  12  │  │
│ │  target = [target_pos, target_angle]  │  │
│ │  error = target - x                   │  │
│ └────────────────────────────────────────┘  │
│ ┌────────────────────────────────────────┐  │
│ │ LQR SOLVER (lqr_solver.py)             │  │
│ │ 1. Compute A matrix (state-dependent) │  │
│ │ 2. Solve SDRE: Riccati equation       │  │
│ │ 3. Compute K = optimal gain           │  │
│ │ 4. u = -K * error                    │  │
│ │ Output: 8 thruster forces (Newtons)  │  │
│ └────────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
        ↓ [thruster_cmd: 8 Newtons]
        ↓
┌─────────────────────────────────────────────┐
│ ACTUATOR NODE (actuator_node.py)            │
│ ┌────────────────────────────────────────┐  │
│ │ Thrust Mapping: Newton → PWM           │  │
│ │ Uses empirical T200 lookup table       │  │
│ │ Handles non-linear thrust curves      │  │
│ └────────────────────────────────────────┘  │
│ ┌────────────────────────────────────────┐  │
│ │ I2C Bus (Thread-safe)                 │  │
│ │ PCA9685 PWM Driver                     │  │
│ └────────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
        ↓ [PWM 1100-1900 µs]
        ↓
Physical Actuators:
├── 8x T200 Thrusters
├── 1x Newton Gripper
├── 1x Lumen Light
└── 1x SER-110X Launcher

Feedback Loop (Sensors):
├─ IMU (VectorNav VN-100)        @≥100 Hz
│  └─ Roll, Pitch, Yaw
│  └─ Angular velocities
│
├─ DVL (A50)                      @≥10 Hz
│  └─ Surge, Sway, Heave velocity
│
└─ Pressure (Bar30)               @20 Hz
   └─ Depth (Z position)
    
        ↓
┌─────────────────────────────────────────────┐
│ EKF FILTER (robot_localization.yaml)        │
│ Fuses all sensors → filtered odometry      │
│ Output: 12-state estimate (pos, vel, ori)  │
└─────────────────────────────────────────────┘
        ↓ [/odometry/filtered]
        ↓
Back to Control Node (closes loop)
```

---

## Development Workflow

### Phase 1: Bench Testing (No Water)

**Goal**: Verify LQR math without physical movement

```bash
# Terminal 1: Start control node with mock odometry
ros2 launch sub_control standalone.launch.yaml

# Terminal 2: Publish fake target (10m North)
ros2 topic pub /debug/target_pose geometry_msgs/PoseStamped \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "odom"}, 
    pose: {position: {x: 10, y: 0, z: 0}, 
           orientation: {x: 0, y: 0, z: 0, w: 1}}}'

# Terminal 3: Monitor output
ros2 topic echo /thruster_cmd
# Should show: [±N1, ±N2, ±N3, ±N4, ±N5, ±N6, ±N7, ±N8]
# e.g., [-5, -5, 0, 0, 0, 0, 5, 5]  (forward surge)
```

**Debug Outputs to Check**:
```bash
ros2 topic echo /debug/lqr_angles              # Current orientation
ros2 topic echo /debug/lqr_velocity            # Current velocities
ros2 topic echo /debug/lqr_accel_cmd           # Desired accelerations
```

**Tuning Questions**:
- Are thrust magnitudes reasonable (< 40 N per thruster)?
- Is thrust distribution sensible (opposing thrusters balanced)?
- Do forces match goal direction?

If all good → **proceed to pool**

---

### Phase 2: Pool Testing (Real Odometry)

**Goal**: Verify system works with real sensors and actual movement

```bash
# Start full system (all hardware + EKF)
ros2 launch sub_control standalone.launch.yaml

# Check sensor health
ros2 topic echo /vectornav/imu                 # IMU: expect ~1000 Hz
ros2 topic echo /dvl/velocities                # DVL: expect ~10 Hz
ros2 topic echo /depth                         # Depth: expect ~20 Hz
ros2 topic echo /odometry/filtered             # EKF: expect ~25 Hz

# Verify control works
ros2 topic pub /debug/target_pose geometry_msgs/PoseStamped \
  '{header: {stamp: now, frame_id: "odom"}, 
    pose: {position: {x: 1, y: 0, z: 0}, 
           orientation: {w: 1}}}'

# Watch submarine approach target
ros2 topic echo /odometry/filtered
```

**Success Criteria**:
- ✓ Sub approaches target position smoothly
- ✓ No oscillation or overshoot
- ✓ Final error < 0.1 m (position), < 5° (heading)
- ✓ Motor power reasonable (< 500 W)

**If problems occur**:

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| No response to waypoint | EKF not fused, odometry NaN | Check sensor topics |
| Jerky, oscillating motion | Q too high, damping too low | Decrease Q, increase velocity Q |
| Sluggish, slow response | Q too low, R too high | Increase Q, decrease R |
| Drifts sideways | Thruster imbalance or offset | Tune `thruster_throttle_offset` |
| Refuses to turn | Yaw Q too low | Increase yaw Q (index 5) |

---

### Phase 3: Field Deployment

**Preparation Checklist**:

```
Hardware:
 ☐ All cables plugged in (power, serial, I2C)
 ☐ Battery fully charged (monitor with /battery topic)
 ☐ Kill switch accessible and functional
 ☐ Pressure sensor in water (for Z feedback)
 ☐ DVL has clear water below (no floor < 5m)
 ☐ IMU mounted level (pitch ≈ 0° when hovering)

Software:
 ☐ params.yaml tuned (Q/R tested in pool)
 ☐ robot_localization.yaml using correct sensors
 ☐ hardware/params.yaml matches PCB design
 ☐ thruster_throttle_offset measured (motor on, no cmd → zero thrust)
 ☐ All topics publishing without errors

Safety:
 ☐ Kill switch armed (magnetic cutoff active)
 ☐ Thrusters watchdog enabled (auto-kill after 1s no command)
 ☐ Max thrust capped at 40 N (software saturation)
 ☐ Buddy system: two qualified operators minimum
 ☐ Tether length verified, not twisted
 ☐ Air supply pressure checked (if using surface tether)
```

**Launch Command**:
```bash
ros2 launch sub_control standalone.launch.yaml
```

**Monitoring During Mission**:
```bash
# Terminal 1: Watch odometry
watch -n 0.1 'ros2 topic echo /odometry/filtered | head -20'

# Terminal 2: Monitor power
watch -n 0.1 'ros2 topic echo /battery'

# Terminal 3: Check CPU load
watch -n 0.1 'ps aux | grep -E "control|actuator|sensor"'

# Terminal 4: Listen for warnings
ros2 topic echo /rosout | grep WARN
```

**Emergency Recovery**:
```bash
# If sub becomes unresponsive:
# 1. Trigger kill switch (physical) → all thrusters stop
# 2. Or via software:
ros2 topic pub /kill_switch std_msgs/Bool '{data: true}'

# Manual recovery:
ros2 run sub_control manual_teleop.py  # Switch to gamepad control
```

---

## Parameter Reference Quick Guide

### Minimal Q Matrix Tuning

```python
Q = diag([
    q_x,      # 1000-5000   → Position N response
    q_y,      # 1000-5000   → Position E response
    q_z,      # 1000-5000   → Position D (depth) response
    q_r,      # 5000-10000  → Roll snap speed
    q_p,      # 5000-10000  → Pitch snap speed
    q_y,      # 5000-20000  → Yaw snap speed (often highest)
    q_vu,     # 100-500     → Surge velocity damping (braking)
    q_vv,     # 100-500     → Sway velocity damping
    q_vw,     # 100-500     → Heave velocity damping
    q_vp,     # 10-100      → Roll rate damping
    q_vq,     # 10-100      → Pitch rate damping
    q_vr      # 10-100      → Yaw rate damping
])
```

**Fast Tuning**:
1. **Conservative** (start here): Q all 1000, R all 30
2. **Aggressive** (target): Q all 5000, R all 15
3. **Smooth** (if oscillating): Q velocity ×5, R ×2

### Minimal Hardware Config Changes

```yaml
# If sub drifts while stationary:
thruster_throttle_offset: ±0.05  # Adjust in ±0.01 steps

# If motors weak in cold water:
max_thruster_force_newton: 50.0  # Increase from 40 if headroom exists

# If DVL gets confused:
dvl_node:
  ros__parameters:
    serial_port_name: "/dev/ttyTHS0"  # Verify port name
```

---

## Monitoring Dashboard (RViz)

**Recommended RViz setup for field testing**:

```
Left Panel (3D View):
├── TF: odom → base_link (should stay level)
├── Odometry: /odometry/filtered (particle cloud shows uncertainty)
└── Marker: /nav_goal (target position indicator)

Right Panel (2D Plots):
├── /odometry/filtered → position.x (should asymptote to target)
├── /odometry/filtered → position.y
├── /odometry/filtered → position.z
├── /thruster_cmd → effort (all 8 thrusters)
└── /battery → voltage (should stay > 10V)
```

---

## Common Issues & Solutions

### Issue 1: "NaN in state vector"
**Cause**: Waiting for first sensor message
**Solution**: 
```bash
# Check that sensors are publishing:
ros2 topic hz /odometry/filtered  # Should show 25 Hz
```

### Issue 2: "Thruster_cmd all zeros"
**Cause**: Control mode not set or target state invalid
**Solution**:
```bash
ros2 param set /control_node control_mode behavior
# Or for tuning mode:
ros2 param set /control_node control_mode lqr_tuning
```

### Issue 3: "High oscillation, jerky movements"
**Cause**: Velocity damping too low
**Solution**: In params.yaml, multiply indices 6-11 by 5:
```yaml
state_cost_matrix: [5000, 5000, 5000, 10000, 10000, 10000,
                    1000, 1000, 1000, 100, 100, 100]  # ← 5× velocity Q
```

### Issue 4: "Slow response, misses waypoints"
**Cause**: Position spring too weak
**Solution**: In params.yaml, increase indices 0-5:
```yaml
state_cost_matrix: [8000, 8000, 8000, 15000, 15000, 15000,
                    500, 500, 500, 50, 50, 50]  # ← Higher Q
```

### Issue 5: "Sub won't hold heading"
**Cause**: Yaw Q too low
**Solution**: Increase index 5 specifically:
```yaml
state_cost_matrix: [5000, 5000, 5000, 10000, 10000, 25000,  # ← yaw=25000
                    500, 500, 500, 50, 50, 50]
```

### Issue 6: "Power consumption too high"
**Cause**: R matrix too low (overly aggressive thrust)
**Solution**: Increase R values:
```yaml
thruster_cost_matrix: [30, 30, 30, 30, 30, 30, 30, 30]  # ← from 15 to 30
```

---

## File Editing Locations

**All critical files are here**:
```
c:\Programmation\ASUQTR\ros2_workspace\
├── src\sub_control\
│   ├── config\params.yaml                    ← MAIN TUNING FILE
│   ├── config\robot_localization.yaml        ← EKF FUSION CONFIG
│   ├── scripts\control_node.py               ← LQR LOOP (read-only)
│   ├── sub_control\lqr_solver.py            ← LQR MATH (read-only)
│   └── scripts\actuator_node.py              ← ACTUATOR (read-only)
│
└── src\sub_hardware\config\params.yaml       ← HARDWARE CONFIG (rarely change)
```

**Never edit without backup**:
```bash
# Before major tuning session:
cp params.yaml params.yaml.backup.$(date +%Y%m%d_%H%M%S)

# Quickly revert if something breaks:
cp params.yaml.backup.20250505_120000 params.yaml
```

---

## Summary: Key Files to Know

| File | Edit? | Frequency | Impact |
|------|-------|-----------|--------|
| `params.yaml` | YES | Every tuning | **HIGH** - LQR gains |
| `robot_localization.yaml` | MAYBE | If sensors noisy | **MEDIUM** - State estimate quality |
| `hardware/params.yaml` | RARELY | Only if HW changes | **MEDIUM** - Thruster calibration |
| `control_node.py` | NO | Never during tuning | - Code, not parameters |
| `lqr_solver.py` | NO | Never during tuning | - Math engine, physics |
| `actuator_node.py` | NO | Never during tuning | - Hardware interface |

