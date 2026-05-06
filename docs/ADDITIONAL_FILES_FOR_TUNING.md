# Additional Configuration and Utility Files for Control Tuning

## Overview

This document lists additional files beyond the core control system that are essential for:
- Testing and tuning the LQR controller
- Configuring hardware sensors
- Debugging the control loop
- Running the system in different modes

---

## 1. LQR Tuning Helper Node

### File: `scripts/lqr_tuning.py`

**Purpose**: Standalone node for real-time LQR parameter testing without high-level autonomy.

**Key Features**:
- Subscribes to both filtered odometry AND raw IMU data
- Publishes Euler angles extracted from both sources separately
- Allows comparison between EKF-fused state and raw IMU
- Useful for debugging sensor fusion errors

**Subscriptions**:
```
odometry/filtered (nav_msgs/Odometry)  → EKF-fused state
vectornav/imu (sensor_msgs/Imu)        → Raw IMU data
```

**Publications**:
```
filtered_ned_euler (geometry_msgs/Quaternion)   → [roll, pitch, yaw] from EKF
imu_ned_euler (geometry_msgs/Quaternion)        → [roll, pitch, yaw] from raw IMU
ned_euler_error (geometry_msgs/Quaternion)      → Difference between both sources
```

**Use Case**: 
- Tune IMU-to-base-link TF2 transform alignment
- Verify that EKF and IMU agree on orientation
- Test LQR with RViz interactive markers before full deployment

**Coordinate Transformations Handled**:
```python
# ROS uses ENU/FLU (East-North-Up / Forward-Left-Up)
# LQR math uses NED/FRD (North-East-Down / Forward-Right-Down)

M_ned_enu = [[0, 1, 0],      # Transform rotation matrix
             [1, 0, 0],
             [0, 0, -1]]

M_frd_flu = [[1,  0,  0],    # Transform body frame
             [0, -1,  0],
             [0,  0, -1]]
```

---

## 2. Launch File

### File: `launch/standalone.launch.yaml`

**Purpose**: ROS 2 launch configuration to start control node with mock odometry for bench testing.

**Contents**:
```yaml
launch:
  # Mock TF2 frame tree (odom → base_link)
  - node:
      pkg: "tf2_ros"
      exec: "static_transform_publisher"
      args: "--x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 
             --frame-id odom --child-frame-id base_link"

  # Control node with params
  - node:
      pkg: "sub_control"
      exec: "control_node.py"
      param:
        - from: "$(find-pkg-share sub_control)/config/params.yaml"
```

**Usage**:
```bash
ros2 launch sub_control standalone.launch.yaml
```

**Why Use This**:
- Start control node without full ROS 2 stack
- Test LQR math without waiting for all hardware to boot
- Safe development environment (no physical thrusters armed)

---

## 3. Hardware Sensor Nodes

### File: `../sub_hardware/scripts/sensor_node.py`

**Purpose**: Publishes depth (Z position) from Bar30 pressure sensor.

**Subscriptions**: None (I2C sensor input only)

**Publications**:
```
depth (nav_msgs/Odometry)  → Z position + covariance
```

**Key Parameters**:
```python
SENSOR_I2C_BUS = 1
PUBLISH_RATE_HZ = 20.0
OVERSAMPLING_RATIO = OSR_4096
FLUID_DENSITY = DENSITY_FRESHWATER  # or DENSITY_SALTWATER
```

**Why Relevant for Control**:
- Provides Z position feedback (vertical position)
- Feeds into EKF sensor fusion for depth control
- Covariance tuning affects how much EKF trusts pressure vs. model

---

### File: `../sub_hardware/scripts/power_node.py`

**Purpose**: Monitors battery voltage and current draw from Arduino power board.

**Serial Interface**:
```
DRIVER_PORT = '/dev/ttyACM0'   # Main power board
HELPER_PORT = '/dev/ttyACM1'   # Secondary board
```

**Publishes**:
```
/battery (sensor_msgs/BatteryState)  → Voltage, current, health
/leak (std_msgs/Bool)                 → Water intrusion detection
```

**Why Relevant for Control**:
- T200 thrusters change performance with battery voltage
- Low battery = reduced max thrust (may need LQR param adjustment)
- Leak detection = emergency shutdown signal

---

## 4. Hardware Configuration

### File: `../sub_hardware/config/params.yaml`

**Complete Hardware Configuration**:

```yaml
actuator_node:
  ros__parameters:
    pca_ref_clk_speed: 24821760              # PCA9685 clock (tuned empirically)
    thruster_throttle_offset: 0.0            # Neutral point drift correction [-0.15, 0.15]
    enable_thrusters_watchdog: true          # Safety: kill thrusters if no cmd for 1s

dvl_node:
  ros__parameters:
    serial_port_name: "/dev/ttyTHS0"         # Jetson Xavier DVL serial port

gpio_node:
  ros__parameters:
    kill_switch_pin: 36                      # Magnetic killswitch (safety)
    magnetic_switch2_pin: 38
    magnetic_switch3_pin: 40
    signal_leak_pin: 7
    alert_temp_pin: 15
    main_pod_fans_pin: 16
    disable_pwms_pin: 11
    kill_switch_debounce_time: 200           # Debounce time (ms)

sonar_node:
  ros__parameters:
    speed_of_sound: 1481                     # Freshwater (1532 in saltwater)
    ping_range: 50                           # Max detection range (meters)
```

**Critical Parameters for Control**:
- `thruster_throttle_offset`: If sub drifts when stationary, adjust this ±0.05
- `enable_thrusters_watchdog`: **ALWAYS true** unless bench testing with power disconnected
- `speed_of_sound`: Affects sonar distance measurements (if used for obstacle avoidance)

---

## 5. Alternative EKF Configurations

### Files: `config/robot_localization_*.yaml`

ASUQTR has **4 different EKF sensor fusion strategies**:

#### `robot_localization.yaml` (Default - IMU + DVL + Depth)
- Uses all 3 sensors (most reliable)
- IMU provides heading
- DVL provides velocity
- Pressure provides depth
- **Recommended for field deployment**

#### `robot_localization_imu_dvl.yaml` (IMU + DVL only)
- Alternative configuration with both IMU and DVL
- Good backup if depth sensor fails
- Z position will drift over time (no depth anchor)

#### `robot_localization_imu_only.yaml` (IMU only - Fallback)
- Only orientation + angular velocity from IMU
- **Position drifts rapidly** (risky for position control)
- Use only in emergency mode
- Consider switching to manual teleop

#### `robot_localization_dvl_only.yaml` (DVL only - Fallback)
- Only velocity from DVL
- **Loses heading information**
- Not recommended for LQR heading control
- Position still drifts (double integration of velocity)

**When to Switch**:
```bash
# Change active EKF config
ros2 param load /ekf_filter_node $(find-pkg-share sub_control)/config/robot_localization_imu_only.yaml
```

**Covariance Tuning**: In each config file, adjust process noise and measurement noise:

```yaml
process_noise_covariance: [
  0.05, 0.0, 0.0, ...      # Position noise (higher = EKF trusts model less)
  0.03, 0.0, 0.0, ...      # Orientation noise
  0.025, 0.0, 0.0, ...     # Velocity noise
  0.01, 0.0, 0.0, ...      # Angular velocity noise
]

initial_estimate_covariance: [
  1e-9, 0.0, 0.0, ...      # Startup confidence
]
```

---

## 6. Testing & Debugging Tools

### File: `tools/mock_odom.py`

**Purpose**: Generates fake odometry for testing control node without hardware.

**Features**:
- Publishes static odometry at 10 Hz
- Simulates submarine at depth 2m, position (8, 4, -2)
- Zero velocity (stationary)

**Usage for Testing**:
```bash
# Terminal 1: Start control node
ros2 launch sub_control standalone.launch.yaml

# Terminal 2: Start mock odometry
ros2 run sub_control mock_odom

# Terminal 3: Publish target waypoint
ros2 topic pub /debug/target_pose geometry_msgs/PoseStamped \
  "{header: {stamp: now, frame_id: 'odom'}, 
    pose: {position: {x: 10, y: 4, z: -5}, 
           orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

**What to Watch**:
- Does control node publish thruster commands?
- Do commands make sense (magnitude, direction)?
- Check `/debug/lqr_*` topics for intermediate values

---

### File: `tools/control_action_client.py`

**Purpose**: Simple client to test Action Server interface for autonomous navigation.

**Old ROS1 Code** - Needs porting to ROS 2 but shows the pattern:

```python
client = ControlActionClient(max_goal_reach_time=5.0)
client.send_pos_increment(ned_ref=True, distance=1.0)  # Move 1m North

# Blocks until goal reached or timeout
while not client.check_goal_reached():
    time.sleep(0.1)
```

**To Port to ROS 2**:
```python
# Replace actionlib with rclpy.action
from rclpy.action import ActionClient
from sub_interfaces.action import Control

self.client = ActionClient(self, Control, 'navigate_sub')
self.client.wait_for_server()
goal_msg = Control.Goal()
goal_msg.target_pose = ...
```

---

### File: `tools/publish_body_position_cmd.sh`

**Purpose**: Shell script to publish debug target pose from command line.

**Usage**:
```bash
# Move to waypoint (x=10, y=5, z=-3 in NED)
./publish_body_position_cmd.sh 10 5 -3 0 0 0 0 1

# Arguments: north east down roll pitch yaw qx qy qz qw
```

---

## 7. Configuration Files Not Yet Covered

### File: `config/Jean_tuned.yaml`

**Status**: Currently empty

**Purpose**: Reserved for previously tuned Q/R parameters from successful field test (named after team member Jean)

**To Use**:
```bash
# Copy working params from params.yaml into Jean_tuned.yaml
# Then reference it in launch file:
ros2 param load /control_node $(find-pkg-share sub_control)/config/Jean_tuned.yaml
```

---

## Complete Testing Workflow

### 1. Bench Test (No Water)
```bash
# Terminal 1
ros2 launch sub_control standalone.launch.yaml

# Terminal 2
ros2 run sub_control mock_odom

# Terminal 3 - Publish target
ros2 topic pub /debug/target_pose geometry_msgs/PoseStamped \
  "{header: {stamp: now, frame_id: 'odom'}, 
    pose: {position: {x: 10, y: 0, z: 0}, 
           orientation: {w: 1}}}"

# Terminal 4 - Monitor
ros2 topic echo /thruster_cmd  # Should see Newtons being commanded
ros2 topic echo /debug/lqr_angles
ros2 topic echo /debug/lqr_velocity
```

### 2. Pool Test (With Real Odometry)
```bash
# Start full ROS 2 stack (hardware + EKF)
ros2 launch sub_control standalone.launch.yaml

# Control node will listen to REAL odometry instead of mock
# Send waypoint via RViz or tool script
```

### 3. Sensor Fusion Verification
```bash
# Monitor EKF filter health
ros2 topic echo /odometry/filtered

# Check individual sensor contributions
ros2 topic echo /vectornav/imu          # Raw IMU
ros2 topic echo /dvl/velocities         # DVL
ros2 topic echo /depth                  # Pressure

# Compare filtered vs. raw with lqr_tuning node
ros2 run sub_control lqr_tuning
ros2 topic echo /filtered_ned_euler     # From EKF
ros2 topic echo /imu_ned_euler          # From IMU only
ros2 topic echo /ned_euler_error        # Difference
```

---

## Parameter Tuning Checklist

### Before Deployment

- [ ] `params.yaml`: Q and R matrices tuned in pool
- [ ] `robot_localization.yaml`: EKF covariances match sensor noise
- [ ] `hardware/params.yaml`: 
  - [ ] DVL serial port correct (`/dev/ttyTHS0`)
  - [ ] `thruster_throttle_offset` = 0 (measured with power on, no command)
  - [ ] `pca_ref_clk_speed` = 24821760 (tuned with `tune_pca.py`)
  - [ ] `enable_thrusters_watchdog` = true
- [ ] Kill switch, leak detector GPIO pins match PCB schematic
- [ ] Speed of sound for sonar = 1481 m/s (freshwater)

### In-Water Tuning

1. Deploy with conservative gains (low Q, high R)
2. Issue small 1m waypoints
3. Monitor `/debug/lqr_*` topics for oscillation
4. If sluggish: increase Q, decrease R
5. If overshooting: increase velocity Q, increase R
6. Save final tuning → `Jean_tuned.yaml`

---

## Summary Table

| File | Purpose | When to Modify |
|------|---------|----------------|
| `params.yaml` | Main LQR control gains | Every tuning iteration |
| `hardware/params.yaml` | Hardware I2C config | Never (unless HW changes) |
| `robot_localization.yaml` | EKF sensor fusion | If sensors noisy |
| `lqr_tuning.py` | Debug helper node | Not needed normally |
| `mock_odom.py` | Bench test without HW | For development only |
| `standalone.launch.yaml` | Quick startup | To change startup params |

