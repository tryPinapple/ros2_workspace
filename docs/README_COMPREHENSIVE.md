# ASUQTR Submarine ROS 2 Control System - Comprehensive Documentation

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Control Node](#control-node)
4. [Actuator Node](#actuator-node)
5. [LQR Solver Module](#lqr-solver-module)
6. [Configuration Files](#configuration-files)
7. [ROS 2 Interface](#ros-2-interface)
8. [Tuning Guide](#tuning-guide)

---

## Overview

This documentation covers the **core control system** for the ASUQTR autonomous submarine. The system implements a **State-Dependent Riccati Equation (SDRE)** based Linear Quadratic Regulator (LQR) controller, providing precise 6-DOF motion control from high-level mission goals to low-level thruster PWM signals.

### System Architecture

```
┌─────────────────────────────────────────────────┐
│  High-Level Behaviors (FlexBE State Machine)   │
├─────────────────────────────────────────────────┤
│  Control Node (control_node.py)                 │
│  ├─ Manages control modes (Behavior/Manual)    │
│  ├─ Runs LQR solver at 25-50 Hz                │
│  └─ Produces thrust commands in Newtons        │
├─────────────────────────────────────────────────┤
│  Actuator Node (actuator_node.py)              │
│  ├─ Converts Newtons → PWM signals             │
│  ├─ Manages I2C bus to PCA9685 PWM driver      │
│  └─ Controls 8 thrusters + gripper + lights    │
├─────────────────────────────────────────────────┤
│  LQR Solver (lqr_solver.py)                    │
│  ├─ Computes SDRE gain matrix K                │
│  ├─ Converts state errors to force commands    │
│  └─ Mathematically proven output is in Newtons │
└─────────────────────────────────────────────────┘
```

---

## Architecture

### Control Flow Diagram

1. **Odometry Input**: Filtered localization data arrives at 25 Hz
2. **State Extraction**: Current position, velocity, orientation extracted
3. **Error Computation**: Compare current vs. target state
4. **LQR Calculation**: SDRE solver computes optimal gain K
5. **Force Command**: K is applied to error vector → force output
6. **Software Clipping**: Forces clipped to ±40 N physical limits
7. **Thruster Allocation**: Forces distributed to 8 thrusters
8. **PWM Generation**: Newtons converted to PWM via lookup table
9. **I2C Transmission**: PWM sent to PCA9685 driver

### Key Design Decisions

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| **Callback Groups** | MutuallyExclusive for each subsystem | Prevents I2C/actuator bus collisions |
| **State Allocation** | Pre-allocated 12-element numpy arrays | Avoids garbage collection stutters |
| **Angle Wrapping** | Wrapped to [-π, π] before LQR | Ensures shortest rotational path |
| **Output Units** | Newtons (proven mathematically) | Direct correspondence to physics |

---

## Control Node

### File: `scripts/control_node.py`

#### Purpose
Acts as the **central nervous system** for submarine motion control. It:
- Bridges high-level behavioral commands with low-level physics solver
- Manages three operational modes: Behavior, LQR Tuning, Manual
- Executes SDRE LQR control loop at 25 Hz minimum

#### Class: `ControlNode`

```python
class ControlNode(Node):
    """Main execution class for AUV Control."""
```

##### Parameters (Dynamic Reconfigurable)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `control_mode` | string | "behavior" | Operational mode: "behavior", "lqr_tuning", "manual" |
| `state_cost_matrix` | float[12] | [1052.48, ...] | Q matrix diagonal - state error penalties |
| `thruster_cost_matrix` | float[8] | [15.0, ...] | R matrix diagonal - thruster energy penalties |
| `max_thruster_force_newton` | float | 40.0 | Hard saturation limit per thruster (Newtons) |
| `publish_lqr_debug_angles` | bool | true | Publish [roll, pitch, yaw] from odometry |
| `publish_lqr_dynamics_debug` | bool | true | Publish A matrix state-dependent values |
| `debug_invert_roll` | bool | false | Flip sign on roll command (tuning) |
| `debug_invert_pitch` | bool | false | Flip sign on pitch command (tuning) |
| `debug_invert_yaw` | bool | false | Flip sign on yaw command (tuning) |
| `damping_sign` | float | 1.0 | ±1.0 to flip sign of all damping terms |

##### Subscriptions

| Topic | Message Type | Frequency | Purpose |
|-------|--------------|-----------|---------|
| `odometry/filtered` | `nav_msgs/Odometry` | 25 Hz | Current submarine state (position, velocity, orientation) |
| `debug/target_pose` | `geometry_msgs/PoseStamped` | Ad-hoc | (LQR_TUNING mode) Manual waypoint target |
| `dashboard/gamepad` | `sensor_msgs/Joy` | ~50 Hz | (MANUAL mode) Joystick commands |

##### Publications

| Topic | Message Type | Frequency | Purpose |
|-------|--------------|-----------|---------|
| `thruster_cmd` | `sub_interfaces/ThrusterCommand` | ~25 Hz | Thrust force in Newtons for each of 8 thrusters |
| `debug/lqr_angles` | `std_msgs/Float64MultiArray` | ~25 Hz | [roll, pitch, yaw] extracted from odometry |
| `debug/lqr_velocity` | `std_msgs/Float64MultiArray` | ~25 Hz | [u, v, w, p, q, r] body velocities |
| `debug/lqr_accel_cmd` | `std_msgs/Float64MultiArray` | ~25 Hz | [ax, ay, az, ap, aq, ar] commanded accelerations |
| `debug/lqr_dynamics` | `std_msgs/Float64MultiArray` | ~25 Hz | A matrix dynamic components for inspection |

#### Control Modes

##### Mode 1: BEHAVIOR (Autonomous)
- Listens to Action Server for high-level goals (e.g., "Go to waypoint X")
- Executes automatic LQR guidance to goal
- Used with FlexBE state machines for mission autonomy

##### Mode 2: LQR_TUNING (Development)
- Listens to `debug/target_pose` topic from RViz interactive markers
- Allows real-time waypoint testing without modifying code
- Useful for tuning Q and R matrices during development

##### Mode 3: MANUAL (Teleop)
- Listens to gamepad joystick commands
- Implements direct thruster mapping for manual control
- Safety dead-zone to eliminate stick drift

#### State Representation

The 12-element state vector $\mathbf{x}$ represents:

```
x = [position_N, position_E, position_D, 
     roll, pitch, yaw,
     velocity_u, velocity_v, velocity_w, 
     angular_p, angular_q, angular_r]

Index:  0          1           2
        3          4     5
        6          7           8
        9          10         11
```

**Frame Convention**: North-East-Down (NED) world frame, Forward-Right-Down (FRD) body frame
(ROS ENU/FLU coordinates are internally translated)

#### Key Methods

##### `localization_callback(msg: Odometry)`
Executes every time odometry is received. **This is the hot control loop.**

1. Extracts current state from odometry message
2. Converts ROS ENU coordinates to NED
3. Computes state error: $e = x_{target} - x_{current}$
4. Calls LQR solver to compute optimal thrust commands
5. Publishes thruster commands and debug information

**Execution Time**: ~2-5 ms on typical hardware

##### `parameter_callback(params: List[Parameter])`
Handles dynamic parameter updates without node restart.

Example: Reconfigure LQR gains while system is running:
```bash
ros2 param set /control_node state_cost_matrix "[1000.0, 1000.0, 1000.0, ...]"
```

##### `wrap_angles_to_pi(angles: np.ndarray)`
Ensures angular errors are computed on the shortest path.

Example:
- Input: 350° (yaw error)
- Output: -10° (equivalent, but shorter rotation)

---

## Actuator Node

### File: `scripts/actuator_node.py`

#### Purpose
The **hardware interface layer** between ROS 2 software and physical actuators. Converts abstract force commands (Newtons) into concrete PWM signals sent over I2C to a PCA9685 PWM driver chip.

#### Hardware Components Managed

| Component | Type | Count | Interface |
|-----------|------|-------|-----------|
| **T200 Thruster** | ContinuousServo | 8 | PCA9685 PWM, 1100-1900 µs |
| **Newton Subsea Gripper** | ContinuousServo | 1 | PCA9685 PWM, 1100-1900 µs |
| **Lumen Subsea Light** | ContinuousServo | 1 | PCA9685 PWM, 1100-1900 µs |
| **SER-110X Torpedo Launcher** | Servo (Positional) | 1 | PCA9685 PWM, 850-2350 µs |
| **Kill Switch** | Digital GPIO | 1 | Magnetic reed switch |

#### Class: `ActuatorNode`

```python
class ActuatorNode(Node):
    """ROS 2 Node managing I2C physical actuators on the Submarine."""
```

##### Hardware Classes

###### `T200Thruster`
- **Purpose**: Blue Robotics T200 underwater thruster
- **Range**: -40.22 N (reverse) to +51.50 N (forward) @ 16V
- **API**: ContinuousServo throttle from -1.0 to +1.0
- **Deadband**: -0.0625 to +0.0625 (no-thrust zone)
- **Mapping**: Empirical lookup table with numpy interpolation

```python
known_forces_n = np.array([
    -40.22, -27.47, -14.71, -5.89, 0.0, 0.0, 0.0, 
    7.85, 19.62, 34.33, 51.50
])
known_api_cmds = np.array([
    -1.0, -0.75, -0.50, -0.25, -0.0625, 0.0, 0.0625,
    0.25, 0.50, 0.75, 1.0
])
```

**Thrust Mapping Algorithm**: Uses `np.interp()` to smoothly map requested force in Newtons to thruster API command (-1.0 to 1.0)

###### `NewtonSubseaGripper`
- **Purpose**: Subsea manipulator arm gripper
- **API**: ContinuousServo (-1.0 = close, 0.0 = stop, 1.0 = open)
- **PWM Range**: 1100-1900 µs

###### `LumenSubseaLight`
- **Purpose**: Brightness-controllable subsea light
- **API**: ContinuousServo (0.0 = off, 1.0 = full brightness)
- **PWM Range**: 1100-1900 µs

###### `SER110XLauncher`
- **Purpose**: Torpedo launcher servo (positional, not continuous)
- **API**: Servo angle from 0° to 180°
- **PWM Range**: 850-2350 µs

##### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pca_ref_clk_speed` | int | 24821760 | Reference clock for PCA9685 (Hz) |
| `thruster_throttle_offset` | float | 0.0 | Neutral point correction ±0.15 |
| `use_flat_thruster_mapping` | bool | false | Use simplified gain-based mapping vs. lookup table |
| `flat_thruster_gain_primary` | float | 0.2 | Gain for horizontal thrusters (flat mode) |
| `flat_thruster_gain_secondary` | float | 0.1 | Gain for vertical thrusters (flat mode) |
| `enable_thrusters_watchdog` | bool | true | Auto-kill thrusters if no command for 1 sec |

##### Subscriptions

| Topic | Message Type | QoS | Purpose |
|-------|--------------|-----|---------|
| `thruster_cmd` | `sub_interfaces/ThrusterCommand` | BEST_EFFORT | 8 × thruster force in Newtons |
| `gripper` | `std_msgs/Float32` | BEST_EFFORT | Gripper throttle [-1.0, 1.0] |
| `subsea_light` | `std_msgs/Float32` | BEST_EFFORT | Light brightness [0, 180] |
| `torpedo` | `std_msgs/Float32` | BEST_EFFORT | Torpedo position [0, 180] degrees |
| `kill_switch` | `sensor_msgs/Bool` | RELIABLE | Magnetic kill switch state |

#### Callback Functions

##### `thrusters_callback(msg: ThrusterCommand)`
Receives commanded forces in Newtons for all 8 thrusters.

1. **Input**: ThrusterCommand message with 8 float32 values
2. **Validation**: Checks [-50, 50] N valid range
3. **Mapping**: Converts each force to thruster API throttle using `np.interp()`
4. **Offset**: Applies `thruster_throttle_offset` to correct neutral drift
5. **I2C Write**: Sets all 8 PCA9685 channels simultaneously

**Thread Safety**: Executed within MutuallyExclusiveCallbackGroup to prevent concurrent I2C access

##### `gripper_callback(msg: Float32)`
Sets gripper servo throttle directly [-1.0, 1.0].

##### `torpedo_callback(msg: Float32)`
Sets torpedo launcher servo angle [0°, 180°].

##### `subsea_light_callback(msg: Float32)`
Sets light brightness [0, 180].

##### `kill_switch_callback(msg: Bool)`
Monitors magnetic kill switch. When activated, all thrusters are disabled for safety.

##### `thrusters_watchdog_callback()`
Executes every 50 ms. If no thruster command received for 1 second, kills all thrusters to prevent uncontrolled drift.

#### Non-Linear Thrust Mapping

The T200 thruster exhibits non-linear behavior: thrust output is not proportional to PWM input.

**Solution**: Empirical lookup table with interpolation.

```python
# Real-world T200 performance @ 16V
thrust_requested = 25.0  # Newtons

# numpy.interp finds the throttle that produces 25N
throttle = np.interp(thrust_requested, known_forces_n, known_api_cmds)
# Result: throttle ≈ 0.62

thruster.throttle = throttle  # Sets PWM to ~1750 µs
```

**Advantages**:
- Physically accurate behavior
- Smooth response across thrust range
- Automatically handles deadband

**Limitations**:
- Requires thruster characterization (empirical data collection)
- Assumes constant voltage (16V in this case)
- Changes with battery voltage and thruster wear

---

## LQR Solver Module

### File: `sub_control/lqr_solver.py`

#### Purpose
Implements the **State-Dependent Riccati Equation (SDRE)** solver, which continuously re-solves the optimal LQR gain matrix during runtime to adapt to changing hydrodynamic drag and Coriolis forces.

#### Key Concept: SDRE vs. Traditional LQR

| Aspect | Traditional LQR | SDRE (State-Dependent) |
|--------|-----------------|----------------------|
| **System Model** | Linear, time-invariant | Non-linear, state-dependent |
| **A Matrix** | Fixed, constant | Changes based on current velocity/orientation |
| **Gain K** | Computed once offline | Recomputed ~25 Hz at runtime |
| **Adaptation** | None | Automatically adapts to speed and drag |

#### Matrices

##### Thrust Allocation Matrix (T)

Defines how 8 individual thruster forces combine into 6-DOF wrench (force + torque):

```python
THRUST_ALLOC_MAT = np.array([
    [-1,  1,  0,  0,  0,  1],  # Thruster 0: horizontal + yaw
    [-1, -1,  0,  0,  0, -1],  # Thruster 1: horizontal + yaw
    [ 0,  0, -1, -1, -1,  0],  # Thruster 2: vertical + roll + pitch
    [ 0,  0, -1,  1, -1,  0],  # Thruster 3: vertical + roll + pitch
    [ 0,  0, -1, -1,  1,  0],  # Thruster 4: vertical + roll + pitch
    [ 0,  0, -1,  1,  1,  0],  # Thruster 5: vertical + roll + pitch
    [ 1,  1,  0,  0,  0, -1],  # Thruster 6: horizontal + yaw (opposite)
    [ 1, -1,  0,  0,  0,  1],  # Thruster 7: horizontal + yaw (opposite)
], dtype=np.int8)
```

**Interpretation**: Each row represents one thruster's effect. Columns = [Fx, Fy, Fz, τx, τy, τz]

##### Control Input Matrix (B Matrix)

Converts thruster forces into accelerations via $\dot{\mathbf{v}} = \mathbf{B} \cdot \mathbf{u}$

```python
Bm = np.zeros((12, 8), dtype=np.float32)

# Surge (u_dot) - forward acceleration
Bm[6][0] = -0.1017  # Thruster 0 contribution
Bm[6][1] = -0.1017  # Thruster 1 contribution
Bm[6][6] = +0.1017  # Thruster 6 contribution (opposite)
Bm[6][7] = +0.1017  # Thruster 7 contribution (opposite)

# Sway (v_dot) - lateral acceleration
Bm[7][0] = +0.1017
Bm[7][1] = -0.1017
Bm[7][6] = +0.1017
Bm[7][7] = -0.1017

# Heave (w_dot) - vertical acceleration
Bm[8][2:6] = -0.1439  # All vertical thrusters

# Roll (p_dot) - roll acceleration
Bm[9][2] = -1.6213
Bm[9][3] = +1.6213
Bm[9][4] = -1.6213
Bm[9][5] = +1.6213

# Pitch (q_dot) - pitch acceleration
Bm[10][2:6] = [-0.572, -0.572, +0.572, +0.572]

# Yaw (r_dot) - yaw acceleration
Bm[11][0] = +1.3277
Bm[11][1] = -1.3277
Bm[11][6] = -1.3277
Bm[11][7] = +1.3277
```

**Mathematical Interpretation**: Each entry represents $\frac{\partial \dot{v}_i}{\partial u_j}$ where $u_j$ is thruster $j$ force.

**Units Proof**: The B matrix coefficients have dimensions [1/kg]. When multiplied by force [N], output is [m/s²] acceleration. Therefore:
- Coefficient ≈ 0.1017 = 1/m
- m = 1/0.1017 ≈ 9.83 kg (submarine mass in the model)

#### State-Dependent A Matrix

The A matrix models the non-linear submarine dynamics, including:
1. **Kinematic coupling**: How body velocities (u,v,w,p,q,r) translate to world motion
2. **Hydrodynamic drag**: Quadratic velocity-dependent forces (added mass, lift, drag)
3. **Coriolis acceleration**: Due to frame rotation
4. **Restoring forces**: Buoyancy effects

**State Dependency**: A matrix is recomputed every LQR iteration using current:
- Position z (depth) → affects buoyancy
- Orientation (roll, pitch, yaw) → affects rotation matrix
- Velocities (u, v, w, p, q, r) → affects drag and Coriolis

#### Class: `SubLQRSolver`

```python
class SubLQRSolver:
    """
    Executes a State-Dependent Riccati Equation (SDRE) controller strategy.
    """
```

##### Key Methods

###### `update_system_dynamics_matrix_A(Am: np.ndarray, state: np.ndarray) -> np.ndarray`

Computes the 12×12 state-dependent A matrix.

**Inputs**:
- `Am`: Pre-allocated 12×12 array
- `state`: Current 12-element state vector

**Outputs**:
- Populated A matrix reflecting current dynamics

**Mathematical Basis**:
- Rows 0-5: Kinematic coupling (rotation matrices)
- Rows 6-11: Kinetic equations (Newton's second law + hydrodynamic drag)

**Computation**: ~50 sin/cos evaluations + 140 multiplications per call

**Performance**: ~0.1 ms on typical hardware

###### `compute_thrust_force(current_state: np.ndarray, target_state: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray`

Solves the SDRE optimal control problem and returns commanded thruster forces.

**Algorithm**:
1. Compute state-dependent A matrix: $\mathbf{A} = f(\mathbf{x})$
2. Compute error: $\mathbf{e} = \mathbf{x}_{target} - \mathbf{x}_{current}$
3. Solve Riccati equation: Find gain matrix **K** such that cost function is minimized
4. Compute control: $\mathbf{u} = -\mathbf{K} \cdot \mathbf{e}$
5. Apply thruster allocation: $\mathbf{f}_{thruster} = \mathbf{T}^{-1} \cdot \mathbf{u}$

**Riccati Solver**: Uses `scipy.linalg.solve_continuous_are(A, B, Q, R)`

**Output**: 8-element array of thruster forces in Newtons

#### Tuning Guide: Q and R Matrices

##### Q Matrix (State Error Cost)

Penalizes deviation from target state. Diagonal elements:

```
Q = diag([q_x, q_y, q_z, q_roll, q_pitch, q_yaw, 
          q_vu, q_vv, q_vw, q_vp, q_vq, q_vr])
```

**Interpretation**:
- **q_x, q_y, q_z** (Position): Act like **springs**. Higher = stronger pull to target. Range: 100-5000
- **q_roll, q_pitch, q_yaw** (Orientation): Act like **torsional springs**. Higher = snap heading faster. Range: 1000-10000
- **q_vu, q_vv, q_vw** (Linear velocity): Act like **dampers**. Higher = harder braking to target. Range: 10-1000
- **q_vp, q_vq, q_vr** (Angular velocity): Act like **angular dampers**. Higher = resist spinning. Range: 1-100

**Default Tuned Values**:
```python
DEFAULT_Q = [
    4*1052.48,   # q_x: Position North
    4*578.78,    # q_y: Position East
    4*670.07,    # q_z: Position Down
    4*3134.68,   # q_roll: Orientation Roll
    4*3959.95,   # q_pitch: Orientation Pitch
    20*1453.97,  # q_yaw: Orientation Yaw (20× higher for heading lock)
    4*265.11,    # q_vu: Velocity Surge damper
    4*303.53,    # q_vv: Velocity Sway damper
    4*204.29,    # q_vw: Velocity Heave damper
    4*12.32,     # q_vp: Angular velocity Roll
    4*19.81,     # q_vq: Angular velocity Pitch
    20*6.24      # q_vr: Angular velocity Yaw
]
```

##### R Matrix (Thruster Energy Cost)

Penalizes thruster usage. Diagonal elements:

```
R = diag([r_0, r_1, r_2, r_3, r_4, r_5, r_6, r_7])
```

**Interpretation**:
- Higher R = "Thrusters are expensive, use conservatively"
- Lower R = "Aggressive thrust, reach target quickly"

**Default Tuned Values**:
```python
DEFAULT_R = [15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0]
```

**Rule of Thumb**: Output force scales roughly as $\sqrt{Q/R}$

Therefore: $u \approx \sqrt{\frac{Q}{R}} \times e$

Example calculation:
- Q_x = 4210 (position penalty)
- R = 15 (thruster cost)
- $\sqrt{Q/R} = \sqrt{4210/15} = \sqrt{280.7} ≈ 16.8$
- For 1 meter position error: $u ≈ 16.8 × 1.0 = 16.8$ N per thruster

##### Tuning Procedure

**Step 1**: Start with aggressive gain (high Q, low R)
```yaml
state_cost_matrix: [5000, 5000, 5000, 10000, 10000, 10000, 500, 500, 500, 100, 100, 100]
thruster_cost_matrix: [5, 5, 5, 5, 5, 5, 5, 5]
```

Test response. Too jerky? Proceed to Step 2.

**Step 2**: Add damping (increase velocity Q)
```yaml
state_cost_matrix: [5000, 5000, 5000, 10000, 10000, 10000, 1000, 1000, 1000, 200, 200, 200]
```

Test for oscillation. Too sluggish? Proceed to Step 3.

**Step 3**: Increase thruster cost (increase R)
```yaml
thruster_cost_matrix: [20, 20, 20, 20, 20, 20, 20, 20]
```

Test for smooth, energy-efficient motion.

**Step 4**: Fine-tune heading (adjust yaw Q separately)
```yaml
state_cost_matrix: [5000, 5000, 5000, 10000, 10000, 25000, 1000, 1000, 1000, 200, 200, 200]
#                                                  ↑ yaw = 25000 for tight heading
```

---

## Configuration Files

### File: `config/params.yaml`

Main ROS 2 parameter configuration for control node.

```yaml
control_node:
  ros__parameters:
    control_mode: "lqr_tuning"
    
    state_cost_matrix: [1052.4762, 578.7808, 670.0731, 
                        3134.6757, 3959.9523, 2453.9726, 
                        265.1124, 303.5306, 204.2912, 
                        12.3207, 19.8116, 12.2442]
    
    thruster_cost_matrix: [15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0]
    
    max_thruster_force_newton: 40.0
    joy_dead_zone: 0.2
```

**Parameter Mapping**:
- Index 0-2: Position penalties (N, E, D)
- Index 3-5: Orientation penalties (roll, pitch, yaw)
- Index 6-11: Velocity damping penalties (u, v, w, p, q, r)

### File: `config/robot_localization.yaml`

Extended Kalman Filter (EKF) configuration for sensor fusion.

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 25.0
    
    # Coordinate frames
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    
    # --- Sensor 0: IMU (VectorNav VN-100) ---
    imu0: vectornav/imu
    imu0_config: [false, false, false,     # Position X, Y, Z
                  true,  true,  true,      # Orientation R, P, Y ✓
                  false, false, false,     # Linear velocity
                  true,  true,  true,      # Angular velocity ✓
                  false, false, false]     # Linear acceleration
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: false
    
    # --- Sensor 1: DVL (Doppler Velocity Log) ---
    twist0: dvl/velocities
    twist0_config: [false, false, false,   # Position
                    false, false, false,   # Orientation
                    true,  true,  true,    # Linear velocity ✓
                    false, false, false,   # Angular velocity
                    false, false, false]   # Acceleration
    twist0_differential: false
    twist0_relative: false
    twist0_queue_size: 10
    
    # --- Sensor 2: Pressure Sensor (Depth) ---
    odom0: depth
    odom0_config: [false, false, true,     # Position Z only ✓
                   false, false, false,    # Orientation
                   false, false, false,    # Linear velocity
                   false, false, false,    # Angular velocity
                   false, false, false]    # Acceleration
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10
    
    # --- Covariance Matrices ---
    dynamic_process_noise_covariance: true
    
    process_noise_covariance: [
      0.05, 0.0, 0.0, ...,  # Position X, Y, Z noise
      0.03, 0.0, 0.0, ...,  # Orientation roll, pitch
      0.06, 0.0, 0.0, ...,  # Orientation yaw
      0.025, 0.0, 0.0, ..., # Velocity X, Y, Z
      0.01, 0.0, 0.0, ...,  # Angular velocity roll, pitch
      0.02, 0.0, 0.0, ...   # Angular velocity yaw
    ]
    
    initial_estimate_covariance: [
      1e-9, 0.0, 0.0, ...,  # Position uncertainty (very confident at start)
      1e-9, 0.0, 0.0, ...,  # Orientation uncertainty
      ...
    ]
```

**Sensor Fusion Strategy**:
1. **IMU** provides heading (roll, pitch, yaw) + angular velocities
2. **DVL** provides linear velocities (surge, sway, heave)
3. **Depth sensor** provides Z position (locked to pressure)
4. **EKF** fuses all three into consistent filtered odometry

**Key Design Decision**: IMU alone is drift-prone. DVL alone loses heading. Together, they provide stable, drift-corrected 6-DOF state estimate.

### File: `config/robot_localization_imu_dvl.yaml`

Alternative configuration using both IMU and DVL (recommended).

### File: `config/robot_localization_imu_only.yaml`

Fallback configuration if DVL fails. Provides heading + angular rates only (Z drifts over time).

### File: `config/Jean_tuned.yaml`

Previously tuned parameters from testing campaign. May be outdated.

---

## ROS 2 Interface

### Custom Message: `sub_interfaces/ThrusterCommand`

```yaml
float32[8] thruster_effort  # Force in Newtons for each thruster
```

### Custom Action: `sub_interfaces/Control`

```
# Goal
geometry_msgs/PoseStamped target_pose
sensor_msgs/Joy     optional_gamepad_override

---
# Result
bool success
string message
---
# Feedback
geometry_msgs/PoseStamped current_pose
float64 distance_to_goal
float64 heading_error_rad
```

### TF2 Frame Structure

```
map (global reference)
 └── odom (local odometry frame)
      └── base_link (submarine body)
           ├── sensors
           ├── thrusters
           └── payload
```

---

## Tuning Guide

### Quick Tuning Steps

#### 1. Test Stability
Set conservative gains:
```yaml
state_cost_matrix: [1000, 1000, 1000, 5000, 5000, 5000, 100, 100, 100, 10, 10, 10]
thruster_cost_matrix: [20, 20, 20, 20, 20, 20, 20, 20]
```

Issue a 1-meter waypoint. Sub should reach it smoothly without oscillation.

#### 2. Increase Position Response
If reaching waypoint too slowly:
```yaml
state_cost_matrix: [5000, 5000, 5000, 5000, 5000, 5000, 100, 100, 100, 10, 10, 10]
```

#### 3. Add Damping if Overshooting
If sub passes waypoint:
```yaml
state_cost_matrix: [5000, 5000, 5000, 5000, 5000, 5000, 500, 500, 500, 50, 50, 50]
```

#### 4. Fine-Tune Heading
If heading is sluggish:
```yaml
state_cost_matrix: [5000, 5000, 5000, 10000, 10000, 15000, 500, 500, 500, 50, 50, 50]
#                                                ↑ increase yaw Q
```

#### 5. Energy Optimization
If thrusters overloading:
```yaml
thruster_cost_matrix: [30, 30, 30, 30, 30, 30, 30, 30]  # Increase R
```

This reduces thrust aggressiveness at the cost of slower response.

### Debugging Commands

```bash
# View current LQR debug output
ros2 topic echo /debug/lqr_angles
ros2 topic echo /debug/lqr_velocity
ros2 topic echo /debug/lqr_accel_cmd

# Dynamically change control mode
ros2 param set /control_node control_mode lqr_tuning

# Reconfigure gains while running
ros2 param set /control_node state_cost_matrix "[5000, 5000, ...]"

# Monitor thruster commands
ros2 topic echo /thruster_cmd

# Check actuator node health
ros2 topic echo /debug/actuator_status
```

### Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| **Localization Update Rate** | 25 Hz | EKF fusion frequency |
| **LQR Control Loop** | 25 Hz min | Synchronized with odometry |
| **Thruster Response Latency** | <50 ms | Command to PWM change |
| **Position Steady-State Error** | <0.1 m | After reaching waypoint |
| **Heading Steady-State Error** | <5° | After reaching target orientation |
| **Max Thrust Per Thruster** | ±40 N | Hardware saturation |
| **Power Budget** | <500 W continuous | 4S LiPo nominal |

---

## Summary

The ASUQTR submarine control system is a **three-layer architecture**:

1. **Control Node** (`control_node.py`): High-level decision making, LQR solver orchestration
2. **Actuator Node** (`actuator_node.py`): Hardware abstraction, PWM generation
3. **LQR Solver** (`lqr_solver.py`): Physics-based optimal control calculations

**Key Innovation**: State-Dependent Riccati Equation (SDRE) continuously adapts controller gains as submarine speed and drag change, providing superior performance over traditional fixed-gain LQR.

**Physical Foundation**: All outputs are in physical units (Newtons, rad/s) with mathematical proofs backing unit consistency.

---

## References

1. Cloutier, J. R., & Beard, R. W. (2005). "State-Dependent Riccati Equation Techniques: An Overview"
2. Blue Robotics T200 Thruster Datasheet: https://bluerobotics.com/store/thrusters/
3. ROS 2 Extended Kalman Filter: https://github.com/cra-ros-pkg/robot_localization
4. PCA9685 PWM Driver Specifications: Adafruit Industries
