# 03_ARCHITECTURE_CODE_ROS2.md

## Objectif du Document

Décrire l'architecture ROS2 complète : flux de données, callbacks, threading, et documentation détaillée des composants clés (`control_node.py`, `lqr_solver.py`, `actuator_node.py`).

---

## Limites du Document

**Fichiers analysés** :
- `control_node.py`, `lqr_solver.py`, `actuator_node.py`
- Messages ROS2 : ThrusterCommand.msg, Control.action
- Configs : params.yaml, robot_localization*.yaml

**Fichiers non analysés** :
- Scripts MATLAB de génération (A, B, modèle dynamique)
- URDF robot (géométrie thrusters confirmée vs supposée)
- Configurations hardware bas-niveau (PCA9685, T200 ESC)
- Historique des commits MATLAB→Python (Bm hardcodée)

**Note** : Ce document décrit ce que le code *fait*, pas ce qu'il *devrait* faire. Les hypothèses scientifiques (damping_sign, points de linéarisation) sont identifiées explicitement comme "À vérifier".

---

## 1. Vue Globale du Flux de Données

```
┌─────────────────────────────────────────────────────────────────────┐
│                     CAPTEURS (VectorNav + DVL)                      │
│                          ENU / FLU                                  │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
        ┌──────────────────▼──────────────────┐
        │  robot_localization (EKF Kalman)   │
        │  Fusion IMU + DVL + Odometry         │
        │  Publie: odometry/filtered (ENU)    │
        └──────────────────┬──────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────────┐
│                    CONTROL_NODE.PY                                  │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │ Callback: localization_callback (Priority HIGH, fréquence à confirmer │ │
│  │            via runtime params)                                   │ │
│  │  1. Reçoit Odometry (ENU/FLU)                                  │ │
│  │  2. Convertit ENU→NED, FLU→FRD                                 │ │
│  │  3. Calcule erreur: lqr_error = target_state - current_state   │ │
│  │  4. Appelle: compute_thrust_force(state, error, Q, R)         │ │
│  │  5. Clip saturation: max ±40.0 N                              │ │
│  │  6. Publie ThrusterCommand(efforts) [Newtons en mode LQR]      │ │
│  │                                                                  │ │
│  │  ⚠️ ALERTE CRITIQUE : Ambiguïté des unités ThrusterCommand     │ │
│  │  Le même message transporte des grandeurs différentes selon     │ │
│  │  le mode : Newtons (mode BEHAVIOR/LQR_TUNING) vs throttle      │ │
│  │  normalisé (mode MANUAL). À vérifier : marquage explicit ou     │ │
│  │  conventions documentées.                                       │ │
│  └────────────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │ Action Server: control_action_callback (async)                 │ │
│  │  1. Reçoit goal: target_pose (via FlexBE behaviors)            │ │
│  │  2. Transform TF2: camera_link → odom                         │ │
│  │  3. Update target_state (thread-safe, lock)                    │ │
│  │  4. Boucle: await is_target_reached() chaque 0.1s             │ │
│  │  5. Retour: Control.Result(success=True/False)                │ │
│  └────────────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │ Callback: gamepad_callback (Manual mode)                        │ │
│  │  1. Reçoit Joy (joystick axes/buttons)                         │ │
│  │  2. Mappe [left_stick, triggers, right_stick] → 6-DOF vector  │ │
│  │  3. Applique THRUST_ALLOC_MAT: 6-DOF → 8 moteurs             │ │
│  │  4. Publie ThrusterCommand (throttle direct, pas LQR)         │ │
│  └────────────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │ Parameter Callback: parameter_callback                          │ │
│  │  Gère updates dynamic: Q, R, damping_sign, max_force, modes   │ │
│  └────────────────────────────────────────────────────────────────┘ │
└──────────────────┬───────────────────────────────────────────────────┘
                   │ ThrusterCommand (8 valeurs moteurs : unités selon mode, à clarifier)
┌──────────────────▼───────────────────────────────────────────────────┐
│                    ACTUATOR_NODE.PY                                  │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │ Callback: thrusters_callback                                   │ │
│  │  1. Reçoit ThrusterCommand(efforts) [Newtons supposés en LQR; ambigu en MANUAL]                 │ │
│  │  2. Kick watchdog timer (reset 1-sec timeout)                 │ │
│  │  3. Si flat_mapping: efforts × gain_flat                      │ │
│  │  4. Sinon: np.interp(efforts, known_forces, known_api_cmds)  │ │
│  │  5. Applique offset: throttle += thruster_throttle_offset     │ │
│  │  6. Clip: [-1.0, +1.0]                                        │ │
│  │  7. T200Thruster.throttle = throttle → PCA9685 PWM           │ │
│  └────────────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │ Callback: kill_switch_callback                                 │ │
│  │  1. Reçoit Bool (magnétique kill switch)                       │ │
│  │  2. Trigger thrusters_init (1-sec lockout)                    │ │
│  │  3. All throttles → 0.0 (safe)                                │ │
│  └────────────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │ Timer Callback: thrusters_watchdog_callback (fréquence à confirmer)  │ │
│  │  1. Si (now - last_cmd_time) > 1 sec:                         │ │
│  │  2. Log warning + clip all throttles to 0.0                   │ │
│  │  3. Sécurité: tue moteurs si contrôle meurt                   │ │
│  └────────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
                           │ PWM I2C
┌──────────────────────────▼──────────────────────────────────────────┐
│                    HARDWARE (PCA9685 + T200)                         │
│                    PWM [1100, 1900] µs (typique, à vérifier)          │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 2. CONTROL_NODE.PY - Architecture Détaillée

### 2.1 Initialisation (`__init__`)

**Pré-allocation de mémoire** (stratégie de latence) :

```python
self.current_state = np.full(12, np.nan, dtype=np.float64)
self.target_state = np.full(12, np.nan, dtype=np.float64)
```

**Raison** : Réduit les allocations répétées; ne garantit pas un comportement temps réel en Python.

**Threading Safety** :

```python
self.target_state_lock = threading.Lock()
```

Protège target_state car action_server thread l'écrit, odometry thread la lit.

**Callback Groups** (MultiThreadedExecutor non-rentrant) :

```python
self.localization_cb_group = MutuallyExclusiveCallbackGroup()
self.action_cb_group = MutuallyExclusiveCallbackGroup()
self.sensor_cb_group = MutuallyExclusiveCallbackGroup()
```

Empêche la même callback de s'interrompre en elle-même.

### 2.2 Localization Callback (fréquence odometry/filtered à confirmer)

**Signature** :

```python
def localization_callback(self, msg: Odometry):
```

**Étapes** :

#### 1️⃣ **Extraction & Conversion ENU→NED**

```python
# Reçu en ENU (ROS standard)
X_ENU = msg.pose.pose.position.x
Y_ENU = msg.pose.pose.position.y
Z_ENU = msg.pose.pose.position.z

# Converti en NED
current_state[0] = Y_ENU     # North ← East
current_state[1] = X_ENU     # East ← North
current_state[2] = -Z_ENU    # Down ← -Up

# Angles: FLU → FRD
roll, pitch, yaw = quaternion_to_euler(...)
current_state[3] = roll              # Roll unchanged
current_state[4] = -pitch            # Pitch inverted
current_state[5] = (π/2 - yaw)       # Yaw shifted + wrapped
```

**Debug flags** :

```python
if debug_invert_roll:
    current_state[3] = -current_state[3]
# (permet d'isoler axes convention issues en production)
```

#### 2️⃣ **Extraction Vitesses Body**

```python
# Reçu en FLU (ROS std)
u_FLU = msg.twist.twist.linear.x
v_FLU = msg.twist.twist.linear.y
w_FLU = msg.twist.twist.linear.z

# Converti en FRD
current_state[6] = u_FLU           # Surge (forward unchanged)
current_state[7] = -v_FLU          # Sway (left→right inverted)
current_state[8] = -w_FLU          # Heave (up→down inverted)

# Angular velocities
current_state[9] = msg.twist.twist.angular.x
current_state[10] = -msg.twist.twist.angular.y
current_state[11] = -msg.twist.twist.angular.z
```

#### 3️⃣ **Initialisation Target (First Run)**

```python
with self.target_state_lock:
    if np.isnan(self.target_state[0]):  # First time?
        self.target_state[0:6] = self.current_state[0:6]  # Hold position
        self.target_state[6:12] = 0.0                      # Zero velocity
```

"Accroche-toi où tu es" au démarrage.

#### 4️⃣ **Exécution LQR** (si mode BEHAVIOR ou LQR_TUNING)

```python
with self.target_state_lock:
    target_state_copy = self.target_state.copy()  # Thread-safe read

lqr_error = target_state_copy - self.current_state

# Wrap angles to [-π, π] (shortest path)
lqr_error[3:6] = wrap_angles_to_pi(lqr_error[3:6])

# Compute thrust
thrusters_force = self.lqr_solver.compute_thrust_force(
    self.current_state, lqr_error, self.q_matrix, self.r_matrix, self.inv_r_matrix
)

# Software saturation
thrusters_force = np.clip(thrusters_force, -self.max_thruster_force_newton, +self.max_thruster_force_newton)
```

#### 5️⃣ **Publication**

```python
thrust_msg = ThrusterCommand(efforts=thrusters_force.tolist())
self.thruster_pub.publish(thrust_msg)
```

**Temps CPU** : Loggé chaque sec (throttle) pour profiling.

---

### 2.3 Action Server Callback (Async)

**Signature** :

```python
async def control_action_callback(self, goal_handle) -> Control.Result:
```

**Opération** :

1. **Validation mode** : Doit être BEHAVIOR
2. **Transform TF2** : `camera_link` → `odom` (si target frame ≠ odom)
3. **Update target_state** (thread-safe) :
   ```python
   with self.target_state_lock:
       # Copier les 6 DOF position/attitude en NED
   ```
4. **Boucle d'attente asynchrone** :
   ```python
   while not is_target_reached():
       if goal_handle.is_cancel_requested:
           # Safe cancel: set target = current
           self.target_state[:] = self.current_state[:]
           return Control.Result(success=False)
       await asyncio.sleep(0.1)
   ```

**Tolérances** (hardcodées) :
- Position : 0.5 m
- Angles : 10° (π/18 rad)

---

### 2.4 Gamepad Callback (Manual Mode)

**Flux** :

```
Joy (axes, buttons)
  ↓
left_stick_y, left_stick_x, triggers, right_stick
  ↓
raw_cmd_vector = [left_y, left_x, triggers, button_diff, -right_y, right_x]
  ↓
apply THRUST_ALLOC_MAT
  ↓
clip throttle ∈ [-0.8, +0.8]
  ↓
ThrusterCommand (efforts = throttle, pas des Newtons)
```

**Allocation Matrix** :

```python
THRUST_ALLOC_MAT = np.array([
    [-1, 1, 0, 0, 0, 1],      # Motor 0
    [-1, -1, 0, 0, 0, -1],    # Motor 1
    [0, 0, -1, -1, -1, 0],    # Motor 2
    [0, 0, -1, 1, -1, 0],     # Motor 3
    [0, 0, -1, -1, 1, 0],     # Motor 4
    [0, 0, -1, 1, 1, 0],      # Motor 5
    [1, 1, 0, 0, 0, -1],      # Motor 6
    [1, -1, 0, 0, 0, 1],      # Motor 7
])  # 8 moteurs × 6 DOF
```

---

### 2.5 Mode Switching

**Trigger** : START button (gamepad)

**Cycle** : BEHAVIOR → LQR_TUNING → MANUAL → BEHAVIOR

Via dynamic parameter update (appelle `parameter_callback` en interne).

---

## 3. LQR_SOLVER.PY - Implémentation SDRE

### 3.1 Classe SubLQRSolver

**Pré-allocation** :

```python
self.system_dynamics_matrix = np.zeros((12, 12), dtype=np.float32)
self.damping_sign = 1.0
```

### 3.2 Fonction Core: `compute_thrust_force()`

```python
def compute_thrust_force(self, state, state_error, q_matrix, r_matrix, inv_r_matrix):
    """
    Cœur mathématique du contrôleur.
    """
    # Step 1: Calculer A(state)
    a_matrix = self.update_system_dynamics_matrix_A(self.system_dynamics_matrix, state)
    
    # Step 2: Résoudre ARE
    x_matrix = scipy.linalg.solve_continuous_are(a_matrix, Bm, q_matrix, r_matrix)
    
    # Step 3: Calculer K
    k_gain = np.dot(inv_r_matrix, np.dot(Bm.T, x_matrix))
    
    # Step 4: T_cmd = -K·e
    return -np.dot(k_gain, state_error)
```

**⚠️ À vérifier : Convention de signe LQR**

Si state_error = target_state - current_state, alors -K·state_error peut inverser le signe par rapport à la convention LQR standard T_cmd = -K(x - x_ref). Test axe par axe requis : vérifier que erreur positive (target > current) → commande positive (vers avant) ✓

### 3.2 Fonction Cinématique & Cinétique: `update_system_dynamics_matrix_A()`

**Input** : state (12 éléments), matrix Am pré-allouée

**Output** : Am remplie (12×12)

**Rows 0-5 (jacobien des équations 0-5 de f(x,u)) : Cinématique**

```python
# Rotation matrices Z(ψ)·Y(θ)·X(φ) convertissent body→world
# Dépend de: φ, θ, ψ, u, v, w (cross-coupling non-linéaire)

Am[0][6] = cos_pitch * cos_yaw       # x_dot = f(u, ...)
Am[0][7] = cos_yaw*sin_pitch*sin_roll - cos_roll*sin_yaw  # ... f(v)
# ... etc (24 termes cinématiques)
```

**Rows 6-11 (jacobien des équations 6-11 de f(x,u)) : Cinétique**

```python
# Drag linéaire sur diagonal
Am[6][6] = damping_sign * 1.5978...  # u_dot = drag(u) + ...

# Termes Coriolis (cross-coupling)
Am[6][7] = r    # u_dot dépend de yaw-rate × sway velocity
Am[6][11] = v   # u_dot dépend de sway

# Restoring (gravité)
Am[6][4] = 0.0719... * cos_pitch * restoring_func(radius, depth)

# ... et 24+ autres termes pour v_dot, w_dot, p_dot, q_dot, r_dot
```

---

## 4. ACTUATOR_NODE.PY - Chaîne D'Actuation

### 4.1 Classe T200Thruster

**Données empiriques** (Blue Robotics 16V) :

```python
known_forces_n = np.array([-40.22, -27.47, ..., 51.50])
known_api_cmds = np.array([-1.0, -0.75, ..., 1.0])
```

Interpolation linéaire 1D :

```python
throttle = np.interp(effort_newtons, known_forces_n, known_api_cmds)
```

### 4.2 Callback Moteurs

```python
def thrusters_callback(self, msg: ThrusterCommand):
    efforts = np.asarray(msg.efforts, dtype=np.float64)  # [Newtons supposés en mode LQR; ambigu en mode MANUAL]
    
    # Interpolation
    throttles = np.interp(efforts, T200Thruster.known_forces_n, T200Thruster.known_api_cmds)
    
    # Offset
    throttles += self.thruster_throttle_offset
    
    # Clip
    throttles = np.clip(throttles, -1.0, 1.0)
    
    # Send to hardware
    for i, throttle in enumerate(throttles):
        self.thrusters[i].throttle = throttle
```

**⚠️ Décision architecturale en attente** : Si le mode MANUAL publie déjà du throttle normalisé ([-1, +1]), actuator_node ne doit pas appliquer une interpolation Newtons→throttle sur ces valeurs. **Décision à prendre** : standardiser tout en Newtons dans ThrusterCommand, ou séparer les messages (ThrusterCommandNewtons vs ThrusterCommandThrottle).

### 4.3 Watchdog Timer (fréquence à confirmer)

```python
def thrusters_watchdog_callback(self):
    if self.thrusters_watchdog_enabled:
        now = self.get_clock().now()
        if (now - self.last_watchdog_kick_time) > Duration(seconds=1.0):
            # KILL: all throttles → 0
            for t in self.thrusters:
                t.throttle = 0.0
```

Sécurité : si control_node crash, les moteurs s'arrêtent en 1 sec.

---

## 5. Topics, Messages, Actions

### Subscriptions (control_node)

| Topic | Type | Callback | QoS | Fréquence |
|-------|------|----------|-----|-----------|
| `odometry/filtered` | nav_msgs/Odometry | localization_callback | BEST_EFFORT | À confirmer |
| `debug/target_pose` | geometry_msgs/PoseStamped | debug_target_callback | BEST_EFFORT | Manual |
| `dashboard/gamepad` | sensor_msgs/Joy | gamepad_callback | BEST_EFFORT | À confirmer |

### Publications (control_node)

| Topic | Type | Contenu | QoS | Fréquence |
|-------|------|---------|-----|-----------|
| `thruster_cmd` | sub_interfaces/ThrusterCommand | 8 valeurs moteurs; Newtons en mode LQR supposé, throttle en mode MANUAL à vérifier | BEST_EFFORT | À confirmer |
| `debug/lqr_angles` | std_msgs/Float64MultiArray | [φ_flu, θ_flu, ψ_flu, φ_ned, θ_ned, ψ_ned] | — | Opt |
| `debug/lqr_velocity` | Float64MultiArray | [u, v, w, p, q, r] | — | Opt |
| `debug/lqr_accel_cmd` | Float64MultiArray | [u̇, v̇, ẇ, ṗ, q̇, ṙ] from Bm | — | Opt |

**Note QoS** : BEST_EFFORT choisi pour réduire latence vs fiabilité. À valider : ce trade-off approprié pour contrôle haute-fréquence ?

### Subscriptions (actuator_node)

| Topic | Type | Callback | QoS |
|-------|------|----------|-----|
| `thruster_cmd` | sub_interfaces/ThrusterCommand | thrusters_callback | BEST_EFFORT |
| `kill_switch` | std_msgs/Bool | kill_switch_callback | RELIABLE |
| `gripper` | std_msgs/Float32 | gripper_callback | BEST_EFFORT |
| `subsea_light` | std_msgs/Float32 | subsea_light_callback | BEST_EFFORT |
| `torpedo` | std_msgs/Float32 | torpedo_callback | BEST_EFFORT |

### Action Server (control_node)

| Action | Type | Goal | Result | Feedback |
|--------|------|------|--------|----------|
| `navigate_sub` | sub_interfaces/Control | geometry_msgs/PoseStamped | bool success | float32 distance_to_goal |

---

## 6. Thread Safety & Executors

### MultiThreadedExecutor (4 threads)

```python
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

**Avantages** :
- Action server bloquant (~0.1 s) ne gêne pas odometry callback (fréquence à confirmer)
- Joystick input (lent) n'interrompt pas LQR (rapide)

**Risques** :
- Race conditions si state partagé mal protégé
- MutuallyExclusiveCallbackGroup réduit les accès concurrents dans ce node, mais ne garantit pas l'absence de conflit si d'autres nodes/drivers accèdent au même bus I2C

---

## 7. Paramètres Dynamiques

**Réconfigurables sans restart** :

```python
ros2 param set /control_node control_mode "behavior"
ros2 param set /control_node state_cost_matrix "[1000, 500, ...]"
ros2 param set /control_node damping_sign -1.0
ros2 param set /actuator_node thruster_throttle_offset 0.05
```

---

## 8. Niveau de Confiance des Informations

### ✅ Confirmé par le Code

- `control_node.py` reçoit **Odometry** (ENU/FLU) et la convertit en NED/FRD en dur dans les callbacks
- `lqr_solver.py` contient matrice **Bm** (12×8) codée explicitement
- ARE résolue à chaque appel de **compute_thrust_force()**; fréquence effective dépend de l'appel control/odométrie runtime
- **MultiThreadedExecutor** avec callback groups pour organiser la concurrence des callbacks
- **Watchdog** 1 sec avec kill-to-safe (tous throttles → 0.0)
- **Mode switching** : BEHAVIOR ↔ LQR_TUNING ↔ MANUAL
- T200Thruster utilise **np.interp()** avec données empiriques

### ⚠️ Supposé (À Valider)

- Fréquences : "50 Hz" citée comme approximation (réelle à confirmer via runtime)
- **ThrusterCommand.efforts** toujours en Newtons (supposé en mode LQR, mais peut être throttle en mode MANUAL)
- **PWM range [1100, 1900] µs** : typique pour servos continus, à vérifier dans datasheet ESC
- **Fréquence Riccati** : résolution ARE à chaque callback = coût CPU à profiler
- **TF2 frames** : nommées odom/base_link par défaut (à valider avec URDF)
- **Callback groups** MutuallyExclusive suffisent pour thread-safety (à tester avec charges haute)

### ❓ À Vérifier d'Urgence

- Cohérence ThrusterCommand : même message type → Newtons ou throttle ? Marquer explicitement
- Fréquence réelle odometry/filtered vs control_node callback (profiler avec logs timestamp)
- Timeout action server : hardcodé où ? (voir is_target_reached())
- Sens/neutre PWM du PCA9685/ESC : vérifier que 1500 µs correspond bien à throttle 0
- Offset `thruster_throttle_offset` : physiquement justifié ou empirique ?

---

## 9. Risques Architecturaux Prioritaires

### 🔴 Critique

**R1: Ambiguïté ThrusterCommand.efforts (Newtons vs Throttle)**
- Même message type transporte des unités différentes selon mode
- Pas de marquage explicite → confusion possible en debug
- **Mitigation** : Ajouter champ `efforts_unit: "newtons" | "throttle"` au message, ou docs strictes

**R2: Riccati CPU vs Latence (P1 du diagnostic)**
- Coût CPU à profiler sur la plateforme embarquée
- Surcharge Jetson → odometry callback latency augmente
- **Mitigation** : Profiler avant/après; implémenter cache ou LQR fixe si CPU > 60%

**R3: Race Conditions sur target_state**
- Action server et odometry callback accèdent target_state en parallel
- Lock protège, mais risque de latence, blocage ou ordre d'exécution inattendu à tester sous charge
- **Mitigation** : Tester MultiThreadedExecutor sous charge; ajouter logs timeout

**R4: Absence de rate limiter moteur identifié**
- Clipping limite l'amplitude (±40 N), mais pas la variation rapide de commande
- Changements abrupts → oscillations, usure moteur, réponse instable
- **Mitigation** : Ajouter slew-rate limiter sur T_cmd ou throttle avant publication/envoi PWM

### 🟡 Modéré

**R5: Conversions ENU↔NED**
- Pas de tests unitaires pour chaque transformation
- Confusion angles possibles (yaw ±180°, pitch inversé)
- **Mitigation** : Ajouter test suite (test_enu_ned_*.py)

**R6: Stabilité A(x) State-Dependent**
- Matrice A change à chaque tick (SDRE complet)
- Si état varie rapidement, Riccati solution X peut sauter
- **Mitigation** : Logger norm(A(t) - A(t-1)); proposer smoothing si instable

---

## À Vérifier

- [ ] Conversions ENU↔NED correctes (tester avec RViz)
- [ ] Action server timeout cohérent avec missions RoboSub
- [ ] Watchdog 1 sec = timing correct ?
- [ ] Mapping actuator_node : np.interp linéaire entre known_forces_n et known_api_cmds; vérifier comportement autour de zéro et doublons éventuels
- [ ] TF2 frames nommées correctement (odom, base_link, etc.)
- [ ] QoS BEST_EFFORT optimal pour latence ?

---

## Décisions Importantes

1. **MultiThreadedExecutor 4 threads** : Permet async action server
2. **TF2 transform** : Flexible pour futur caméra-centric missions
3. **Debug topics optionnels** : Impact performance à mesurer si activés à haute fréquence
4. **Watchdog 1 sec** : Compromis sécurité/latence à valider en tests à sec puis piscine
5. **BEST_EFFORT QoS** : Choix de latence à valider; RELIABLE pourrait être préférable pour certaines commandes critiques

---

**⚠️ Mise à jour requise** : Ce README doit être mis à jour après validation runtime des fréquences, des unités ThrusterCommand, du signe LQR et des conversions de repères.
