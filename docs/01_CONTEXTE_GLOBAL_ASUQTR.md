# 01_CONTEXTE_GLOBAL_ASUQTR.md

**Généré** : Mai 2026  
**Analyse basée sur** : control_node.py, lqr_solver.py, actuator_node.py, params.yaml, messages ROS2  
**Fichiers non analysés** : Scripts MATLAB generation_A_B, documentations externes  
**Limites** : Ce document capture l'état actuel du code. Des hypothèses y sont identifiées explicitement. **Utilisation recommandée** : lire ce fichier en premier pour le contexte, puis vérifier les détails techniques dans les fichiers code. Ne pas traiter les hypothèses comme des certitudes.

---

## Objectif du Document

Ce document établit le contexte global du projet ASUQTR et définit le rôle attendu d'un GPT personnalisé spécialisé dans l'architecture et le contrôle du sous-marin.

---

## 1. Présentation du Projet ASUQTR

### Contexte Compétition

**À vérifier dans la documentation officielle du projet** :
- Acronyme ASUQTR exact
- Organisme de la compétition (IEEE RoboSub ?)
- Objectifs de missions spécifiques à RoboSub

**État actuel du code** : Le projet implémente une architecture ROS2 avec contrôle autonome, pas de limitation apparente à une seule compétition.

### Objectifs Techniques Confirmés par le Code

- **Navigation autonome** : Via FlexBE (machine à états comportementaux, voir `sub_autonomy/`)
- **Contrôle de position** : LQR/SDRE recalculé en boucle fermée (voir `control_node.py`, `lqr_solver.py`)
- **Fusion capteurs** : IMU + DVL (voir `robot_localization` config)
- **Sécurité** : Watchdog timer 1sec (voir `actuator_node.py`)

### Actuation (À Confirmer)

**Hypothèse actuelle** : 8 moteurs Blue Robotics T200
- 4 moteurs horizontaux (indices 0, 1, 6, 7)
- 4 moteurs verticaux (indices 2, 3, 4, 5)

**Confirmé** : Le système commande 8 sorties moteurs.  
**À confirmer** : Répartition exacte horizontaux/verticaux et géométrie réelle (voir URDF).

### Architecture Système

```
┌─────────────────────────────────────────────────────────────┐
│ sub_autonomy (FlexBE behaviors)                             │
└──────────────────────┬──────────────────────────────────────┘
                       │ Action: navigate_sub(target_pose)
┌──────────────────────▼──────────────────────────────────────┐
│ sub_control (LQR controller + state fusion)                 │
│  ├─ control_node.py      [50 Hz] Reçoit odometry            │
│  ├─ lqr_solver.py        [inline] SDRE math                 │
│  └─ robot_localization   [EKF] IMU + DVL fusion             │
└──────────────────────┬──────────────────────────────────────┘
                       │ Topic: thruster_cmd (8× Newtons)
┌──────────────────────▼──────────────────────────────────────┐
│ sub_hardware (Actuators + sensors)                          │
│  ├─ actuator_node.py     [I2C] Interpole Newtons→PWM       │
│  ├─ vectornav_imu        [USB] IMU 6-DOF                    │
│  ├─ dvl_node             [Ethernet] Altitude + velocity     │
│  └─ gpio_node            [GPIO] Kill switch                 │
└──────────────────────┬──────────────────────────────────────┘
                       │ I2C: PCA9685 PWM driver
                ┌──────▼──────┐
                │  T200 ESCs  │
                │  T200 Motors│
                └─────────────┘
```

---

## 2. Vecteur d'État et Conventions de Repères

### Vecteur d'État Complet

**Confirmé par le code** (`control_node.py`, `lqr_solver.py`) :

```
x = [X, Y, Z, φ, θ, ψ, u, v, w, p, q, r]ᵀ ∈ ℝ¹²

où :
  [X, Y, Z]           : Position (repère NED)
  [φ, θ, ψ]           : Attitude (Roll, Pitch, Yaw en NED)
  [u, v, w]           : Vitesses linéaires (corps NED/FRD)
  [p, q, r]           : Vitesses angulaires (corps NED/FRD)
```

### Repères Utilisés

| Composant | Entrée | Interne | Sortie |
|-----------|--------|---------|--------|
| **Capteurs** | ENU / FLU | — | Odometry (ENU) |
| **EKF** | Odometry (ENU) | ENU / FLU | Odometry filtrée (ENU) |
| **control_node (LQR)** | Odometry (ENU) | **NED / FRD** | Forces (Newtons) |
| **lqr_solver** | État NED/FRD | NED / FRD | Gains K |
| **actuator_node** | Forces (Newtons) | — | PWM |

### Conversions ENU → NED (dans control_node)

**Confirmé par code** (lines 397-405, control_node.py) :

```python
current_state[0] = msg.pose.pose.position.y     # X_NED ← Y_ENU
current_state[1] = msg.pose.pose.position.x     # Y_NED ← X_ENU
current_state[2] = -msg.pose.pose.position.z    # Z_NED ← -Z_ENU

# Angles
current_state[3] = roll                         # Roll (unchanged)
current_state[4] = -pitch                       # Pitch (inverted)
current_state[5] = (π/2 - yaw) wrapped          # Yaw (shifted 90°)
```

**À vérifier** : Les conversions doivent être validées par tests unitaires, notamment yaw, pitch et vitesses angulaires.

### Conversions FLU → FRD (Body Velocities)

**Confirmé par code** (lines 417-422) :

```python
current_state[6] = msg.twist.twist.linear.x     # u (Forward, unchanged)
current_state[7] = -msg.twist.twist.linear.y    # v (Left→Right, inverted)
current_state[8] = -msg.twist.twist.linear.z    # w (Up→Down, inverted)
current_state[9:12] = angular velocities (inverted)
```

---

## 3. Pipeline Complet : Capteurs → PWM

```
┌───────────────────────────┐
│ VectorNav IMU (ENU/FLU)   │
│ DVL (ENU/FLU)             │
└──────────┬────────────────┘
           │
      ┌────▼─────────────────────────────┐
      │ robot_localization (EKF)         │
      │ Fusion IMU + DVL                 │
      │ Publie: odometry/filtered (ENU)  │
      └────┬─────────────────────────────┘
           │
      ┌────▼────────────────────────────────────┐
      │ control_node.localization_callback      │
      │ [HEARTBEAT 50 Hz]                       │
      │                                          │
      │ 1. Reçoit Odometry (ENU/FLU)            │
      │ 2. Convertit ENU→NED, FLU→FRD           │
      │ 3. Calcule erreur: e = x_target - x     │
      │ 4. Appelle: lqr_solver.compute_thrust...│
      │    IN: state, error, Q, R               │
      │    OUT: [T1, T2, ..., T8] en Newtons    │
      │ 5. Clip: [-40.0, +40.0] N               │
      │ 6. Publie: ThrusterCommand              │
      └────┬────────────────────────────────────┘
           │ Topic: thruster_cmd
           │ Message: ThrusterCommand(efforts=[...])
      ┌────▼───────────────────────────────────┐
      │ actuator_node.thrusters_callback        │
      │ [REAL-TIME I2C]                         │
      │                                         │
      │ 1. Reçoit: ThrusterCommand [Newtons]   │
      │ 2. Kick watchdog (1-sec timeout)        │
      │ 3. np.interp(Newtons → [-1.0, 1.0])    │
      │ 4. Applique offset neutral              │
      │ 5. Clip: [-1.0, +1.0]                   │
      │ 6. PCA9685 PWM: [1100, 1900] µs         │
      └────┬───────────────────────────────────┘
           │ I2C
      ┌────▼───────────────────────┐
      │ T200 ESC                    │
      │ → Motor speed/direction     │
      └─────────────────────────────┘
```

---

## 4. Distinctions Clés : Forces, Throttle, PWM

### Chaîne de Conversion Détaillée

| Étape | Grandeur | Unités | Plage | Généré par | Note |
|-------|----------|--------|-------|-----------|------|
| **Calcul LQR** | Forces moteurs (T_i) | Newtons | ±40.0 N (clip) | lqr_solver.compute_thrust_force() | Output u = -K·e |
| **Saturation logicielle** | Clipped T_i | Newtons | ±40.0 N | control_node.py | Saturation physique |
| **Interpolation** | Throttle | Normalized | [-1.0, +1.0] | actuator_node (np.interp) | Utilise courbe T200 empirique |
| **PCA9685 PWM** | Pulse width | Microsecondes | [1100, 1900] µs | ContinuousServo API | Signal ESC |
| **Moteur** | Vitesse/sens | RPM | Variable | Hardware | Dépend tension 16V |

**Note** : Les conversions doivent être validées par tests unitaires, surtout yaw, pitch et vitesses angulaires.

### Points Critiques de Confusion

**Hypothèse actuelle** : Les efforts sortis par le LQR sont déjà en Newtons (pas des poussées généralisées τ).

**Justification** (voir `lqr_solver.py` docstring) :
```
B matrix [6:12, :] convertit Newtons → accélérations
Si entrée = 1.0 N → accélération = B[i,j] m/s²
→ Entrée en Newtons confirmée par Newton's 2nd law
```

**À vérifier** :
- [ ] Bm [6:12, :] correspond à ∂ν̇/∂T avec T en Newtons ?
- [ ] Ou à ∂ν̇/∂du avec du normalisé ? (très important !)

---

## 5. Lien MATLAB → Python

### Génération du Modèle d'État

**Hypothèse** : Un script MATLAB génère (ou a généré) les matrices A et B du modèle linéarisé :

```
ẋ = A·x + B·u
```

**État actuel du code Python** :
- Matrice **Bm** (12×8) : **Codée directement** dans `lqr_solver.py` (hardcodée)
- Matrice **A(x)** : **Reconstruite en ligne** dans `update_system_dynamics_matrix_A()`

**Workflow supposé** (à valider tant que le script MATLAB exact et l'historique de copie vers Bm n'ont pas été vérifiés):

```
MATLAB Script (generation_A_B.m)
  ├─ Définit dynamique non-linéaire ẋ = f(x, u)
  ├─ Calcule jacobiens: A = ∂f/∂x, B = ∂f/∂u
  ├─ Linéarise autour (x₀, u₀) = ?
  └─ Exporte A, B ou les coefficients Bm

↓

Python (lqr_solver.py)
  ├─ Bm hardcodée (copié depuis MATLAB ou calculé)
  ├─ A reconstruite chaque tick via update_system_dynamics_matrix_A()
  └─ Résout ARE: A^T X + XA - XBm R^-1 Bm^T X + Q = 0
```

### Point Critique à Vérifier

**Question cruciale** : La matrice Bm codée en Python correspond-elle à :

1. **∂ν̇/∂T** où T = forces moteurs **en Newtons** ? ✓ (confirmé par docstring)
2. Ou à **∂ν̇/∂du** où du = throttle/PWM normalisé ? ✗ (incompatible avec LQR output)

**Implication** :
- Si Bm = ∂ν̇/∂T (Newtons) → u = T_cmd en Newtons ✓ cohérent
- Si Bm = ∂ν̇/∂du (throttle) → u = throttle → contradiction avec actuator_node

**À vérifier d'urgence** : Lire script MATLAB original, comparer coefficients Bm

---

## 6. Rôle Attendu du GPT Personnalisé

### Périmètre Principal

- **Contrôle LQR/SDRE** : Architecture, tuning Q/R, diagnostics mathématiques
- **Architecture ROS2** : Flux données, callbacks, threading, QoS
- **Modélisation dynamique** : Matrices A/B/M/C/D/G, linéarisation
- **Conversion forces moteurs** : Newtons → throttle → PWM
- **Tests à sec** : Unit tests, simulations
- **Tests piscine** : Comportement empirique, tuning
- **Diagnostics** : Logs, comportements anormaux, debugging

### Périmètre Secondaire (Peut Aider)

- **Électronique embarquée** : PCA9685, T200 ESC, connexions I2C
- **Sécurité système** : Kill switch, watchdog, timeouts
- **Organisation RoboSub** : Calendrier, allocation tâches, missions types
- **Stratégies comportementales** : FlexBE states, transitions, mission sequencing

### Hors Périmètre (Sauf Demande Explicite)

- Design mécanique détaillé
- PCB avancé, schématiques complexes
- Fabrication, usinage
- Électronique analogique complexe

---

## 7. Niveau de Confiance des Informations

### ✅ Confirmé par le Code

- `control_node.py` reçoit **Odometry** (ENU/FLU) et la convertit en NED/FRD
- `lqr_solver.py` retourne un vecteur de 8 commandes, interprétées ensuite comme des efforts en Newtons par actuator_node.py
- `actuator_node.py` reçoit **ThrusterCommand** (efforts) et interp vers PWM
- **THRUST_ALLOC_MAT** (8×6) mappe 6-DOF input vers 8 moteurs
- **Watchdog timer** 1 sec si pas de commande thruster
- **Modes** : BEHAVIOR (autonome), LQR_TUNING (debug), MANUAL (joystick)
- Recalcul ARE à chaque callback de contrôle/odométrie (fréquence exacte à confirmer)

### ⚠️ Supposé (À Valider)

- Les efforts T_cmd sont **en Newtons** (supposé basé sur docstring, à vérifier empiriquement)
- Matrice Bm représente **∂ν̇/∂T** avec T en Newtons (très critique)
- Le repère interne du LQR est **NED/FRD** (déduit des conversions ENU→NED)
- Point de linéarisation A/B : **x₀ = équilibre, u₀ = 0** (à confirmer MATLAB)
- Plage PWM moteurs : **[1100, 1900] µs** (à vérifier dans datasheet ESC/config)
- 8 moteurs = 4 horizontal + 4 vertical (supposé, à confirmer géométrie)

### ❓ À Vérifier

- **Signe de l'erreur** : `e = x_ref - x` (confirmé code) mais validation empirique ?
- **Signe du damping** : `damping_sign = +1` correct physiquement ? (voir P2 diagnostic)
- **Cohérence ENU↔NED** : Conversions correctes ? (tests unitaires manquent)
- **Cohérence Bm Python ↔ B MATLAB** : Valeurs concordent-elles ?
- **Unités T200** : Courbe known_forces_n/known_api_cmds exacte pour 16V ?
- **Conversions FLU→FRD** : Signes des vitesses angulaires correct ?
- **Tolerance is_target_reached()** : 0.5m / 10° sont-elles réalistes pour RoboSub ?

---

## 8. Problèmes Actuels Critiques pour le Contrôle

### 🔴 Risques Majeurs

1. **Recalcul Riccati à fréquence élevée** : Selon le code, l'ARE est résolu à chaque callback de contrôle/odométrie. Fréquence exacte à confirmer avec les paramètres runtime et profiling.
   - Coût CPU : à profiler avant d'affirmer une valeur.
   - Perte déterminisme temps-réel potentielle si charge élevée.
   - À profiler et optimiser au besoin

2. **Signe du damping ambigu** : `damping_sign = ±1.0`
   - Drag linéaire doit être négatif (dissipation)
   - Si signé mal → instabilité
   - À valider contre théorie Fossen + tests

3. **Signe de l'erreur** : `e = target - current`
   - Code cohérent mais validation empirique manque
   - Si inversé → feedback déstabilisant

4. **Unités Bm** : Newtons ou throttle normalisé ?
   - Docstring dit Newtons mais source MATLAB incertaine
   - **Critique pour validité LQR**

5. **Absence trim/intégrateur/anti-windup identifiée** : Aucun trim/feedforward/intégrateur identifié dans les fichiers analysés.
   - Erreur DC persistent possible (creep)
   - Pas de compensation offset statique (gravité/flottabilité)
   - À ajouter pour robustesse

### 🟡 Risques Modérés

6. **Conversions ENU↔NED** : Pas de tests unitaires
   - Confusion possible (yaw ±180°, pitch inversé)
   - À documenter + tester systématiquement

7. **Matrice A(x) state-dependent**
   - Peut changer rapidement si état varie
   - Riccati solution peut devenir instable

---

## 9. Paramètres Dynamiques ROS2

### control_node

```yaml
control_mode: "lqr_tuning"        # behavior | lqr_tuning | manual
state_cost_matrix: [1052, 578, ...]  # Q diagonal (12 éléments)
thruster_cost_matrix: [15, 15, ...]  # R diagonal (8 éléments)
max_thruster_force_newton: 40.0    # Clipping software
damping_sign: 1.0                  # ±1 pour inverser drag
publish_lqr_debug_angles: true
```

### actuator_node

```yaml
thruster_throttle_offset: 0.0      # Correction neutral drift
use_flat_thruster_mapping: false   # Linear (debug) vs interp
enable_thrusters_watchdog: true    # Sécurité 1sec
```

### robot_localization

Voir `config/robot_localization_imu_dvl.yaml` pour poids IMU/DVL

---

## 10. Fichiers Clés du Projet

| Fichier | Rôle | Lien |
|---------|------|------|
| control_node.py | Orchestration LQR + modes | `../src/sub_control/scripts/control_node.py` |
| lqr_solver.py | Mathématiques SDRE/ARE | `../src/sub_control/sub_control/lqr_solver.py` |
| actuator_node.py | Newtons→PWM + watchdog | `../src/sub_hardware/scripts/actuator_node.py` |
| params.yaml | Config contrôle | `../src/sub_control/config/params.yaml` |
| ThrusterCommand.msg | Message efforts | `../src/sub_interfaces/msg/ThrusterCommand.msg` |
| Control.action | Action serveur | `../src/sub_interfaces/action/Control.action` |
| robot_localization*.yaml | EKF config | `../src/sub_control/config/` |
| URDF robot | Géométrie | `../src/sub_launch/urdf/asuqtr_sub.urdf` |

---

## À Vérifier (Checklist pour Validation)

- [ ] Acronyme ASUQTR et contexte RoboSub (doc officielle)
- [ ] Configuration réelle moteurs (4 horiz + 4 vert) vs géométrie URDF
- [ ] Plage PWM [1100, 1900] µs confirmée dans datasheet ESC + config
- [ ] Script MATLAB `generation_A_B.m` : point linéarisation (x₀, u₀)
- [ ] Matrice Bm : coefficients concordent avec B MATLAB en Newtons ?
- [ ] Signe damping physiquement correct (tests step response)
- [ ] Conversions ENU↔NED → tests unitaires complets
- [ ] Signe erreur e = x_ref - x validé empiriquement
- [ ] Coordonnées repères : world = NED ?, body = FRD ?
- [ ] Tolerances is_target_reached() justifiées pour RoboSub

---

## Décisions Importantes (État Actuel vs À Décider)

1. **État actuel** : Riccati recalculé fréquemment pour approche SDRE
   **Décision à prendre** : Garder SDRE complet ? Passer LQR fixe K₀ ? Implémenter cache ?

2. **État actuel** : Aucun trim/feedforward/intégrateur
   **Décision à prendre** : Ajouter compensations statiques ? Anti-windup ?

3. **État actuel** : Tolérances `is_target_reached()` hardcodées
   **Décision à prendre** : Paramétriser ? Adapter par mission ?

4. **État actuel** : damping_sign paramètre dynamique
   **Décision à prendre** : Hardcoder signe correct (retirer ambiguïté)

5. **État actuel** : Pas de tests unitaires conversions repères
   **Décision à prendre** : Ajouter test suite pour ENU↔NED ?

---

## 11. Comment Utiliser ce README dans un GPT Personnalisé

Pour un assistant IA spécialisé sur ASUQTR :

1. **Lire ce fichier en premier** : Il établit le contexte global et identifie les hypothèses.
2. **Utiliser ce fichier pour comprendre le contexte** : Flux données, repères, unités, modes de contrôle.
3. **Vérifier les détails techniques dans les fichiers code** : Ne pas supposer que ce README est exhaustif.
4. **Ne pas traiter les hypothèses comme des faits** : Les sections "Supposé" et "À vérifier" sont critiques.
5. **Mettre à jour après validation** : Notamment après analyse du script MATLAB, vérification de Bm, et tests de repères.

---

**⚠️ Ce document doit être mis à jour après validation du script MATLAB, de la matrice Bm et des tests de repères.** 

Une fois ces vérifications effectuées, ce README servira de base solide pour un GPT spécialisé ASUQTR sans risque de propagation d'erreurs.
