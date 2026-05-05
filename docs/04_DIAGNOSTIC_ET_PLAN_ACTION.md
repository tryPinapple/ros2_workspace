# 04_DIAGNOSTIC_ET_PLAN_ACTION.md

## Objectif du Document

Identifier les problèmes actuels de l'implémentation LQR/SDRE, évaluer leur criticité, et fournir un plan d'action détaillé avec ordre de priorité.

---

## 1. Problèmes Identifiés

### 1.0 🔴 CRITIQUE - Nouveaux Problèmes Identifiés

#### **P0: Ambiguïté ThrusterCommand (Newtons vs Throttle)**

**Description** :
- En mode LQR, ThrusterCommand semble contenir des efforts interprétés comme Newtons
- En mode MANUAL, il peut contenir des valeurs throttle/directes
- actuator_node applique np.interp Newtons→throttle sur les deux, ce qui est risqué

**Problème** :
- Même message type transporte des unités différentes selon le mode
- Pas de marquage explicite → confusion possible en debug
- Si mode MANUAL publie déjà throttle, l'interpolation Newtons→throttle donne un résultat incorrect

**Correction** :
1. Vérifier en détail ce que control_node.py publie en mode MANUAL
2. Vérifier ce qu'actuator_node.py reçoit et comment il interprète
3. Standardiser ThrusterCommand en Newtons partout, ou créer deux messages distincts

---

#### **P0b: Cohérence Bm MATLAB/Python Non Validée**

**Description** :
- Matrice Bm (12×8) est hardcodée dans `lqr_solver.py`
- Origine, unités, point de linéarisation et cohérence avec B MATLAB ne sont pas validés

**Problème** :
- Si Bm provient du mauvais point de linéarisation, tous les calculs LQR sont invalides
- Unités ambiguës : Newtons ou commande normalisée du ?
- Coefficients numériques non documentés

**Correction** :
1. Localiser script MATLAB original `generation_A_B.m`
2. Comparer coefficients Bm Python ↔ B MATLAB
3. Clarifier unités et point de linéarisation (x₀, T₀ ou du₀)
4. Régénérer Bm si nécessaire avec entrées T en Newtons

---

### 1.1 🔴 CRITIQUE

#### **P1: Recalcul Riccati à Chaque Callback (SDRE Complet)**

**Description** :
- À chaque réception d'odométrie ou appel de compute_thrust_force(), selon l'architecture runtime, on résout ARE complet via `scipy.linalg.solve_continuous_are()`
- Fréquence effective à confirmer runtime

**Symptôme** :
- Surcharge potentielle du processeur embarqué
- Déterminisme temps-réel dégradé si Riccati échoue ou prend trop longtemps

**Options de correction** :

| Option | Coût CPU | Adaptation | Complexité | Notes |
|--------|----------|-----------|-----------|-------|
| **LQR fixe** (K₀ calculé offline) | À profiler | Locale (pt lin) | Très bas | Rapide; revalidation si A₀/B₀ changent |
| **SDRE complet** (actuel) | À profiler | Locale à chaque tick | Bas | Adapte gain au modèle A(x), sans garantie stabilité globale |
| **SDRE avec cache** | À profiler | Locale | Moyen | Hash state→K, réutilise si proches |
| **Gain scheduling** | À profiler | Regimes prédéfinis | Haut | Pré-calc K(speed_regime) |

**Recommandation court-terme** :
```
→ PRIORITÉ : Implémenter et tester d'abord un LQR fixe nominal K₀ calculé une seule fois.
  - Calculer K₀ au démarrage depuis A₀ = A(x₀) 
  - Ajouter paramètre `use_sdre=false` pour utiliser K₀
  - Garder SDRE complet comme option comparative use_sdre=true après validation
→ Profiler sur la plateforme embarquée (utiliser time.perf_counter())
→ Seuils CPU et fréquence à définir après mesure réelle
→ Valider signes, unités, repères AVANT toute optimisation Riccati
```

---

#### **P2: Signes du Damping Ambigus**

**Description** :
```python
# Dans lqr_solver.py
damping_sign = 1.0  # ou -1.0 ?
Am[6][6] = damping_sign * 1.5978...  # Drag linéaire
```

**Problème** :
- Drag est une **dissipation** (énergie → chaleur)
- Physiquement : acceleration = -drag·velocity
- Code applique : drag avec damping_sign, dont le signe exact n'est pas validé

**Ambiguïté** :
- damping_sign = +1 ou damping_sign = -1 ?
- Dépend de la convention exacte de la matrice D dans le modèle MATLAB
- Mauvais signe → instabilité ou freinage inversé

**À vérifier** :
- [ ] Quel signe correct pour l'amortissement hydrodynamique du véhicule dans le modèle ?
- [ ] D'où provient le coefficient 1.5978... (MATLAB, source, linéarisation) ?
- [ ] Est-ce une approximation linéaire ou combinaison linéaire + quadratique ?

**Test empirique requis** :
```
1. Construire A avec damping_sign = +1
2. Simuler : u_dot = A[6,6] × u (sans moteur, T_cmd = 0)
3. Observer le signe de u_dot quand u > 0
4. Si u_dot < 0 : freinage physique correct ✓
5. Si u_dot > 0 : freinage inversé, inverser damping_sign
6. Répéter avec damping_sign = -1 et comparer
7. Garder le signe qui donne u_dot < 0 (dissipation)
```

**Correction** :
1. Tester les deux signes empiriquement
2. Documenter résultat du test
3. Fixer le signe validé et le documenter clairement
4. Ajouter commentaire mathématique avec convention utilisée et référence Fossen

---

#### **P3: Convention Erreur d'État Potentiellement Inversée**

**Description** :
```python
# control_node.py
lqr_error = target_state_copy - self.current_state
```

**Ambiguïté critique** :
```
Notation standard LQR:
  δx = x - x_ref        (erreur position)
  δu = -K δx            (commande feedback)

Code utilise:
  e = target_state - current_state   (e = x_ref - x)
  T_cmd = -K e                        (applique T_cmd = -K(x_ref - x) = K(x - x_ref))

→ Signe inversé par rapport à la convention standard !
```

**Problème** :
- Si e = x_ref - x et T_cmd = -K e, alors T_cmd = K(x - x_ref)
- En LQR standard, δu = -K(x - x_ref) qui est le contraire
- À moins que K soit calculé avec une autre convention ou que Bm inclue un signe, cela peut être faux

**À vérifier obligatoirement** :
- [ ] Test axe par axe : target X = 1m, current X = 0m → error = +1m
- [ ] Vérifier que la combinaison des 8 moteurs produit un effort global dans la direction attendue
- [ ] Répéter test pour Y, Z, φ, θ, ψ
- [ ] Vérifier comportement à l'opposite (target X = 0m, current X = 1m → T_cmd global opposé)

**Correction** :
```
1. Documenter explicitement la convention utilisée
2. Test physique : publier commande connue, observer réaction
3. Si comportement inversé → ajouter signe négatif à K ou à e
4. Bm peut inclure des signes de convention; vérifier avec script MATLAB
```

**Risque** :
Commande inversée → feedback déstabilisant, vehicle refuse de converger vers cible

---

#### **P4: Point de Linéarisation et Unités de B Flous**

**Description** :
- Matrices A/B linéarisées autour de quel point x₀, T₀ ou du₀ ?
- Unités d'entrée ambiguës : Newtons ou commande normalisée du ?

**Problème connu** :
- Script MATLAB semble évaluer B autour de du_i = 1, pas nécessairement autour de du_i = 0
- Si T_i = du_i |du_i|, la dérivée autour de du_i = 0 est nulle/problématique
- Point de linéarisation et unités doivent être clairs pour LQR valide

**À vérifier** :
```
Dans script MATLAB generation_A_B.m (si existe):
  syms x u
  f = dynamics(x, u)
  A = jacobian(f, x)  % Évalué où exactement ?
  B = jacobian(f, u)  % Évalué où exactement ?
```

- [ ] Origine exacte de Bm (généré par script MATLAB ?)
- [ ] Unités entrée : Newtons (T) ou commande normalisée (du) ?
- [ ] Point de linéarisation : x₀ = [0...], T₀ = [0...], du₀ = [1...]... ?
- [ ] Cohérence Bm Python ↔ B MATLAB : coefficients concordent ?

**Correction** :
1. Localiser script MATLAB original
2. Noter explicitement : entrée T en Newtons ou du normalisé ?
3. Noter point de linéarisation exact (x₀, T₀ ou du₀)
4. Ajouter docstring dans lqr_solver.py avec math proof
5. Si entrée du : régénérer Bm avec entrées T en Newtons pour cohérence LQR
6. Valider Bm[6:12, :] = M⁻¹ B_T (si entrée Newtons)

---

### 1.2 🟡 MODÉRÉ

#### **P5: B_T vs Bm Distinction Floue**

**Description** :
- B_T (6×8) : mappe forces moteurs en Newtons → efforts généralisés (forces/torques body)
- Bm (12×8) : mappe entrée moteur → dérivée d'état complète

**Actuellement** :
```python
# lqr_solver.py
Bm = np.zeros((12, 8), dtype=np.float32)  # Hardcodée directement
Bm[6:12, :] = ...                          # Lignes 6-11 remplies (accélération vitesses)
                                            # Lignes 0-5 restent 0 (position = intégration vitesse)
```

**Relation entre Bm et B_T** dépend du type d'entrée :

```
Si entrée = forces T en Newtons :
  Bm ≈ [0_{6×8}          ]
       [M⁻¹ B_T]

Si entrée = commande normalisée du :
  Bm ≈ [0_{6×8}                 ]
       [M⁻¹ B_T ∂T/∂du]
```

**À vérifier** :
- [ ] Lignes Bm[0:6, :] correctement nulles (position = intégration vitesse) ?
- [ ] Valeurs Bm[6:12, :] concordent avec M⁻¹ B_T si entrée en Newtons ?
- [ ] Unités cohérentes avec ce que compute_thrust_force() retourne ?
- [ ] Bm correspond-elle réellement à une entrée Newtons ou à commande du ?

---

#### **P6: Conversions ENU↔NED Inconsistentes**

**Description** :
```python
# control_node.py (odometry → NED)
current_state[0] = msg.pose.pose.position.y     # X_NED ← Y_ENU

# vs control_node.py (action callback → NED)
self.target_state[0] = target_pose_in_map.pose.position.y  # Idem

# vs lqr_solver.py
# Assume entrée NED (aucune conversion interne)
```

**Risque** :
- Si un endroit oublie conversion, erreur 180° dans yaw par exemple

**Correction** :
```
1. Tests unitaires conversion:
   test_enu_to_ned_position()
   test_enu_to_ned_orientation()
2. Invariant: LQR toujours reçoit NED/FRD
3. Documenter chaque input: "ENU" ou "NED" en commentaire
```

---

#### **P7: Linéarisation à État Courant vs Point Fixe**

**Description** :
- Matrice A recalculée à chaque tick avec état courant
- → Approche de type SDRE / LQR à gain variable

**Mais** :
- État change à chaque tick (mouvement du véhicule)
- A(x(t)) et A(x(t+dt)) peuvent différer significativement
- Riccati solution X peut changer rapidement

**Risques** :
- Instabilité si A change trop vite
- Gains K(t) non lissés peuvent causer oscillations
- Aucune garantie de stabilité globale

**À tester** :
```
1. Logger norm(A(t) - A(t-1)) chaque tick
2. Identifier si gain K change abruptement
3. Analyser stabilité avec changements rapides de A
4. Proposer lissage si instable : A_smooth = α·A + (1-α)·A_prev
```

**Décision** :
Cette approche ne doit être conservée que si elle apporte un gain mesurable par rapport au LQR fixe K₀

---

### 1.3  🟢 DOCUMENTATION & DESIGN

#### **P8: Pas de Trim / Feedforward**

**Description** :
```
Actuellement selon le code analysé : T_cmd semble calculé à partir de -K·e,
avec e = x_ref - x à confirmer par inspection du code.

Idéal:       T_cmd = T_trim - K(x - x_ref)
             où T_trim compense offset statique/dynamique
```

**Exemple problème** :
- Sub veut station-keep à Z = 2m (immergé)
- Gravité ≠ Flottabilité exactement
- LQR seul → creeping vertical lent

**Approche : Trim**

Le trim doit satisfaire une condition d'équilibre :
```
f(x_trim, T_trim) = 0   ⇔ ẋ_trim = 0 pour un état cible donné
```

**Solution future** :
```
Le trim/feedforward doit être calculé pour maintenir x_ref stationnaire.

Approximation linéaire :
  T_cmd = T_trim - K(x - x_ref)
  
  ou si le code utilise e = x_ref - x :
  
  T_cmd = T_trim + K e
  
  où T_trim satisfait : A·x_ref + Bm·T_trim ≈ 0 (x_ref en état stationnaire)

Implémenter via pseudo-inverse ou optimisation sous contraintes :
  T_ff = Bm⁺ (-A x_ref)      [pseudo-inverse]
  
  ou
  
  T_ff = argmin_T ||Bm T + A x_ref||  with motor saturations
```

**Note** : Ne pas utiliser B⁻¹ car Bm n'est pas carrée (12×8)

---

#### **P9: Pas d'Intégrateur**

**Description** :
- LQR proportional feedback seul
- Erreur DC persistent → jamais éliminée complètement

**Exemple** :
- Courant océan constant 0.1 m/s vers Est
- LQR converge → creeping résiduel

**Solution future** :
```
Ajouter e_I pour certains axes seulement (ex: Z et yaw, pas tous).

x_aug = [x; e_I]  où e_I = [∫e_Z dt, ∫e_ψ dt]ᵀ

Convention (si e = x_ref - x) :
  T_cmd = T_trim + K_x e + K_I e_I
  
Convention (si code utilise e = x - x_ref) :
  T_cmd = T_trim - K_x e - K_I e_I
```

**Important** : Ne pas intégrer automatiquement les 12 états; choisir sélectivement ceux qui ont erreur DC résiduelle

---

#### **P10: Pas d'Anti-Windup**

**Description** :
- Anti-windup nécessaire seulement si un intégrateur est ajouté (voir P9)
- Si T_cmd clipé (max 40N), intégrateur continue → windup

**Solution future** (si intégrateur implémenté) :
```
Si |T_cmd| > T_max:
    back_calculate e_I pour que T_cmd = T_max
    → Ramène l'intégrateur en arrière-plan
```

**À court terme** : Priorité au clipping propre + rate limiter moteur

---

#### **P0c: Absence de Rate Limiter Moteur**

**Description** :
- Clipping limite l'amplitude (±40 N), mais pas la variation rapide
- Changements abrupts T_cmd → oscillations, usure moteur, réponse instable

**Problème** :
```
T_cmd change peut sauter de -40 à +40 N en 1 tick
→ Moteur doit accepter dT/dt extrême
→ ESC/moteur peut non répondre ou osciller
```

**Solution future** :
```
Ajouter slew-rate limiter :
  T_cmd_limited[i] = clip(T_cmd[i], T_cmd_prev[i] - max_rate*dt, T_cmd_prev[i] + max_rate*dt)
  
  Exemple de plage initiale à tuner expérimentalement (ne pas utiliser sans validation moteur/ESC).
  Valeur typique initiale : 10-50 N/s à adapter selon réponse moteur/ESC.
```

---

#### **P11: Tolerances is_target_reached() Hardcodées**

**Description** :
```python
def is_target_reached(self):
    pos_tolerance = 0.5  # meters
    angle_tolerance = math.radians(10)  # degrees
```

**Problème** :
- Fixées pour toute mission
- Piscine ≠ mer : tolerances différentes

**Solution** :
```python
# Paramètre dynamique
self.declare_parameter('pos_tolerance_m', 0.5)
self.declare_parameter('angle_tolerance_deg', 10.0)
```

---

## 2. Plan d'Action Détaillé

### Phase 1: Validation Prioritaire (2-3 jours)

**Objectif** : Valider fondamentaux avant toute optimisation Riccati

| ID | Tâche | Fichier | Test | Acceptance |
|----|-------|---------|------|-----------|
| **1.0** | Vérifier unités ThrusterCommand | control_node.py + actuator_node.py | Tracer mode MANUAL vs LQR | Unités clarifiées (Newtons vs throttle) |
| **1.0b** | Valider Bm MATLAB/Python | lqr_solver.py | Localiser generation_A_B.m, comparer coefficients | Cohérence confirmée ou Bm régénérée |
| **1.1** | Tester convention signe erreur LQR | control_node.py | Unit test + step response | Erreur positive → T_cmd correct (vers cible) |
| **1.2** | Valider damping_sign empiriquement | lqr_solver.py | Step response u > 0 sans moteur | Observe u_dot < 0 (freinage) ✓ |
| **1.3** | Tester conversions ENU↔NED | test/ (nouveau) | Unit + integration tests | [1,0,0]_ENU → [0,1,0]_NED ✓ à confirmer selon convention exacte utilisée dans control_node.py |
| **1.4** | Profiler CPU Riccati | control_node.py | Ajouter time.perf_counter() | Coût mesuré, seuils déterminés |
| **1.5** | Comparer LQR fixe vs SDRE | — | Implémenter K₀ offline, comparer | LQR fixe baseline établie |

---

### Phase 2: Corrections Critiques (3-5 jours)

**Objectif** : Fixer P0, P0b, P1-P4 et implémenter LQR fixe

| ID | Tâche | Fichier | Action | Vérifier |
|----|-------|---------|--------|----------|
| **2.0** | Standardiser ThrusterCommand | msg/interfaces | Décider : Newtons partout ou deux messages | Ambiguïté éliminée |
| **2.0b** | Régénérer/valider Bm | lqr_solver.py | Comparer avec MATLAB ou régénérer | Cohérence confirmée |
| **2.1** | Implémenter LQR fixe K₀ | lqr_solver.py | Calculer K₀ offline, ajouter paramètre use_sdre | K₀ stable baseline |
| **2.2** | Documenter point de linéarisation | lqr_solver.py | Ajouter docstring x₀, T₀/du₀, A/B formules | Mathématiquement rigoureux |
| **2.3** | Fixer damping_sign validé | lqr_solver.py | Fixer le signe validé par test, le documenter, et ajouter un test unitaire | Comportement déterministe |
| **2.4** | Clarifier convention erreur | control_node.py | Ajouter bloc détaillé + test d'axe | 100% cohérent avec code |
| **2.5** | Implémenter fallback Riccati | lqr_solver.py | Si solve_continuous_are échoue → utiliser K₀ | Robustesse accrue |

---

### Phase 3: Amélioration Robustesse (1-2 semaines)

**Objectif** : Ajouter rate limiter, trim/feedforward, intégrateur

| ID | Tâche | Fichier | Effort | Gain |
|----|-------|---------|--------|------|
| **3.0** | Implémenter rate limiter moteur | control_node.py | 1-2h | Commandes plus progressives, moins d'oscillation |
| **3.1** | Implémenter feedforward/trim optionnel | control_node.py | 2-4h | Réduit creep statique (Z, yaw) |
| **3.2** | Implémenter intégrateur partiel (Z, ψ) | control_node.py | 3-5h | Erreur DC → 0 sur axes critiques |
| **3.3** | Ajouter anti-windup (si 3.2) | control_node.py | 1-2h | Prévient saturation oscillations |
| **3.4** | Tolérances paramétriques | control_node.py | 1h | Flexible par mission |
| **3.5** | Tests piscine complets | — | 1 jour | Validation empirique, tuning Q/R |

---

### Phase 4: Documentation (1-2 jours)

**Objectif** : README complets pour futur mainteneur

| Fichier | Contenu |
|---------|---------|
| CONTROL.md | Flux complet + diagrams |
| TUNING.md | Comment ajuster Q/R/tolerances |
| TROUBLESHOOTING.md | Common issues + fixes |

---

## 3. Checklist de Validation (Par Correction)

### ✅ Après P1 (CPU profile)

- [ ] Script profiling créé (`lqr_solver_profile.py`)
- [ ] Résultats loggés dans fichier CSV
- [ ] Coût CPU mesuré pour ARE solve
- [ ] Seuils de latence à définir après mesure selon fréquence de contrôle visée

### ✅ Après P2 (damping_sign)

- [ ] Signe correct documenté avec référence MATLAB
- [ ] Test unitaire : `test_drag_direction()` passe
- [ ] Vérification : v↑ → thrust needed ↓ (freinage) ✓

### ✅ Après P3 (error convention)

- [ ] Bloc commentaires ajouté (25+ lignes)
- [ ] Test : `test_error_direction()` passe
- [ ] Example: target +1m, current 0 → error[i] signe correct

### ✅ Après P4 (linéarisation)

- [ ] Script MATLAB `generation_A_B.m` trouvé
- [ ] x₀ et T₀ ou du₀ explicites en docstring
- [ ] Matrice imprimée: `print(A0)`, `print(B0)`
- [ ] Concordance Bm ↔ B0[6:12, :] vérifiée

### ✅ Après P5-P6-P7 (matrices & conversions)

- [ ] Unit tests `test_enu_ned_*.py` all pass
- [ ] Invariant: LQR input = NED/FRD everywhere
- [ ] A(x) norm stability logged chaque sec
- [ ] No NaN ou Inf dans matrices

---

## 4. Risques & Mitigations

| Risque | Probabilité | Impact | Mitigation |
|--------|-------------|--------|-----------|
| **P0 : ThrusterCommand unités ambiguës** | Haute | Critique | Standardiser en Newtons partout ou créer messages distincts |
| **P0b : Bm incoherent** | Haute | Critique | Localiser MATLAB, valider unités, régénérer si besoin |
| **Riccati solver fail** | Basse | Critique | Try-except, fallback K₀ |
| **Signe erreur inversé** | Moyenne | Critique | Test axe par axe obligatoire |
| **damping_sign faux** | Moyenne | Critique | Test empirique u_dot avec u>0 sans moteur |
| **ENU↔NED confusion → yaw 180°** | Moyenne | Modérée | Unit test comprehensive |
| **Rate limiter absent → oscillations** | Moyenne | Modérée | Implémenter slew-rate limiter |
| **CPU spike → odometry latency** | Basse | Modérée | Profiler, LQR fixe K₀ comme baseline |
| **Riccati A change rapidement** | Basse | Modérée | Logger norm(A(t)-A(t-1)), proposer lissage |

---

## 5. Dépendances Externes

| Ressource | Status | Action |
|-----------|--------|--------|
| Script MATLAB `generation_A_B.m` | À localiser | Chercher dans Control_Future/ ou archives |
| Handbook Fossen (hydrodynamics) | ✓ Disponible | Référence pour P2 (damping), P4 (linéarisation) |
| Table T200 force-throttle | ✓ Présente dans actuator_node.py | Cohérence avec tension, ESC et configuration réelle à vérifier |
| Plateforme embarquée (Jetson) et marge CPU | À profiler | Mesurer coût réel Riccati après Phase 1 |

---

## À Vérifier

- [ ] Tous les problèmes P0-P11 représentatifs du code actuel ?
- [ ] Ordre priorité (P0/P0b puis P1→P4) aligné avec équipe ?
- [ ] Tests acceptation clairs et mesurables ?
- [ ] Resources (temps, personnel) estimées réalistes ?
- [ ] Dépendances externes disponibles ?

---

## Décisions Importantes

1. **Priorité : valider LQR fixe nominal K₀ d'abord**
   - Calculer K₀ offline au démarrage depuis A₀ = A(x₀)
   - Ajouter paramètre `use_sdre=false` pour utiliser K₀
   - Garder SDRE comme option comparative `use_sdre=true` après validation
   - Établir baseline de performance et stabilité
   
2. **Implémenter fallback K₀ si Riccati échoue**
   - SDRE complet ne doit être conservé que s'il apporte gain mesurable
   - Sans validation de signes/unités/Bm, SDRE risqué

3. **Tests unitaires AVANT modifications code**
   - Chaque correction (signe erreur, damping, conversions) nécessite test physique/unitaire
   - Phased rollout avec bench test d'abord

4. **Documenter math explicitement**
   - Chaque matrice (A, B, Bm) = équation + point de linéarisation + références Fossen
   - Unités d'entrée et sorties claires

5. **Valider unités ThrusterCommand et Bm dès Phase 1**
   - Plus critique que profiling Riccati
   - Fondation pour tout le reste

6. **Piscine tests requis**
   - Validation empirique critique post-correction Phase 2
   - Tuning Q/R et seuils acceptation en Phase 3

---

**⚠️ Mise à jour requise** : Ce document doit être mis à jour après les tests Phase 1, surtout après validation de ThrusterCommand, Bm, damping_sign et signe LQR.
