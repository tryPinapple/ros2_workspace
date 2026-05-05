# 02_ARCHITECTURE_CONTROLE_LQR_SDRE.md

## Objectif du Document

Décrire précisément l'implémentation LQR/SDRE actuelle du sous-marin : les modèles dynamiques, les matrices, la stratégie de contrôle et les choix mathématiques sous-jacents.

---

## 1. Modèle Dynamique Non-Linéaire

### Équation d'État Générale

Le système sous-marin est décrit par :

```
ẋ = f(x, u)
```

avec le vecteur d'état et de commande :

```
x = [X, Y, Z, φ, θ, ψ, u, v, w, p, q, r]ᵀ  ∈ ℝ¹²

du = [du₀, du₁, du₂, du₃, du₄, du₅, du₆, du₇]ᵀ  ∈ ℝ⁸

ou équivalemment :

T = [T₁, T₂, T₃, T₄, T₅, T₆, T₇, T₈]ᵀ  ∈ ℝ⁸  (poussées moteurs)
```

**Note** : Les 8 composantes représentent soit les commandes normalisées du moteur `du_i`, soit les forces moteurs `T_i` selon le point de linéarisation choisi dans MATLAB.

### Cinématique : Équations 0-5 de f(x,u)

Les vitesses world-frame (X_dot, Y_dot, Z_dot) proviennent de la rotation des vitesses body-frame :

```
η̇ = J(η) ν
```

où :
- η = [X, Y, Z, φ, θ, ψ]ᵀ (position + attitude)
- ν = [u, v, w, p, q, r]ᵀ (vitesses body)
- J(η) ∈ ℝ^{6×6} est la matrice de rotation, structurée en blocs :

```
J(η) = [ R(φ,θ,ψ)   0₃ₓ₃  ]
       [ 0₃ₓ₃       T(φ,θ) ]
```

où R(φ,θ,ψ) est la matrice de rotation (ZYX Euler), et T(φ,θ) relie les vitesses angulaires body aux dérivées d'angles.

### Cinétique : Équations 6-11 de f(x,u)

Les accélérations body-frame sont gouvernées par :

```
ν̇ = M⁻¹ (B_T T - C(ν)ν + D(ν)ν - G(η))
```

où :

| Terme | Description | Dimension | Notes |
|-------|-------------|-----------|-------|
| **M** | Matrice d'inertie (RB + added mass) | 6×6 | Symétrique, définie positive |
| **B_T T** | Efforts moteurs généralisés | 6×1 | τ = B_T · T (où T = forces moteurs) |
| **C(ν)** | Matrice Coriolis-centrifuge | 6×6 | État-dépendante (non-linéaire) |
| **D(ν)** | Matrice de damping hydrodynamique | 6×6 | Signes inclus ou non selon convention (à vérifier) |
| **G(η)** | Vecteur de restoring (gravité/flottabilité) | 6×1 | État-dépendant via angles |

---

## 2. Matrices Physiques

### Inertie M (6×6)

```
M = M_RB + M_A

M_RB = diag(m, m, m, I_x, I_y, I_z)   [Corps rigide]
M_A  = matrice added mass (couplages possibles)
```

**Valeurs ASUQTR** (à confirmer dans MATLAB) :
- Masse et moments d'inertie : à confirmer dans les paramètres mécaniques ou script MATLAB
- **Note** : Ne pas déduire la masse depuis Bm sans preuve mathématique claire

### Matrice Coriolis C(ν) (6×6)

Terme standard pour rotation rigide + inertie ajoutée :

```
C(ν) = [ 0₃ₓ₃      -S(M₁₁ν₁ + M₁₂ν₂)  ]
       [ (symétrique)    -S(M₂₁ν₁ + M₂₂ν₂) ]
```

où S(·) est la matrice antisymétrique (produit vectoriel).

### Matrice Damping D(ν) (6×6)

**Convention actuelle du code** : À vérifier avec le script MATLAB

Notre implémentation semble utiliser :
```
ν̇ = M⁻¹(τ - C(ν)ν + D(ν)ν - G(η))
```

où D(ν) contient des signes qui doivent amortir le mouvement. Cependant, la convention exacte (si les signes négatifs sont inclus dans D ou appliqués avant) doit être confirmée avec le script MATLAB original.

**Forme standard en théorie marine** :
```
ν̇ = M⁻¹(τ - C(ν)ν - D(ν)ν - G(η))
```
où D est définie positive.

| Composante | Drag Linéaire | Drag Quadratique | Total |
|-----------|--------------|-----------------|-------|
| **u (Surge)** | 1.598 | ρ·A·C_d·u·\|u\| | D[0,0] |
| **v (Sway)** | 2.006 | ρ·A·C_d·v·\|v\| | D[1,1] |
| **w (Heave)** | 3.032 | ρ·A·C_d·w·\|w\| | D[2,2] |
| **p (Roll rate)** | 4.819 | I_x·c_p·p·\|p\| | D[3,3] |
| **q (Pitch rate)** | 4.319 | I_y·c_q·q·\|q\| | D[4,4] |
| **r (Yaw rate)** | 2.973 | I_z·c_r·r·\|r\| | D[5,5] |

**Valeurs extraites de lqr_solver.py** (diagonal terms) :

```python
Am[6][6] = damping_sign * 1.5978417266187050359712230215827  # u
Am[7][7] = damping_sign * 2.005755395683453237410071942446   # v
Am[8][8] = damping_sign * 3.0316546762589928057553956834532  # w
Am[9][9] = damping_sign * 4.8191481416942570511065196285878  # p
Am[10][10] = damping_sign * 4.3185544587585745357202610005019 # q
Am[11][11] = damping_sign * 2.9727509347911212118885467933018 # r
```

### Vecteur Restoring G(η) (6×1)

Gravité + flottabilité (dépend des angles) :

```
G(η) = [  (W - B) sin(θ)          ]
       [  -(W - B) sin(φ) cos(θ)  ]
       [  -(W - B) cos(φ) cos(θ)  ]
       [  (y_b·B - y_g·W) cos(φ) cos(θ) + ... ]
       [  (z_b·B - z_g·W) sin(φ) + ... ]
       [  ... ]
```

où W = poids (N), B = buoyancy (N), calculés depuis position du centre de mass et du centre de flottabilité.

---

## 3. Matrice d'Allocation des Moteurs B_T (6×8)

Génération depuis MATLAB basée sur **position et direction de chaque thruster** :

Pour chaque moteur i avec position **r_i** et direction **d_i** (vecteur unitaire) :

```
b_{T,i} = [ d_i           ]  ∈ ℝ⁶
          [ r_i × d_i     ]
```

**Matrice complète** :

```
B_T = [b_{T,1} | b_{T,2} | ... | b_{T,8}]  ∈ ℝ⁶ˣ⁸
```

Relation efforts généralisés ← poussées moteurs :

```
τ = B_T · T    où τ ∈ ℝ⁶, T ∈ ℝ⁸
```

Pour obtenir les poussées depuis les efforts généralisés (allocation inverse) :
```
T_cmd = B_T⁻ᵍ τ_cmd
```
(pseudoinverse, car B_T n'est pas carrée)

**ASUQTR configuration** (à confirmer avec géométrie réelle et câblage moteur) : 8 moteurs T200
- Moteurs 0-1, 6-7 : Hypothèse horizontaux (surge/yaw)
- Moteurs 2-5 : Hypothèse verticaux (heave/roll/pitch)

---

## 4. Passage du Modèle Non-Linéaire au LQR

### Dynamique MATLAB

Le script MATLAB modélise la poussée moteur par :

```
T_i = du_i · |du_i|   ∀ i ∈ [0, 7]
```

où `du_i ∈ ℝ` est la commande normalisée (sans lien direct aux Newtons du LQR).

Dynamique complète :

```
ẋ = f(x, du) = [ J(η)ν
                 M⁻¹ (B_T T(du) - C(ν)ν + D(ν)ν - G(η)) ]
```

### Linéarisation par Jacobien

**Matrice A** (12×12) : Jacobienne de f par rapport à x

```
A = ∂f/∂x |_(x₀, du₀)
```

**Matrice B** (12×8) : Jacobienne de f par rapport à du

```
B = ∂f/∂du |_(x₀, du₀)
```

**Point de linéarisation** :
- **À confirmer** : Le script MATLAB actuel semble évaluer B autour de du_i = 1 plutôt que du_i = 0.
- **Raison** : Si T_i = du_i |du_i|, la dérivée ∂T_i/∂du_i = 2·du_i. À du_i = 0, cette dérivée est nulle, ce qui rend du_i = 0 un mauvais point de linéarisation.
- **Important** : Confirmer le point exact (x₀, du₀) dans le script MATLAB original pour valider la cohérence.

### Matrice Bm (12×8) dans le Code Python

Implémentée directement dans `lqr_solver.py` :

```python
Bm = np.zeros((12, 8), dtype=np.float32)

# Surge (row 6)
Bm[6][0] = -0.10172661870503597122302158273381
Bm[6][1] = -0.10172661870503597122302158273381
# ... (total 8 colonnes, 6 lignes remplies)

# Heave (row 8)
Bm[8][2:6] = -0.14388489208633093525179856115108
# ... etc
```

**Bm et B_T : distinction critique** :

- **B_T ∈ ℝ^{6×8}** : mappe les poussées moteurs → efforts généralisés
- **Bm ∈ ℝ^{12×8}** : mappe l'entrée moteur → dérivée d'état complète

**Relation entre Bm et B_T** dépend du type d'entrée :

Si l'entrée est des forces moteurs en Newtons (T en ℝ⁸) :
```
Bm ≈ [0_{6×8}          ]
     [M⁻¹ B_T]
```

Si l'entrée est des commandes normalisées (du en ℝ⁸) :
```
Bm ≈ [0_{6×8}                 ]
     [M⁻¹ B_T ∂T/∂du]
```

où ∂T/∂du dépend de la relation T_i = f(du_i) (e.g., quadratique).

**Dans le pipeline ROS actuel** : Les 8 sorties du LQR sont interprétées comme des efforts moteurs par `actuator_node.py`. À vérifier : Bm correspond-elle réellement à une entrée en Newtons ou à une commande normalisée du_i ?

### Matrice A(x, t) State-Dependent

Dans `lqr_solver.update_system_dynamics_matrix_A()` :

```python
def update_system_dynamics_matrix_A(Am, state):
    # Rows 0-5 : Cinématique (dépend de φ, θ, ψ, u, v, w)
    # Rows 6-11 : Cinétique (dépend de z, φ, θ, u, v, w, p, q, r)
    # Retourne Am remplie
```

**État-dépendance clé** :
- Termes de Coriolis (cross-coupling u-w, v-w, etc.)
- Drag non-linéaire (D(ν) ~= damping_sign × const pour linéarisation)
- Restoring (termes sin/cos des angles)

---

## 5. Stratégie de Contrôle : LQR vs SDRE

### LQR Fixe (Linear Quadratic Regulator)

```
K = R⁻¹ Bₘᵀ X

où X résout l'Équation Algébrique de Riccati :
  AᵀX + XA - XBₘR⁻¹BₘᵀX + Q = 0
```

**Avantages** :
- Gain K calculé une seule fois offline
- Pas de CPU à runtime

**Inconvénients** :
- Assume modèle linéaire valide partout
- Perd optimalité si trajectoires s'éloignent du point de lin

### SDRE (State-Dependent Riccati Equation)

**Approche actuelle du code ASUQTR** (avec mise en garde) :

```
À chaque callback de contrôle/odométrie (fréquence à confirmer) :
  1. Récupérer état x(t) courant
  2. Calculer A(x(t)) depuis update_system_dynamics_matrix_A()
  3. Résoudre scipy.linalg.solve_continuous_are(A, Bm, Q, R)
  4. Calculer K(t) = R⁻¹ Bₘᵀ X(t)
  5. Appliquer u(t) = -K(t) · e(t)
```

**Avantages** :
- Adapte le gain K au modèle local A(x)

**Inconvénients et Limites** :
- Coût CPU à profiler (pas de valeurs numériques sans benchmark)
- Ne garantit pas optimalité globale ni stabilité globale (contrairement au LQR fixe)
- Peut causer instabilité si A(x) change trop vite ou si le solver ARE échoue

**Recommandation** : Pour les premiers essais, privilégier un **LQR fixe** :
```
A0 = A(x0), B0 = Bm, K0 calculé une seule fois offline.
Recalculer Riccati seulement si Q/R changent ou si on choisit volontairement SDRE avec cache.
```

**Raison** : LQR fixe donne un comportement plus prévisible et une stabilité locale attendue **si A0/B0 sont corrects** et si le système réel reste dans la zone de validité de la linéarisation. Moins de CPU, comportement plus transparent pour le debug.

---

## 6. Rôle des Matrices Q et R

### Q Matrix (12×12, diagonal) : Pénalité d'Erreur d'État

```
Q = diag(q₀, q₁, ..., q₁₁)
```

**Interprétation physique** :

| Indices | Variable | Rôle | Unités | Tuning |
|---------|----------|------|--------|--------|
| 0-2 | Position (X, Y, Z) | Ressort : attire vers cible | m⁻² | Augmenter = converger plus vite |
| 3-5 | Angles (φ, θ, ψ) | Ressort torsionnel : snappe heading | rad⁻² | Augmenter = heading plus rigide |
| 6-8 | Vitesses linéaires (u, v, w) | Amortisseur : frein overshoot | (m/s)⁻² | Augmenter = freinage plus agressif |
| 9-11 | Vitesses angulaires (p, q, r) | Amortisseur : frein rotation | (rad/s)⁻² | Augmenter = rotation moins oscillante |

**Valeurs ASUQTR actuelles** (à confirmer dans le fichier de paramètres utilisé au runtime) :

```
state_cost_matrix: [
  1052.4762,  578.7808,  670.0731,     # Position X, Y, Z
  3134.6757, 3959.9523, 2453.9726,     # Angles φ, θ, ψ
  265.1124,  303.5306,  204.2912,      # Vit lin u, v, w
  12.3207,   19.8116,   12.2442        # Vit ang p, q, r
]
```

**Note** : Les valeurs par défaut peuvent être dans params.yaml, mais le runtime peut utiliser un autre fichier ou override ROS2. À vérifier avec `ros2 param get`.

### R Matrix (8×8, diagonal) : Pénalité d'Effort Moteur

```
R = diag(r₀, r₁, ..., r₇)
```

**Interprétation physique** : "Coût de la batterie"

```
r_i élevé  → "Épargner moteur i" → moins de thrust
r_i bas    → "Utiliser moteur i" → plus de thrust
```

**Valeurs ASUQTR actuelles** :

```
thruster_cost_matrix: [15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0]
```

Tous égaux → Pas de préférence pour un moteur vs autre.

### Relation Approximative K ∝ √(Q/R)

Règle utile pour tuning :

```
Thrust ≈ K · erreur ≈ √(Q/R) · erreur

Exemples :
  1m position error, Q=1, R=1   →  ~1.0 N
  1m position error, Q=100, R=1 →  ~10 N
  1m position error, Q=1, R=100 →  ~0.1 N
```

---

## 7. Feedforward, Trim, Intégrateur, Anti-Windup

### ❌ Feedforward (Pas implémenté)

Ajout prédictif avant feedback :

```
u = u_ff(x_ref, ẋ_ref) - K · e
```

**Bénéfice** : Réaction plus rapide pour trajectoires connues

**Implémentation future** : Ajouter dans `control_node.py` si missions = patterns répétables

---

### ❌ Trim (Pas implémenté)

Le trim doit satisfaire une condition d'équilibre :

```
f(x_trim, T_trim) = 0
```

c.à.d. ẋ_trim = 0 pour un état cible donné.

Si approximation linéaire autour d'un point de lin :
```
0 ≈ A·x_trim + Bm·T_trim
T_trim ≈ -Bm⁻ᵍ A·x_trim  (pseudoinverse)
```

Ou par optimisation sous contraintes :
```
T_trim = argmin ||τ_cmd - B_T T|| avec saturations moteurs
```

**Bénéfice** : Compense offset statique (gravité/flottabilité) pour maintien en position sans intégrateur

**Implémentation future** : Pré-calculer T_trim pour missions répétables

---

### ❌ Intégrateur (Pas implémenté)

Ajout sélectif d'intégrateurs pour réduire erreur DC persistent :

```
x_aug = [x; e_I]

où e_I contient seulement les erreurs à intégrer (ex. e_Z, e_yaw).
Ne pas intégrer automatiquement les 12 états.

K_aug = [K_state | K_integral]
T_cmd = T_trim - K_x(x - x_ref) - K_I · e_I
```

Où T_trim est le trim de base et K_I affine l'intégration pour chaque DOF sélectionné.

**Bénéfice** : Converge vers zéro même avec disturbances constantes (e.g., courant, dérive)

**Important** : L'intégrateur doit être accompagné d'anti-windup si jamais implémenté.

---

### ❌ Anti-Windup (Implémentation conditionnelle)

L'anti-windup est nécessaire **seulement si un intégrateur est ajouté**.

**À court terme** : Les problèmes principaux sont plutôt :
- Saturation des sorties (clipping à ±40N)
- Rate limiting des moteurs (accélération PWM limitée)
- Asservissement pneumatique (délai de réponse)

Solution : Ajouter saturation propre + rate limiter sur les forces.

**À long terme**, si intégrateur ajouté :
```
Si |u_total| > u_max :
  e_I ← e_I - (u_total - saturate(u_total))
  → Ramène l'intégrateur en arrière-plan
```

**Bénéfice** : Évite windup oscillations après saturation physique

---

## 8. Paramètre damping_sign

### Problème et Validation

Dans `lqr_solver.py` :

```python
self.damping_sign = 1.0  # ou -1.0 ?
Am[6][6] = self.damping_sign * 1.5978...
```

**Validation empirique requise** (ne pas conclure sans test) :

Procédure de test :
```
1. Construire A avec damping_sign = +1
2. Simuler : u_dot = A[6,6] × u (sans moteur, T_cmd = 0)
3. Observer le signe de u_dot quand u > 0
4. Si u_dot < 0 : freinage physique correct ✓
5. Répéter avec damping_sign = -1 et comparer
6. Garder le signe qui donne le comportement attendu
```

**Important** : Le signe correct dépend de la convention MATLAB utilisée dans le modèle original. Ne pas supposer.

---

## 9. Niveau de Confiance des Informations

### ✅ Confirmé par le Code

- Le modèle utilise 12 états (position + attitude + vitesses)
- Le contrôleur utilise Q diagonal 12×12 et R diagonal 8×8
- Le solver utilise Bm matrice 12×8 codée en dur
- Les sorties LQR sont envoyées à `actuator_node.py` comme 8 commandes moteurs
- Riccati est résolu à chaque callback (fréquence exacte à confirmer)

### ⚠️ Supposé (À Valider)

- Les 8 commandes moteurs du LQR sont des forces en Newtons
- Bm représente l'effet de forces moteurs en Newtons
- Point de linéarisation : x₀ = équilibre, mais du₀ pourrait être ≠ 0
- Les signes NED/FRD sont cohérents partout
- damping_sign = +1 est physiquement correct (freinage)
- Convention damping : signes négatifs inclus ou non dans D

### ❓ À Vérifier d'Urgence

- Origine exacte de Bm (généré par script MATLAB ?)
- Cohérence Bm Python vs B généré par MATLAB
- Point exact de linéarisation (du₀ = 0 ou du₀ = 1 ?)
- Signe du damping (test physique : u > 0 sans moteur → u_dot < 0 ?)
- Signe de l'erreur : e = x_ref - x ou x - x_ref ?
- Conversions ENU↔NED réellement correctes ?
- Fréquence réelle de Riccati (50 Hz ou autre ?)

---

## 10. Recommandation Principale

**Pour les premiers essais du système :** Privilégier un **LQR fixe nominal** plutôt que SDRE complet.

```
Approche recommandée :

1. Choisir point de lin : x₀, du₀ (à valider depuis MATLAB)
2. Calculer A₀ = A(x₀) et utiliser Bm
3. Calculer K₀ une seule fois en offline
4. Appliquer en boucle fermée : T_cmd = -K₀ · e
5. Mesurer performance et CPU
6. Si nécessaire seulement : recalc Riccati avec cache ou SDRE partiel
```

**Raison** : LQR fixe donne un comportement plus prévisible et une stabilité locale attendue **si A₀/B0 sont corrects**. Zéro CPU à runtime. SDRE complexité + CPU non justifiés avant validation complète de A/B/Bm.

---

## À Vérifier (Checklist Prioritaire)

- [ ] Script MATLAB original : point de linéarisation exact (x₀, du₀)
- [ ] Cohérence Bm Python ↔ B MATLAB : valeurs concordent-elles ?
- [ ] Unités de Bm : Newtons ou throttle normalisé du_i ?
- [ ] damping_sign : test simple de freinage (u > 0 sans moteur → u_dot < 0)
- [ ] Conversions ENU↔NED : tests unitaires pour chaque transformation
- [ ] Signe erreur : e = x_ref - x validé par comportement attendu
- [ ] Fréquence réelle Riccati : profiler le système en operation
- [ ] Matrice M : inertie symétrique et définie positive ?
- [ ] C(ν) : termes Coriolis calculés correctement ?
- [ ] G(η) : poids/buoyancy/centres de masse corrects ?

---

## Décisions Importantes (État Actuel vs À Décider)

1. **État actuel** : Riccati recalculé à chaque callback (SDRE complet)
   **Recommandation** : Démarrer avec LQR fixe K₀, puis SDRE seulement si validé + besoin CPU acceptable

2. **État actuel** : Point de linéarisation (x₀, du₀) supposé
   **À faire** : Confirmer depuis script MATLAB original

3. **État actuel** : damping_sign = +1.0 paramètre dynamique
   **À faire** : Valider physiquement, puis hardcoder signe correct

4. **État actuel** : Pas de trim/feedforward/intégrateur/anti-windup
   **À envisager** : Ajouter trim et intégrateur pour robustesse (phased)

5. **État actuel** : Clipping software à ±40N
   **Recommandation** : Garder ce choix, ajouter aussi rate limiter si besoin

---

**⚠️ Mise à jour requise** : Ce README doit être mis à jour après vérification de Bm, du point de linéarisation MATLAB et des tests de signe damping/repères.
