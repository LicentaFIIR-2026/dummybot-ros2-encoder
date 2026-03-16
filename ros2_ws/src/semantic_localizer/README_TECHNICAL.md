# Semantic Localizer — Documentație Tehnică
## SAIM Xplorer / FIIR — Pas 1: Fuziune Camera-LiDAR

---

## 1. Analiza Geometriei Senzorilor

### 1.1 Camera (camera_link → camera_link_optical)

| Parametru | Valoare | Sursă |
|-----------|---------|-------|
| Rezoluție | 640 × 480 | CameraInfo |
| fx (focal length X) | 687.54 px | K[0] |
| fy (focal length Y) | 688.97 px | K[4] |
| cx (centrul optic X) | 308.11 px | K[2] |
| cy (centrul optic Y) | 232.75 px | K[5] |
| Distortion model | plumb_bob | CameraInfo |
| FOV orizontal | ±24.9° (total ~49.8°) | 2×atan(320/687.54) |
| FOV vertical | ±19.3° (total ~38.6°) | 2×atan(240/688.97) |
| Poziție vs base_link | x=0.228m, y=0.0, z=0.095m | TF static |
| Rotație vs base_link | Identitate (0,0,0) | TF static |

**Observație:** Camera privește direct înainte pe axa +X a robotului. Nu există rotație relativă față de base_link.

### 1.2 LiDAR 2D (base_laser)

| Parametru | Valoare | Sursă |
|-----------|---------|-------|
| angle_min | 0.0 rad (0°) | /scan |
| angle_max | 6.2832 rad (360°) | /scan |
| angle_increment | 0.01249 rad (0.716°) | /scan |
| Număr raze | ~503 | 6.2832/0.01249 |
| range_min | 0.02 m | /scan |
| range_max | 25.0 m | /scan |
| scan_time | 100.4 ms (~10 Hz) | /scan |
| Poziție vs base_link | x=0.222m, y=0.0, z=0.160m | TF static |
| Rotație vs base_link | Identitate (0,0,0) | TF static |

### 1.3 Relația geometrică Camera ↔ LiDAR

```
          Privit de sus (planul XY al robotului)
          
          +X (înainte)
           ↑
           |
    LiDAR ●─── z=0.160m, x=0.222m
   Camera ●─── z=0.095m, x=0.228m
           |
      ─────●───── base_link (x=0, y=0)
           |
          +Y (stânga)
```

**Offset lateral (Y):** 0.0m — perfect aliniate pe aceeași axă
**Offset longitudinal (X):** 6mm (0.228 - 0.222) — neglijabil
**Offset vertical (Z):** 65mm (0.160 - 0.095) — LiDAR deasupra camerei

**Concluzie:** Senzorii sunt practic co-locați din perspectiva proiecției orizontale.
Nu e nevoie de compensare de paralaxă semnificativă.

### 1.4 Suprapunerea FOV

Camera FOV: ±24.9° de la axa +X
LiDAR: scanează 360° complet, cu 0° = +X (forward)

**Raze LiDAR corespunzătoare FOV camera:**
- Stânga camerei: razele 0 → ~35 (0° → +25°)
- Dreapta camerei: razele ~468 → ~502 (335° → 360°)
- **Total: ~70 raze LiDAR acoperă FOV-ul camerei**

Aceasta înseamnă o rezoluție angulară de ~0.72° per rază,
adică la 3m distanță fiecare rază acoperă ~3.8cm lateral.

---

## 2. Pipeline-ul de Fuziune

```
┌──────────────┐     ┌─────────────┐     ┌──────────────┐
│  /yolo26/    │     │   /scan     │     │ /camera/     │
│  detections  │     │  (LaserScan)│     │ camera_info  │
└──────┬───────┘     └──────┬──────┘     └──────┬───────┘
       │                    │                    │
       │  Detection2DArray  │  cached            │  fx, cx (once)
       ▼                    ▼                    ▼
  ┌─────────────────────────────────────────────────────┐
  │           Semantic Localizer Node (3 Hz)            │
  │                                                     │
  │  1. BB pixels → camera angles                       │
  │     angle = atan2(x_px - cx, fx)                    │
  │                                                     │
  │  2. Camera angles → LiDAR angles                    │
  │     lidar_angle = -camera_angle                     │
  │                                                     │
  │  3. Select LiDAR rays → percentile(25) distance     │
  │                                                     │
  │  4. (distance, angle) → point in base_laser         │
  │     x = d·cos(θ), y = d·sin(θ)                     │
  │                                                     │
  │  5. TF2: base_laser → map                           │
  │     → (x_map, y_map)                                │
  │                                                     │
  │  6. Temporal tracking + median filter               │
  └────────────┬────────────────────┬───────────────────┘
               │                    │
               ▼                    ▼
     ┌─────────────────┐  ┌──────────────────┐
     │ /semantic_markers│  │ /semantic_        │
     │ (MarkerArray)   │  │  detections/point │
     │  → RViz2        │  │ (PointStamped)    │
     └─────────────────┘  └──────────────────┘
```

---

## 3. Limitări Cunoscute

1. **Obiecte care nu intersectează planul LiDAR:**
   LiDAR-ul la 16cm înălțime va vedea picioarele oamenilor și baza mobilierului,
   dar NU obiecte pe masă, pe perete la înălțime, sau suspendate.
   → Detecția YOLO le va recunoaște, dar distanța va fi indisponibilă.

2. **Obiecte la limita FOV:**
   La marginile FOV-ului camerei (±25°), doar 1-2 raze LiDAR pot fi disponibile.
   → Parametrul `min_valid_rays` protejează contra măsurătorilor nesigure.

3. **Obiecte în spatele altor obiecte:**
   LiDAR-ul returnează prima suprafață. Dacă YOLO detectează un om în spatele
   unui scaun, distanța LiDAR va fi cea a scaunului.
   → Risc de eroare în estimarea poziției.

4. **Obiecte foarte apropiate (<0.5m):**
   Bounding box-ul poate fi mai larg decât FOV-ul, și razele LiDAR pot conține
   reflecții de la corpul robotului.

---

## 4. Instrucțiuni de Deploy pe RPi5

### 4.1 Copiere package

```bash
# De pe PC-ul de dezvoltare:
scp -r semantic_localizer/ pi@<IP_RPi5>:~/saim_xplorer/src/

# SAU direct pe RPi5:
cd ~/saim_xplorer/src/
# (copiați fișierele din structura furnizată)
```

### 4.2 Build

```bash
cd ~/saim_xplorer
colcon build --packages-select semantic_localizer --symlink-install
source install/setup.bash
```

### 4.3 Lansare

```bash
# Asigurați-vă că rulează deja:
# - Nav2 stack (cu AMCL / harta)
# - Camera driver (publică pe /camera/image_raw + /camera/camera_info)
# - LiDAR driver (publică pe /scan)
# - YOLO26 (publică pe /yolo26/detections)

# Apoi:
ros2 launch semantic_localizer semantic_localizer_launch.py
```

### 4.4 Verificare

```bash
# Terminal 1: Monitor topic
ros2 topic echo /semantic_detections/point

# Terminal 2: Verifică rate
ros2 topic hz /semantic_markers

# Terminal 3: RViz2 (pe PC remote)
# Adaugă MarkerArray pe topic /semantic_markers
```

### 4.5 Estimare impact CPU

| Componentă | CPU estimat |
|------------|-------------|
| Subscribe /scan + /yolo26/detections | <1% |
| Calcul geometric (numpy) | <1% per ciclu |
| TF2 lookup | <0.5% |
| Publisher markers | <0.5% |
| **Total la 3 Hz** | **~2-4%** |

---

## 5. Pașii Următori (Pas 2+)

- **Pas 2:** Export detecții stabile → GeoJSON (format Route Server)
- **Pas 3:** Plugin Route Server care citește semantica din GeoJSON
- **Pas 4:** Task-uri de navigație semantică via Commander API
- **Pas 5:** Integrare AI pentru generare automată de task-uri
