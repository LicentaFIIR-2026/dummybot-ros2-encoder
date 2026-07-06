# Semantic Localizer — Documentație Tehnică v3
## SAIM Xplorer / FIIR — Navigație Semantică Adaptivă

**Proiect:** Integrarea detecției semantice a obiectelor în graful de navigație al Nav2 Route Server  
**Platformă:** Raspberry Pi 5 (16GB) | ROS 2 Jazzy | Nav2 1.3.10  
**Autori:** FIIR / Politehnica București  
**Ultima actualizare:** 2026-02-17

---

## Cuprins

1. [Analiza Geometriei Senzorilor](#1-analiza-geometriei-senzorilor)
2. [Pipeline-ul de Fuziune Camera-LiDAR (Pas 1)](#2-pipeline-ul-de-fuziune-camera-lidar-pas-1)
3. [Persistența Semantică și Anotarea Grafului (Pas 2)](#3-persistența-semantică-și-anotarea-grafului-pas-2)
4. [Format GeoJSON — Compatibilitate Nav2 Route Server](#4-format-geojson--compatibilitate-nav2-route-server)
5. [Clasificare Obiecte și Strategii de Update](#5-clasificare-obiecte-și-strategii-de-update)
6. [Integrare Nav2 Route Server (Pas 3)](#6-integrare-nav2-route-server-pas-3)
7. [Rezultate Navigație Semantică — Sprint 1](#7-rezultate-navigație-semantică--sprint-1)
8. [Limitări Cunoscute](#8-limitări-cunoscute)
9. [Instrucțiuni de Deploy pe RPi5](#9-instrucțiuni-de-deploy-pe-rpi5)
10. [Pașii Următori](#10-pașii-următori)

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
LiDAR: scanează 360° complet, cu 0° = +X (forward) — **confirmat prin test fizic**

**Raze LiDAR corespunzătoare FOV camera:**
- Stânga camerei: razele 0 → ~35 (0° → +25°)
- Dreapta camerei: razele ~468 → ~502 (335° → 360°)
- **Total: ~70 raze LiDAR acoperă FOV-ul camerei**

Aceasta înseamnă o rezoluție angulară de ~0.72° per rază,
adică la 3m distanță fiecare rază acoperă ~3.8cm lateral.

---

## 2. Pipeline-ul de Fuziune Camera-LiDAR (Pas 1)

### 2.1 Arhitectura

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
  │  5. TF2: base_laser → map (Time=0 for latest TF)   │
  │     → (x_map, y_map)                                │
  │                                                     │
  │  6. Temporal tracking + median filter               │
  │                                                     │
  │  7. Push to SemanticMapManager (persistence)        │
  └──┬──────────────┬──────────────┬────────────────────┘
     │              │              │
     ▼              ▼              ▼
┌──────────┐ ┌────────────┐ ┌──────────────────────┐
│/semantic │ │/semantic_  │ │ SemanticMapManager   │
│_markers  │ │detections/ │ │ ├─ semantic_objects   │
│(Markers) │ │point       │ │ │  .geojson           │
│→ RViz2   │ │(PointStamp)│ │ └─ route_graph_fiir   │
└──────────┘ └────────────┘ │    _semantic.geojson  │
                            └──────────────────────┘
```

### 2.2 Detalii tehnice TF2

**Problemă rezolvată:** AMCL publică transformarea `map→odom` cu întârziere (1-5s) față de timestamp-ul scan-ului. Utilizarea `self.get_clock().now()` sau `Time.from_msg(scan.header.stamp)` cauzează erori de extrapolare.

**Soluție:** `ros_now = Time(seconds=0, nanoseconds=0)` — folosește cea mai recentă transformare TF disponibilă, indiferent de timestamp. Aceasta este practica standard când sursa TF (AMCL) publică mai rar decât consumatorul.

### 2.3 Rezultate validate (Pas 1)

| Test | Rezultat |
|------|----------|
| Sticlă la 1m în fața robotului | Detectată la (3.21, 2.88) — robot la (1.97, 2.98) → distanță 1.25m ✓ |
| Rată de publicare markere | ~2.4 Hz (sub limita 3 Hz) ✓ |
| Clase detectate corect | bottle (70%), person (93%) ✓ |
| CPU overhead | ~2-4% suplimentar ✓ |

---

## 3. Persistența Semantică și Anotarea Grafului (Pas 2)

### 3.1 Arhitectura duală — 2 fișiere GeoJSON

Sistemul produce **două fișiere GeoJSON** cu scopuri diferite:

**Fișier 1: `semantic_objects.geojson`** — Harta obiectelor detectate
- FeatureCollection de Points (GeoJSON standard)
- Fiecare obiect detectat stabil = un Feature cu:
  - Poziție (x, y) în frame-ul `/map`
  - Clasă, confidence, mobility, observation_count, timestamps
  - TTL (Time-To-Live) pentru obiecte dinamice
- **Scop:** Persistență, vizualizare, baza de cunoștințe a robotului
- **Nu e citit direct de Route Server**

**Fișier 2: `route_graph_fiir_semantic.geojson`** — Graful de navigație anotat (complet)
- Copie a grafului original (`route_graph_fiir.geojson`) cu metadata semantică injectată pe edges
- Conține metadata completă: `penalty`, `speed_limit`, `class`, `semantic_objects[]`
- **NU e citit direct de Route Server** (array-ul `semantic_objects` cauzează `bad any_cast`)
- Servește ca output informativ și sursă pentru conversie

**Fișier 3: `route_graph_fiir_nav2.geojson`** — Graful Nav2-compatibil (minimal) ✨ **NOU**
- Generat din `route_graph_fiir_semantic.geojson` prin `convert_geojson_for_route_server.py`
- Metadata minimală: **doar `penalty` (float) + `speed_limit` (float)**
- **Citit direct de Route Server** — compatibil 100% cu GeoJsonGraphFileLoader

### 3.2 SemanticMapManager — Logica de persistență

```
Observație nouă (clasă, x, y, confidence, distance, rays)
         │
         ▼
  Există obiect similar în baza de date?
  (aceeași clasă + distanță < 0.5m)
         │
    ┌────┴────┐
    Nu        Da
    │         │
    ▼         ▼
  Adaugă    E dinamic?
  obiect    │
  nou       ┌──┴──┐
            Nu     Da
            │      │
            ▼      ▼
         Fused   Overwrite
         update  poziție +
         (α=0.7) reset TTL
```

### 3.3 Anotarea edge-urilor din graf

Pentru fiecare edge din graful de navigație:
1. Se calculează distanța punct-segment de la fiecare obiect persistent la edge
2. Obiectele aflate la distanță < `edge_proximity` (default 1.5m) sunt asociate edge-ului
3. Se calculează penalty agregat (scalat cu proximity și confidence)
4. Se setează speed_limit minim bazat pe tipul obiectului cel mai restrictiv
5. Se adaugă array `semantic_objects` cu detaliile tuturor obiectelor apropiate

**Formula penalty:**
```
penalty_per_object = base_penalty × proximity_factor × confidence
proximity_factor = max(0.1, 1.0 - distance_to_edge / edge_proximity)
total_penalty = Σ(penalty_per_object) for all nearby objects
```

### 3.4 Timere și persistență pe disk

| Timer | Interval | Funcție |
|-------|----------|---------|
| **save** | 30s | Salvează `semantic_objects.geojson` + `route_graph_fiir_semantic.geojson` |
| **cleanup** | 10s | Șterge obiecte dinamice/minor care au depășit TTL |
| **diagnostics** | 10s | Logare statistici |

La shutdown (Ctrl+C), se face o salvare finală a ambelor fișiere.

### 3.5 Rezultate validate cu navigație reală

**Test cu 11 obiecte detectate în navigație prin holul FIIR:**

Obiecte detectate:

| obj_id | Poziție (x, y) | Tip | Confidence | Observații |
|--------|---------------|-----|------------|------------|
| bottle_0 | (3.21, 2.87) | minor | 0.764 | 5565 |
| refrigerator_5 | (5.05, 2.01) | static | 0.754 | 52 |
| bottle_6 | (6.33, 1.94) | minor | 0.610 | 14 |
| tv_7 | (1.73, 1.66) | static | 0.512 | 1 |
| person_8 | (0.58, 2.04) | dynamic | 0.671 | 4 |
| person_9 | (-0.06, 1.65) | dynamic | 0.629 | 8 |
| person_10 | (-1.16, 1.28) | dynamic | 0.599 | 7 |
| person_11 | (-0.70, 4.13) | dynamic | 0.565 | 2 |
| person_12 | (1.06, 3.45) | dynamic | 0.764 | 10 |
| bench_13 | (0.33, 2.77) | static | 0.724 | 7 |
| bottle_14 | (1.27, 3.46) | minor | 0.795 | 1342 |

**TTL funcțional validat:** `Expired 3 objects: ['person_2', 'person_3', 'person_4']` — persoanele care nu mai sunt observate dispar după 60s, iar penalty-urile pe edges se ajustează la următoarea salvare.

---

## 4. Format GeoJSON — Compatibilitate Nav2 Route Server

### 4.1 Cerințe obligatorii (din documentația Nav2)

| Câmp | Node (Point) | Edge (Line/MultiLineString) | Status |
|------|-------------|-------------------|--------|
| `id` | ✅ unic, int | ✅ unic, int | **OBLIGATORIU** |
| `coordinates` | ✅ [x, y] | ✅ [[x1,y1],[x2,y2]] | **OBLIGATORIU** |
| `startid` / `endid` | — | ✅ | **OBLIGATORIU** |
| `frame` | "map" | — | Recomandat |
| `overridable` | — | true/false | Recomandat |

### 4.2 Constrângere critică — Metadata flat (float only) ✨ **NOU**

**Problemă descoperită:** Nav2 GeoJsonGraphFileLoader (Jazzy 1.3.10) **NU poate parsa** valori nested (array-uri, obiecte) sau string-uri în metadata edges. PenaltyScorer și DistanceScorer iterează toate cheile metadata și apelează `any_cast<double>()` — orice valoare non-numerică cauzează eroarea:

```
[ERROR] Failed to convert the key: semantic_objects to a value
[FATAL] Failed to configure route server: Failed to convert
```

**Chiar și metadata flat cu string-uri** (ex: `dominant_class: "bench"`, `obj_0_id: "tv_7"`) cauzează `bad any_cast` deoarece plugin-urile încearcă conversia la double pe **toate** cheile.

**Soluția validată:** Metadata edges trebuie să conțină **exclusiv** cheile pe care plugin-urile le citesc:

```json
"metadata": {
  "penalty": 1.61,
  "speed_limit": 60.0
}
```

**Nimic altceva** — nici string-uri, nici int-uri care nu sunt citite de un plugin configurat.

### 4.3 Pipeline de conversie GeoJSON

```
semantic_localizer (la 30s)
        │
        ▼
route_graph_fiir_semantic.geojson    ← metadata completă (penalty, speed_limit,
        │                                class, semantic_objects[])
        │                            ← INFORMATIV, nu e citit de Route Server
        ▼
convert_geojson_for_route_server.py  ← strip metadata → doar penalty + speed_limit
        │
        ▼
route_graph_fiir_nav2.geojson        ← metadata minimală (doar float)
                                     ← CITIT DE ROUTE SERVER ✓
```

### 4.4 Convenții metadata Nav2 (utilizate de plugin-uri)

| Cheie | Tip | Descriere | Plugin care o citește |
|-------|-----|-----------|----------------------|
| `speed_limit` | float | Procent viteză 0-100 | DistanceScorer, AdjustSpeedLimit |
| `penalty` | float | Cost suplimentar pe edge | PenaltyScorer |

**Notă:** `speed_limit` e interpretat ca **procentaj** din viteza maximă (0-100), NU valoare absolută în m/s. Valorile curente (60, 90) înseamnă 60% și 90% din `max_speed`.

### 4.5 Direcționalitate edges

Toate edge-urile în Route Server sunt **direcționale**. Navigația bidirecțională necesită 2 edges (forward + reverse). Graful FIIR curent are 6 perechi bidirecționale (12 edges total).

### 4.6 Suport geometrie

Modulul suportă atât `LineString` cât și `MultiLineString` pentru edge-uri, asigurând compatibilitate cu grafuri generate de QGIS, RViz Route Tool, LIF Editor sau manual.

---

## 5. Clasificare Obiecte și Strategii de Update

### 5.1 Clasificare mobilitate

| Tip | Clase | Penalty base | Speed limit | TTL |
|-----|-------|-------------|-------------|-----|
| **dynamic** | person, cat, dog, bird, horse, etc. | 10.0 | 30% | 60s |
| **static** | chair, bench, refrigerator, tv, couch, etc. | 3.0 | 60% | ∞ (permanent) |
| **minor** | bottle, cup, remote, book, etc. | 0.5 | 90% | 120s |

### 5.2 Strategii de update la re-observare

| Tip obiect | Strategie poziție | Strategie TTL |
|------------|------------------|---------------|
| **dynamic** | Overwrite (obiectul se mișcă) | Reset la 60s |
| **static** | Fused update: `pos = 0.7×old + 0.3×new` | Nu expiră niciodată |
| **minor** | Fused update: `pos = 0.7×old + 0.3×new` | Reset la 120s |

### 5.3 Matching obiecte existente

Un obiect nou este considerat același cu unul existent dacă:
- **Aceeași clasă** (class_name match exact)
- **Distanță euclidiană < 0.5m** (parametru configurabil: `tracking_distance_threshold`)

Se alege cel mai apropiat obiect existent ca match (nearest-neighbor).

---

## 6. Integrare Nav2 Route Server (Pas 3) ✨ **NOU**

### 6.1 Decizia de arhitectură

**Abordare:** Route Server standalone (Opțiunea A), separat de stack-ul Nav2 existent.

**Motivare:**
- Stack-ul Nav2 (AMCL, planner_server, controller_server, bt_navigator) funcționează și nu e atins
- Route Server rulează independent, nu e în lifecycle manager-ul de navigare
- Pentru articol nu contează dacă Route Server e în lifecycle manager — contează că demonstrăm alegerea traseului semantic
- Risc zero de a strica navigația existentă

**Flux de navigație:**

```
                            ┌──────────────────────────┐
                            │      Route Server        │
                            │  (standalone lifecycle)  │
                            │                          │
ComputeRoute Action ───────►│  GeoJsonGraphFileLoader  │
(start pose, goal pose)     │  PenaltyScorer (w=5.0)   │
                            │  DistanceScorer (w=1.0)  │
                            │  AdjustSpeedLimit        │
                            └──────────┬───────────────┘
                                       │ dense path (102 pts)
                                       ▼
                            ┌──────────────────────────┐
FollowPath Action ─────────►│   Controller Server      │
                            │   (MPPI, 20 Hz)          │
                            │   din stack-ul Nav2      │
                            └──────────────────────────┘
```

**Arhitectura folosită:** Arhitectura #1+#5 din documentația Nav2 Route Server:
- **#1:** ComputeRoute → dense path → FollowPath (MPPI controller direct)
- **#5:** AdjustSpeedLimit operation — publică `/speed_limit` pe baza metadata edges
- **Fără SmacPlanner intermediar** — justificat: graf mic (6 noduri), edges scurte (1-2m), mediu interior

### 6.2 Configurare Route Server

**Fișier:** `route_server_params.yaml`

```yaml
route_server:
  ros__parameters:
    base_frame: "base_link"
    route_frame: "map"
    max_planning_time: 2.0
    path_density: 0.05              # Densitate puncte pe dense path (m)
    smooth_corners: true
    smoothing_radius: 0.5           # Redus pentru spații interioare mici

    graph_file_loader: "GeoJsonGraphFileLoader"
    graph_file_loader_plugin:
      plugin: "nav2_route::GeoJsonGraphFileLoader"
    graph_filepath: "/home/pi/saim_xplorer/maps/route_graph_fiir_nav2.geojson"

    edge_cost_functions: ["DistanceScorer", "PenaltyScorer"]

    DistanceScorer:
      plugin: "nav2_route::DistanceScorer"
      weight: 1.0
      speed_tag: "speed_limit"

    PenaltyScorer:
      plugin: "nav2_route::PenaltyScorer"
      weight: 5.0                   # Weight mare → penalty-urile semantice domină
      penalty_tag: "penalty"

    operations: ["AdjustSpeedLimit"]
    AdjustSpeedLimit:
      plugin: "nav2_route::AdjustSpeedLimit"
      speed_limit_topic: "speed_limit"
      speed_tag: "speed_limit"

    tracker_update_rate: 20.0       # Redus de la 50 Hz pentru RPi5
```

### 6.3 Launch file

**Fișier:** `route_server_launch.py` — lansare standalone cu auto-configure + auto-activate

```bash
# Lansare (default: route_graph_fiir_nav2.geojson)
ros2 launch semantic_localizer route_server_launch.py

# Sau cu argument explicit:
ros2 launch semantic_localizer route_server_launch.py \
  graph_filepath:=/home/pi/saim_xplorer/maps/route_graph_fiir_nav2.geojson
```

**Notă:** Remapare `/plan` → `/route_plan` pentru a evita conflictul cu planner_server.

**Notă importantă:** Dacă launch file-ul nu funcționează (problemă cunoscută de prioritate parametri YAML vs launch override), se poate lansa manual:

```bash
ros2 run nav2_route route_server --ros-args \
  --params-file ~/saim_xplorer/src/semantic_localizer/config/route_server_params.yaml \
  -r plan:=route_plan

# Apoi în alt terminal:
ros2 lifecycle set /route_server configure
ros2 lifecycle set /route_server activate
```

### 6.4 Graful de navigație FIIR

```
              Calea A (penalty MIC)
        ┌───── altA (3) ─────┐
        │   penalty=0.58      │
        │   speed=60%         │
start (0) ── int1 (1) ── int2 (2)         goal (5)
        │                     │
        │   penalty=1.52      │
        └───── altB (4) ─────┘
              Calea B (penalty MARE)
```

| Traseu | Edges | Penalty total | Speed min |
|--------|-------|---------------|-----------|
| **Calea A:** 0→1→2→3→5 | 10,12,14,18 | **2.86** | 30% |
| **Calea B:** 0→1→2→4→5 | 10,12,16,20 | **4.24** | 30% |

Route Server preferă corect **Calea A** (penalty total 2.86 < 4.24).

### 6.5 ComputeRoute — Detalii tehnice API

**Câmpuri critice ComputeRoute.Goal (Jazzy 1.3.10):**

| Câmp | Tip | Descriere | **IMPORTANT** |
|------|-----|-----------|---------------|
| `use_start` | bool | Folosește start PoseStamped vs TF lookup | Setează `True` |
| `use_poses` | bool | Folosește PoseStamped vs node IDs | **OBLIGATORIU `True`** pentru coordonate |
| `start` | PoseStamped | Poziția de start | frame_id = "map" |
| `goal` | PoseStamped | Destinația | frame_id = "map" |

**Fără `use_poses=True`**, Route Server interpretează cererea ca navigare între node IDs (default 0→0), returnând un path cu 1 singur punct.

**Structura rezultatului:**

| Câmp | Tip | Acces |
|------|-----|-------|
| `result.path` | nav_msgs/Path | Dense path (102 puncte @ path_density=0.05) |
| `result.route.nodes` | RouteNode[] | Atribut: `nodeid` (NU `node_id`) |
| `result.route.edges` | RouteEdge[] | Atribut: `edgeid` (NU `edge_id`) |
| `result.route.route_cost` | float | Cost total rută |
| `result.error_code` | int | 0=SUCCESS, 400=UNKNOWN, 403=INDETERMINANT |

---

## 7. Rezultate Navigație Semantică — Sprint 1 ✨ **NOU**

### 7.1 Condiții experimentale

| Parametru | Valoare |
|-----------|---------|
| Platformă | Raspberry Pi 5 (16GB), CPU-only |
| ROS 2 | Jazzy |
| Nav2 | 1.3.10 |
| Detector | YOLO11n (COCO pretrained) |
| Controller | MPPI @ 20 Hz |
| Planner (baseline) | SmacPlannerHybrid (Reeds-Shepp) |
| Mediu | Holul FIIR, Politehnica București |
| Graf | 6 noduri, 12 edges bidirecționale |

### 7.2 Teste funcționale — Toate PASS ✓

| Test | Descriere | Metodă | Rezultat |
|------|-----------|--------|----------|
| **Test 1** | ComputeRoute (planificare) | Route Server: start→goal | Calea A (0→1→2→3→5), 102 pts, 39ms ✓ |
| **Test 2** | ComputeRoute + FollowPath | Semantic route → MPPI | Robotul parcurge Calea A, 16.67s ✓ |
| **Test 2r** | ComputeRoute + FollowPath (reverse) | Goal→start, semantic | Calea A inversă (5→3→2→1→0), 25.76s ✓ |
| **Test 3** | NavigateToPose baseline | SmacPlannerHybrid, fără semantică | Navigație directă, 22.04s ✓ |

### 7.3 Metrici de planificare

| Metric | Valoare |
|--------|---------|
| Timp planificare ComputeRoute | 25-39 ms |
| Dense path (puncte) | 102 |
| Cost rută (Calea A) | 14.37 |
| Noduri rută | [0, 1, 2, 3, 5] |
| Edges rută | [10, 12, 14, 18] |
| Verificare bidirecționalitate | nodes=[5, 3, 2, 1, 0], edges=[19, 15, 13, 11] ✓ |

### 7.4 Comparație preliminară Semantic vs Baseline

| Condiție | Timp parcurgere | Traseu |
|----------|----------------|--------|
| Semantic (forward) | 16.67s | Calea A (penalty minim) |
| Semantic (reverse) | 25.76s | Calea A inversă |
| Baseline (forward) | 22.04s | SmacPlannerHybrid (fără semantică) |

**Observații:**
- Route Server alege corect traseul cu penalty minim în ambele direcții
- PenaltyScorer funcționează bidirecțional cu același cost (14.37)
- Baseline-ul (SmacPlannerHybrid) nu ține cont de obiecte semantice
- Pentru statistici robuste, sunt necesare 5-10 run-uri per condiție (Sprint 3)

### 7.5 Verificări runtime

```bash
# Topics Route Server
ros2 topic list | grep -E "(route|speed)"
# /route_graph, /route_plan, /speed_limit ✓

# Actions disponibile
ros2 action list | grep -i route
# /compute_route, /compute_and_track_route ✓

# Lifecycle state
ros2 lifecycle get /route_server
# active [3] ✓
```

---

## 8. Limitări Cunoscute

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

5. **Fals pozitive YOLO la navigație:**
   La viteze de deplasare, YOLO poate genera detecții fugitive (ex: refrigerator_5
   detectat cu 52 observații pe scurt, apoi stabil). Confidence threshold (0.45)
   și observation_count ajută la filtrare, dar pot apărea obiecte efemere.

6. **Graful de navigație nu se reîncarcă automat în Route Server:**
   Fișierul GeoJSON se salvează pe disk periodic (30s), dar Route Server trebuie
   notificat separat prin `set_route_graph` service (Sprint 2 — neimplementat încă).

7. **Metadata GeoJSON — restricție Nav2 (float only):** ✨ **NOU**
   GeoJsonGraphFileLoader din Nav2 1.3.10 nu suportă metadata nested sau string-uri.
   Soluție implementată: conversie la metadata minimală (doar `penalty` + `speed_limit`).

8. **Fără fallback freespace pe ruta semantică:** ✨ **NOU**
   Arhitectura ComputeRoute → FollowPath (MPPI) nu include un planner intermediar
   pentru obstacole neprevăzute pe traseul dens. Dacă un obstacol apare pe path-ul
   generat de Route Server, MPPI va încerca evitare locală dar poate eșua.
   Justificare: graf mic (6 noduri, 1-2m edges), mediu interior controlat.

---

## 9. Instrucțiuni de Deploy pe RPi5

### 9.1 Structura pachetului

```
semantic_localizer/
├── semantic_localizer/
│   ├── __init__.py
│   ├── semantic_localizer_node.py     ← Nodul principal: fuziune + tracking + persistence
│   └── semantic_map_manager.py        ← Persistență obiecte + anotare graf Nav2
├── scripts/
│   └── create_fiir_route_graph.py     ← Generator graf demo Nav2
├── launch/
│   ├── semantic_localizer_launch.py
│   └── route_server_launch.py         ← Lansare Route Server standalone ✨ NOU
├── config/
│   ├── semantic_localizer_params.yaml
│   └── route_server_params.yaml       ← Configurare Route Server ✨ NOU
├── resource/semantic_localizer
├── package.xml
├── setup.py
└── setup.cfg
```

### 9.2 Fișiere generate la runtime

```
~/saim_xplorer/maps/
├── route_graph_fiir.geojson            ← Graf de bază (creat manual, NU se modifică)
├── semantic_objects.geojson            ← Harta obiectelor (auto-generat, salvat la 30s)
├── route_graph_fiir_semantic.geojson   ← Graf anotat complet (auto-generat, salvat la 30s)
└── route_graph_fiir_nav2.geojson       ← Graf Nav2-compatibil (generat prin conversie) ✨ NOU
```

### 9.3 Utilitare

```
~/saim_xplorer/
├── convert_geojson_for_route_server.py  ← Conversie semantic → Nav2 (doar penalty + speed_limit) ✨ NOU
└── test_semantic_navigation.py          ← Suite teste: ComputeRoute, FollowPath, baseline ✨ NOU
```

### 9.4 Copiere și Build

```bash
# De pe PC-ul de dezvoltare:
scp -r semantic_localizer/ pi@<IP_RPi5>:~/saim_xplorer/src/

# Build:
cd ~/saim_xplorer
colcon build --packages-select semantic_localizer --symlink-install
source install/setup.bash
```

### 9.5 Lansare

```bash
# 1. Asigurați-vă că rulează deja:
# - Nav2 stack (cu AMCL / harta)
# - Camera driver (publică pe /camera/image_raw + /camera/camera_info)
# - LiDAR driver (publică pe /scan)
# - YOLO26 (publică pe /yolo26/detections)
terminal1:

terminal2:
terminal3:
ros2 launch yolo26_cpp yolo26_cpp_launch.py
# Apoi:  ros2 lifecycle set /yolo26_detector configure && ros2 lifecycle set /yolo26_detector activate

# 2. Lansați Semantic Localizer:
ros2 launch semantic_localizer semantic_localizer_launch.py

# 3. Convertiți GeoJSON-ul (după ce semantic_localizer a salvat cel puțin o dată):
python3 ~/saim_xplorer/convert_geojson_for_route_server.py \
  --input  maps/route_graph_fiir_semantic.geojson \
  --output maps/route_graph_fiir_nav2.geojson

# 4. Lansați Route Server:
ros2 run nav2_route route_server --ros-args \
  --params-file ~/saim_xplorer/src/semantic_localizer/config/route_server_params.yaml \
  -r plan:=route_plan
# Apoi: ros2 lifecycle set /route_server configure && ros2 lifecycle set /route_server activate

# 5. Rulați teste:
python3 ~/saim_xplorer/test_semantic_navigation.py --test 1  # ComputeRoute
python3 ~/saim_xplorer/test_semantic_navigation.py --test 2  # ComputeRoute + FollowPath
python3 ~/saim_xplorer/test_semantic_navigation.py --test 3  # Baseline NavigateToPose
```

### 9.6 Verificare funcționare

```bash
# Diagnostice semantic_localizer (în log, la fiecare 10s):
# Semantic: 7314 det | 3 tracked (3 stable) | map: 4 persistent {'minor': 1, 'dynamic': 3} | graph=True

# Verificare Route Server:
ros2 lifecycle get /route_server           # Trebuie: active [3]
ros2 topic list | grep -E "(route|speed)"  # /route_graph, /route_plan, /speed_limit
ros2 action list | grep route              # /compute_route, /compute_and_track_route

# Verificare GeoJSON Nav2-compatibil:
python3 -c "
import json
with open('maps/route_graph_fiir_nav2.geojson') as f:
    data = json.load(f)
for feat in data['features']:
    meta = feat['properties'].get('metadata', {})
    if meta:
        print(list(meta.keys()))  # Trebuie: ['penalty', 'speed_limit']
        break
"
```

### 9.7 Parametri configurabili

| Parametru | Default | Descriere |
|-----------|---------|-----------|
| `min_confidence` | 0.45 | Threshold YOLO minimum |
| `max_process_rate_hz` | 3.0 | Rate limiter fuziune (Hz) |
| `max_object_distance` | 8.0 | Ignoră obiecte >8m |
| `tracking_distance_threshold` | 0.5 | Rază matching obiecte (m) |
| `tracking_timeout` | 5.0 | Timeout tracking live (s) |
| `min_detections_stable` | 3 | Nr. detecții pentru "stabil" |
| `median_filter_window` | 5 | Fereastră medianei temporale |
| `ray_percentile` | 25.0 | Percentila LiDAR (25 = cel mai aproape) |
| `min_valid_rays` | 2 | Raze minime pentru măsurătoare validă |
| `camera_lidar_yaw_offset` | 0.0 | Offset angular cameră-LiDAR (rad) |
| `semantic_objects_filepath` | `~/saim_xplorer/maps/semantic_objects.geojson` | Fișier persistență obiecte |
| `route_graph_filepath` | `~/saim_xplorer/maps/route_graph_fiir.geojson` | Graful de bază Nav2 |
| `output_graph_filepath` | `~/saim_xplorer/maps/route_graph_fiir_semantic.geojson` | Graful anotat output |
| `save_interval` | 30.0 | Salvare periodică pe disk (s) |
| `cleanup_interval` | 10.0 | Cleanup TTL periodic (s) |
| `edge_proximity` | 1.5 | Distanță max obiect↔edge pentru anotare (m) |

### 9.8 Estimare impact CPU (RPi5)

| Componentă | CPU estimat |
|------------|-------------|
| Subscribe /scan + /yolo26/detections | <1% |
| Calcul geometric (numpy) | <1% per ciclu |
| TF2 lookup | <0.5% |
| Publisher markers | <0.5% |
| SemanticMapManager (save/cleanup) | <0.5% |
| **Route Server (ComputeRoute)** | **~1% per request** ✨ NOU |
| **Total la 3 Hz** | **~2-5%** |

### 9.9 Notă importantă — TF2 timestamp

În `_detections_cb`, timestamp-ul pentru TF lookup **trebuie** să fie:
```python
ros_now = Time(seconds=0, nanoseconds=0)
```
Aceasta folosește cea mai recentă transformare TF disponibilă. Utilizarea `self.get_clock().now()` sau `Time.from_msg(scan.stamp)` cauzează erori de extrapolare deoarece AMCL publică TF-ul `map→odom` cu întârziere.

---

## 10. Pașii Următori

### Implementat ✅

- **Pas 1:** Fuziune YOLO + LiDAR 2D → poziții obiecte în `/map`
- **Pas 2.1:** Persistență obiecte — `semantic_objects.geojson`
- **Pas 2.2:** Graf de navigație FIIR — `route_graph_fiir.geojson`
- **Pas 2.3:** Asociere obiecte ↔ edges (geometrie punct-segment)
- **Pas 2.4:** Injectare metadata Nav2 (`penalty`, `speed_limit`, `class`)
- **Pas 2.4+:** Suport MultiLineString + LineString în graf
- **Pas 3.1:** ✨ Conversie GeoJSON → format Nav2-compatibil (doar penalty + speed_limit float)
- **Pas 3.2:** ✨ Route Server standalone cu PenaltyScorer + DistanceScorer + AdjustSpeedLimit
- **Pas 3.3:** ✨ Test ComputeRoute — alegere corectă traseu semantic (Calea A vs Calea B)
- **Pas 3.4:** ✨ Test ComputeRoute + FollowPath — navigație fizică pe traseu semantic
- **Pas 3.5:** ✨ Test baseline NavigateToPose — comparație fără semantică

### De implementat 🔧

- **Sprint 2: Reîncărcare dinamică graf (1 zi)**
  - Service call `set_route_graph` din semantic_localizer după fiecare salvare GeoJSON
  - Alternativă: timer node care monitorizează timestamp fișier
  - Validează claimul de "navigație semantică adaptivă"

- **Sprint 3: Scenarii de navigație repetate (2-3 zile)**
  - 5-10 run-uri per condiție (semantic forward, semantic reverse, baseline)
  - Metrici: timp, distanță parcursă, distanță minimă față de obiecte semantice
  - Scenariul adaptiv: persoană apare → penalty crește → traseu se schimbă

- **Sprint 4: Ground truth și metrici (2-3 zile)**
  - Marcare 20-30 poziții obiecte (bandă pe podea, măsurători de la landmarks)
  - Comparație semantic_objects.geojson vs ground truth → RMSE, MAE per clasă
  - Ablation: percentilă (10/25/50/75), confidence (0.3/0.45/0.6)

**Estimare totală Sprint 2-4:** ~8-10 zile lucru → Gata pentru validare cross-robot/cross-environment

### Amânat / Future Work 📋

- **Pas 5:** Integrare AI pentru generare automată de task-uri din harta semantică (articol separat)
- Navigare la obiect detectat (ex: "mergi la frigider") — necesită nod lookup semantic
- Comparație cu deep fusion sau metode RGBD — discutat teoretic în articol
- Training custom YOLO — COCO pretrained e suficient

---

## Troubleshooting Reference ✨ **NOU**

| Problemă | Cauză | Soluție |
|----------|-------|---------|
| `bad any_cast` la configure | Metadata nested/string în GeoJSON | Folosește `route_graph_fiir_nav2.geojson` (doar penalty + speed_limit) |
| Path cu 0-1 puncte | `use_poses=False` în ComputeRoute | Setează `use_poses=True` |
| Rută [0] only | start_id=0, goal_id=0 | Folosește PoseStamped cu `use_poses=True` |
| Route Server nu pornește | GeoJSON invalid | Validează cu `python3 -m json.tool < file.geojson` |
| Traseu greșit ales | PenaltyScorer weight prea mic | Crește weight-ul (curent: 5.0) |
| Speed limit nu se publică | Folosești ComputeRoute | Folosește ComputeAndTrackRoute |
| Launch file ignoră graph_filepath | Conflict YAML vs launch override | Lansează manual cu `ros2 run` + `--params-file` |
| Două instanțe Route Server | Proces vechi în background | `pkill -f route_server` apoi relansare |
