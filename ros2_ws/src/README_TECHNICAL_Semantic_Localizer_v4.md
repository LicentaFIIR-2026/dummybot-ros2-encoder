# Semantic Localizer — Documentație Tehnică v4
## DummyBot (AIONIS) — Navigație Semantică Adaptivă

**Proiect:** Integrarea detecției semantice a obiectelor în graful de navigație al Nav2 Route Server  
**Platformă:** Raspberry Pi 5 (16GB) + Desktop PC | ROS 2 Jazzy | Nav2 1.3.10  
**Autori:** FIIR / Politehnica București  
**Ultima actualizare:** 2026-02-19

---

## Cuprins

1. [Analiza Geometriei Senzorilor](#1-analiza-geometriei-senzorilor)
2. [Pipeline-ul de Fuziune Camera-LiDAR (Pas 1)](#2-pipeline-ul-de-fuziune-camera-lidar-pas-1)
3. [Persistența Semantică și Anotarea Grafului (Pas 2)](#3-persistenta-semantica-si-anotarea-grafului-pas-2)
4. [Format GeoJSON — Compatibilitate Nav2 Route Server](#4-format-geojson--compatibilitate-nav2-route-server)
5. [Clasificare Obiecte și Strategii de Update](#5-clasificare-obiecte-si-strategii-de-update)
6. [Integrare Nav2 Route Server (Pas 3)](#6-integrare-nav2-route-server-pas-3)
7. [Reload Dinamic Graf (Sprint 2)](#7-reload-dinamic-graf-sprint-2)
8. [Rezultate Navigație Semantică — Sprint 1-2](#8-rezultate-navigatie-semantica--sprint-1-2)
9. [Limitări Cunoscute](#9-limitari-cunoscute)
10. [Instrucțiuni de Deploy](#10-instructiuni-de-deploy)
11. [Pașii Următori](#11-pasii-urmatori)

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

**Concluzie:** Senzorii sunt practic co-locați din perspectiva proiecției orizontale. Nu e nevoie de compensare de paralaxă semnificativă.

### 1.4 Suprapunerea FOV

Camera FOV: ±24.9° de la axa +X  
LiDAR: scanează 360° complet, cu 0° = +X (forward) — **confirmat prin test fizic**

**Raze LiDAR corespunzătoare FOV camera:**
- Stânga camerei: razele 0 → ~35 (0° → +25°)
- Dreapta camerei: razele ~468 → ~502 (335° → 360°)
- **Total: ~70 raze LiDAR acoperă FOV-ul camerei**

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
┌──────────┐ ┌────────────┐ ┌──────────────────────────────┐
│/semantic │ │/semantic_  │ │ SemanticMapManager           │
│_markers  │ │detections/ │ │ ├─ semantic_objects.geojson  │
│(Markers) │ │point       │ │ └─ route_graph_dummybot_     │
│→ RViz2   │ │(PointStamp)│ │    semantic.geojson          │
└──────────┘ └────────────┘ └──────────────────────────────┘
```

### 2.2 Detalii tehnice TF2

**Problemă rezolvată:** AMCL publică transformarea `map→odom` cu întârziere (1-5s) față de timestamp-ul scan-ului. Utilizarea `self.get_clock().now()` sau `Time.from_msg(scan.header.stamp)` cauzează erori de extrapolare.

**Soluție:** `ros_now = Time(seconds=0, nanoseconds=0)` — folosește cea mai recentă transformare TF disponibilă, indiferent de timestamp. Aceasta este practica standard când sursa TF (AMCL) publică mai rar decât consumatorul.

### 2.3 Rezultate validate (Pas 1)

| Test | Rezultat |
|------|----------|
| Sticlă la 1m în fața robotului | Detectată corect în frame-ul map ✓ |
| Rată de publicare markere | ~2.4 Hz (sub limita 3 Hz) ✓ |
| Clase detectate corect | bottle (70%), person (93%) ✓ |
| CPU overhead | ~2-4% suplimentar ✓ |

---

## 3. Persistența Semantică și Anotarea Grafului (Pas 2)

### 3.1 Arhitectura duală — 3 fișiere GeoJSON

**Fișier 1: `semantic_objects.geojson`** — Harta obiectelor detectate
- FeatureCollection de Points (GeoJSON standard)
- Fiecare obiect detectat stabil = un Feature cu poziție (x, y) în `/map`, clasă, confidence, mobility, observation_count, timestamps, TTL
- **Scop:** Persistență, vizualizare, baza de cunoștințe a robotului
- **Nu e citit direct de Route Server**

**Fișier 2: `route_graph_dummybot_semantic.geojson`** — Graful de navigație anotat (complet)
- Copie a grafului original cu metadata semantică injectată pe edges
- Conține metadata completă: `penalty`, `speed_limit`, `class`, `semantic_objects[]`
- **NU e citit direct de Route Server** (array-ul `semantic_objects` cauzează `bad any_cast`)
- Servește ca output informativ și sursă pentru conversie

**Fișier 3: `route_graph_dummybot_nav2.geojson`** — Graful Nav2-compatibil (minimal)
- Generat automat din `route_graph_dummybot_semantic.geojson` prin funcția internă de conversie
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
| **save** | 30s | Salvează `semantic_objects.geojson` + `route_graph_dummybot_semantic.geojson` → conversie → reload Route Server |
| **cleanup** | 10s | Șterge obiecte dinamice/minor care au depășit TTL |
| **diagnostics** | 10s | Logare statistici |

La shutdown (Ctrl+C), se face o salvare finală a ambelor fișiere.

---

## 4. Format GeoJSON — Compatibilitate Nav2 Route Server

### 4.1 Cerințe obligatorii

| Câmp | Node (Point) | Edge (MultiLineString) | Status |
|------|-------------|----------------------|--------|
| `id` | ✅ unic, int | ✅ unic, int | **OBLIGATORIU** |
| `coordinates` | ✅ [x, y] | ✅ [[[x1,y1],[x2,y2]]] | **OBLIGATORIU** |
| `startid` / `endid` | — | ✅ | **OBLIGATORIU** |
| `frame` | "map" | — | Recomandat |
| `overridable` | — | true/false | Recomandat |

### 4.2 Constrângere critică — Metadata flat (float only)

**Problemă descoperită:** Nav2 GeoJsonGraphFileLoader (Jazzy 1.3.10) **NU poate parsa** valori nested (array-uri, obiecte) sau string-uri în metadata edges. PenaltyScorer și DistanceScorer iterează toate cheile metadata și apelează `any_cast<double>()` — orice valoare non-numerică cauzează eroarea:

```
[ERROR] Failed to convert the key: semantic_objects to a value
[FATAL] Failed to configure route server: Failed to convert
```

**Soluția validată:** Metadata edges trebuie să conțină **exclusiv**:

```json
"metadata": {
  "penalty": 1.61,
  "speed_limit": 60.0
}
```

### 4.3 Pipeline de conversie GeoJSON

```
semantic_localizer (la 30s)
        │
        ▼
route_graph_dummybot_semantic.geojson    ← metadata completă
        │                                ← INFORMATIV, nu e citit de Route Server
        │
        ▼  (conversie internă în _save_maps_cb)
route_graph_dummybot_nav2.geojson        ← metadata minimală (doar penalty + speed_limit float)
        │                                ← CITIT DE ROUTE SERVER ✓
        ▼
/route_server/set_route_graph (service call automat)
```

### 4.4 Convenții metadata Nav2

| Cheie | Tip | Descriere | Plugin care o citește |
|-------|-----|-----------|----------------------|
| `speed_limit` | float | Procent viteză 0-100 | DistanceScorer, AdjustSpeedLimit |
| `penalty` | float | Cost suplimentar pe edge | PenaltyScorer |

**Notă:** `speed_limit` e interpretat ca **procentaj** din viteza maximă (0-100), NU valoare absolută în m/s.

### 4.5 Direcționalitate edges

Toate edge-urile în Route Server sunt **direcționale**. Navigația bidirecțională necesită 2 edges (forward + reverse). Graful DummyBot curent are 14 perechi bidirecționale (28 edges total).

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

---

## 6. Integrare Nav2 Route Server (Pas 3)

### 6.1 Decizia de arhitectură

**Abordare:** Route Server integrat în lifecycle manager-ul Nav2 existent (nu standalone).

**Motivare:**
- Route Server pornește și se oprește sincron cu restul stack-ului Nav2
- Nu e nevoie de un launch file separat sau lifecycle manual
- `route_server_launch.py` există în pachet dar este **redundant** — nu se folosește

**Flux de navigație:**

```
                            ┌──────────────────────────┐
                            │      Route Server        │
                            │  (în lifecycle Nav2)     │
                            │                          │
ComputeRoute Action ───────►│  GeoJsonGraphFileLoader  │
(start pose, goal pose)     │  PenaltyScorer (w=5.0)   │
                            │  DistanceScorer (w=1.0)  │
                            │  AdjustSpeedLimit        │
                            └──────────┬───────────────┘
                                       │ dense path (102-116 pts)
                                       ▼
                            ┌──────────────────────────┐
FollowPath Action ─────────►│   Controller Server      │
                            │   (MPPI, 20 Hz)          │
                            │   din stack-ul Nav2      │
                            └──────────────────────────┘
```

### 6.2 Configurare Route Server (xplorer.yaml)

Route Server este configurat direct în `xplorer.yaml` din pachetul `amr2ax_nav2`:

```yaml
route_server:
  ros__parameters:
    use_sim_time: false
    enable_stamped_cmd_vel: true

    graph_file_loader: "GeoJsonGraphFileLoader"
    graph_filepath: "/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_dummybot_nav2.geojson"

    max_iterations: 100000
    max_planning_time: 10.0

    edge_cost_functions: ["DistanceScorer", "PenaltyScorer"]

    DistanceScorer:
      plugin: "nav2_route::DistanceScorer"
      weight: 1.0

    PenaltyScorer:
      plugin: "nav2_route::PenaltyScorer"
      weight: 5.0

    route_operations: ["AdjustSpeedLimit"]
    AdjustSpeedLimit:
      plugin: "nav2_route::AdjustSpeedLimit"
```

**Notă importantă:** `AdjustSpeedLimit` este de tip `RouteOperation` (nu `EdgeCostFunction`) și se configurează sub `route_operations:`, nu sub `edge_cost_functions:`.

### 6.3 Lifecycle Manager

Route Server e inclus în lista `node_names` din lifecycle_manager_navigation:

```yaml
lifecycle_manager_navigation:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names: ['controller_server', 'planner_server', 'behavior_server', 'route_server',
                 'bt_navigator', 'waypoint_follower', 'velocity_smoother']
```

**Verificare:**
```bash
ros2 lifecycle get /route_server
# active [3] ✓
```

### 6.4 Graful de navigație DummyBot

**Fișier de bază:** `/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph.json`

**10 noduri, 28 edges bidirecționale:**

| Node | Nume | Coordonate (x, y) |
|------|------|-------------------|
| 0 | start | (1.097, 2.588) |
| 1 | in1 | (2.313, 3.215) |
| 2 | in2 | (2.316, 2.540) |
| 3 | int2 | (3.334, 2.844) |
| 4 | int3 | (3.337, 2.473) |
| 5 | altB | (4.464, 2.563) |
| 6 | altA | (4.353, 3.299) |
| 7 | int4 | (5.302, 2.564) |
| 8 | goal_old | (5.635, 3.335) |
| 9 | goal | (6.002, 2.649) |

**Structura căilor principale (start→goal):**

```
Calea A: 0 → 1 → 3 → 6 → 7 → 9   (prin altA, node 6)
Calea B: 0 → 2 → 4 → 5 → 7 → 9   (prin altB, node 5)
```

**Penalty-uri actuale (fără obiecte semantice):**

| Traseu | Penalty total aproximativ |
|--------|--------------------------|
| Calea A | ~57-74 (variabil cu obiecte) |
| Calea B | ~50-70 (variabil cu obiecte) |

Route Server alege calea cu penalty total minim — dependent de obiectele detectate de YOLO în apropierea edges-urilor.

### 6.5 ComputeRoute — Detalii tehnice API

**Câmpuri critice ComputeRoute.Goal (Jazzy 1.3.10):**

| Câmp | Tip | Descriere | IMPORTANT |
|------|-----|-----------|-----------|
| `use_start` | bool | Folosește start PoseStamped vs TF lookup | Setează `True` |
| `use_poses` | bool | Folosește PoseStamped vs node IDs | **OBLIGATORIU `True`** |
| `start` | PoseStamped | Poziția de start | frame_id = "map" |
| `goal` | PoseStamped | Destinația | frame_id = "map" |

**Fără `use_poses=True`**, Route Server interpretează cererea ca navigare între node IDs (default 0→0), returnând un path cu 1 singur punct.

**Structura rezultatului:**

| Câmp | Tip | Acces |
|------|-----|-------|
| `result.path` | nav_msgs/Path | Dense path (102-116 puncte) |
| `result.route.nodes` | RouteNode[] | Atribut: `nodeid` (NU `node_id`) |
| `result.route.edges` | RouteEdge[] | Atribut: `edgeid` (NU `edge_id`) |
| `result.route.route_cost` | float | Cost total rută |

---

## 7. Reload Dinamic Graf (Sprint 2)

### 7.1 Arhitectura

Reîncărcarea automată a grafului în Route Server este implementată direct în `semantic_localizer_node.py`, în funcția `_save_maps_cb()`.

**Flux complet la fiecare 30s:**

```
_save_maps_cb() se declanșează (timer 30s)
        │
        ▼
1. map_manager.save_objects()
   → semantic_objects.geojson
        │
        ▼
2. map_manager.save_annotated_graph()
   → route_graph_dummybot_semantic.geojson
        │
        ▼
3. _convert_geojson_fn(data)          ← funcție importată din convert_geojson_for_route_server.py
   → strip metadata la penalty + speed_limit
   → route_graph_dummybot_nav2.geojson
        │
        ▼
4. /route_server/set_route_graph (service call async)
   → nav2_msgs/srv/SetRouteGraph
   → Route Server reîncarcă graful fără restart
        │
        ▼
5. _route_graph_reload_cb() → log succes/eroare
```

### 7.2 Implementare

**Funcția de conversie** este importată dinamic la startup din `convert_geojson_for_route_server.py`:

```python
spec = importlib.util.spec_from_file_location('convert_geojson_mod', script_path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
self._convert_geojson_fn = mod.convert_geojson
```

**Service client** creat în `__init__`:

```python
self._route_graph_client = self.create_client(
    SetRouteGraph, '/route_server/set_route_graph')
```

### 7.3 Parametri noi în semantic_localizer_params.yaml

```yaml
semantic_localizer:
  ros__parameters:
    # ... parametrii existenți ...

    # Route Server auto-reload
    nav2_graph_filepath: '/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_dummybot_nav2.geojson'
    convert_script_path: '/home/saim/dummybot-ros2-encoder/ros2_ws/convert_geojson_for_route_server.py'
```

### 7.4 Mesaje de log așteptate

```
[INFO] Funcție conversie încărcată din convert_geojson_for_route_server.py
[INFO] Saved annotated route graph to .../route_graph_dummybot_semantic.geojson
[INFO] GeoJSON convertit: 10 noduri, 28 edges → route_graph_dummybot_nav2.geojson
[INFO] Route Server: graf reîncărcat cu succes
```

### 7.5 Problemă de timing — Race condition

**Problemă identificată:** Dacă scriptul de test trimite un goal ComputeRoute exact în momentul în care semantic_localizer reîncarcă graful (apel `set_route_graph`), Route Server returnează:

```
[WARN] Failed to send goal response (timeout): client will not receive response
```

Scriptul de test primește `goal REJECTED` și raportează FAIL, chiar dacă planificarea ar fi reușit în alt moment.

**Soluție:** Rulează testele la minimum 5s după apariția mesajului `Route Server: graf reîncărcat cu succes` în log.

### 7.6 Limitare — Nu reactiv în timp real

Sistemul actual salvează și reîncarcă la fiecare 30s (configurabil prin `save_interval`). Nu există replanificare în timp real în timp ce robotul se deplasează. Dacă un obiect apare lângă traseul curent în timp ce robotul merge, schimbarea traseului va fi aplicată la următorul ciclu de 30s, nu instant.

Aceasta este acceptabilă pentru demonstrația din teză — sistemul arată că **poate** adapta traseul bazat pe semantică, nu că o face cu latență minimă.

---

## 8. Rezultate Navigație Semantică — Sprint 1-2

### 8.1 Condiții experimentale

| Parametru | Valoare |
|-----------|---------|
| Platformă | Raspberry Pi 5 (16GB) + Desktop PC |
| ROS 2 | Jazzy |
| Nav2 | 1.3.10 |
| Detector | YOLO (COCO pretrained) — topic `/yolo26/detections` |
| Controller | MPPI @ 20 Hz |
| Planner (baseline) | SmacPlannerHybrid |
| Graf | 10 noduri, 28 edges bidirecționale |

### 8.2 Teste funcționale — Toate PASS ✓

| Test | Descriere | Rezultat |
|------|-----------|----------|
| **Test 1 (fără obiecte)** | ComputeRoute start→goal | Calea B (0→2→4→5→7→9), 102 pts, 18ms ✓ |
| **Test 1 (cu persoană lângă Calea B)** | ComputeRoute start→goal | Calea A parțială (0→1→3→5→7→9), rută schimbată ✓ |
| **Test 1 (penalizare manuală Calea B)** | ComputeRoute cu penalty 50.0 pe Calea B | Calea A (0→1→3→6→8→9), 116 pts, 9ms ✓ |
| **Test 2** | ComputeRoute + FollowPath | Robotul parcurge traseul ales, 25-31s ✓ |
| **Reload dinamic** | semantic_localizer reîncarcă graf automat | `Route Server: graf reîncărcat cu succes` la 30s ✓ |

### 8.3 Metrici de planificare

| Metric | Valoare |
|--------|---------|
| Timp planificare ComputeRoute | 9-230 ms (variabil cu load sistem) |
| Dense path (puncte) | 102-116 puncte |
| Noduri rută Calea B | [0, 2, 4, 5, 7, 9] — edges [12, 16, 18, 26, 37] |
| Noduri rută Calea A | [0, 1, 3, 6, 8, 9] — edges [10, 14, 20, 28, 34] |
| Interval reload graf | 30s |

### 8.4 Validare selecție traseu semantic

**Demonstrat:** Route Server alege corect calea cu penalty total mai mic.

| Condiție | Calea aleasă | Comportament |
|----------|-------------|-------------|
| Fără obiecte | B (50.46 < 57.76) | Calea mai scurtă/directă ✓ |
| Persoană lângă Calea B | A (parțial) | Evită zona cu persoană ✓ |
| Penalty manual 50.0 pe Calea B | A | Schimbare completă traseu ✓ |

---

## 9. Limitări Cunoscute

1. **Obiecte care nu intersectează planul LiDAR:** LiDAR-ul la 16cm înălțime vede picioarele oamenilor și baza mobilierului, dar nu obiecte pe masă sau suspendate.

2. **Obiecte la limita FOV:** La marginile FOV-ului camerei (±25°), doar 1-2 raze LiDAR pot fi disponibile. Parametrul `min_valid_rays` protejează contra măsurătorilor nesigure.

3. **Obiecte în spatele altor obiecte:** LiDAR-ul returnează prima suprafață. Dacă YOLO detectează un om în spatele unui scaun, distanța LiDAR va fi cea a scaunului.

4. **Metadata GeoJSON — restricție Nav2 (float only):** GeoJsonGraphFileLoader din Nav2 1.3.10 nu suportă metadata nested sau string-uri. Soluție implementată: conversie la metadata minimală (doar `penalty` + `speed_limit`).

5. **MPPI poate eșua pe path-uri dense aproape de costmap:** Path-ul dens generat de Route Server trece pe coordonatele exacte ale grafului. Dacă acele coordonate coincid cu zone inflate în costmap, MPPI raportează `Failed to make progress` și abortează. Soluție: ajustarea coordonatelor nodurilor din graf pentru a fi mai departe de obstacole.

6. **Race condition la reload:** Dacă scriptul de test trimite ComputeRoute simultan cu `set_route_graph`, Route Server poate refuza goal-ul. Soluție: așteptare 5s după `Route Server: graf reîncărcat cu succes`.

7. **Acumulare obiecte vechi:** `semantic_objects.geojson` acumulează obiecte din toate sesiunile anterioare. Obiectele `static` nu expiră niciodată. Dacă obiectele vechi poluează grafică, resetează manual:

```bash
echo '{"type": "FeatureCollection", "features": []}' > \
  /home/saim/dummybot-ros2-encoder/ros2_ws/maps/semantic_objects.geojson
```

8. **Nu reactiv în timp real:** Traseul se recalculează la fiecare 30s, nu instant la apariția unui obstacol.

---

## 10. Instrucțiuni de Deploy

### 10.1 Structura pachetului

```
semantic_localizer/
├── semantic_localizer/
│   ├── __init__.py
│   ├── semantic_localizer_node.py     ← Nodul principal: fuziune + tracking + persistence + reload
│   └── semantic_map_manager.py        ← Persistență obiecte + anotare graf Nav2
├── launch/
│   ├── semantic_localizer_launch.py   ← Launch file activ
│   └── route_server_launch.py         ← REDUNDANT — Route Server e în Nav2 lifecycle
├── config/
│   └── semantic_localizer_params.yaml ← Toți parametrii, inclusiv nav2_graph_filepath
├── resource/semantic_localizer
├── package.xml
├── setup.py
└── setup.cfg

ros2_ws/
├── maps/
│   ├── route_graph.json                      ← Graf de bază (NU se modifică manual)
│   ├── semantic_objects.geojson              ← Harta obiectelor (auto-generat, 30s)
│   ├── route_graph_dummybot_semantic.geojson ← Graf anotat complet (auto-generat, 30s)
│   └── route_graph_dummybot_nav2.geojson     ← Graf Nav2-compatibil (auto-generat, 30s)
├── convert_geojson_for_route_server.py       ← Script conversie (importat de nod)
├── test_semantic_navigation.py               ← Suite teste: ComputeRoute, FollowPath, baseline
└── test_reverse_navigation.py                ← Test navigație reverse: goal(9) → start(0)
```

### 10.2 Build și lansare

```bash
# Build
cd /home/saim/dummybot-ros2-encoder/ros2_ws
colcon build --packages-select amr2ax_nav2 semantic_localizer --symlink-install
source install/setup.bash

# Pornire Nav2 (include Route Server automat prin lifecycle manager)
# [folosești comanda ta obișnuită de lansare Nav2]

# Verificare Route Server activ
ros2 lifecycle get /route_server  # trebuie: active [3]

# Pornire Semantic Localizer
ros2 launch semantic_localizer semantic_localizer_launch.py
```

### 10.3 Bootstrap — fișier nav2.geojson lipsă

Dacă `route_graph_dummybot_nav2.geojson` nu există (prima rulare sau după resetare), Route Server nu pornește. Generează-l manual:

```bash
python3 -c "
import json
with open('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph.json') as f:
    data = json.load(f)
for feat in data['features']:
    if 'startid' in feat['properties']:
        feat['properties']['metadata'] = {'penalty': 0.0, 'speed_limit': 100.0}
with open('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_dummybot_nav2.geojson', 'w') as f:
    json.dump(data, f, indent=2)
print('Done')
"
```

Semantic_localizer va suprascrie cu valorile reale la primul ciclu de 30s.

### 10.4 Resetare bază de date semantică

Pentru teste curate (fără obiecte acumulate din sesiuni anterioare):

```bash
# Opreste semantic_localizer mai intai
echo '{"type": "FeatureCollection", "features": []}' > \
  /home/saim/dummybot-ros2-encoder/ros2_ws/maps/semantic_objects.geojson
rm -f /home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_dummybot_semantic.geojson
rm -f /home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_dummybot_nav2.geojson
# Regenereaza nav2.geojson (bootstrap) si reporneste totul
```

### 10.5 Rulare teste

```bash
# Test 1: Planificare semantică (fără mișcare fizică)
python3 /home/saim/dummybot-ros2-encoder/ros2_ws/test_semantic_navigation.py --test 1

# Test 2: Navigație semantică fizică (ROBOTUL SE MIȘCĂ)
# Asteapta mesajul "Route Server: graf reîncărcat cu succes" in log, apoi:
python3 /home/saim/dummybot-ros2-encoder/ros2_ws/test_semantic_navigation.py --test 2

# Test 3: Baseline NavigateToPose (fără semantică, ROBOTUL SE MIȘCĂ)
python3 /home/saim/dummybot-ros2-encoder/ros2_ws/test_semantic_navigation.py --test 3

# Test reverse: navigație înapoi goal(9) → start(0)
python3 /home/saim/dummybot-ros2-encoder/ros2_ws/test_reverse_navigation.py
```

### 10.6 Verificare funcționare

```bash
# Diagnostice semantic_localizer (la fiecare 10s în log):
# [INFO] Semantic: 3 det | 2 tracked (2 stable) | map: 5 persistent {'static': 2, 'dynamic': 3} | graph=True

# Verificare penalty-uri curente în graf:
python3 -c "
import json
with open('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_dummybot_nav2.geojson') as f:
    data = json.load(f)
for feat in data['features']:
    props = feat['properties']
    if 'startid' in props:
        meta = props.get('metadata', {})
        print(f'  {props[\"startid\"]}→{props[\"endid\"]}: penalty={meta.get(\"penalty\",0):.2f}')
"

# Verificare Route Server:
ros2 lifecycle get /route_server           # active [3]
ros2 service list | grep set_route_graph   # /route_server/set_route_graph disponibil
ros2 action list | grep route              # /compute_route disponibil
```

### 10.7 Parametri configurabili

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
| `semantic_objects_filepath` | `.../maps/semantic_objects.geojson` | Fișier persistență obiecte |
| `route_graph_filepath` | `.../maps/route_graph.json` | Graful de bază |
| `output_graph_filepath` | `.../maps/route_graph_dummybot_semantic.geojson` | Graful anotat output |
| `nav2_graph_filepath` | `.../maps/route_graph_dummybot_nav2.geojson` | Graful Nav2-compatibil |
| `convert_script_path` | `.../convert_geojson_for_route_server.py` | Script conversie |
| `save_interval` | 30.0 | Salvare + reload periodic (s) |
| `cleanup_interval` | 10.0 | Cleanup TTL periodic (s) |
| `edge_proximity` | 1.5 | Distanță max obiect↔edge pentru anotare (m) |

### 10.8 Notă importantă — TF2 timestamp

În `_detections_cb`, timestamp-ul pentru TF lookup **trebuie** să fie:
```python
ros_now = Time(seconds=0, nanoseconds=0)
```
Aceasta folosește cea mai recentă transformare TF disponibilă. Utilizarea `self.get_clock().now()` sau `Time.from_msg(scan.stamp)` cauzează erori de extrapolare deoarece AMCL publică TF-ul `map→odom` cu întârziere.

---

## 11. Pașii Următori

### Implementat ✅

- **Pas 1:** Fuziune YOLO + LiDAR 2D → poziții obiecte în `/map`
- **Pas 2.1:** Persistență obiecte — `semantic_objects.geojson`
- **Pas 2.2:** Graf de navigație DummyBot — `route_graph.json` (10 noduri, 28 edges)
- **Pas 2.3:** Asociere obiecte ↔ edges (geometrie punct-segment)
- **Pas 2.4:** Injectare metadata Nav2 (`penalty`, `speed_limit`)
- **Pas 3.1:** Conversie GeoJSON → format Nav2-compatibil (doar penalty + speed_limit float)
- **Pas 3.2:** Route Server integrat în lifecycle Nav2 cu PenaltyScorer + DistanceScorer + AdjustSpeedLimit
- **Pas 3.3:** Test ComputeRoute — alegere corectă traseu semantic
- **Pas 3.4:** Test ComputeRoute + FollowPath — navigație fizică pe traseu semantic
- **Sprint 2:** Reload dinamic automat graf prin `set_route_graph` service la fiecare 30s ✅

### De implementat 🔧

- **Sprint 3: Scenarii de navigație repetate (2-3 zile)**
  - 5-10 run-uri per condiție (semantic forward, semantic reverse, baseline)
  - Metrici: timp, distanță parcursă, distanță minimă față de obiecte semantice
  - Scenariul adaptiv demonstrat: obiect apare → penalty crește → traseu se schimbă la 30s

- **Sprint 4: Ground truth și metrici (2-3 zile)**
  - Marcare 20-30 poziții obiecte (bandă pe podea, măsurători de la landmarks)
  - Comparație `semantic_objects.geojson` vs ground truth → RMSE, MAE per clasă
  - Ablation: percentilă (10/25/50/75), confidence (0.3/0.45/0.6)

**Estimare totală Sprint 3-4:** ~5-7 zile lucru

### Amânat / Future Work 📋

- Replanificare în timp real (sub 1s) la detectarea unui obiect nou lângă traseu
- Navigare la obiect detectat (ex: "mergi la frigider") — necesită nod lookup semantic
- Comparație cu deep fusion sau metode RGBD

---

## Troubleshooting Reference

| Problemă | Cauză | Soluție |
|----------|-------|---------|
| `bad any_cast` la configure | Metadata nested/string în GeoJSON | Folosește `route_graph_dummybot_nav2.geojson` (doar penalty + speed_limit) |
| Path cu 0-1 puncte | `use_poses=False` în ComputeRoute | Setează `use_poses=True` |
| Route Server nu pornește | `route_graph_dummybot_nav2.geojson` lipsă | Rulează scriptul de bootstrap (secțiunea 10.3) |
| `ComputeRoute goal REJECTED` | Race condition cu reload graf | Așteaptă 5s după `Route Server: graf reîncărcat cu succes` |
| MPPI `Failed to make progress` | Path dens trece prin zona inflată din costmap | Ajustează coordonatele nodurilor din `route_graph.json` |
| `Nu am putut încărca scriptul de conversie: load_from_spec` | Eroare importlib | Înlocuiește `load_from_spec` cu `module_from_spec` în `_load_convert_fn` |
| Penalty-uri prea mici pentru a schimba traseul | Obiecte de tip `minor` (penalty_base=0.5) | Folosește obiecte `dynamic` (persoană) sau `static` (scaun) |
| Obiectele vechi influențează toate edges-urile | Acumulare din sesiuni anterioare | Resetează `semantic_objects.geojson` (secțiunea 10.4) |
| `Traseu greșit ales` | PenaltyScorer weight prea mic | Crește weight-ul (curent: 5.0) |
