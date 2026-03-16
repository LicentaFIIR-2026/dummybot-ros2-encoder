# nav2-semantic-route-server

**AIONIS / DummyBot (Xplorer-C)** — Lightweight semantic-aware route planning on edge hardware.  
Dual Raspberry Pi 5 architecture with YOLO26 inference offloading, Angular Sector Fusion (ASF), Nav2 Route Server with penalty-weighted replanning.

> Based on: *"Lightweight Semantic-Aware Route Planning on Edge Hardware for Indoor Mobile Robots: Monocular Camera–2D LiDAR Fusion with Penalty-Weighted Nav2 Route Server Replanning"* — Abaza, Staicu, Doicin — MDPI Sensors (under review)

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red) ![Nav2](https://img.shields.io/badge/Nav2-1.3.10-green) ![Branch](https://img.shields.io/badge/branch-nav2--semantic--route--server-orange)

---

## Quick Start — Terminal Launch Order

> All terminals on **saim** (RPi5 principal, `192.168.53.159 / 10.2.0.2`) unless specified.  
> SSH into **nav** (RPi5 inferenta, `10.2.0.1`) din terminalele de pe saim.

### Terminal 1 — Robot bringup (saim)
```bash
cd dummybot-ros2-encoder/ros2_ws
source install/setup.bash
ros2 launch dummybot_bringup robot.launch.py
```

### Terminal 2 — Navigatie Nav2 + AMCL (saim)
```bash
cd dummybot-ros2-encoder/ros2_ws
source install/setup.bash
ros2 launch amr2ax_nav2 navxplorer.launch.py \
  localization_type:=2D \
  slam:=False \
  use_sim_time:=False
```

### Terminal 3 — SSH in nav + YOLO26 inference (saim → nav)
```bash
ssh nav@10.2.0.1
cd ros2_ws
source install/setup.bash
ros2 launch yolo26_cpp yolo26_cpp_launch.py \
  model_path:=/home/nav/yolo26/yolo26n_ncnn_model
```

### Terminal 4 — Activare lifecycle YOLO (saim → nav, terminal nou)
```bash
ssh nav@10.2.0.1
ros2 lifecycle set /yolo26_detector configure
ros2 lifecycle set /yolo26_detector activate
```

### Terminal 5 — Semantic Localizer (saim)
```bash
cd dummybot-ros2-encoder/ros2_ws
source install/setup.bash
ros2 launch semantic_localizer semantic_localizer_launch.py
```

### Terminal 6 — Route Server (saim)
```bash
cd dummybot-ros2-encoder/ros2_ws
source install/setup.bash
ros2 run nav2_route route_server --ros-args \
  --params-file /home/saim/dummybot-ros2-encoder/ros2_ws/src/semantic_localizer/config/route_server_params.yaml \
  -r plan:=route_plan
```

### Terminal 7 — Activare lifecycle Route Server (saim)
```bash
ros2 lifecycle set /route_server configure && ros2 lifecycle set /route_server activate
```

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    RPi5 "saim" (10.2.0.2)                       │
│                                                                  │
│  ┌─────────────┐    ┌──────────────────┐    ┌────────────────┐  │
│  │  dummybot   │    │  amr2ax_nav2     │    │   semantic_    │  │
│  │  _bringup   │    │  Nav2 + AMCL +   │    │   localizer    │  │
│  │             │    │  MPPI + EKF      │    │   (ASF node)   │  │
│  │ ESP32 Motor │    │                  │    │                │  │
│  │ LD19 LiDAR  │    │  Route Server    │    │  semantic_map  │  │
│  │ C920 Camera │    │  (standalone     │    │  _manager      │  │
│  │ BNO055 IMU  │    │   lifecycle)     │    │                │  │
│  └──────┬──────┘    └────────┬─────────┘    └───────┬────────┘  │
│         │                   │                       │           │
│         └───────────────────┴───────────────────────┘           │
│                         ROS2 DDS (ROS_DOMAIN_ID=67)             │
│                         Ethernet eth0  10.2.0.2/24              │
└──────────────────────────────┬──────────────────────────────────┘
                               │ Ethernet direct (~0.2ms latency)
┌──────────────────────────────┴──────────────────────────────────┐
│                    RPi5 "nav" (10.2.0.1)                        │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  yolo26_cpp (lifecycle node)                             │   │
│  │  NCNN backend — 5.5 ± 0.7 FPS — 167 ± 21 ms latency    │   │
│  │  Subscribes: /camera/image_raw (de pe saim via DDS)     │   │
│  │  Publishes:  /yolo26/detections  /yolo26/diagnostics    │   │
│  └──────────────────────────────────────────────────────────┘   │
│                         ROS_DOMAIN_ID=67                        │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow

```
Camera (C920)                LiDAR (LD19)
     │                            │
     ▼                            ▼
/camera/image_raw          /scan (503 rays, 10Hz)
     │                            │
     ▼                            │
yolo26_cpp (nav RPi)              │
[NCNN, 4 threads, 416x416]        │
     │                            │
     ▼                            │
/yolo26/detections ───────────────┤
                                  ▼
                       semantic_localizer (ASF)
                       [3 Hz, <1ms/detection]
                                  │
                                  ▼
                       semantic_objects.geojson
                       route_graph_fiir_nav2.geojson
                                  │
                                  ▼
                          Route Server
                    [ComputeRoute → FollowPath]
                    [PenaltyScorer weight=5.0]
                    [DistanceScorer weight=1.0]
                                  │
                                  ▼
                         MPPI Controller
                      [vx_max=0.5 m/s, 20Hz]
```

---

## Hardware

| Component | Spec |
|---|---|
| Compute (saim) | Raspberry Pi 5, 16 GB RAM, ARM Cortex-A76, Ubuntu 24.04 |
| Compute (nav) | Raspberry Pi 5, dedicat inferenta YOLO |
| Camera | Logitech C920 HD, 640×480 @ 30fps, fx=687.54px, cx=308.11px |
| LiDAR | STL-19P / LD19, 360°, 503 raze, 0.716° increment, 10Hz |
| IMU | BNO055 (I2C) |
| Motor control | ESP32 cu encodere Hall effect, UART → ttyUSB0 (CP2102) |
| Drivetrain | 4WD differential, 223 rpm, quadrature encoders |
| Power | 12V LiPo + BMS, RPi5 via USB-PD 5V |

### TF Frames

| Transform | Translation | Rotation |
|---|---|---|
| base_link → camera_link_optical | [0.225, 0, 0.140] m | -90° roll, -90° yaw |
| base_link → base_laser | [0.185, 0, 0.210] m | zero |

---

## Software Packages

```
ros2_ws/src/
├── semantic_localizer/          # ASF pipeline + SemanticMapManager
│   ├── semantic_localizer/
│   │   ├── semantic_localizer_node.py     # nod principal (activ)
│   │   └── semantic_map_manager.py        # persistenta GeoJSON
│   ├── config/
│   │   ├── route_server_params.yaml       # config Route Server
│   │   └── semantic_localizer_params.yaml
│   └── launch/
│       └── semantic_localizer_launch.py
│
├── yolo26_cpp/                  # Detector C++ YOLO26 via NCNN (pe nav)
│   ├── src/
│   │   ├── yolo26_detector_ncnn.cpp       # backend NCNN
│   │   └── yolo26_node.cpp                # lifecycle node
│   ├── config/yolo26_params.yaml
│   └── launch/yolo26_cpp_launch.py
│
├── yolo26_ros/                  # Detector Python YOLO26 (alternativa saim)
│
├── amr2ax_nav2/                 # Stack navigatie Nav2
│   ├── config/
│   │   ├── xplorer.yaml                   # config Nav2 + Route Server
│   │   └── route_graph.json               # graf navigatie activ
│   ├── scripts/
│   │   ├── semantic_localizer.py          # versiune standalone
│   │   ├── semantic_navigation.py         # logica navigatie semantica
│   │   └── dynamic_blocker.py             # CLI blocare muchii manuale
│   └── launch/
│       └── navxplorer.launch.py
│
├── dummybot_bringup/            # Launch robot fizic
│   ├── bringup/launch/robot.launch.py
│   └── description/urdf/dummybot.urdf.xacro
│
└── media_pipe_ros2/             # MediaPipe (versiune anterioara, pastrat)
    └── media_pipe_ros2/
        └── mediapipe_standalone.py
```

### Experimente

```
ros2_ws/
├── experiments/                 # 27 sesiuni, 115 legs, JSON per run
│   └── session_YYYY-MM-DD_HH-MM-SS/
│       ├── session_config.json
│       ├── semantic_snapshot_start.geojson
│       ├── test1_compute_route.json
│       ├── test2_runNN_forward.json
│       ├── test2_runNN_return.json
│       ├── test3_runNN_forward.json
│       └── summary.json
├── maps/                        # GeoJSON graf + harti semantice
│   ├── route_graph_fiir_nav2.geojson      # graf Nav2-compatible
│   ├── route_graph_fiir_semantic.geojson  # graf complet cu metadata
│   └── semantic_objects.geojson           # harta semantica persistenta
└── test_semantic_navigation_v5.*.py       # framework test automatizat
```

---

## Angular Sector Fusion (ASF)

Metoda geometrica determinista pentru localizarea obiectelor detectate de camera in frame-ul `/map`, prin fuziunea bounding box-urilor YOLO cu masuratorile LiDAR 2D.

```
Bounding box (px)          LiDAR scan (/scan)
  [x_min, x_max]               503 raze
       │                           │
       ▼                           │
  θ_left  = atan2((x_min - cx) / fx, 1.0)
  θ_right = atan2((x_max - cx) / fx, 1.0)
       │                           │
       ▼                           ▼
  sector LiDAR [idx_min, idx_max] ──► raze valide in sector
                                           │
                                           ▼
                                   d = percentile_25(ranges)
                                   (robustete la ocluzie partiala)
                                           │
                                           ▼
                              (x_laser, y_laser) polar → cartezian
                                           │
                                           ▼
                                    TF2: base_laser → map
                              (timestamp=0 pentru ultima transformare)
                                           │
                                           ▼
                               median filter (W=5 pozitii)
                                           │
                                           ▼
                              semantic_objects.geojson
```

**Parametri auto-configurati la runtime din topic-uri ROS2:**
- `fx`, `cx` — din `/camera/camera_info` (CameraInfo)
- geometrie scan — din `/scan` (LaserScan)
- transformari — din TF2 / URDF (fara hardcode)

**Overhead computational:** sub 1ms/detectie, ~2-5% CPU la 3Hz pe RPi5.

---

## Navigation Graph

Graful de navigatie contine 7 noduri si 20 de muchii directionate (10 perechi bidirecţionale), encodat ca GeoJSON in formatul Nav2 Route Server.

```
           [3] ──────── [2] ──────── [5]
          / SUS          │ HUB        \ SUS
[0] ────/               │             \──── [7]
START   \               │ DIRECT      /    GOAL
         \ JOS          │            / JOS
           [4] ──────── .  ──────── [6]

Rute disponibile (start → goal):
  Upper (SUS):  0 → 3 → 2 → 5 → 7
  Lower (JOS):  0 → 4 → 2 → 6 → 7
  Direct:       0 → 2 → 7          (selectata in 52% din cazuri)
```

### Scoring Route Server

| Plugin | Weight | Functie |
|---|---|---|
| `DistanceScorer` | 1.0 | Cost geometric (lungime muchie × speed_limit) |
| `PenaltyScorer` | 5.0 | Citeste `penalty` din metadata GeoJSON |

**Exemplu:** o persoana detectata langa o muchie → penalty ≈ 25-40 → cost ponderat 125-200 unitati vs. cost distanta 1-3 unitati pentru o muchie de 1-2m. Routerul evita activ muchia.

### Clasificare mobilitate obiecte

| Categorie | Exemple | Penalty baza | Speed limit | TTL |
|---|---|---|---|---|
| `dynamic` | persoana | 50 | 30% | 60s |
| `static` | mobila, scaun | 20 | 60% | permanent |
| `minor` | sticla, cana | 10 | 90% | 120s |

---

## Ethernet Setup: saim ↔ nav

### 1. Pregatirea mediului pe nav

Instalarea dependentelor si compilarea NCNN din surse (ARM nu are pachete precompilate in apt):

```bash
sudo apt install cmake git build-essential libopencv-dev
git clone https://github.com/Tencent/ncnn.git
cd ncnn && git submodule update --init
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DNCNN_BUILD_EXAMPLES=OFF -DNCNN_SHARED_LIB=ON ..
make -j4 && sudo make install && sudo ldconfig
```

### 2. Copierea pachetului si modelului de pe saim

```bash
scp -r saim@192.168.53.159:/home/saim/dummybot-ros2-encoder/ros2_ws/src/yolo26_cpp ~/ros2_ws/src/
scp -r saim@192.168.53.159:/home/saim/yolo26 ~/yolo26
```

### 3. Compilarea pachetului pe nav

```bash
cd ~/ros2_ws
colcon build --packages-select yolo26_cpp \
  --cmake-args -Dncnn_DIR=/usr/local/lib/cmake/ncnn
```

### 4. Environment pe nav (de adaugat in ~/.bashrc)

```bash
export ROS_DOMAIN_ID=67
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 5. Configurare conexiune Ethernet directa

Nav avea deja IP static `10.2.0.1/24` configurat pe `eth0`. Pe saim se adauga IP-ul static prin netplan, editand `/etc/netplan/50-cloud-init.yaml`:

```yaml
ethernets:
  eth0:
    dhcp4: false
    dhcp6: false
    addresses:
      - 10.2.0.2/24
```

Aplicarea configuratiei:

```bash
sudo netplan apply
ping 10.2.0.1   # verificare conectivitate, latenta ~0.2ms
```

### 6. Dezactivarea WiFi pe nav

Odata conexiunea ethernet stabilita, WiFi-ul de pe nav se dezactiveaza pentru a forta tot traficul ROS2 prin ethernet:

```bash
sudo ip link set wlan0 down
```

Accesul SSH la nav se face exclusiv prin ethernet de pe saim:

```bash
ssh nav@10.2.0.1
```

**Topicuri cross-board:** nodul YOLO subscrie la `/camera/image_raw` de pe saim si publica detectiile pe `/yolo26/detections`, topic vizibil pe intreaga retea ROS2 prin `ROS_DOMAIN_ID=67`.

---

## Experimental Results

Evaluare pe **115 navigation legs** (45 baseline, 70 adaptive) pe 3 roboti (Xplorer-A, B, C) in coridorul FIIR, etajul 2, UPB.

| Metric | Baseline | Adaptive |
|---|---|---|
| Success rate | 100% (45/45) | 94% (66/70) |
| Forward nav time (mean) | 37.3 ± 31.1 s | 32.5 ± 11.0 s |
| Forward nav time (median) | 25.4 s | 29.8 s |
| Replanning events | — | 42 (in 57% din legs) |
| CPU load (saim) | ~85% | ~85% |
| CPU load (nav, YOLO offload) | — | ~48% |
| YOLO FPS (nav) | — | 5.5 ± 0.7 FPS |
| YOLO latency | — | 167 ± 21 ms |

**Observatia cheie:** adaptivul nu reduce semnificativ timpul mediu (Mann-Whitney U p=0.157), dar reduce dramatic varianta (σ: 31.1s → 11.0s, ratio varianta ≈ 8x; Levene W=3.14, p=0.082). Rutarea bazata pe graf produce navigatie mult mai predictibila decat free-space planning.

Toate cele 4 esecuri adaptive au fost cauzate de AMCL localization drift pe Xplorer-B, nu de pipeline-ul semantic.

---

## Known Issues / Nav2 Quirks

### 1. GeoJSON metadata constraint
Nav2 Route Server 1.3.10 accepta **doar valori scalare float** in metadata edge-urilor. Orice array, object sau string cauzeaza `bad any_cast` la configurare. Solutia: dual-graph architecture:
- `route_graph_fiir_semantic.geojson` — metadata completa (pentru analiza)
- `route_graph_fiir_nav2.geojson` — doar `penalty` si `speed_limit` ca float (pentru Route Server)

### 2. Path-gap phenomenon
Route Server genereaza path-ul dens pornind de la cel mai apropiat nod din graf, nu de la pozitia curenta a robotului. Daca distanta robot-start_path > 1.0m, MPPI termina imediat raportand success fara sa se miste. **Fix:** se prepend waypoints interpolate la 0.15m intervale de la robot la primul punct al path-ului.

### 3. `use_start=True` dupa AMCL reinitializare
Dupa orice reinitializare AMCL, `ComputeRoute` trebuie apelat cu `use_start=True` si pozitia explicita din AMCL pose. Fara asta, transformarea `map→odom` poate sa nu fi converge inca si Route Server genereaza un start gresit.

### 4. TF2 timestamp zero
In ASF pipeline, TF lookup-urile se fac cu `Time(seconds=0, nanoseconds=0)` pentru a obtine cea mai recenta transformare disponibila. AMCL publica `map→odom` cu o intarziere variabila de 1-5s; folosind timestamp-ul scan-ului ar rezulta erori de extrapolate.

### 5. Route Server standalone lifecycle
Route Server-ul **nu trebuie** inclus in Nav2 lifecycle manager. Trebuie pornit ca nod lifecycle standalone pentru a putea fi reloaded independent (via `set_route_graph` service) fara a afecta stack-ul de navigatie.

---

## Citation

Daca folosesti acest cod sau arhitectura in lucrarea ta, citeaza:

```bibtex
@article{abaza2026lightweight,
  title={Lightweight Semantic-Aware Route Planning on Edge Hardware for Indoor Mobile Robots:
         Monocular Camera--2D LiDAR Fusion with Penalty-Weighted Nav2 Route Server Replanning},
  author={Abaza, Bogdan Felician and Staicu, Andrei-Alexandru and Doicin, Cristian Vasile},
  journal={Sensors},
  year={2026},
  publisher={MDPI}
}
```

---

## Repository Structure

```
dummybot-ros2-encoder/
├── ros2_ws/
│   ├── src/                     # pachete ROS2
│   ├── experiments/             # dataset complet (27 sesiuni)
│   ├── maps/                    # GeoJSON grafuri + harti semantice
│   └── test_semantic_navigation_v5.*.py   # framework test
├── firmware/                    # ESP32 motor control
└── scripts/
```

**Branch history:**
- `semantic-navigation-1.0` — Route Server + MediaPipe + dynamic blocker
- `semantic-navigation-1.1` — MediaPipe standalone + semantic localizer (Python)
- `nav2-semantic-route-server` — YOLO26 C++ NCNN + dual-RPi + ASF complet + experimente (acest branch)
