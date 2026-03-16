#!/usr/bin/env python3
"""
SAIM Xplorer — Semantic Navigation Experimental Test Suite v5.9
===============================================================

SCHIMBARI FATA DE v5.7:
  - Zero valori hardcodate: GRAPH_NODES, EDGE_PAIRS, TARGET_YAW, paths,
    start/goal node IDs, thresholds — toate vin din route_config.yaml
  - RouteConfig: clasa care citeste YAML si construieste GRAPH_NODES +
    EDGE_PAIRS automat din GeoJSON sursa (nu mai e nevoie de sync manual)
  - identify_route() generica: logica SUS/JOS vine din route_alternatives
    in config, nu mai e hardcodata pentru topologia cb204v1
  - nav2_graph generat automat la startup daca lipseste sau e outdated
    (foloseste SemanticMapManager.save_nav2_graph — fara script extern)
  - Argparse: adaugat --config pentru specificarea fisierului de config

Protocol:
  Setup  (--test 0): Reset map, AMCL reinit, wait repopulation, regen graph
  Test 1 (--test 1): ComputeRoute only (verifica decizia de routing)
  Test 2 (--test 2): Baseline NavigateToPose, fara semantica
  Test 3 (--test 3): Adaptive semantic pre-scan + replanning
  All    (--test all): Setup + Test1 + Test2 + Test3

Usage:
  python3 test_semantic_navigation_v5_9.py --config route_config.yaml --test all --runs 5
  python3 test_semantic_navigation_v5_9.py --config route_config_7nodes.yaml --test 2 --runs 10
"""

import sys
import os
import json
import time
import shutil
import argparse
import math
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, List, Dict, Any, Tuple

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputeRoute, FollowPath, NavigateToPose, Spin
from nav2_msgs.srv import SetRouteGraph
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False


# =============================================================================
# ROUTE CONFIG — citeste YAML + construieste GRAPH_NODES / EDGE_PAIRS din GeoJSON
# =============================================================================

class RouteConfig:
    """
    Singura sursa de adevar pentru topologie si parametri experiment.

    La instantiere:
      1. Citeste route_config.yaml
      2. Citeste graph_filepath (GeoJSON) -> construieste graph_nodes si edge_pairs
      3. Asigura ca nav2_graph exista si e actualizat (genereaza daca nu)

    Toate campurile sunt disponibile ca atribute publice dupa __init__.
    """

    # Valori default pentru parametri optionali din YAML
    _DEFAULTS = {
        'odom_topic':              '/odom',
        'odom_sample_hz':          2.0,
        'max_start_offset_m':      2.0,
        'max_success_error_m':     2.0,
        'min_success_dist_m':      1.0,
        'min_success_duration_s':  3.0,
        'max_consecutive_fails':   2,
        'prescan_wait_secs':       35,
        'replan_check_interval_s': 2.0,
        'node_proximity_thresh_m': 0.5,
        'route_alternatives':      {},
        'valid_classes': [
            'person', 'chair', 'bench', 'bottle', 'cup', 'backpack',
            'suitcase', 'potted plant', 'tv', 'laptop', 'book',
        ],
    }

    def __init__(self, config_path: str):
        self._config_path = Path(config_path).resolve()
        if not self._config_path.exists():
            raise FileNotFoundError(f'Config not found: {self._config_path}')

        raw = self._load_yaml(self._config_path)

        # ── Campuri obligatorii ──────────────────────────────────────
        for req in ('graph_filepath', 'start_node_id', 'goal_node_id',
                    'start_yaw_deg', 'goal_yaw_deg',
                    'semantic_objects_path', 'nav2_graph_path',
                    'semantic_graph_path', 'experiments_dir'):
            if req not in raw:
                raise ValueError(f'route_config.yaml: camp obligatoriu lipsa: {req}')

        # ── Identitate ───────────────────────────────────────────────
        self.route_name:   str = raw.get('route_name', 'unnamed')
        self.description:  str = raw.get('description', '')

        # ── Topologie: din GeoJSON ───────────────────────────────────
        self.graph_filepath = Path(raw['graph_filepath'])
        if not self.graph_filepath.exists():
            raise FileNotFoundError(f'graph_filepath nu exista: {self.graph_filepath}')

        self.graph_nodes, self.edge_pairs = self._parse_geojson(self.graph_filepath)

        # ── Noduri terminale ─────────────────────────────────────────
        self.start_node_id: int = int(raw['start_node_id'])
        self.goal_node_id:  int = int(raw['goal_node_id'])

        for nid, label in [(self.start_node_id, 'start_node_id'),
                           (self.goal_node_id,  'goal_node_id')]:
            if nid not in self.graph_nodes:
                raise ValueError(
                    f'{label}={nid} nu exista in {self.graph_filepath.name}. '
                    f'Noduri disponibile: {sorted(self.graph_nodes.keys())}'
                )

        # ── Orientari terminale (radiani) ────────────────────────────
        self.start_yaw: float = math.radians(float(raw['start_yaw_deg']))
        self.goal_yaw:  float = math.radians(float(raw['goal_yaw_deg']))

        # ── Route alternatives ───────────────────────────────────────
        raw_alt = raw.get('route_alternatives', {})
        # Normalizeaza: valorile pot fi liste de int sau string
        self.route_alternatives: Dict[str, List[int]] = {
            label: [int(n) for n in nodes]
            for label, nodes in raw_alt.items()
        }

        # ── Paths sistem ─────────────────────────────────────────────
        self.semantic_objects_path = Path(raw['semantic_objects_path'])
        self.nav2_graph_path       = Path(raw['nav2_graph_path'])
        self.semantic_graph_path   = Path(raw['semantic_graph_path'])
        self.experiments_dir       = Path(raw['experiments_dir'])

        # ── ROS / parametri ──────────────────────────────────────────
        self.odom_topic:              str   = raw.get('odom_topic',              self._DEFAULTS['odom_topic'])
        self.odom_sample_hz:          float = float(raw.get('odom_sample_hz',    self._DEFAULTS['odom_sample_hz']))
        self.max_start_offset_m:      float = float(raw.get('max_start_offset_m',     self._DEFAULTS['max_start_offset_m']))
        self.max_success_error_m:     float = float(raw.get('max_success_error_m',    self._DEFAULTS['max_success_error_m']))
        self.min_success_dist_m:      float = float(raw.get('min_success_dist_m',     self._DEFAULTS['min_success_dist_m']))
        self.min_success_duration_s:  float = float(raw.get('min_success_duration_s', self._DEFAULTS['min_success_duration_s']))
        self.max_consecutive_fails:   int   = int(raw.get('max_consecutive_fails',    self._DEFAULTS['max_consecutive_fails']))
        self.prescan_wait_secs:       int   = int(raw.get('prescan_wait_secs',        self._DEFAULTS['prescan_wait_secs']))
        self.replan_check_interval_s: float = float(raw.get('replan_check_interval_s', self._DEFAULTS['replan_check_interval_s']))
        self.node_proximity_thresh_m: float = float(raw.get('node_proximity_thresh_m', self._DEFAULTS['node_proximity_thresh_m']))

        raw_vc = raw.get('valid_classes', self._DEFAULTS['valid_classes'])
        self.valid_classes: set = set(raw_vc)

        # ── Asigura nav2_graph exista ─────────────────────────────────
        self._ensure_nav2_graph()

    # ----------------------------------------------------------------- YAML

    @staticmethod
    def _load_yaml(path: Path) -> dict:
        """Citeste YAML cu fallback la JSON daca PyYAML nu e instalat."""
        text = path.read_text()
        if HAS_YAML:
            return yaml.safe_load(text)
        # Fallback minimal: incearca JSON (daca utilizatorul a salvat ca .json)
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            raise ImportError(
                'PyYAML nu e instalat si fisierul nu e JSON valid.\n'
                'Instaleaza cu: pip install pyyaml --break-system-packages'
            )

    # ----------------------------------------------------------------- GeoJSON -> graph

    @staticmethod
    def _parse_geojson(path: Path) -> Tuple[Dict[int, Dict], Dict[int, int]]:
        """
        Parseaza graph_filepath si construieste:
          graph_nodes: {node_id: {'name': str, 'x': float, 'y': float}}
          edge_pairs:  {edge_id_forward: edge_id_reverse, ...}

        Structura GeoJSON asteptata:
          - Noduri: Feature cu geometry.type='Point',
                    properties: {id: int, name: str (optional), frame: str}
          - Edges:  Feature cu geometry.type='LineString',
                    properties: {id: int, startid: int, endid: int, ...}

        edge_pairs e construit prin auto-discovery:
          Pentru fiecare edge A->B cu id=X, cauta edge B->A cu id=Y.
          => edge_pairs[X] = Y, edge_pairs[Y] = X
        """
        with open(path) as f:
            data = json.load(f)

        features = data.get('features', [])

        # ── Noduri ────────────────────────────────────────────────────
        graph_nodes: Dict[int, Dict] = {}
        for feat in features:
            if feat.get('geometry', {}).get('type') != 'Point':
                continue
            props = feat.get('properties', {})
            nid = props.get('id')
            if nid is None:
                continue
            coords = feat['geometry']['coordinates']
            graph_nodes[int(nid)] = {
                'name': props.get('name', f'node_{nid}'),
                'x':    float(coords[0]),
                'y':    float(coords[1]),
            }

        if not graph_nodes:
            raise ValueError(
                f'Niciun nod (Point feature) gasit in {path.name}. '
                'Verifica structura GeoJSON: features cu geometry.type=Point '
                'si properties.id (integer).'
            )

        # ── Edges — indexate dupa (startid, endid) ────────────────────
        edges_by_endpoints: Dict[Tuple[int, int], int] = {}  # (startid, endid) -> edge_id
        for feat in features:
            geom_type = feat.get('geometry', {}).get('type')
            if geom_type not in ('LineString', 'MultiLineString'):
                continue
            props = feat.get('properties', {})
            eid     = props.get('id')
            startid = props.get('startid')
            endid   = props.get('endid')
            if None in (eid, startid, endid):
                continue
            edges_by_endpoints[(int(startid), int(endid))] = int(eid)

        # ── Auto-discovery edge pairs ──────────────────────────────────
        # Per fiecare edge A->B cu id X, cauta perechea B->A cu id Y
        edge_pairs: Dict[int, int] = {}
        for (sa, ea), xid in edges_by_endpoints.items():
            rev = edges_by_endpoints.get((ea, sa))
            if rev is not None:
                edge_pairs[xid] = rev

        return graph_nodes, edge_pairs

    # ----------------------------------------------------------------- nav2 graph

    def _ensure_nav2_graph(self):
        """
        Asigura ca nav2_graph_path exista si e cel putin la fel de nou ca graph_filepath.
        Daca nu, genereaza fara obiecte semantice (structura pura).

        Foloseste SemanticMapManager.save_nav2_graph() — acelasi mecanism
        ca semantic_localizer_node, fara ROS node, fara script extern.
        """
        needs_regen = False

        if not self.nav2_graph_path.exists():
            print(f'  [RouteConfig] nav2_graph nu exista, il generez...')
            needs_regen = True
        else:
            src_mtime = self.graph_filepath.stat().st_mtime
            nav_mtime = self.nav2_graph_path.stat().st_mtime
            if src_mtime > nav_mtime:
                print(f'  [RouteConfig] graph_filepath mai nou decat nav2_graph, regenerez...')
                needs_regen = True

        if not needs_regen:
            return

        try:
            # Import local — semantic_map_manager.py trebuie sa fie in acelasi director
            sys.path.insert(0, str(Path(__file__).parent))
            from semantic_map_manager import SemanticMapManager

            mgr = SemanticMapManager(
                route_graph_filepath=str(self.graph_filepath),
                output_graph_filepath=str(self.semantic_graph_path),
            )
            ok = mgr.save_nav2_graph(str(self.nav2_graph_path))
            if ok:
                print(f'  [RouteConfig] nav2_graph generat: {self.nav2_graph_path.name}')
            else:
                print(f'  [RouteConfig] WARN: generare nav2_graph esuata, continui cu ce exista')
        except ImportError:
            print(
                '  [RouteConfig] WARN: semantic_map_manager.py nu e in PATH, '
                'nav2_graph nu a putut fi generat automat.'
            )
        except Exception as e:
            print(f'  [RouteConfig] WARN: eroare la generare nav2_graph: {e}')

    # ----------------------------------------------------------------- helpers

    def node(self, nid: int) -> Dict:
        """Returneaza datele unui nod. Raise daca nu exista."""
        if nid not in self.graph_nodes:
            raise KeyError(f'Node {nid} nu exista in graf. Noduri: {sorted(self.graph_nodes.keys())}')
        return self.graph_nodes[nid]

    def terminal_yaw(self, node_id: int) -> float:
        """Returneaza yaw-ul (radiani) pentru un nod terminal."""
        if node_id == self.start_node_id:
            return self.start_yaw
        if node_id == self.goal_node_id:
            return self.goal_yaw
        raise ValueError(f'Node {node_id} nu e nod terminal (start={self.start_node_id}, goal={self.goal_node_id})')

    def summary(self) -> str:
        n_nodes = len(self.graph_nodes)
        n_edges = len(self.edge_pairs) // 2
        alts = list(self.route_alternatives.keys()) or ['DIRECT']
        return (
            f'{self.route_name}: {n_nodes} noduri, ~{n_edges} perechi edges, '
            f'start={self.start_node_id} goal={self.goal_node_id}, '
            f'rute={alts}'
        )


# =============================================================================
# UTILITIES
# =============================================================================

# cfg e initializat in main() si folosit global in functii helper
cfg: Optional[RouteConfig] = None


def make_pose(x, y, yaw=0.0, stamp_sec=None, stamp_nanosec=None):
    p = PoseStamped()
    p.header.frame_id = 'map'
    if stamp_sec is not None:
        p.header.stamp.sec = stamp_sec
        p.header.stamp.nanosec = stamp_nanosec or 0
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def node_pose(nid: int, yaw=0.0) -> PoseStamped:
    n = cfg.node(nid)
    return make_pose(n['x'], n['y'], yaw)


def yaw_from_quat(oz, ow):
    return 2.0 * math.atan2(oz, ow)


def hdr(t):
    print('\n' + '=' * 70 + f'\n  {t}\n' + '=' * 70)


def sub(t):
    print(f'\n  --- {t} ---')


def ts_str():
    return datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


def iso_now():
    return datetime.now(timezone.utc).isoformat()


def identify_route(path_poses) -> Tuple[str, Dict]:
    """
    Identifica tipul rutei comparand path-ul cu route_alternatives din config.

    Generica: nu mai contine logica SUS/JOS hardcodata.
    Fiecare alternativa e caracterizata de noduri intermediare din config.
    Daca path-ul trece la < 0.5m de cel putin un nod din lista, acea ruta e activa.
    Daca multiple alternative sunt active: 'MIX'.
    Daca nicio alternativa nu se potriveste: 'DIRECT'.
    """
    if not path_poses:
        return 'UNKNOWN', {}

    MATCH_DIST = 0.5  # metri

    # Precalculeaza coordonatele nodurilor pentru fiecare alternativa
    alt_nodes: Dict[str, List[Dict]] = {}
    for label, nids in cfg.route_alternatives.items():
        alt_nodes[label] = [cfg.graph_nodes[n] for n in nids if n in cfg.graph_nodes]

    if not alt_nodes:
        return 'DIRECT', {}

    # Distanta minima de la path la fiecare nod intermediar
    min_dists: Dict[int, float] = {}
    for label, nodes in alt_nodes.items():
        for n in nodes:
            key = (label, n['name'])
            min_dists[key] = float('inf')

    for p in path_poses:
        px, py = p.pose.position.x, p.pose.position.y
        for label, nodes in alt_nodes.items():
            for n in nodes:
                d = math.sqrt((px - n['x'])**2 + (py - n['y'])**2)
                key = (label, n['name'])
                if d < min_dists.get(key, float('inf')):
                    min_dists[key] = d

    # Determina care alternative sunt vizitate
    active = set()
    visited_per_label: Dict[str, List[str]] = {}
    for label, nodes in alt_nodes.items():
        visited = [n['name'] for n in nodes
                   if min_dists.get((label, n['name']), float('inf')) < MATCH_DIST]
        if visited:
            active.add(label)
        visited_per_label[label] = visited

    if len(active) == 0:
        rt = 'DIRECT'
    elif len(active) == 1:
        rt = next(iter(active))
    else:
        rt = 'MIX'

    details = {
        'active_alternatives': sorted(active),
        'visited_per_label': visited_per_label,
        'min_dists': {f'{lb}/{n}': round(v, 3)
                      for (lb, n), v in min_dists.items()},
    }
    return rt, details


def load_semantic_snapshot() -> Dict:
    p = cfg.semantic_objects_path
    if not p.exists():
        return {'objects_count': 0, 'objects_by_class': {}, 'valid_count': 0}
    try:
        with open(p) as f:
            data = json.load(f)
        bc: Dict[str, int] = {}
        vc = 0
        for feat in data.get('features', []):
            c = feat.get('properties', {}).get('class_name', 'unknown')
            bc[c] = bc.get(c, 0) + 1
            if c in cfg.valid_classes:
                vc += 1
        return {'objects_count': len(data.get('features', [])),
                'objects_by_class': bc, 'valid_count': vc,
                'timestamp': data.get('metadata', {}).get('last_updated', '')}
    except Exception as e:
        return {'objects_count': 0, 'error': str(e)}


def load_nav2_penalties() -> Dict[str, float]:
    p = cfg.nav2_graph_path
    if not p.exists():
        return {}
    try:
        with open(p) as f:
            data = json.load(f)
        pen: Dict[str, float] = {}
        for feat in data.get('features', []):
            props = feat.get('properties', {})
            eid = props.get('id')
            meta = props.get('metadata', {})
            if 'penalty' in meta and eid is not None:
                pen[str(eid)] = meta['penalty']
        return pen
    except Exception:
        return {}


def load_semantic_objects_positions() -> List[Tuple[float, float, str]]:
    p = cfg.semantic_objects_path
    if not p.exists():
        return []
    try:
        with open(p) as f:
            data = json.load(f)
        objs = []
        for feat in data.get('features', []):
            coords = feat.get('geometry', {}).get('coordinates', [0, 0])
            cls = feat.get('properties', {}).get('class_name', '')
            if cls in cfg.valid_classes:
                objs.append((coords[0], coords[1], cls))
        return objs
    except Exception:
        return []


def min_distance_to_objects(trajectory, objects):
    if not trajectory or not objects:
        return None
    min_d = float('inf')
    closest_obj = None
    for tx, ty, _ in trajectory:
        for ox, oy, cls in objects:
            d = math.sqrt((tx - ox)**2 + (ty - oy)**2)
            if d < min_d:
                min_d = d
                closest_obj = cls
    if min_d == float('inf'):
        return None
    return {'min_distance_m': round(min_d, 3), 'closest_class': closest_obj}


def penalties_really_changed(pen_fwd: Dict, pen_ret: Dict) -> bool:
    """Verifica daca penalizarile pe edges s-au schimbat intre forward si return."""
    for eid_f, val_f in pen_fwd.items():
        rev_eid = str(cfg.edge_pairs.get(int(eid_f), -1))
        val_r = pen_ret.get(rev_eid)
        if val_r is not None and abs(val_f - val_r) > 0.01:
            return True
    return False


# =============================================================================
# SYSTEM METRICS COLLECTOR
# =============================================================================

class SystemMetricsCollector:
    """Colecteaza metrici sistem la 1Hz in background thread."""

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._active = False
        self._samples = []
        self._battery_pct = -1.0
        self._thread = None

        # YOLO diagnostics (published by yolo26_cpp at ~5s interval)
        self._yolo_diag = {}  # latest diagnostics snapshot
        try:
            from std_msgs.msg import String as StdString
            yolo_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE, depth=1)
            node.create_subscription(StdString, '/yolo26/diagnostics',
                                     self._yolo_diag_cb, yolo_qos)
        except Exception:
            pass

        try:
            from sensor_msgs.msg import BatteryState
            batt_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE, depth=1)
            node.create_subscription(BatteryState, '/platform/bms/state',
                                     self._batt_cb, batt_qos)
        except Exception:
            pass

    def _batt_cb(self, msg):
        with self._lock:
            self._battery_pct = round(msg.percentage * 100, 1)

    def _yolo_diag_cb(self, msg):
        """Parse JSON diagnostics from yolo26_cpp (/yolo26/diagnostics)."""
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._yolo_diag = data
        except (json.JSONDecodeError, AttributeError):
            pass

    def _read_cpu_temp(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            return None

    def _sample(self):
        if not HAS_PSUTIL:
            return None
        s = {
            't':           round(time.time(), 3),
            'cpu_pct':     psutil.cpu_percent(interval=None),
            'mem_pct':     psutil.virtual_memory().percent,
            'swap_pct':    psutil.swap_memory().percent,
            'disk_pct':    psutil.disk_usage('/').percent,
            'cpu_temp_c':  self._read_cpu_temp(),
        }
        with self._lock:
            s['battery_pct'] = self._battery_pct
            if self._yolo_diag:
                s['yolo_fps']          = self._yolo_diag.get('fps')
                s['yolo_inference_ms'] = self._yolo_diag.get('inference_ms')
                s['yolo_detections']   = self._yolo_diag.get('detections')
        per_cpu = psutil.cpu_percent(percpu=True)
        if per_cpu:
            s['cpu_per_core'] = per_cpu
        return s

    def _collection_loop(self):
        while self._active:
            s = self._sample()
            if s:
                with self._lock:
                    self._samples.append(s)
            time.sleep(1.0)

    def start(self):
        with self._lock:
            self._samples = []
            self._active = True
        if HAS_PSUTIL:
            psutil.cpu_percent(interval=None)
        self._thread = threading.Thread(target=self._collection_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._active = False
        if self._thread:
            self._thread.join(timeout=2.0)
        with self._lock:
            samples = list(self._samples)
        if not samples:
            return {'samples': 0}

        def stats(key):
            vals = [s[key] for s in samples if s.get(key) is not None]
            if not vals:
                return None
            import statistics as st
            return {
                'mean': round(st.mean(vals), 1),
                'max':  round(max(vals), 1),
                'min':  round(min(vals), 1),
                'std':  round(st.stdev(vals), 1) if len(vals) > 1 else 0.0,
            }

        result = {
            'samples':    len(samples),
            'duration_s': round(samples[-1]['t'] - samples[0]['t'], 1) if len(samples) > 1 else 0,
            'cpu':        stats('cpu_pct'),
            'memory':     stats('mem_pct'),
            'swap':       stats('swap_pct'),
            'cpu_temp':   stats('cpu_temp_c'),
            'battery_start': samples[0].get('battery_pct', -1),
            'battery_end':   samples[-1].get('battery_pct', -1),
            'time_series':   samples,
        }

        # YOLO inference diagnostics (from /yolo26/diagnostics topic)
        yolo_fps   = stats('yolo_fps')
        yolo_infer = stats('yolo_inference_ms')
        yolo_dets  = stats('yolo_detections')
        if yolo_fps or yolo_infer or yolo_dets:
            result['yolo'] = {}
            if yolo_fps:   result['yolo']['fps']          = yolo_fps
            if yolo_infer: result['yolo']['inference_ms']  = yolo_infer
            if yolo_dets:  result['yolo']['detections']    = yolo_dets

        b_start = samples[0].get('battery_pct', -1)
        b_end   = samples[-1].get('battery_pct', -1)
        if b_start > 0 and b_end > 0:
            result['battery_delta_pct'] = round(b_end - b_start, 2)
        return result


# =============================================================================
# ODOM TRACKER
# =============================================================================

class OdomTracker:
    """Urmareste odometria si traiectoria in frame odom si map."""

    def __init__(self, node):
        self._pts      = []   # traiectorie frame ODOM
        self._pts_map  = []   # traiectorie frame MAP (din /amcl_pose)
        self._dist     = 0.0
        self._lx = self._ly   = None
        self._lt = self._lt_map = 0.0
        self._iv       = 1.0 / cfg.odom_sample_hz
        self._vels     = []
        self._fp       = None
        self._map_pose = None
        self._active   = False
        self._lock     = threading.Lock()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=1)
        node.create_subscription(Odometry, cfg.odom_topic, self._cb, qos)

        amcl_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amcl_cb, amcl_qos)

    def start(self):
        with self._lock:
            self._pts, self._pts_map, self._vels = [], [], []
            self._dist = 0.0
            self._lx = self._ly = None
            self._lt = self._lt_map = 0.0
            self._fp = None
            self._active = True

    def stop(self):
        with self._lock:
            self._active = False

    def get_results(self):
        with self._lock:
            av = sum(self._vels) / len(self._vels) if self._vels else 0
            mv = max(self._vels) if self._vels else 0
            return {
                'distance_traveled_m':  round(self._dist, 3),
                'avg_velocity_ms':      round(av, 3),
                'max_velocity_ms':      round(mv, 3),
                'final_pose':           self._fp,
                'trajectory_points':    len(self._pts),
            }

    def get_trajectory(self):
        """Traiectorie raw din odom (poate avea drift vs map frame)."""
        with self._lock:
            return list(self._pts)

    def get_trajectory_map(self):
        """Traiectorie in frame MAP (din /amcl_pose).
        AMCL publica la ~2Hz — rezolutie mai mica decat odom.
        Fallback la odom cu corectie offset daca nu s-au acumulat suficiente puncte.
        """
        with self._lock:
            if len(self._pts_map) >= 3:
                return list(self._pts_map)
            if self._pts and self._map_pose:
                ax, ay = self._map_pose[0], self._map_pose[1]
                ox, oy = self._pts[0][0], self._pts[0][1]
                dx, dy = ax - ox, ay - oy
                return [[round(p[0]+dx, 4), round(p[1]+dy, 4)] + p[2:] for p in self._pts]
            return list(self._pts)

    def get_current_pose(self):
        with self._lock:
            return self._fp

    def get_map_pose(self):
        with self._lock:
            return self._map_pose

    def _amcl_cb(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        with self._lock:
            self._map_pose = (round(x, 4), round(y, 4), round(yaw, 4))
            if self._active:
                now = time.monotonic()
                if now - self._lt_map >= self._iv:
                    self._pts_map.append([round(x, 4), round(y, 4), round(now, 3)])
                    self._lt_map = now

    def _cb(self, msg):
        with self._lock:
            x   = msg.pose.pose.position.x
            y   = msg.pose.pose.position.y
            yaw = yaw_from_quat(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            if not self._active:
                self._fp = (round(x, 4), round(y, 4), round(yaw, 4))
                return
            now = time.monotonic()
            vx  = msg.twist.twist.linear.x
            if self._lx is not None:
                self._dist += math.sqrt((x - self._lx)**2 + (y - self._ly)**2)
            self._lx, self._ly = x, y
            self._vels.append(abs(vx))
            if now - self._lt >= self._iv:
                self._pts.append([round(x, 4), round(y, 4), round(now, 3)])
                self._lt = now
            self._fp = (round(x, 4), round(y, 4), round(yaw, 4))


# =============================================================================
# EXPERIMENTAL TESTER
# =============================================================================

class ExperimentalTester(Node):

    def __init__(self, sdir: Path):
        super().__init__('experimental_tester')
        self.sdir = sdir
        self.sdir.mkdir(parents=True, exist_ok=True)

        self._cr       = ActionClient(self, ComputeRoute,    '/compute_route')
        self._fp_ac    = ActionClient(self, FollowPath,      '/follow_path')
        self._ntp      = ActionClient(self, NavigateToPose,  '/navigate_to_pose')
        self._spin_ac  = ActionClient(self, Spin,            '/spin')
        self._srg      = self.create_client(SetRouteGraph,   '/route_server/set_route_graph')
        self._reset_map = self.create_client(Trigger,        '/semantic_localizer/reset_map')

        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.odom    = OdomTracker(self)
        self.metrics = SystemMetricsCollector(self)

        if HAS_PSUTIL:
            psutil.cpu_percent(interval=None)

    # ------------------------------------------------------------------ servers

    def wait_for_servers(self, timeout=15.0) -> bool:
        for name, client in [
            ('ComputeRoute', self._cr),
            ('FollowPath',   self._fp_ac),
            ('NavigateToPose', self._ntp),
            ('Spin',         self._spin_ac),
        ]:
            print(f'  Wait {name}...', end=' ', flush=True)
            if not client.wait_for_server(timeout):
                print('TIMEOUT!')
                return False
            print('OK')
        return True

    # ------------------------------------------------------------------ actions

    def _send_action(self, client, goal_msg, timeout=180.0):
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        gh = future.result()
        if not gh or not gh.accepted:
            return None
        rf = gh.get_result_async()
        t0 = time.time()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - t0 > timeout:
                gh.cancel_goal_async()
                return None
        return rf.result()

    def compute_route(self, start: PoseStamped, goal: PoseStamped, use_start=True):
        g = ComputeRoute.Goal()
        now_stamp = self.get_clock().now().to_msg()
        start.header.stamp = now_stamp
        goal.header.stamp  = now_stamp
        g.start, g.goal = start, goal
        g.use_start = use_start
        g.use_poses = True
        r = self._send_action(self._cr, g, 30.0)
        if r is None:
            return None, None
        return r.result.path, r.result

    def exec_follow(self, path, timeout=180.0) -> bool:
        g = FollowPath.Goal()
        g.path = path
        return self._send_action(self._fp_ac, g, timeout) is not None

    def exec_nav(self, goal: PoseStamped, timeout=180.0) -> bool:
        g = NavigateToPose.Goal()
        goal.header.stamp = self.get_clock().now().to_msg()
        g.pose = goal
        return self._send_action(self._ntp, g, timeout) is not None

    def exec_spin(self, angle=math.pi, timeout=30.0) -> Tuple[bool, float]:
        g = Spin.Goal()
        g.target_yaw = angle
        t0 = time.time()
        r = self._send_action(self._spin_ac, g, timeout)
        return r is not None, time.time() - t0

    def spin_to_heading(self, target_yaw: float, timeout=30.0) -> Tuple[bool, float, float]:
        """Roteste la yaw absolut in map frame. Returns (ok, duration_s, delta_rad)."""
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        fp = self.odom.get_map_pose()
        if fp is None:
            print('    [spin_to_heading] no AMCL pose, fallback exec_spin(pi)')
            ok, dt = self.exec_spin(math.pi, timeout)
            return ok, dt, math.pi

        current_yaw = fp[2]
        delta = target_yaw - current_yaw
        while delta >  math.pi: delta -= 2 * math.pi
        while delta < -math.pi: delta += 2 * math.pi

        print(f'    spin_to_heading: current={math.degrees(current_yaw):.1f}deg '
              f'target={math.degrees(target_yaw):.1f}deg '
              f'delta={math.degrees(delta):.1f}deg')

        if abs(delta) < 0.05:
            print('    Heading OK, no spin needed')
            return True, 0.0, delta

        ok, dt = self.exec_spin(delta, timeout)
        return ok, dt, delta

    def exec_follow_with_replan(self, goal_pose: PoseStamped, timeout=180.0):
        """Follow path cu replanning periodic la schimbarea penalizarilor.

        FIX v5.9.1: compute_route foloseste pozitia explicita din AMCL ca start
        (use_start=True) in loc de use_start=False care lasa Route Server sa
        ghiceasca pozitia din TF. Dupa ensure_localization, TF-ul poate fi
        inconsistent cateva cicluri si Route Server poate genera un path din
        mijlocul grafului, pe care FollowPath il termina instant (0s, 0m).
        """
        t_global = time.time()
        reroute_log = []
        current_route = None
        segment = 0

        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Dreneaza callback queue: asigura ca _amcl_cb a primit cel putin
        # un mesaj recent inainte de primul compute_route.
        # Critic pentru return leg unde ensure_localization tocmai a terminat.
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

        while time.time() - t_global < timeout:
            segment += 1

            fp = self.odom.get_map_pose() or self.odom.get_current_pose()
            if fp:
                dist_to_goal = math.sqrt((fp[0] - goal_x)**2 + (fp[1] - goal_y)**2)
                if dist_to_goal < cfg.node_proximity_thresh_m:
                    reroute_log.append({
                        'segment': segment, 'timestamp': iso_now(),
                        'plan_ms': 0, 'route_type': 'AT_GOAL',
                        'route_nodes': [], 'route_edges': [], 'route_cost': 0,
                        'penalties': {}, 'reason': 'at_goal', 'outcome': 'success',
                    })
                    return True, {
                        'segments': reroute_log, 'total_reroutes': segment - 1,
                        'final_route': current_route or [], 'outcome': 'success',
                    }

            # Construieste start_pose explicit din AMCL — nu lasa Route Server
            # sa deduca pozitia din TF (use_start=False), care poate fi stale
            # dupa reinitializare AMCL sau dupa spinuri de convergenta.
            plan_t0 = time.time()
            amcl_fp = self.odom.get_map_pose()
            # La linia 952, după amcl_fp = self.odom.get_map_pose():
            print(f'      [Seg {segment}] AMCL start=({amcl_fp[0]:.2f}, {amcl_fp[1]:.2f}) yaw={math.degrees(amcl_fp[2]):.1f}deg')
            if amcl_fp:
                start_pose = make_pose(amcl_fp[0], amcl_fp[1], amcl_fp[2])
                path, result = self.compute_route(start_pose, goal_pose, use_start=True)
            else:
                # Fallback: lasa Route Server sa determine pozitia
                print(f'      [Seg {segment}] AMCL unavail, use_start=False fallback')
                path, result = self.compute_route(goal_pose, goal_pose, use_start=False)
            plan_dt = time.time() - plan_t0

            if path is None or not path.poses:
                print(f'      [Seg {segment}] Route planning failed, fallback NavigateToPose')
                nav_ok = self.exec_nav(goal_pose, timeout=max(30, timeout - (time.time() - t_global)))
                reroute_log.append({
                    'segment': segment, 'timestamp': iso_now(),
                    'plan_ms': round(plan_dt * 1000, 2), 'route_type': 'FALLBACK_NAV',
                    'route_nodes': [], 'route_edges': [], 'route_cost': 0,
                    'penalties': {}, 'reason': 'fallback' if segment == 1 else 'reroute_fallback',
                    'outcome': 'success' if nav_ok else 'fail',
                })
                return nav_ok, {
                    'segments': reroute_log, 'total_reroutes': segment - 1,
                    'final_route': current_route or [],
                    'outcome': 'success' if nav_ok else 'plan_fail',
                }
# =============================================================================
# PATCH for test_semantic_navigation_v5_10.py
# Replace lines 978-1007 (from "rt, rd = identify_route" to 
# "send_future = self._fp_ac.send_goal_async(fp_goal)")
# with the block below.
# Everything before line 978 and after line 1007 stays UNCHANGED.
# =============================================================================

            # ── Fix path gap: if first path point is far from robot,
            #    prepend interpolated poses from robot to path start ──
            PATH_GAP_THRESHOLD = 1.0   # meters
            PATH_GAP_STEP      = 0.15  # interpolation step (meters)

            robot_fp = self.odom.get_map_pose() or self.odom.get_current_pose()
            if robot_fp and path.poses:
                p0 = path.poses[0].pose.position
                gap = math.sqrt((robot_fp[0] - p0.x)**2 + (robot_fp[1] - p0.y)**2)
                if gap > PATH_GAP_THRESHOLD:
                    n_pts = max(2, int(gap / PATH_GAP_STEP))
                    patch = []
                    for i in range(n_pts):
                        frac = i / n_pts
                        px = robot_fp[0] + frac * (p0.x - robot_fp[0])
                        py = robot_fp[1] + frac * (p0.y - robot_fp[1])
                        patch.append(make_pose(px, py))
                    path.poses = patch + list(path.poses)
                    print(f'      [fix_path_gap] {gap:.2f}m gap, '
                          f'prepended {n_pts} poses')

            rt, rd      = identify_route(path.poses)
            ri          = self._route_info(result)
            pen_snapshot = load_nav2_penalties()
            route_edges  = ri.get('route_edges', [])
            ep           = {str(e): pen_snapshot.get(str(e), 0.0) for e in route_edges}
            route_nodes  = ri.get('route_nodes', [])

            seg_info = {
                'segment':     segment,
                'timestamp':   iso_now(),
                'plan_ms':     round(plan_dt * 1000, 2),
                'route_type':  rt,
                'route_nodes': route_nodes,
                'route_edges': route_edges,
                'route_cost':  ri.get('route_cost'),
                'penalties':   ep,
                'reason':      'initial' if segment == 1 else 'reroute',
            }

            if current_route and route_nodes != current_route:
                seg_info['route_changed_from'] = current_route
                print(f'      REROUTE! {current_route} -> {route_nodes}')
            current_route = route_nodes

            print(f'      [Seg {segment}] {rt} nodes={route_nodes} '
                  f'cost={ri.get("route_cost","?")} ({plan_dt*1000:.0f}ms)')

            fp_goal = FollowPath.Goal()
            fp_goal.path = path
            send_future = self._fp_ac.send_goal_async(fp_goal)
            
            rt, rd      = identify_route(path.poses)
            ri          = self._route_info(result)
            pen_snapshot = load_nav2_penalties()
            route_edges  = ri.get('route_edges', [])
            ep           = {str(e): pen_snapshot.get(str(e), 0.0) for e in route_edges}
            route_nodes  = ri.get('route_nodes', [])

            seg_info = {
                'segment':     segment,
                'timestamp':   iso_now(),
                'plan_ms':     round(plan_dt * 1000, 2),
                'route_type':  rt,
                'route_nodes': route_nodes,
                'route_edges': route_edges,
                'route_cost':  ri.get('route_cost'),
                'penalties':   ep,
                'reason':      'initial' if segment == 1 else 'reroute',
            }

            if current_route and route_nodes != current_route:
                seg_info['route_changed_from'] = current_route
                print(f'      REROUTE! {current_route} -> {route_nodes}')
            current_route = route_nodes

            print(f'      [Seg {segment}] {rt} nodes={route_nodes} '
                  f'cost={ri.get("route_cost","?")} ({plan_dt*1000:.0f}ms)')

            fp_goal = FollowPath.Goal()
            fp_goal.path = path
            send_future = self._fp_ac.send_goal_async(fp_goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
            goal_handle = send_future.result()

            if not goal_handle or not goal_handle.accepted:
                print(f'      [Seg {segment}] FollowPath rejected')
                seg_info['outcome'] = 'rejected'
                reroute_log.append(seg_info)
                return False, {'segments': reroute_log, 'total_reroutes': segment - 1,
                               'outcome': 'rejected'}

            result_future = goal_handle.get_result_async()
            last_check = time.time()
            did_reroute = False

            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.3)

                if time.time() - t_global > timeout:
                    goal_handle.cancel_goal_async()
                    seg_info['outcome'] = 'global_timeout'
                    reroute_log.append(seg_info)
                    return False, {'segments': reroute_log, 'total_reroutes': segment - 1,
                                   'outcome': 'timeout'}

                if time.time() - last_check >= cfg.replan_check_interval_s:
                    last_check = time.time()

                    rp = self.odom.get_map_pose() or self.odom.get_current_pose()
                    if rp:
                        d2g = math.sqrt((rp[0] - goal_x)**2 + (rp[1] - goal_y)**2)
                        if d2g < cfg.node_proximity_thresh_m * 2:
                            continue

                    new_pen = load_nav2_penalties()
                    changed = any(
                        abs(new_pen.get(eid_s, old_val) - old_val) > 2.0
                        for eid_s, old_val in ep.items()
                    )
                    if changed:
                        print('      Penalties changed, cancelling + replanning')
                        goal_handle.cancel_goal_async()
                        for _ in range(10):
                            rclpy.spin_once(self, timeout_sec=0.2)
                            if result_future.done():
                                break
                        seg_info['outcome'] = 'rerouted'
                        reroute_log.append(seg_info)
                        did_reroute = True
                        break

            if did_reroute:
                time.sleep(0.5)
                continue

            fp_result = result_future.result()
            success = fp_result is not None
            seg_info['outcome'] = 'success' if success else 'fail'
            reroute_log.append(seg_info)
            return success, {
                'segments': reroute_log, 'total_reroutes': segment - 1,
                'final_route': current_route, 'outcome': 'success' if success else 'fail',
            }

        return False, {'segments': reroute_log, 'total_reroutes': segment - 1,
                       'outcome': 'timeout'}

    # ------------------------------------------------------------------ services

    def call_reset_map(self) -> bool:
        if not self._reset_map.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('reset_map service unavailable')
            return False
        req = Trigger.Request()
        future = self._reset_map.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        try:
            return future.result().success
        except Exception:
            return False

    def reload_route_graph(self) -> bool:
        if not self._srg.wait_for_service(timeout_sec=3.0):
            return False
        req = SetRouteGraph.Request()
        req.graph_filepath = str(cfg.nav2_graph_path)
        future = self._srg.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        try:
            return future.result().success
        except Exception:
            return False

    # ------------------------------------------------------------------ AMCL

    def reinitialize_amcl(self, node_id: int):
        """Publica /initialpose la coordonatele nodului specificat."""
        n = cfg.node(node_id)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        # stamp=0: mecanismul ROS2 standard pentru "cel mai recent TF disponibil"
        # Robust dupa repornire — evita extrapolation errors din TF buffer
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose.pose.position.x = n['x']
        msg.pose.pose.position.y = n['y']
        fp = self.odom.get_map_pose()
        yaw = fp[2] if fp else 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0]  = 0.05
        msg.pose.covariance[7]  = 0.05
        msg.pose.covariance[35] = 0.02
        self._initial_pose_pub.publish(msg)

        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f'  AMCL re-initialized at node {node_id} ({n["x"]}, {n["y"]})')

    def ensure_localization(self, expected_node_id: int) -> bool:
        """Reinitializeaza AMCL, asteapta convergenta, realiniaza heading."""
        self.reinitialize_amcl(expected_node_id)
        time.sleep(2.0)

        print('    [AMCL convergence spin]')
        self.exec_spin(math.pi / 6, timeout=10.0)
        time.sleep(1.5)

        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
            print('    No AMCL pose after reinit')
            return False

        n = cfg.node(expected_node_id)
        d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
        print(f'    AMCL pose: ({fp[0]:.2f}, {fp[1]:.2f}), {d:.2f}m from '
              f'{n["name"]} ({n["x"]}, {n["y"]})')

        if d > 1.5:
            print('    Still drifted, retry AMCL reinit')
            self.reinitialize_amcl(expected_node_id)
            time.sleep(2.0)
            self.exec_spin(math.pi / 3, timeout=15.0)
            time.sleep(1.5)
            for _ in range(15):
                rclpy.spin_once(self, timeout_sec=0.2)
            fp = self.odom.get_map_pose()
            if fp:
                d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
                print(f'    AMCL retry: {d:.2f}m from {n["name"]}')

        converged = d <= 1.5 if fp else False

        # Realiniaza la heading-ul nodului terminal (start sau goal)
        # Nodurile intermediare nu au target_yaw definit — skip
        if converged:
            try:
                target_yaw = cfg.terminal_yaw(expected_node_id)
                print(f'    [Post-reinit realign to node {expected_node_id} heading]')
                self.spin_to_heading(target_yaw, timeout=15.0)
            except ValueError:
                pass  # nod intermediar, fara yaw definit

        return converged

    def _find_nearest_node(self) -> int:
        """Returneaza ID-ul nodului cel mai apropiat de pozitia curenta."""
        fp = self.odom.get_map_pose()
        if fp is None:
            return cfg.start_node_id
        best_id, best_d = cfg.start_node_id, float('inf')
        for nid, n in cfg.graph_nodes.items():
            d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
            if d < best_d:
                best_id, best_d = nid, d
        return best_id

    def align_to_path(self, path_poses):
        if not path_poses or len(path_poses) < 2:
            return True, 0.0, None
        look = min(5, len(path_poses) - 1)
        p0 = path_poses[0].pose.position
        p1 = path_poses[look].pose.position
        target_yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)
        fp = self.odom.get_map_pose()
        if fp is None:
            return True, 0.0, target_yaw
        current_yaw = fp[2]
        delta = target_yaw - current_yaw
        while delta >  math.pi: delta -= 2 * math.pi
        while delta < -math.pi: delta += 2 * math.pi
        if abs(delta) < 0.3:
            print(f'    Heading OK (delta={math.degrees(delta):.0f}deg)')
            return True, 0.0, target_yaw
        print(f'    Aligning: {math.degrees(current_yaw):.0f}deg -> '
              f'{math.degrees(target_yaw):.0f}deg (delta={math.degrees(delta):.0f}deg)')
        ok, dt = self.exec_spin(delta)
        return ok, dt, target_yaw

    # ------------------------------------------------------------------ logging

    def _save(self, fn: str, data: Dict):
        p = self.sdir / fn
        with open(p, 'w') as f:
            json.dump(data, f, indent=2, default=str)
        print(f'  {p.name}')

    def _pos_error(self, goal_pose: PoseStamped) -> Dict:
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
            r = self.odom.get_results()
            fp = r.get('final_pose')
        if fp is None:
            return {'xy_error_m': None, 'yaw_error_rad': None}
        gx  = goal_pose.pose.position.x
        gy  = goal_pose.pose.position.y
        g_y = yaw_from_quat(goal_pose.pose.orientation.z, goal_pose.pose.orientation.w)
        xy  = math.sqrt((fp[0] - gx)**2 + (fp[1] - gy)**2)
        ye  = abs(fp[2] - g_y)
        if ye > math.pi:
            ye = 2 * math.pi - ye
        return {'xy_error_m': round(xy, 4), 'yaw_error_rad': round(ye, 4)}

    def _route_info(self, result) -> Dict:
        info: Dict = {}
        if result and hasattr(result, 'route') and result.route:
            r = result.route
            if hasattr(r, 'nodes') and r.nodes:
                info['route_nodes']      = [n.nodeid for n in r.nodes]
                info['route_node_names'] = [
                    cfg.graph_nodes.get(n.nodeid, {}).get('name', '?') for n in r.nodes
                ]
            if hasattr(r, 'edges') and r.edges:
                info['route_edges'] = [e.edgeid for e in r.edges]
            if hasattr(r, 'route_cost'):
                info['route_cost'] = round(r.route_cost, 3)
        return info

    def _validate_start(self, start_id: int) -> Tuple[bool, Optional[float], Optional[tuple]]:
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
            print('    AMCL unavailable, running ensure_localization')
            self.ensure_localization(start_id)
            fp = self.odom.get_map_pose() or self.odom.get_current_pose()
            if fp is None:
                return False, None, None

        n = cfg.node(start_id)
        d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
        if d > cfg.max_start_offset_m:
            print(f'    Robot {d:.2f}m from node {start_id}, running ensure_localization')
            self.ensure_localization(start_id)
            fp = self.odom.get_map_pose() or self.odom.get_current_pose()
            if fp:
                d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
        return d <= cfg.max_start_offset_m, round(d, 3), fp

    def _validate_success(self, nav2_ok: bool, odom_r: Dict, err: Dict) -> bool:
        if not nav2_ok:
            return False
        dist   = odom_r.get('distance_traveled_m', 0)
        xy_err = err.get('xy_error_m')
        if dist < cfg.min_success_dist_m:
            return False
        if xy_err is not None and xy_err > cfg.max_success_error_m:
            return False
        return True

    # =================================================================
    # TEST 0: SETUP
    # =================================================================

    def run_setup(self, wait_secs=60) -> bool:
        hdr('TEST 0: SESSION SETUP')

        sub('Pas 1: Backup + reset via service')
        if cfg.semantic_objects_path.exists():
            bk = self.sdir / f'semantic_objects_BACKUP_{ts_str()}.geojson'
            shutil.copy2(cfg.semantic_objects_path, bk)
            print(f'  Backup: {bk.name}')

        ok = self.call_reset_map()
        if ok:
            print('  Reset via service OK')
        else:
            print('  Service unavailable, fallback disk reset')
            empty = {
                'type': 'FeatureCollection',
                'metadata': {'description': 'Reset', 'frame': 'map',
                             'last_updated': iso_now(), 'object_count': 0},
                'features': [],
            }
            cfg.semantic_objects_path.parent.mkdir(parents=True, exist_ok=True)
            with open(cfg.semantic_objects_path, 'w') as f:
                json.dump(empty, f, indent=2)

        sub(f'Pas 2: Astept repopulare ({wait_secs}s)')
        for i in range(wait_secs, 0, -1):
            rclpy.spin_once(self, timeout_sec=1.0)
            if i % 10 == 0:
                s = load_semantic_snapshot()
                print(f'  {i}s, {s["objects_count"]} obj ({s.get("valid_count", 0)} valid)')

        sub('Pas 3: Snapshot')
        snap = load_semantic_snapshot()
        print(f'  Total: {snap["objects_count"]} | Valid: {snap.get("valid_count", 0)}')
        if cfg.semantic_objects_path.exists():
            shutil.copy2(cfg.semantic_objects_path,
                         self.sdir / 'semantic_snapshot_start.geojson')

        sub('Pas 4: Astept regenerare graf')
        for i in range(35):
            rclpy.spin_once(self, timeout_sec=1.0)
            if cfg.semantic_graph_path.exists():
                age = time.time() - os.path.getmtime(cfg.semantic_graph_path)
                if age < 35:
                    print(f'  Graf semantic actualizat (age={age:.0f}s)')
                    break

        sub('Pas 5: Reload Route Server')
        print(f'  {"OK" if self.reload_route_graph() else "FAILED"}')

        sub('Pas 6: AMCL localization')
        if not self.ensure_localization(expected_node_id=cfg.start_node_id):
            print('  AMCL could not be recovered, results may be invalid')

        pen = load_nav2_penalties()
        session_cfg = {
            'session_id':         self.sdir.name,
            'timestamp':          iso_now(),
            'route_config':       cfg._config_path.name,
            'route_name':         cfg.route_name,
            'graph_file':         cfg.graph_filepath.name,
            'start_node_id':      cfg.start_node_id,
            'goal_node_id':       cfg.goal_node_id,
            'start_yaw_deg':      math.degrees(cfg.start_yaw),
            'goal_yaw_deg':       math.degrees(cfg.goal_yaw),
            'route_alternatives': cfg.route_alternatives,
            'graph': {
                'nodes': len(cfg.graph_nodes),
                'coords': {str(k): v for k, v in cfg.graph_nodes.items()},
            },
            'semantic':    snap,
            'penalties_at_start': pen,
            'validation': {
                'max_start_offset_m':     cfg.max_start_offset_m,
                'min_success_dist_m':     cfg.min_success_dist_m,
                'max_success_error_m':    cfg.max_success_error_m,
                'min_success_duration_s': cfg.min_success_duration_s,
                'max_consecutive_fails':  cfg.max_consecutive_fails,
            },
            'params': {
                'odom_topic':    cfg.odom_topic,
                'sample_hz':     cfg.odom_sample_hz,
                'valid_classes': sorted(cfg.valid_classes),
                'has_psutil':    HAS_PSUTIL,
            },
        }
        self._save('session_config.json', session_cfg)
        print('\n  Setup complet!')
        return True

    # =================================================================
    # TEST 1: COMPUTE ROUTE
    # =================================================================

    def run_test1(self) -> bool:
        hdr('TEST 1: ComputeRoute')
        t0 = time.time()
        path, result = self.compute_route(
            node_pose(cfg.start_node_id),
            node_pose(cfg.goal_node_id),
        )
        dt = time.time() - t0

        if path is None:
            print('  [ERR] ComputeRoute returned None')
            return False

        rt, rd = identify_route(path.poses)
        ri  = self._route_info(result)
        pen = load_nav2_penalties()
        ep  = {str(e): pen.get(str(e), 0.0) for e in ri.get('route_edges', [])}

        data = {
            'test_type': 'compute_route', 'timestamp': iso_now(),
            'planning': {
                'duration_ms': round(dt * 1000, 2),
                'points':      len(path.poses),
                'route_type':  rt,
                **ri,
                'penalties_per_edge': ep,
            },
            'details':  rd,
            'semantic': load_semantic_snapshot(),
        }
        self._save('test1_compute_route.json', data)
        print(f'  {dt*1000:.1f}ms | {len(path.poses)} pts | {rt}')
        print(f'  Nodes: {ri.get("route_nodes", "?")} | Cost: {ri.get("route_cost", "?")}')
        return True

    # =================================================================
    # TEST 2: BASELINE
    # =================================================================

    def _run_baseline_leg(self, start_id: int, goal_id: int,
                          run_num: int, direction: str) -> Tuple[Dict, bool]:
        """Baseline: NavigateToPose fara routing semantic."""
        goal_p = node_pose(goal_id)

        start_ok, start_dist, robot_pose = self._validate_start(start_id)
        if not start_ok:
            print(f'    Robot {start_dist}m from node {start_id} after recovery, skipping')
            data = {
                'test_type':   f'baseline_{direction}',
                'run_number':  run_num,
                'timestamp':   iso_now(),
                'robot_start_pose': robot_pose,
                'start_offset_m':   start_dist,
                'execution': {
                    'duration_s': 0, 'nav2_success': False,
                    'validated_success': False,
                    'distance_traveled_m': 0, 'avg_velocity_ms': 0,
                    'max_velocity_ms': 0, 'final_pose': robot_pose,
                    'trajectory_points': 0,
                    'xy_error_m': start_dist, 'yaw_error_rad': 0,
                },
                'trajectory': [], 'trajectory_odom': [],
                'min_object_distance': None,
                'system_metrics': {'samples': 0},
                'semantic': load_semantic_snapshot(),
                'skipped': True,
                'skip_reason': f'start_offset {start_dist}m after recovery',
            }
            return data, False

        n_goal = cfg.node(goal_id)
        print(f'    NavigateToPose -> node {goal_id} ({n_goal["name"]})')
        print('    ROBOT MOVING')
        time.sleep(0.5)

        self.metrics.start()
        self.odom.start()
        t0 = time.time()
        nav2_ok = self.exec_nav(goal_p)
        dt = time.time() - t0
        self.odom.stop()
        sys_metrics = self.metrics.stop()

        odom_r   = self.odom.get_results()
        traj     = self.odom.get_trajectory()
        traj_map = self.odom.get_trajectory_map()
        err      = self._pos_error(goal_p)
        real_ok  = self._validate_success(nav2_ok, odom_r, err)

        sem_objs     = load_semantic_objects_positions()
        traj_for_dist = traj_map if len(traj_map) >= 2 else traj
        min_obj_dist  = min_distance_to_objects(
            [(p[0], p[1], '') for p in traj_for_dist], sem_objs)

        cpu_s = sys_metrics.get('cpu', {})
        print(f'    Exec: {dt:.1f}s | {odom_r["distance_traveled_m"]}m | '
              f'err={err["xy_error_m"]}m | nav2={"OK" if nav2_ok else "FAIL"} '
              f'valid={"OK" if real_ok else "FAIL"}')
        if cpu_s:
            mem_s  = sys_metrics.get('memory', {})
            temp_s = sys_metrics.get('cpu_temp', {})
            print(f'    Sys: CPU={cpu_s.get("mean",0):.0f}% '
                  f'Mem={mem_s.get("mean",0):.0f}% '
                  f'Temp={temp_s.get("max","?") if temp_s else "?"}C')

        data = {
            'test_type':  f'baseline_{direction}',
            'run_number': run_num,
            'timestamp':  iso_now(),
            'robot_start_pose': robot_pose,
            'start_offset_m':   start_dist,
            'timing': {
                'prescan_duration_s':   0.0,
                'navigation_duration_s': round(dt, 3),
                'total_duration_s':      round(dt, 3),
            },
            'execution': {
                'duration_s':        round(dt, 3),
                'nav2_success':      nav2_ok,
                'validated_success': real_ok,
                **odom_r, **err,
            },
            'trajectory':      traj_map,   # MAP frame
            'trajectory_odom': traj,       # ODOM frame
            'min_object_distance': min_obj_dist,
            'system_metrics':  sys_metrics,
            'semantic':        load_semantic_snapshot(),
        }
        return data, real_ok

    def run_test2(self, n_runs=5) -> bool:
        hdr(f'TEST 2: Baseline, {n_runs} runs')
        S = cfg.start_node_id
        G = cfg.goal_node_id
        results = []
        consec_fail = 0

        for run in range(1, n_runs + 1):
            sub(f'RUN {run}/{n_runs}')

            print(f'  [Pre-forward AMCL reinit at node {S}]')
            self.ensure_localization(S)

            print(f'  [Forward] start({S}) -> goal({G})')
            fwd_data, fwd_ok = self._run_baseline_leg(S, G, run, 'forward')
            if fwd_data:
                self._save(f'test2_run{run:02d}_forward.json', fwd_data)

            print(f'    [Spin -> node {G} heading]')
            s_ok, s_dt, s_delta = self.spin_to_heading(cfg.goal_yaw)
            print(f'    {s_dt:.1f}s | delta={math.degrees(s_delta):.1f}deg | '
                  f'{"OK" if s_ok else "FAIL"}')

            ret_start = G if fwd_ok else self._find_nearest_node()
            if not fwd_ok:
                print(f'    Forward failed, nearest node for return: {ret_start}')

            print(f'  [Pre-return AMCL reinit at node {ret_start}]')
            self.ensure_localization(ret_start)

            print(f'  [Return] -> start({S})')
            ret_data, ret_ok = self._run_baseline_leg(ret_start, S, run, 'return')
            if ret_data:
                ret_data['spin_before'] = {
                    'duration_s': round(s_dt, 2),
                    'success':    s_ok,
                    'delta_deg':  round(math.degrees(s_delta), 1),
                }
                self._save(f'test2_run{run:02d}_return.json', ret_data)

            s2_ok, s2_dt, s2_delta = self.spin_to_heading(cfg.start_yaw)
            print(f'    [Spin -> node {S} heading] {s2_dt:.1f}s | '
                  f'delta={math.degrees(s2_delta):.1f}deg | {"OK" if s2_ok else "FAIL"}')

            ok = (fwd_ok or False) and (ret_ok or False)
            results.append(ok)
            print(f'  Run {run}: {"OK" if ok else "FAIL"}')

            consec_fail = consec_fail + 1 if not ok else 0
            if consec_fail >= cfg.max_consecutive_fails:
                print(f'\n  {cfg.max_consecutive_fails} consecutive failures, aborting')
                break

        p = sum(results)
        print(f'\n  Test 2: {p}/{len(results)} ok')
        return p == len(results)

    # =================================================================
    # TEST 3: ADAPTIVE SEMANTIC
    # =================================================================

    def _prescan(self):
        """360deg rotation pentru detectie obiecte + asteptare pipeline semantic."""
        print('    [Pre-scan] 360deg rotation for object detection...')

        nav2_ts_before = 0
        if cfg.nav2_graph_path.exists():
            nav2_ts_before = os.path.getmtime(cfg.nav2_graph_path)

        ok1, dt1 = self.exec_spin(math.pi, timeout=90.0)
        ok2, dt2 = self.exec_spin(math.pi, timeout=90.0)
        print(f'    Scan: {dt1 + dt2:.1f}s total | {"OK" if ok1 and ok2 else "WARN"}')

        snap_before = load_semantic_snapshot()
        print(f'    Objects before wait: {snap_before.get("objects_count", 0)} '
              f'(valid: {snap_before.get("valid_count", 0)})')

        print(f'    Waiting {cfg.prescan_wait_secs}s for semantic pipeline...')
        graph_updated = False
        for i in range(cfg.prescan_wait_secs, 0, -1):
            rclpy.spin_once(self, timeout_sec=1.0)
            if i % 10 == 0:
                s = load_semantic_snapshot()
                nav2_ts_now = (os.path.getmtime(cfg.nav2_graph_path)
                               if cfg.nav2_graph_path.exists() else 0)
                if nav2_ts_now > nav2_ts_before:
                    graph_updated = True
                tag = 'graph ok' if graph_updated else 'waiting'
                print(f'      {i}s, {s.get("objects_count",0)} obj '
                      f'(valid: {s.get("valid_count",0)}) | {tag}')

        if not graph_updated:
            print('    Nav2 graph NOT updated, pipeline may be broken!')
        else:
            print('    Nav2 graph confirmed updated')

        snap_after = load_semantic_snapshot()
        print(f'    Objects after wait: {snap_after.get("objects_count", 0)} '
              f'(valid: {snap_after.get("valid_count", 0)})')

        self.reload_route_graph()
        time.sleep(1.0)

        # Realiniaza la heading start dupa 360 (spinul poate acumula eroare)
        print('    [Post-scan realign]')
        align_ok, align_dt, align_delta = self.spin_to_heading(cfg.start_yaw)
        print(f'    Realign: {align_dt:.1f}s | delta={math.degrees(align_delta):.1f}deg | '
              f'{"OK" if align_ok else "WARN"}')

        return snap_after

    def _run_adaptive_leg(self, start_id: int, goal_id: int,
                          run_num: int, direction: str) -> Tuple[Optional[Dict], bool]:
        """Adaptive semantic leg: pre-scan + follow cu replanning."""
        goal_p = node_pose(goal_id)

        start_ok, start_dist, robot_pose = self._validate_start(start_id)
        if not start_ok:
            print(f'    Robot {start_dist}m from node {start_id} after recovery, skipping')
            return None, False

        prescan_snap = None
        prescan_duration = 0.0
        if direction == 'forward':
            t_prescan = time.time()
            prescan_snap = self._prescan()
            prescan_duration = time.time() - t_prescan

        pen_before = load_nav2_penalties()

        print('    ROBOT MOVING (adaptive)')
        time.sleep(0.5)

        self.metrics.start()
        self.odom.start()
        t0 = time.time()
        nav2_ok, replan_log = self.exec_follow_with_replan(goal_p)
        nav_dt = time.time() - t0
        self.odom.stop()
        sys_metrics = self.metrics.stop()

        odom_r   = self.odom.get_results()
        traj     = self.odom.get_trajectory()
        traj_map = self.odom.get_trajectory_map()
        err      = self._pos_error(goal_p)
        real_ok  = self._validate_success(nav2_ok, odom_r, err)

        sem_objs     = load_semantic_objects_positions()
        traj_for_dist = traj_map if len(traj_map) >= 2 else traj
        min_obj_dist  = min_distance_to_objects(
            [(p[0], p[1], '') for p in traj_for_dist], sem_objs)

        pen_after = load_nav2_penalties()

        segments          = replan_log.get('segments', [])
        initial_route     = segments[0].get('route_nodes', []) if segments else []
        final_route       = segments[-1].get('route_nodes', []) if segments else []
        initial_route_type = segments[0].get('route_type', '?') if segments else '?'
        planning_total_ms = sum(s.get('plan_ms', 0) for s in segments)
        n_reroutes        = replan_log.get('total_reroutes', 0)
        route_changed     = initial_route != final_route

        print(f'    Exec: {nav_dt:.1f}s | {odom_r["distance_traveled_m"]}m | '
              f'err={err.get("xy_error_m","?")}m | reroutes={n_reroutes} | '
              f'nav2={"OK" if nav2_ok else "FAIL"} valid={"OK" if real_ok else "FAIL"}')

        data = {
            'test_type':        f'adaptive_{direction}',
            'run_number':       run_num,
            'timestamp':        iso_now(),
            'robot_start_pose': robot_pose,
            'start_offset_m':   start_dist,
            'replanning':       replan_log,
            'route_analysis': {
                'initial_route':       initial_route,
                'final_route':         final_route,
                'initial_route_type':  initial_route_type,
                'route_changed':       route_changed,
                'planning_total_ms':   planning_total_ms,
                'n_segments':          len(segments),
            },
            'timing': {
                'prescan_duration_s':    round(prescan_duration, 2),
                'navigation_duration_s': round(nav_dt, 3),
                'total_duration_s':      round(prescan_duration + nav_dt, 3),
            },
            'execution': {
                'duration_s':        round(nav_dt, 3),
                'nav2_success':      nav2_ok,
                'validated_success': real_ok,
                **odom_r, **err,
            },
            'trajectory':          traj_map,   # MAP frame
            'trajectory_odom':     traj,       # ODOM frame
            'min_object_distance': min_obj_dist,
            'penalties_before':    pen_before,
            'penalties_after':     pen_after,
            'system_metrics':      sys_metrics,
            'semantic':            load_semantic_snapshot(),
        }
        if prescan_snap is not None:
            data['prescan_snapshot'] = prescan_snap
        return data, real_ok

    def run_test3(self, n_runs=5) -> bool:
        hdr(f'TEST 3: Adaptive Semantic, {n_runs} runs')
        S = cfg.start_node_id
        G = cfg.goal_node_id
        results = []
        consec_fail = 0

        for run in range(1, n_runs + 1):
            sub(f'RUN {run}/{n_runs}')

            print(f'  [Forward adaptive] start({S}) -> goal({G})')
            fwd_data, fwd_ok = self._run_adaptive_leg(S, G, run, 'forward')
            if fwd_data:
                self._save(f'test3_run{run:02d}_forward.json', fwd_data)

            print(f'    [Spin -> node {G} heading]')
            s_ok, s_dt, s_delta = self.spin_to_heading(cfg.goal_yaw)
            print(f'    {s_dt:.1f}s | delta={math.degrees(s_delta):.1f}deg | '
                  f'{"OK" if s_ok else "FAIL"}')

            ret_start_id = G if fwd_ok else self._find_nearest_node()
            if not fwd_ok:
                print(f'    Forward failed, nearest node for return: {ret_start_id}')

            print(f'  [Pre-return AMCL reinit at node {ret_start_id}]')
            self.ensure_localization(ret_start_id)

            print(f'  [Return adaptive] goal({G}) -> start({S})')
            ret_data, ret_ok = self._run_adaptive_leg(ret_start_id, S, run, 'return')
            if ret_data:
                ret_data['spin_before'] = {
                    'duration_s': round(s_dt, 2),
                    'success':    s_ok,
                    'delta_deg':  round(math.degrees(s_delta), 1),
                }
                self._save(f'test3_run{run:02d}_return.json', ret_data)

            s2_ok, s2_dt, s2_delta = self.spin_to_heading(cfg.start_yaw)
            print(f'    [Spin -> node {S} heading] {s2_dt:.1f}s | '
                  f'delta={math.degrees(s2_delta):.1f}deg | {"OK" if s2_ok else "FAIL"}')

            ok = (fwd_ok or False) and (ret_ok or False)
            results.append(ok)

            fwd_rr = fwd_data.get('replanning', {}).get('total_reroutes', 0) if fwd_data else 0
            ret_rr = ret_data.get('replanning', {}).get('total_reroutes', 0) if ret_data else 0
            print(f'  Run {run}: {"OK" if ok else "FAIL"} | reroutes: fwd={fwd_rr} ret={ret_rr}')

            consec_fail = consec_fail + 1 if not ok else 0
            if consec_fail >= cfg.max_consecutive_fails:
                print(f'\n  {cfg.max_consecutive_fails} consecutive failures, aborting')
                break

        p = sum(results)
        total_reroutes = 0
        for f in self.sdir.glob('test3_*.json'):
            with open(f) as fh:
                d = json.load(fh)
                total_reroutes += d.get('replanning', {}).get('total_reroutes', 0)
        print(f'\n  Test 3: {p}/{len(results)} ok | Total reroutes: {total_reroutes}')
        return p == len(results)

    # =================================================================
    # SUMMARY
    # =================================================================

    def gen_summary(self):
        import statistics as st

        def stats(v):
            if not v: return {}
            return {
                'mean': round(st.mean(v), 3),
                'std':  round(st.stdev(v), 3) if len(v) > 1 else 0,
                'min':  round(min(v), 3),
                'max':  round(max(v), 3),
                'n':    len(v),
            }

        summary = {
            'session':    self.sdir.name,
            'timestamp':  iso_now(),
            'route_name': cfg.route_name,
            'tests':      {},
        }

        for tname, prefix in [('baseline', 'test2_'), ('adaptive', 'test3_')]:
            fwd, ret = [], []
            for f in sorted(self.sdir.glob(f'{prefix}*_forward.json')):
                with open(f) as fh: fwd.append(json.load(fh))
            for f in sorted(self.sdir.glob(f'{prefix}*_return.json')):
                with open(f) as fh: ret.append(json.load(fh))
            if not fwd:
                continue

            ok_fwd = [r for r in fwd if r['execution'].get('validated_success')]
            ok_ret = [r for r in ret if r['execution'].get('validated_success')]

            ft  = [r['execution']['duration_s'] for r in ok_fwd]
            fd  = [r['execution']['distance_traveled_m'] for r in ok_fwd]
            fe  = [r['execution']['xy_error_m'] for r in ok_fwd
                   if r['execution'].get('xy_error_m') is not None]
            rt  = [r['execution']['duration_s'] for r in ok_ret]
            rd_l = [r['execution']['distance_traveled_m'] for r in ok_ret]

            ts_data = {
                'total_runs':           len(fwd),
                'validated_success_fwd': f'{len(ok_fwd)}/{len(fwd)}',
                'validated_success_ret': f'{len(ok_ret)}/{len(ret)}' if ret else '0/0',
                'forward': {'time_s': stats(ft), 'dist_m': stats(fd), 'error_m': stats(fe)},
                'return':  {'time_s': stats(rt), 'dist_m': stats(rd_l)},
            }

            nav_fwd     = [r.get('timing', {}).get('navigation_duration_s',
                            r['execution']['duration_s']) for r in ok_fwd]
            nav_ret     = [r.get('timing', {}).get('navigation_duration_s',
                            r['execution']['duration_s']) for r in ok_ret]
            prescan_fwd = [r.get('timing', {}).get('prescan_duration_s', 0) for r in ok_fwd]
            ts_data['navigation_only'] = {
                'fwd_time_s':   stats(nav_fwd),
                'ret_time_s':   stats(nav_ret),
                'prescan_time_s': stats(prescan_fwd) if any(p > 0 for p in prescan_fwd) else {},
            }

            if prefix == 'test3_':
                reroutes_fwd = [r.get('replanning', {}).get('total_reroutes', 0) for r in fwd]
                reroutes_ret = [r.get('replanning', {}).get('total_reroutes', 0) for r in ret]
                ts_data['reroutes_fwd'] = stats(reroutes_fwd)
                ts_data['reroutes_ret'] = stats(reroutes_ret)
                ts_data['total_reroutes']     = sum(reroutes_fwd) + sum(reroutes_ret)
                ts_data['runs_with_reroute']  = sum(1 for r in reroutes_fwd + reroutes_ret if r > 0)

                plan_times = [
                    seg.get('plan_ms', 0)
                    for r in fwd + ret
                    for seg in r.get('replanning', {}).get('segments', [])
                ]
                if plan_times:
                    ts_data['planning_ms'] = stats(plan_times)

                route_types = []
                routes_changed = 0
                for r in fwd + ret:
                    ra = r.get('route_analysis', {})
                    rt_type = ra.get('initial_route_type', '?')
                    if rt_type != '?':
                        route_types.append(rt_type)
                    if ra.get('route_changed', False):
                        routes_changed += 1
                ts_data['route_analysis'] = {
                    'route_types':    route_types,
                    'routes_changed': routes_changed,
                    'total_legs':     len(fwd) + len(ret),
                }

            mod_all = [
                r['min_object_distance']['min_distance_m']
                for r in ok_fwd + ok_ret
                if r.get('min_object_distance') is not None
                and isinstance(r['min_object_distance'], dict)
                and r['min_object_distance'].get('min_distance_m') is not None
            ]
            if mod_all:
                ts_data['min_object_distance_m'] = stats(mod_all)

            all_runs = ok_fwd + ok_ret
            cpu_means  = [r['system_metrics']['cpu']['mean']
                          for r in all_runs if r.get('system_metrics', {}).get('cpu')]
            mem_means  = [r['system_metrics']['memory']['mean']
                          for r in all_runs if r.get('system_metrics', {}).get('memory')]
            temp_maxs  = [r['system_metrics']['cpu_temp']['max']
                          for r in all_runs if r.get('system_metrics', {}).get('cpu_temp')]
            batt_deltas = [r['system_metrics']['battery_delta_pct']
                           for r in all_runs
                           if r.get('system_metrics', {}).get('battery_delta_pct') is not None]

            if cpu_means or mem_means or temp_maxs:
                sm = {}
                if cpu_means:   sm['cpu_pct']       = stats(cpu_means)
                if mem_means:   sm['mem_pct']        = stats(mem_means)
                if temp_maxs:   sm['cpu_temp_max_c'] = stats(temp_maxs)
                if batt_deltas: sm['battery_delta_pct'] = stats(batt_deltas)
                ts_data['system_metrics_summary'] = sm

            # YOLO inference diagnostics aggregated across runs
            yolo_fps_means = [r['system_metrics']['yolo']['fps']['mean']
                              for r in all_runs
                              if r.get('system_metrics', {}).get('yolo', {}).get('fps')]
            yolo_infer_means = [r['system_metrics']['yolo']['inference_ms']['mean']
                                for r in all_runs
                                if r.get('system_metrics', {}).get('yolo', {}).get('inference_ms')]
            if yolo_fps_means or yolo_infer_means:
                yd = {}
                if yolo_fps_means:   yd['fps']          = stats(yolo_fps_means)
                if yolo_infer_means: yd['inference_ms']  = stats(yolo_infer_means)
                ts_data['yolo_summary'] = yd

            summary['tests'][tname] = ts_data

        if 'baseline' in summary['tests'] and 'adaptive' in summary['tests']:
            b = summary['tests']['baseline']
            a = summary['tests']['adaptive']
            b_nav = b.get('navigation_only', {}).get('fwd_time_s', {})
            a_nav = a.get('navigation_only', {}).get('fwd_time_s', {})
            b_mod = b.get('min_object_distance_m', {})
            a_mod = a.get('min_object_distance_m', {})
            summary['comparison'] = {
                'nav_time_fwd_s': {
                    'baseline': b_nav.get('mean', 0),
                    'adaptive': a_nav.get('mean', 0),
                },
                'min_obj_dist_m': {
                    'baseline': b_mod.get('mean', 0),
                    'adaptive': a_mod.get('mean', 0),
                },
                'success_rate': {
                    'baseline': b.get('validated_success_fwd', '?'),
                    'adaptive': a.get('validated_success_fwd', '?'),
                },
            }

        self._save('summary.json', summary)
        for tn, td in summary.get('tests', {}).items():
            print(f'  {tn}: fwd={td["validated_success_fwd"]} ret={td["validated_success_ret"]}')
            nav = td.get('navigation_only', {}).get('fwd_time_s', {})
            d   = td.get('forward', {}).get('dist_m', {})
            if nav:
                print(f'    Fwd nav: {nav.get("mean",0):.1f}+-{nav.get("std",0):.1f}s | '
                      f'{d.get("mean",0):.2f}+-{d.get("std",0):.2f}m')
            mod = td.get('min_object_distance_m', {})
            if mod:
                print(f'    Min obj dist: {mod.get("mean",0):.2f}+-{mod.get("std",0):.2f}m')

        if 'comparison' in summary:
            c = summary['comparison']
            print('  --- Comparison ---')
            print(f'    Nav time fwd: baseline={c["nav_time_fwd_s"]["baseline"]:.1f}s '
                  f'adaptive={c["nav_time_fwd_s"]["adaptive"]:.1f}s')
            print(f'    Min obj dist: baseline={c["min_obj_dist_m"]["baseline"]:.2f}m '
                  f'adaptive={c["min_obj_dist_m"]["adaptive"]:.2f}m')


# =============================================================================
# MAIN
# =============================================================================

def main():
    global cfg

    parser = argparse.ArgumentParser(
        description='SAIM Xplorer Experimental Suite v5.9 — parametrizat din route_config.yaml'
    )
    parser.add_argument(
        '--config', type=str, default='route_config.yaml',
        help='Calea catre fisierul de configuratie YAML (default: route_config.yaml)'
    )
    parser.add_argument(
        '--test', type=str, default='1',
        choices=['0', '1', '2', '3', 'all'],
        help='Testul de rulat: 0=setup, 1=compute_route, 2=baseline, 3=adaptive, all=toate'
    )
    parser.add_argument('--runs', type=int, default=5,
                        help='Numar runs pentru ambele teste')
    parser.add_argument('--runs-baseline', type=int, default=None,
                        help='Numar runs Test 2 (suprascrie --runs)')
    parser.add_argument('--runs-adaptive', type=int, default=None,
                        help='Numar runs Test 3 (suprascrie --runs)')
    parser.add_argument('--session', type=str, default=None,
                        help='Nume sesiune (default: session_YYYY-MM-DD_HH-MM-SS)')
    parser.add_argument('--wait-secs', type=int, default=60,
                        help='Secunde asteptare repopulare semantica in setup (default: 60)')
    args = parser.parse_args()

    # ── Incarca configuratia ───────────────────────────────────────────
    try:
        cfg = RouteConfig(args.config)
    except (FileNotFoundError, ValueError, ImportError) as e:
        print(f'\n[FATAL] Nu pot incarca configuratia: {e}')
        sys.exit(1)

    runs_baseline = args.runs_baseline if args.runs_baseline is not None else args.runs
    runs_adaptive = args.runs_adaptive if args.runs_adaptive is not None else args.runs

    sdir = cfg.experiments_dir / (args.session or f'session_{ts_str()}')

    rclpy.init()
    tester = ExperimentalTester(sdir)

    hdr('SAIM Xplorer, Experimental Suite v5.9')
    print(f'  Config:          {args.config}')
    print(f'  {cfg.summary()}')
    print(f'  Session:         {sdir}')
    print(f'  Runs baseline:   {runs_baseline} | Runs adaptive: {runs_adaptive} | Test: {args.test}')
    print(f'  CPU monitoring:  {"psutil OK" if HAS_PSUTIL else "not installed"}')
    print(f'  Start yaw:       {math.degrees(cfg.start_yaw):.2f}deg (node {cfg.start_node_id})')
    print(f'  Goal yaw:        {math.degrees(cfg.goal_yaw):.2f}deg (node {cfg.goal_node_id})')
    print(f'  Route alts:      {list(cfg.route_alternatives.keys()) or ["DIRECT"]}')
    print(f'  Nav2 graph:      {cfg.nav2_graph_path.name}')

    if not tester.wait_for_servers():
        tester.destroy_node()
        rclpy.shutdown()
        return

    res = {}
    try:
        if args.test in ('0', 'all'):
            res['setup'] = tester.run_setup(args.wait_secs)
        if args.test in ('1', 'all'):
            res['test1'] = tester.run_test1()
        if args.test in ('2', 'all'):
            res['test2'] = tester.run_test2(runs_baseline)
        if args.test in ('3', 'all'):
            res['test3'] = tester.run_test3(runs_adaptive)
        if args.test in ('2', '3', 'all'):
            tester.gen_summary()
    except KeyboardInterrupt:
        print('\n  Interrupted, saving summary...')
        tester.gen_summary()

    hdr('FINAL REPORT')
    for n, p in res.items():
        print(f'  {n}: {"OK" if p else "FAIL"}')
    print(f'  Data: {sdir}')

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
