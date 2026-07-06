#!/usr/bin/env python3
"""
Semantic Map Manager
====================
Manages two GeoJSON files for Nav2 Route Server integration:

1. semantic_objects.geojson — Persistent map of detected objects (Points)
   - Tracks all objects with position, class, confidence, timestamps
   - Handles static vs dynamic classification and TTL expiration
   - Fused position updates on re-observation

2. route_graph enrichment — Reads existing Nav2 route graph, injects
   semantic metadata (penalty, speed_limit, class) on edges near
   detected objects.

Nav2 Route Server GeoJSON format compliance:
  - Nodes: Feature with Point geometry, properties: {id, frame, metadata{}}
  - Edges: Feature with LineString geometry, properties: {id, startid, endid, overridable, metadata{}}
  - metadata keys used: penalty, speed_limit, class, semantic_objects (custom)

Author: FIIR / SAIM Xplorer Project
"""

import json
import math
import os
import copy
from datetime import datetime, timezone
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Tuple, Any


# ─── Object mobility classification ─────────────────────────────────

DYNAMIC_CLASSES = {
    'person', 'cat', 'dog', 'bird', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe',
}

STATIC_CLASSES = {
    'chair', 'couch', 'bench', 'dining table', 'potted plant',
    'bed', 'toilet', 'tv', 'laptop', 'refrigerator', 'oven',
    'microwave', 'sink', 'toaster', 'fire hydrant', 'stop sign',
    'parking meter', 'suitcase', 'backpack',
}

# Objects too small to affect navigation — still tracked but low penalty
MINOR_CLASSES = {
    'bottle', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'remote',
    'cell phone', 'book', 'scissors', 'teddy bear',
    'mouse', 'keyboard',
}


# ─── Indoor environment whitelist ────────────────────────────────
# Only these classes are persisted to the semantic map.
# False positives from YOLO (car, truck, airplane, etc.) are filtered out.
# Configurable at init via allowed_classes parameter.
DEFAULT_ALLOWED_CLASSES = (
    DYNAMIC_CLASSES | STATIC_CLASSES | MINOR_CLASSES
)


def classify_mobility(class_name: str) -> str:
    """Classify an object class as static, dynamic, or minor."""
    if class_name in DYNAMIC_CLASSES:
        return 'dynamic'
    elif class_name in STATIC_CLASSES:
        return 'static'
    elif class_name in MINOR_CLASSES:
        return 'minor'
    return 'static'  # default: assume static


# ─── Penalty configuration per mobility type ─────────────────────────

DEFAULT_PENALTIES = {
    'dynamic': 50.0,     # person → very high penalty, strongly prefer alternate
    'static': 2.0,      # chair → significant penalty
    'minor': 0.5,        # bottle → meaningful penalty (was 0.5 — too low for routing)
}

DEFAULT_SPEED_LIMITS = {
    'dynamic': 30,       # 30% speed near people
    'static': 60,        # 60% speed near furniture
    'minor': 90,         # 90% speed near small objects
}

DEFAULT_TTL = {
    'dynamic': 60.0,     # forget people after 60s without re-observation
    'static': -1.0,      # static objects never expire
    'minor': 120.0,      # minor objects expire after 2 min
}


# ─── Persistent object data structure ────────────────────────────────

@dataclass
class PersistentObject:
    """An object in the persistent semantic map."""
    obj_id: str                  # unique ID: "{class_name}_{index}"
    class_name: str
    class_id: int
    mobility: str                # 'static', 'dynamic', 'minor'
    x: float                     # position in /map frame
    y: float
    confidence: float            # best/latest confidence
    observation_count: int = 1
    best_score: float = 0.0      # confidence / distance * rays
    first_seen: str = ''         # ISO timestamp
    last_seen: str = ''          # ISO timestamp
    ttl_seconds: float = -1.0    # -1 = never expires
    fused_alpha: float = 0.7     # position fusion weight for existing


# ─── Semantic Map Manager ────────────────────────────────────────────

class SemanticMapManager:
    """
    Manages persistent semantic object map and route graph annotation.

    Designed for minimal CPU on RPi5 — all operations are dict/list-based,
    no heavy computation. Disk writes are periodic (not per-detection).
    """

    def __init__(
        self,
        objects_filepath: str = '',
        route_graph_filepath: str = '',
        output_graph_filepath: str = '',
        match_distance: float = 0.5,
        edge_proximity: float = 1.5,
        allowed_classes: set = None,
        logger=None,
    ):
        self.objects_filepath = objects_filepath
        self.route_graph_filepath = route_graph_filepath
        self.output_graph_filepath = output_graph_filepath
        self.match_distance = match_distance
        self.edge_proximity = edge_proximity  # max distance object↔edge to annotate
        self.allowed_classes = allowed_classes if allowed_classes is not None else DEFAULT_ALLOWED_CLASSES
        self.logger = logger

        # In-memory object store
        self.objects: Dict[str, PersistentObject] = {}
        self._next_id_counter: int = 0
        self._dirty: bool = False  # True if objects changed since last save

        # Route graph (loaded once, annotated dynamically)
        self.route_graph: Optional[Dict] = None

        # Load existing data
        self._load_objects()
        self._load_route_graph()

    # ═══════════════════════════════════════════════════════════════
    # OBJECT PERSISTENCE — Load/Save semantic_objects.geojson
    # ═══════════════════════════════════════════════════════════════

    def _load_objects(self):
        """Load persistent objects from GeoJSON file."""
        if not self.objects_filepath or not os.path.exists(self.objects_filepath):
            return

        try:
            with open(self.objects_filepath, 'r') as f:
                data = json.load(f)

            if data.get('type') != 'FeatureCollection':
                return

            for feat in data.get('features', []):
                props = feat.get('properties', {})
                coords = feat.get('geometry', {}).get('coordinates', [0, 0])
                obj = PersistentObject(
                    obj_id=props.get('obj_id', ''),
                    class_name=props.get('class_name', ''),
                    class_id=props.get('class_id', -1),
                    mobility=props.get('mobility', 'static'),
                    x=coords[0],
                    y=coords[1],
                    confidence=props.get('confidence', 0.0),
                    observation_count=props.get('observation_count', 1),
                    best_score=props.get('best_score', 0.0),
                    first_seen=props.get('first_seen', ''),
                    last_seen=props.get('last_seen', ''),
                    ttl_seconds=props.get('ttl_seconds', -1.0),
                )
                self.objects[obj.obj_id] = obj
                # Track ID counter
                try:
                    idx = int(obj.obj_id.rsplit('_', 1)[-1])
                    self._next_id_counter = max(self._next_id_counter, idx + 1)
                except (ValueError, IndexError):
                    pass

            if self.logger:
                self.logger.info(
                    f'Loaded {len(self.objects)} persistent objects '
                    f'from {self.objects_filepath}'
                )
        except Exception as e:
            if self.logger:
                self.logger.warn(f'Failed to load objects file: {e}')

    def save_objects(self) -> bool:
        """Save persistent objects to GeoJSON FeatureCollection of Points."""
        if not self.objects_filepath:
            return False

        if not self._dirty:
            return True

        features = []
        for obj in self.objects.values():
            feature = {
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': [round(obj.x, 4), round(obj.y, 4)]
                },
                'properties': {
                    'obj_id': obj.obj_id,
                    'class_name': obj.class_name,
                    'class_id': obj.class_id,
                    'mobility': obj.mobility,
                    'confidence': round(obj.confidence, 3),
                    'observation_count': obj.observation_count,
                    'best_score': round(obj.best_score, 3),
                    'first_seen': obj.first_seen,
                    'last_seen': obj.last_seen,
                    'ttl_seconds': obj.ttl_seconds,
                }
            }
            features.append(feature)

        geojson = {
            'type': 'FeatureCollection',
            'metadata': {
                'description': 'SAIM Xplorer Semantic Object Map',
                'frame': 'map',
                'generated_by': 'semantic_localizer',
                'last_updated': datetime.now(timezone.utc).isoformat(),
                'object_count': len(features),
            },
            'features': features,
        }

        try:
            os.makedirs(os.path.dirname(self.objects_filepath), exist_ok=True)
            with open(self.objects_filepath, 'w') as f:
                json.dump(geojson, f, indent=2)
            self._dirty = False
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f'Failed to save objects: {e}')
            return False

    # ═══════════════════════════════════════════════════════════════
    # OBJECT UPDATE LOGIC — Matching, fusion, TTL
    # ═══════════════════════════════════════════════════════════════

    def update_object(
        self,
        class_name: str,
        class_id: int,
        x: float,
        y: float,
        confidence: float,
        distance: float,
        lidar_rays: int,
    ) -> str:
        """
        Add or update a detected object in the persistent map.

        Returns the obj_id of the matched/created object, or '' if filtered.

        Strategy:
        - Filter: reject classes not in allowed_classes whitelist
        - If matching existing object (same class + within match_distance):
          - Dynamic: overwrite position, reset TTL
          - Static: fused position update (weighted average)
        - If no match: create new object
        """
        # ── Whitelist filter ──
        if class_name not in self.allowed_classes:
            return ''
        now_iso = datetime.now(timezone.utc).isoformat()
        mobility = classify_mobility(class_name)
        score = confidence * lidar_rays / max(distance, 0.1)

        # Try to match existing object
        best_match: Optional[PersistentObject] = None
        best_dist = float('inf')

        for obj in self.objects.values():
            if obj.class_name != class_name:
                continue
            dx = obj.x - x
            dy = obj.y - y
            d = math.sqrt(dx * dx + dy * dy)
            if d < self.match_distance and d < best_dist:
                best_match = obj
                best_dist = d

        if best_match is not None:
            # ── Update existing object ──
            obj = best_match

            if mobility == 'dynamic':
                # Dynamic: overwrite position (object moves)
                obj.x = x
                obj.y = y
            else:
                # Static/minor: fused update
                alpha = obj.fused_alpha
                obj.x = alpha * obj.x + (1 - alpha) * x
                obj.y = alpha * obj.y + (1 - alpha) * y

            obj.confidence = max(obj.confidence, confidence)
            obj.observation_count += 1
            obj.last_seen = now_iso
            if score > obj.best_score:
                obj.best_score = score

            # Reset TTL for dynamic objects
            ttl = DEFAULT_TTL.get(mobility, -1.0)
            obj.ttl_seconds = ttl

            self._dirty = True
            return obj.obj_id

        else:
            # ── Create new object ──
            obj_id = f'{class_name}_{self._next_id_counter}'
            self._next_id_counter += 1

            obj = PersistentObject(
                obj_id=obj_id,
                class_name=class_name,
                class_id=class_id,
                mobility=mobility,
                x=x,
                y=y,
                confidence=confidence,
                observation_count=1,
                best_score=score,
                first_seen=now_iso,
                last_seen=now_iso,
                ttl_seconds=DEFAULT_TTL.get(mobility, -1.0),
            )
            self.objects[obj_id] = obj
            self._dirty = True

            if self.logger:
                self.logger.info(
                    f'New object: {obj_id} at ({x:.2f}, {y:.2f}) '
                    f'[{mobility}] conf={confidence:.0%}'
                )
            return obj_id

    def cleanup_expired(self) -> int:
        """
        Remove dynamic/minor objects that have exceeded their TTL.
        Returns number of objects removed.
        """
        now = datetime.now(timezone.utc)
        to_remove = []

        for obj_id, obj in self.objects.items():
            if obj.ttl_seconds <= 0:
                continue  # never expires
            try:
                last = datetime.fromisoformat(obj.last_seen)
                if last.tzinfo is None:
                    last = last.replace(tzinfo=timezone.utc)
                elapsed = (now - last).total_seconds()
                if elapsed > obj.ttl_seconds:
                    to_remove.append(obj_id)
            except (ValueError, TypeError):
                continue

        for obj_id in to_remove:
            del self.objects[obj_id]

        if to_remove:
            self._dirty = True
            if self.logger:
                self.logger.info(f'Expired {len(to_remove)} objects: {to_remove}')

        return len(to_remove)

    def reset_objects(self) -> int:
        """
        Clear all objects from in-memory store and mark dirty for save.
        Used by experimental protocol to start with a clean slate.
        Returns count of objects cleared.
        """
        count = len(self.objects)
        self.objects.clear()
        self._next_id_counter = 0
        self._dirty = True
        if self.logger:
            self.logger.info(f'Reset: cleared {count} objects from memory')
        return count

    def reload_route_graph(self) -> bool:
        """Reload the route graph from disk. Useful after external edits."""
        self._load_route_graph()
        return self.route_graph is not None

    # ═══════════════════════════════════════════════════════════════
    # ROUTE GRAPH — Load, annotate, save
    # ═══════════════════════════════════════════════════════════════

    def _load_route_graph(self):
        """Load the Nav2 route graph GeoJSON."""
        if not self.route_graph_filepath or not os.path.exists(self.route_graph_filepath):
            return

        try:
            with open(self.route_graph_filepath, 'r') as f:
                self.route_graph = json.load(f)
            n_features = len(self.route_graph.get('features', []))
            if self.logger:
                self.logger.info(
                    f'Loaded route graph: {n_features} features '
                    f'from {self.route_graph_filepath}'
                )
        except Exception as e:
            if self.logger:
                self.logger.warn(f'Failed to load route graph: {e}')

    def annotate_route_graph(self) -> Optional[Dict]:
        """
        Create an annotated copy of the route graph with semantic metadata
        injected on edges near detected objects.

        For each edge (LineString), checks proximity to all persistent objects.
        If an object is within edge_proximity meters, adds:
          - metadata.penalty (additive, from object type)
          - metadata.speed_limit (minimum across all nearby objects)
          - metadata.semantic_objects (list of nearby objects with details)
          - metadata.class (set to most significant nearby object class)

        Returns the annotated graph dict, or None if no graph loaded.
        """
        if self.route_graph is None:
            return None

        if not self.objects:
            return copy.deepcopy(self.route_graph)

        annotated = copy.deepcopy(self.route_graph)

        for feature in annotated.get('features', []):
            geom = feature.get('geometry', {})

            # Only annotate edges (LineString or MultiLineString)
            geom_type = geom.get('type', '')
            if geom_type == 'LineString':
                coords = geom.get('coordinates', [])
            elif geom_type == 'MultiLineString':
                multi_coords = geom.get('coordinates', [[]])
                coords = multi_coords[0] if multi_coords else []
            else:
                continue

            if len(coords) < 2:
                continue

            props = feature.get('properties', {})
            metadata = props.setdefault('metadata', {})

            # Find objects near this edge
            nearby = self._find_objects_near_edge(coords)

            if not nearby:
                # No objects near this edge — clear any old semantic metadata
                metadata.pop('penalty', None)
                metadata.pop('speed_limit', None)
                metadata.pop('class', None)
                metadata.pop('semantic_objects', None)
                continue

            # Calculate aggregate penalty and speed limit
            # IMPORTANT: Start from 0, not from existing metadata.
            # PenaltyScorer in Route Server adds this to the edge's
            # intrinsic cost (length). We only write the SEMANTIC penalty.
            total_penalty = 0.0
            min_speed = 100

            semantic_objs = []
            most_significant_class = None
            highest_penalty = 0.0

            for obj, dist_to_edge in nearby:
                mob = obj.mobility
                penalty = DEFAULT_PENALTIES.get(mob, 1.0)

                # Scale penalty by proximity (closer = higher penalty)
                proximity_factor = max(0.1, 1.0 - dist_to_edge / self.edge_proximity)
                scaled_penalty = penalty * proximity_factor

                # Scale by observation confidence
                scaled_penalty *= min(obj.confidence, 1.0)

                total_penalty += scaled_penalty
                speed = DEFAULT_SPEED_LIMITS.get(mob, 100)
                min_speed = min(min_speed, speed)

                if scaled_penalty > highest_penalty:
                    highest_penalty = scaled_penalty
                    most_significant_class = obj.class_name

                semantic_objs.append({
                    'obj_id': obj.obj_id,
                    'class': obj.class_name,
                    'mobility': obj.mobility,
                    'x': round(obj.x, 3),
                    'y': round(obj.y, 3),
                    'confidence': round(obj.confidence, 2),
                    'observations': obj.observation_count,
                    'distance_to_edge': round(dist_to_edge, 3),
                })

            # Write metadata using Nav2 conventions
            metadata['penalty'] = round(total_penalty, 2)
            metadata['speed_limit'] = min_speed
            if most_significant_class:
                metadata['class'] = most_significant_class
            metadata['semantic_objects'] = semantic_objs

            # Ensure edge is overridable so scoring plugins can use it
            props['overridable'] = True

        return annotated

    def save_annotated_graph(self) -> bool:
        """Annotate the route graph and save to output filepath."""
        if not self.output_graph_filepath:
            return False

        annotated = self.annotate_route_graph()
        if annotated is None:
            return False

        try:
            os.makedirs(os.path.dirname(self.output_graph_filepath), exist_ok=True)
            with open(self.output_graph_filepath, 'w') as f:
                json.dump(annotated, f, indent=2)
            if self.logger:
                self.logger.info(
                    f'Saved annotated route graph to {self.output_graph_filepath}'
                )
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f'Failed to save annotated graph: {e}')
            return False

    def save_nav2_graph(self, nav2_filepath: str) -> bool:
        """Annotate → flatten metadata → save Nav2-compatible graph.

        This replaces the external convert_geojson_for_route_server.py script.
        Nav2 Route Server only accepts flat metadata (string, number, bool).
        We strip nested 'semantic_objects' arrays and keep only scalar fields.
        """
        annotated = self.annotate_route_graph()
        if annotated is None:
            return False

        # Flatten metadata on edges
        edges_converted = 0
        for feature in annotated.get('features', []):
            props = feature.get('properties', {})
            if 'startid' in props and 'endid' in props and 'metadata' in props:
                meta = props['metadata']
                flat = {}
                if 'penalty' in meta:
                    flat['penalty'] = float(meta['penalty'])
                if 'speed_limit' in meta:
                    flat['speed_limit'] = float(meta['speed_limit'])
                # All other keys (class, semantic_objects) are dropped —
                # Nav2 Route Server can't parse nested arrays
                props['metadata'] = flat
                edges_converted += 1

        try:
            os.makedirs(os.path.dirname(nav2_filepath), exist_ok=True)
            with open(nav2_filepath, 'w') as f:
                json.dump(annotated, f, indent=2)
            if self.logger:
                self.logger.info(
                    f'Nav2 graph saved: {edges_converted} edges → {nav2_filepath}'
                )
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f'Failed to save Nav2 graph: {e}')
            return False

    # ═══════════════════════════════════════════════════════════════
    # GEOMETRY — Point-to-segment distance
    # ═══════════════════════════════════════════════════════════════

    def _find_objects_near_edge(
        self, edge_coords: List[List[float]]
    ) -> List[Tuple[PersistentObject, float]]:
        """
        Find all persistent objects within edge_proximity of an edge segment.

        Args:
            edge_coords: [[x1,y1], [x2,y2]] coordinates of the edge

        Returns:
            List of (PersistentObject, distance_to_edge) tuples
        """
        if len(edge_coords) < 2:
            return []

        # Edge segment: from start to end
        ax, ay = edge_coords[0][0], edge_coords[0][1]
        bx, by = edge_coords[-1][0], edge_coords[-1][1]

        nearby = []
        for obj in self.objects.values():
            d = self._point_to_segment_distance(obj.x, obj.y, ax, ay, bx, by)
            if d <= self.edge_proximity:
                nearby.append((obj, d))

        return nearby

    @staticmethod
    def _point_to_segment_distance(
        px: float, py: float,
        ax: float, ay: float,
        bx: float, by: float,
    ) -> float:
        """
        Compute minimum distance from point (px, py) to line segment (ax,ay)-(bx,by).
        """
        dx = bx - ax
        dy = by - ay
        len_sq = dx * dx + dy * dy

        if len_sq < 1e-10:
            # Degenerate segment (start == end)
            return math.sqrt((px - ax) ** 2 + (py - ay) ** 2)

        # Projection parameter t ∈ [0, 1]
        t = ((px - ax) * dx + (py - ay) * dy) / len_sq
        t = max(0.0, min(1.0, t))

        # Closest point on segment
        cx = ax + t * dx
        cy = ay + t * dy

        return math.sqrt((px - cx) ** 2 + (py - cy) ** 2)

    # ═══════════════════════════════════════════════════════════════
    # UTILITIES
    # ═══════════════════════════════════════════════════════════════

    def get_stats(self) -> Dict[str, Any]:
        """Return statistics about the semantic map."""
        by_mobility = {}
        by_class = {}
        for obj in self.objects.values():
            by_mobility[obj.mobility] = by_mobility.get(obj.mobility, 0) + 1
            by_class[obj.class_name] = by_class.get(obj.class_name, 0) + 1

        return {
            'total_objects': len(self.objects),
            'by_mobility': by_mobility,
            'by_class': by_class,
            'has_route_graph': self.route_graph is not None,
            'dirty': self._dirty,
        }

    def get_objects_list(self) -> List[Dict]:
        """Return all objects as a simple list of dicts."""
        return [
            {
                'obj_id': o.obj_id,
                'class': o.class_name,
                'mobility': o.mobility,
                'x': o.x,
                'y': o.y,
                'confidence': o.confidence,
                'observations': o.observation_count,
            }
            for o in self.objects.values()
        ]