# Waypoint-uri Robot DummyBot
```json
{
  "usa-spate": {
    "position": {"x": 1.103, "y": 2.728, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": -0.779338, "w": 0.626604}
  },
  "hol-spate": {
    "position": {"x": 4.293, "y": 2.709, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.006109, "w": 0.999981}
  },
  "home": {
    "position": {"x": -0.754, "y": -1.425, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": -0.999848, "w": 0.017452}
  }
}
```

## Instrucțiuni

Pentru a naviga robotul la un waypoint:
1. Extrage coordonatele din JSON de mai sus
2. Publică pe `/goal_pose` (geometry_msgs/PoseStamped) cu `frame_id="map"`