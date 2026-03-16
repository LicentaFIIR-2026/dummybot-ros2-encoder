#!/usr/bin/env python3
"""
Create a sample Nav2 Route Server compatible GeoJSON graph
for the FIIR CB ground floor hallway.

Usage:
    python3 create_fiir_route_graph.py [output_path]

The graph should be edited with real coordinates from your map.
Use RViz2 "Publish Point" to get coordinates, or the Nav2 Route Tool.

This script creates a template with the correct format that you can
then modify with actual coordinates from the FIIR hallway map.
"""

import json
import sys
import os


def create_sample_graph(output_path: str):
    """
    Create a sample route graph with Nav2 Route Server compatible format.

    Format requirements (from Nav2 docs):
    - FeatureCollection with Point (nodes) and LineString (edges)
    - Nodes: id (int, unique), coordinates [x, y], frame
    - Edges: id (int, unique, convention >=10000), startid, endid
    - Edges are DIRECTIONAL — need 2 for bidirectional
    - metadata: arbitrary key-value pairs for plugins
    """

    # ─── NODES ───
    # Replace these coordinates with actual positions from your FIIR map.
    # Use: ros2 topic echo /clicked_point  (click in RViz2)
    # or:  ros2 run nav2_rviz_plugins route_tool

    nodes = [
        {
            "id": 0,
            "name": "entrance",
            "coordinates": [0.5, 3.0],
            "metadata": {"class": "entrance", "description": "FIIR CB entrance"}
        },
        {
            "id": 1,
            "name": "corridor_start",
            "coordinates": [2.0, 3.0],
            "metadata": {"class": "corridor"}
        },
        {
            "id": 2,
            "name": "corridor_mid",
            "coordinates": [4.0, 3.0],
            "metadata": {"class": "corridor"}
        },
        {
            "id": 3,
            "name": "corridor_junction",
            "coordinates": [6.0, 3.0],
            "metadata": {"class": "junction"}
        },
        {
            "id": 4,
            "name": "corridor_end",
            "coordinates": [8.0, 3.0],
            "metadata": {"class": "corridor"}
        },
        {
            "id": 5,
            "name": "lab_door",
            "coordinates": [6.0, 5.0],
            "metadata": {
                "class": "door",
                "operations": {
                    "open_door": {
                        "type": "open_door",
                        "trigger": "ON_ENTER",
                        "metadata": {"door_id": 1}
                    }
                }
            }
        },
        {
            "id": 6,
            "name": "side_corridor",
            "coordinates": [4.0, 1.0],
            "metadata": {"class": "corridor"}
        },
    ]

    # ─── EDGES ───
    # Bidirectional edges: each physical connection needs 2 entries
    # IDs start at 10000 (convention)

    edge_pairs = [
        # (start, end, metadata)
        (0, 1, {"class": "hallway"}),
        (1, 2, {"class": "hallway"}),
        (2, 3, {"class": "hallway", "speed_limit": 80}),
        (3, 4, {"class": "hallway"}),
        (3, 5, {"class": "hallway_to_door", "speed_limit": 60}),
        (2, 6, {"class": "side_hallway"}),
    ]

    edges = []
    edge_id = 10000
    for start, end, meta in edge_pairs:
        # Forward edge
        edges.append({
            "id": edge_id,
            "startid": start,
            "endid": end,
            "overridable": True,
            "metadata": dict(meta),
        })
        edge_id += 1

        # Reverse edge (bidirectional)
        edges.append({
            "id": edge_id,
            "startid": end,
            "endid": start,
            "overridable": True,
            "metadata": dict(meta),
        })
        edge_id += 1

    # ─── Build GeoJSON FeatureCollection ───

    features = []

    # Add nodes as Point features
    for node in nodes:
        feature = {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": node["coordinates"]
            },
            "properties": {
                "id": node["id"],
                "frame": "map",
            }
        }
        # Add metadata if present
        meta = node.get("metadata", {})
        if meta:
            feature["properties"]["metadata"] = meta
        features.append(feature)

    # Add edges as LineString features
    # We need to look up node coordinates for the LineString geometry
    node_coords = {n["id"]: n["coordinates"] for n in nodes}

    for edge in edges:
        start_coords = node_coords[edge["startid"]]
        end_coords = node_coords[edge["endid"]]

        feature = {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": [start_coords, end_coords]
            },
            "properties": {
                "id": edge["id"],
                "startid": edge["startid"],
                "endid": edge["endid"],
                "overridable": edge.get("overridable", True),
            }
        }
        meta = edge.get("metadata", {})
        if meta:
            feature["properties"]["metadata"] = meta
        features.append(feature)

    geojson = {
        "type": "FeatureCollection",
        "features": features
    }

    # Save
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(geojson, f, indent=2)

    print(f'Created route graph: {output_path}')
    print(f'  Nodes: {len(nodes)}')
    print(f'  Edges: {len(edges)} ({len(edge_pairs)} bidirectional pairs)')
    print()
    print('IMPORTANT: Edit the node coordinates to match your actual FIIR map!')
    print('Use RViz2 "Publish Point" tool or Nav2 Route Tool to get coordinates.')


if __name__ == '__main__':
    output = sys.argv[1] if len(sys.argv) > 1 else 'route_graph_fiir.geojson'
    create_sample_graph(output)
