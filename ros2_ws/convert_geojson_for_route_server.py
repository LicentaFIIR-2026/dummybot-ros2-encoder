#!/usr/bin/env python3
"""
convert_geojson_for_route_server.py
====================================

Convertește route_graph_fiir_semantic.geojson în format compatibil
cu Nav2 Route Server GeoJsonGraphFileLoader.

Problema: Route Server-ul nu poate parsa valori nested (array/obiecte)
în metadata. Acceptă doar flat key-value (string, number, bool).

Soluția: Flatten-uiește semantic_objects array-ul în chei individuale
și codează informația compactă ca string-uri.

Exemplu transformare:
  ÎNAINTE (nested - INCOMPATIBIL):
    "metadata": {
      "penalty": 1.61,
      "speed_limit": 60,
      "class": "bench",
      "semantic_objects": [
        {"obj_id": "tv_7", "class": "tv", "distance_to_edge": 1.119, ...},
        {"obj_id": "bench_13", "class": "bench", "distance_to_edge": 0.789, ...}
      ]
    }

  DUPĂ (flat - COMPATIBIL):
    "metadata": {
      "penalty": 1.61,
      "speed_limit": 60,
      "dominant_class": "bench",
      "num_objects": 2,
      "obj_0_id": "tv_7",
      "obj_0_class": "tv",
      "obj_0_distance": 1.119,
      "obj_0_confidence": 0.51,
      "obj_0_mobility": "static",
      "obj_1_id": "bench_13",
      "obj_1_class": "bench",
      "obj_1_distance": 0.789,
      "obj_1_confidence": 0.72,
      "obj_1_mobility": "static",
      "objects_summary": "tv_7:1.12|bench_13:0.79|bottle_14:0.80"
    }

Utilizare:
  python3 convert_geojson_for_route_server.py \
    --input  maps/route_graph_fiir_semantic.geojson \
    --output maps/route_graph_fiir_nav2.geojson
"""

import json
import argparse
import sys
import copy
from pathlib import Path


def flatten_edge_metadata(metadata: dict) -> dict:
    flat = {}
    if 'penalty' in metadata:
        flat['penalty'] = float(metadata['penalty'])
    if 'speed_limit' in metadata:
        flat['speed_limit'] = float(metadata['speed_limit'])
    return flat


def convert_geojson(input_data: dict) -> dict:
    """Convertește întregul GeoJSON — flatten metadata pe toate edges."""
    output = copy.deepcopy(input_data)

    edges_converted = 0
    nodes_kept = 0

    for feature in output.get('features', []):
        props = feature.get('properties', {})

        # Detectăm edges (au startid + endid)
        if 'startid' in props and 'endid' in props:
            if 'metadata' in props:
                props['metadata'] = flatten_edge_metadata(props['metadata'])
                edges_converted += 1
        else:
            # Noduri — le păstrăm neschimbate
            nodes_kept += 1

    return output, nodes_kept, edges_converted


def main():
    parser = argparse.ArgumentParser(
        description='Convert semantic GeoJSON to Nav2 Route Server compatible format')
    parser.add_argument('--input', '-i', required=True,
                        help='Input GeoJSON (cu semantic_objects nested)')
    parser.add_argument('--output', '-o', required=True,
                        help='Output GeoJSON (flat metadata, compatibil Route Server)')
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        print(f'[EROARE] Fișierul nu există: {input_path}')
        sys.exit(1)

    # Citire
    with open(input_path, 'r') as f:
        data = json.load(f)

    # Conversie
    converted, nodes, edges = convert_geojson(data)

    # Scriere
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(converted, f, indent=2)

    print(f'Conversie completă:')
    print(f'  Noduri păstrate: {nodes}')
    print(f'  Edges convertite: {edges}')
    print(f'  Output: {output_path}')

    # Verificare — arată un exemplu
    for feature in converted.get('features', []):
        props = feature.get('properties', {})
        if 'startid' in props:
            print(f'\n  Exemplu edge {props["id"]} ({props["startid"]}→{props["endid"]}):')
            meta = props.get('metadata', {})
            for k, v in meta.items():
                print(f'    {k}: {v}')
            break


if __name__ == '__main__':
    main()
