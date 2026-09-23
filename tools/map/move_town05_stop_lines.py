#!/usr/bin/env python3
"""Build an offline candidate moving existing stop lines before crosswalk centers.

Only unshared stop-line node XY/geographic coordinates are edited. Crosswalks,
road geometry, IDs, regulatory references, elevations and planner settings stay
unchanged. Unmatched stop lines are reported, never assigned across an intersection.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from shapely.geometry import LineString, Polygon


def tags(element):
    return {t.get("k"): t.get("v") for t in element.findall("tag")}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def move_lines(data, manifest, source, native_report, offset=3.0):
    if not math.isfinite(offset) or offset <= 0:
        raise ValueError("Offset must be finite and positive")
    if digest(data) != manifest["map_sha256"]:
        raise ValueError("Map and traffic signal manifest do not match")
    root = ET.fromstring(data)
    nodes = {n.get("id"): n for n in root.findall("node")}
    xy = {nid: tuple(float(tags(n)[k]) for k in ("local_x", "local_y"))
          for nid, n in nodes.items()}
    ways = {int(w.get("id")): w for w in root.findall("way")}
    refs = {wid: [n.get("ref") for n in w.findall("nd")] for wid, w in ways.items()}
    usage = Counter(n for ns in refs.values() for n in ns)
    relations = {int(r.get("id")): r for r in root.findall("relation")}
    crosswalks = {}
    for rid, relation in relations.items():
        if tags(relation).get("type") == "regulatory_element" and tags(relation).get("subtype") == "crosswalk":
            polygon_id = int(relation.find("member[@role='crosswalk_polygon']").get("ref"))
            poly = Polygon([xy[n] for n in refs[polygon_id]])
            if not poly.is_valid:
                raise ValueError(f"Invalid crosswalk {rid}")
            crosswalks[rid] = poly
    signals = {s["opendrive_id"]: s for s in source["signals"]}
    signs = {s["opendrive_id"]: s for s in source["stop_signs"]}
    entries = manifest["groups"] + native_report["stop_signs"]
    if {e["stop_line_id"] for e in entries} != {
        wid for wid, w in ways.items() if tags(w).get("type") == "stop_line"
    }:
        raise ValueError("Stop-line inventory differs from recorded native metadata")
    replacements, moved, unchanged = {}, [], []
    for entry in entries:
        wid = entry["stop_line_id"]
        coordinates = [xy[n] for n in refs[wid]]
        line = LineString(coordinates)
        midpoint = line.interpolate(0.5, normalized=True)
        native = signals if "group_id" in entry else signs
        record = native[entry["opendrive_id"]]
        # Native signal lines were constructed perpendicular to the first stop's
        # heading. Keep that fixed heading across reruns; nearest-lane selection
        # can flip at a multi-lane line's midpoint after a translation.
        stop = (record["stops"][0] if "group_id" in entry else
                min(record["stops"], key=lambda s: math.dist(s["position"][:2], midpoint.coords[0])))
        fx, fy = stop["forward"]
        length = math.hypot(fx, fy)
        fx, fy = fx / length, fy / length
        # A bounded forward ray rejects crossings behind this approach and
        # crossings on the far side of the junction. Preserve all unmatched lines.
        ray = LineString([(midpoint.x, midpoint.y),
                          (midpoint.x + 20 * fx, midpoint.y + 20 * fy)])
        candidates = []
        for cid, poly in crosswalks.items():
            if not ray.intersects(poly):
                continue
            along = (poly.centroid.x - midpoint.x) * fx + (poly.centroid.y - midpoint.y) * fy
            if 0 < along <= 15:
                candidates.append((along, cid, poly))
        candidates.sort(key=lambda row: row[0])
        if not candidates:
            unchanged.append({"stop_line_id": wid, "reason": "no_crosswalk_on_forward_approach_within_15m"})
            continue
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 3:
            raise ValueError(f"Ambiguous crosswalk for stop line {wid}")
        along, cid, poly = candidates[0]
        dx, dy = (along - offset) * fx, (along - offset) * fy
        new_points = [(x + dx, y + dy) for x, y in coordinates]
        new_line = LineString(new_points)
        center_distance = ((poly.centroid.x - new_line.interpolate(.5, normalized=True).x) * fx +
                           (poly.centroid.y - new_line.interpolate(.5, normalized=True).y) * fy)
        if abs(center_distance - offset) > 1e-7:
            raise ValueError(f"Incorrect center offset for {wid}")
        if new_line.intersects(poly) or new_line.distance(poly) < 0.1:
            raise ValueError(f"Stop line {wid} would overlap/touch crosswalk {cid}")
        if abs(new_line.length - line.length) > 1e-7:
            raise ValueError(f"Stop line {wid} changed length")
        for nid, point in zip(refs[wid], new_points):
            if usage[nid] != 1:
                raise ValueError(f"Stop line {wid} shares node {nid} with another geometry")
            if abs(along - offset) > 1e-7:
                replacements[nid] = point
        moved.append({"stop_line_id": wid, "crosswalk_regulatory_id": cid,
                      "crosswalk_center_xy": list(poly.centroid.coords[0]), "forward": [fx, fy],
                      "old_points": coordinates, "new_points": new_points,
                      "old_center_distance_m": along, "new_center_distance_m": center_distance,
                      "translation_m": along - offset, "edge_clearance_m": new_line.distance(poly)})
    # Patch only the affected original node blocks, retaining all other XML bytes.
    def replace_node(match):
        block = match.group(0)
        nid = re.search(rb'\bid="([^"]+)"', block).group(1).decode()
        if nid not in replacements:
            return block
        x, y = replacements[nid]
        for key, value in (("local_x", x), ("local_y", y)):
            pattern = rb'(<tag\s+k="' + key.encode() + rb'"\s+v=")[^"]+("\s*/>)'
            block, count = re.subn(pattern, lambda m: m[1] + f"{value:.9f}".encode() + m[2], block)
            if count != 1:
                raise ValueError(f"Unexpected coordinate tag format for node {nid}")
        for key, value in (("lon", x / 111319.49079327358), ("lat", y / 110692.0)):
            block, count = re.subn(rb'\b' + key.encode() + rb'="[^"]+"',
                                  key.encode() + b'="' + repr(value).encode() + b'"', block)
            if count != 1:
                raise ValueError(f"Unexpected geographic coordinates for node {nid}")
        return block
    output = re.sub(rb'<node\b[^>]*>.*?</node>', replace_node, data, flags=re.DOTALL)
    after = ET.fromstring(output)
    before_elements = {(e.tag, e.get("id")): ET.tostring(e) for e in root}
    changed = [(e.tag, e.get("id")) for e in after
               if ET.tostring(e) != before_elements[(e.tag, e.get("id"))]]
    if set(changed) != {("node", n) for n in replacements}:
        raise ValueError("Unexpected non-stop-line map edits")
    updated_manifest = dict(manifest, map_sha256=digest(output))
    associated = {row["crosswalk_regulatory_id"] for row in moved}
    report = {"source_sha256": digest(data), "map_sha256": digest(output),
              "center_offset_m": offset, "crosswalk_count": len(crosswalks),
              "matched_crosswalk_count": len(associated), "matched_stop_line_count": len(moved),
              "changed_node_count": len(changed), "unchanged_stop_lines": unchanged,
              "crosswalks_without_matched_existing_stop_line": sorted(set(crosswalks) - associated),
              "moves": moved, "preserved": ["crosswalk_geometry", "road_geometry", "way_and_relation_ids",
                  "regulatory_references", "stop_line_length_and_direction", "elevation", "speed_tags"]}
    return output, updated_manifest, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--offset", type=float, default=3.0)
    args = parser.parse_args()
    if args.output_dir.resolve() == args.map_dir.resolve():
        parser.error("Output must be a separate candidate directory")
    load = lambda p: json.loads(p.read_text())
    osm, manifest, report = move_lines(
        (args.map_dir / "lanelet2_map.osm").read_bytes(),
        load(args.map_dir / "carla_traffic_signals.json"), load(args.source),
        load(args.map_dir / "carla_map_report.json"), args.offset)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "lanelet2_map.osm").write_bytes(osm)
    (args.output_dir / "carla_traffic_signals.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (args.output_dir / "carla_stop_line_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "moves"}, indent=2))


if __name__ == "__main__":
    main()
