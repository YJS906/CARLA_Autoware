#!/usr/bin/env python3
"""Add native CARLA Town05 semantics without rebuilding its road lanelets.

Capture uses read-only CARLA APIs: never load a world, tick, move actors or set lights.
Output is a candidate. Validate with Autoware's Lanelet2 loader before installing it.
The source JSON can be reused offline with the matching OpenDRIVE file.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import carla
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

SOURCE_TAG = "carla_town05_native_v1"


def digest(data):
    return hashlib.sha256(data).hexdigest()


def tags(element):
    return {t.get("k"): t.get("v") for t in element.findall("tag")}


def tag(element, key, value):
    for child in element.findall("tag"):
        if child.get("k") == key:
            child.set("v", str(value))
            return
    ET.SubElement(element, "tag", k=key, v=str(value))


def xyz(location):
    return [location.x, -location.y, location.z]


def waypoint_record(wp):
    t = wp.transform
    f = t.get_forward_vector()
    return {"road_id": wp.road_id, "lane_id": wp.lane_id, "s": wp.s,
            "position": xyz(t.location), "forward": [f.x, -f.y],
            "width": wp.lane_width}


def capture_source(host, port, xodr):
    client = carla.Client(host, port)
    client.set_timeout(5)
    world = client.get_world()
    wmap = world.get_map()
    if wmap.name.rsplit("/", 1)[-1] not in ("Town05", "Town05_Opt"):
        raise ValueError(f"Expected native Town05; world is {wmap.name}")
    # A new CARLA client's actor cache can initially be empty.
    lights = []
    for _ in range(20):
        lights = list(world.get_actors().filter("traffic.traffic_light"))
        if lights:
            break
        time.sleep(0.2)
    expected = {e.get("id") for e in ET.fromstring(xodr).findall(".//signal")
                if e.get("type") == "1000001" and e.get("dynamic") == "yes"}
    records = []
    for actor in lights:
        signal_id = actor.get_opendrive_id()
        boxes = []
        for box in actor.get_light_boxes():
            boxes.append({"center": xyz(box.location),
                          "extent": [box.extent.x, box.extent.y, box.extent.z],
                          "yaw": -box.rotation.yaw})
        stops = [waypoint_record(w) for w in actor.get_stop_waypoints()]
        if not stops or not boxes:
            raise ValueError(f"Traffic light {signal_id} has no stop waypoints/light boxes")
        records.append({"opendrive_id": signal_id, "boxes": boxes, "stops": stops,
                        "affected": [waypoint_record(w) for w in actor.get_affected_lane_waypoints()]})
    found = [r["opendrive_id"] for r in records]
    if set(found) != expected or len(found) != len(set(found)):
        raise ValueError(f"Light inventory mismatch: expected {len(expected)}, got {len(found)}")
    xroot = ET.fromstring(xodr)
    stop_definitions = {e.get("id"): e for e in xroot.findall(".//signal") if e.get("type") == "206"}
    stop_actors = list(world.get_actors().filter("traffic.stop"))
    triggers = [a.get_transform().transform(a.trigger_volume.location) for a in stop_actors]
    landmarks = {lm.id: lm for lm in wmap.get_all_landmarks_of_type("206")}
    stop_signs = {sid: {"opendrive_id": sid, "position": xyz(landmarks[sid].transform.location),
                        "width": float(definition.get("width")),
                        "height": float(definition.get("height")), "stops": [], "affected": []}
                  for sid, definition in stop_definitions.items()}
    used_triggers = set()
    for road in xroot.findall("road"):
        for reference in road.findall("./signals/signalReference"):
            sid = reference.get("id")
            if sid not in stop_signs:
                continue
            for validity in reference.findall("validity"):
                a, b = int(validity.get("fromLane")), int(validity.get("toLane"))
                for lane_id in range(min(a, b), max(a, b)+1):
                    if lane_id == 0:
                        continue
                    wp = wmap.get_waypoint_xodr(int(road.get("id")), lane_id, float(reference.get("s")))
                    samples = [wp]
                    for step in range(1, 45):
                        samples.extend(wp.previous(step*.5))
                    matches = []
                    for index, position in enumerate(triggers):
                        near = min(samples, key=lambda q: math.hypot(q.transform.location.x-position.x,
                                                                  q.transform.location.y-position.y))
                        error = math.hypot(near.transform.location.x-position.x,
                                           near.transform.location.y-position.y)
                        if error < .7:
                            matches.append((index, position, near))
                    if len(matches) != 1:
                        raise ValueError(f"Stop sign {sid}, connector {wp.road_id}/{wp.lane_id}: "
                                         f"expected one real upstream stop trigger, found {len(matches)}")
                    index, position, near = matches[0]
                    used_triggers.add(index)
                    f = near.transform.get_forward_vector()
                    along = (position.x-near.transform.location.x)*f.x+(position.y-near.transform.location.y)*f.y
                    projected = wmap.get_waypoint_xodr(near.road_id, near.lane_id,
                                                       near.s+along*(-1 if near.lane_id > 0 else 1))
                    if projected is None:
                        raise ValueError("Physical stop trigger projection left its road")
                    record = waypoint_record(projected)
                    record["physical_trigger_center"] = xyz(position)
                    record["lateral_projection_error_m"] = math.hypot(
                        position.x-projected.transform.location.x, position.y-projected.transform.location.y)
                    if record["lateral_projection_error_m"] > .7:
                        raise ValueError("Stop trigger is too far from the controlled lane")
                    prior = stop_signs[sid]["stops"]
                    if not any((q["road_id"], q["lane_id"]) == (record["road_id"], record["lane_id"]) for q in prior):
                        prior.append(record)
                    stop_signs[sid]["affected"].append(waypoint_record(wp))
    if any(not item["stops"] for item in stop_signs.values()):
        raise ValueError("An OpenDRIVE stop sign has no validated physical stopping lane")
    scene_stops = []
    for index, position in enumerate(triggers):
        if index in used_triggers:
            continue
        wp = wmap.get_waypoint(carla.Location(x=position.x, y=position.y, z=position.z))
        if wp is None or math.hypot(wp.transform.location.x-position.x,
                                   wp.transform.location.y-position.y) > .7:
            raise ValueError("A scene stop trigger cannot be associated with its driving lane")
        record = waypoint_record(wp)
        record["physical_trigger_center"] = xyz(position)
        record["source_key"] = "scene_stop_"+digest(
            json.dumps([round(position.x, 4), round(position.y, 4)]).encode())[:16]
        scene_stops.append(record)
    return {"schema": 1, "map_name": "Town05", "world_name": wmap.name,
            "xodr_sha256": digest(xodr.encode()),
            "live_xodr_sha256": digest(wmap.to_opendrive().encode()),
            "stop_signs": sorted(stop_signs.values(), key=lambda r: int(r["opendrive_id"])),
            "scene_stops": sorted(scene_stops, key=lambda r: r["source_key"]),
            "signals": sorted(records, key=lambda r: int(r["opendrive_id"]))}


class MapBuilder:
    def __init__(self, root):
        self.root = root
        self.next_id = max(int(e.get("id", "0")) for e in root) + 1
        self.nodes = {n.get("id"): [float(tags(n)["local_x"]),
                      float(tags(n)["local_y"]), float(tags(n).get("ele", "0"))]
                      for n in root.findall("node")}
        self.ways = {e.get("id"): e for e in root.findall("way")}
        original_points = {wid: self.points(wid) for wid in self.ways}
        self.lanes = []
        self.deduplicated = []
        for way in self.ways.values():
            refs = [n.get("ref") for n in way.findall("nd")]
            if not refs:
                continue
            groups = []
            for ref in refs:
                if groups and self.nodes[ref] == self.nodes[groups[-1][-1]]:
                    groups[-1].append(ref)
                else:
                    groups.append([ref])
            if len(groups) < 2:
                raise ValueError(f"Degenerate original way {way.get('id')}")
            cleaned = [g[-1] if i == len(groups)-1 else g[0] for i, g in enumerate(groups)]
            if len(cleaned) != len(refs):
                self.deduplicated.append({"way_id": int(way.get("id")),
                                         "removed": len(refs)-len(cleaned)})
                for child in list(way.findall("nd")):
                    way.remove(child)
                for i, ref in enumerate(cleaned):
                    way.insert(i, ET.Element("nd", ref=ref))
        for relation in root.findall("relation"):
            t = tags(relation)
            if t.get("type") != "lanelet" or t.get("subtype") != "road":
                continue
            members = {m.get("role"): m.get("ref") for m in relation.findall("member")}
            left, right = self.points(members["left"]), self.points(members["right"])
            if math.dist(left[0], right[0])+math.dist(left[-1], right[-1]) > \
                    math.dist(left[0], right[-1])+math.dist(left[-1], right[0]):
                right.reverse()
            # Original Town05 boundaries have paired cross-section samples. Keep that
            # pairing for geometric matching, independent of the deduplication above.
            ol, ort = original_points[members["left"]], original_points[members["right"]]
            if len(ol) != len(ort):
                raise ValueError("Original Town05 cross-section samples are not paired")
            if math.dist(ol[0], ort[0])+math.dist(ol[-1], ort[-1]) > \
                    math.dist(ol[0], ort[-1])+math.dist(ol[-1], ort[0]):
                ort = list(reversed(ort))
            center = LineString([tuple((a[j]+b[j])/2 for j in range(3)) for a, b in zip(ol, ort)])
            poly = Polygon(left+list(reversed(right)))
            self.lanes.append({"id": int(relation.get("id")), "element": relation,
                               "polygon": poly, "query_polygon": poly if poly.is_valid else poly.buffer(0),
                               "center": center, "left": left, "right": right,
                               "z": sum(p[2] for p in left+right)/(len(left)+len(right))})
        self.index = STRtree([r["query_polygon"] for r in self.lanes])

    def points(self, way_id):
        return [self.nodes[n.get("ref")] for n in self.ways[way_id].findall("nd")]

    def element(self, kind):
        element = ET.Element(kind, id=str(self.next_id), visible="true", version="1")
        self.next_id += 1
        self.root.append(element)
        tag(element, "source", SOURCE_TAG)
        return element

    def node(self, point):
        node = self.element("node")
        x, y, z = point
        # Geographic values match the original map's near-origin convention;
        # Autoware's Local projector uses local_x/local_y directly.
        node.set("lon", str(x/111319.49079327358))
        node.set("lat", str(y/110692.0))
        tag(node, "local_x", f"{x:.9f}")
        tag(node, "local_y", f"{y:.9f}")
        tag(node, "ele", f"{z:.9f}")
        return node.get("id")

    def way(self, points, attributes):
        closed = len(points) > 2 and math.dist(points[0], points[-1]) < 1e-9
        refs = [self.node(point) for point in (points[:-1] if closed else points)]
        if closed:
            refs.append(refs[0])
        way = self.element("way")
        for ref in refs:
            ET.SubElement(way, "nd", ref=ref)
        for key, value in attributes.items():
            tag(way, key, value)
        return int(way.get("id"))

    def relation(self, members, attributes):
        relation = self.element("relation")
        for kind, ref, role in members:
            ET.SubElement(relation, "member", type=kind, ref=str(ref), role=role)
        for key, value in attributes.items():
            tag(relation, key, value)
        return int(relation.get("id"))

    def attach(self, lane, regulatory_id):
        refs = {m.get("ref") for m in lane["element"].findall("member")
                if m.get("role") == "regulatory_element"}
        if str(regulatory_id) not in refs:
            lane["element"].append(ET.Element("member", type="relation",
                                            ref=str(regulatory_id), role="regulatory_element"))

    def match_candidates(self, record, tolerance=0.8):
        x, y, z = record["position"]
        point = Point(x, y)
        f = record["forward"]
        candidates = []
        for index in self.index.query(point.buffer(tolerance)):
            lane = self.lanes[index]
            line = lane["center"]
            along = line.project(point)
            if abs(line.interpolate(along).z-z) > 2:
                continue
            a, b = line.interpolate(max(0, along-.5)), line.interpolate(min(line.length, along+.5))
            dx, dy = b.x-a.x, b.y-a.y
            alignment = (dx*f[0]+dy*f[1])/max(1e-9, math.hypot(dx, dy))
            distance = line.distance(point)
            if alignment > .95 and distance < tolerance:
                candidates.append((distance, lane))
        candidates.sort(key=lambda item: item[0])
        return candidates

    def match(self, record, tolerance=0.8):
        candidates = self.match_candidates(record, tolerance)
        if not candidates or (len(candidates) > 1 and candidates[1][0]-candidates[0][0] < .10):
            raise ValueError(f"Ambiguous/missing road match {record}: {[x[0] for x in candidates]}")
        return candidates[0][1], candidates[0][0]


def crosswalk_boundaries(points):
    """Split exact source outline at its two short end caps, retaining middle vertices."""
    polygon = Polygon(points)
    if not polygon.is_valid or polygon.area < 1:
        raise ValueError("Invalid CARLA crosswalk outline")
    box = list(polygon.minimum_rotated_rectangle.exterior.coords)[:4]
    a, b = max(zip(box, box[1:]+box[:1]), key=lambda pair: math.dist(*pair))
    direction = (b[0]-a[0], b[1]-a[1])
    scores = []
    for i, point in enumerate(points):
        nxt = points[(i+1) % len(points)]
        dx, dy = nxt[0]-point[0], nxt[1]-point[1]
        scores.append((abs(dx*direction[0]+dy*direction[1])/max(1e-9, math.hypot(dx, dy)), i))
    caps = sorted(i for _, i in sorted(scores)[:2])
    first, second = caps
    left = points[first+1:second+1]
    right = list(reversed(points[second+1:]+points[:first+1]))
    if len(left) < 2 or len(right) < 2:
        raise ValueError("Crosswalk cap split is degenerate")
    vx = (left[-1][0]+right[-1][0]-left[0][0]-right[0][0])/2
    vy = (left[-1][1]+right[-1][1]-left[0][1]-right[0][1])/2
    if vx*(left[0][1]-right[0][1])-vy*(left[0][0]-right[0][0]) < 0:
        left, right = right, left
    if Polygon(left+list(reversed(right))).symmetric_difference(polygon).area > 1e-6:
        raise ValueError("Crosswalk split changed source outline")
    return left, right


def augment(map_path, xodr, source):
    if source["xodr_sha256"] != digest(xodr.encode()):
        raise ValueError("OpenDRIVE differs from captured signal source")
    root = ET.parse(map_path).getroot()
    if any(tags(e).get("source") == SOURCE_TAG for e in root):
        raise ValueError("Input already augmented; regenerate from the saved original backup")
    if any(tags(e).get("type") == "regulatory_element" for e in root.findall("relation")):
        raise ValueError("Input already has regulations; do not merge without review")
    builder = MapBuilder(root)
    if len(builder.lanes) != 486:
        raise ValueError("Expected original Town05's 486 road lanelets")
    meta = root.find("MetaInfo")
    if meta is None:
        root.insert(0, ET.Element("MetaInfo", format_version="1", map_version=SOURCE_TAG))
    wmap = carla.Map("Town05", xodr)
    xroot = ET.fromstring(xodr)
    road_lengths = {int(r.get("id")): float(r.get("length")) for r in xroot.findall("road")}
    report = {"original_sha256": digest(Path(map_path).read_bytes()),
              "invalid_original_polygons_preserved": [r["id"] for r in builder.lanes if not r["polygon"].is_valid],
              "duplicate_boundary_points_removed": builder.deduplicated,
              "signals": [], "stop_signs": [], "crosswalks": [],
              "turn_directions": {}, "unmatched_connectors": []}
    manifest = {"schema": 1, "map_name": "Town05", "xodr_sha256": source["xodr_sha256"],
                "live_xodr_sha256": source["live_xodr_sha256"], "groups": []}
    for signal in source["signals"]:
        stop_lanes, candidates, errors = [], [], []
        forward = signal["stops"][0]["forward"]
        normal = [-forward[1], forward[0]]
        for stop in signal["stops"]:
            if sum(a*b for a, b in zip(stop["forward"], forward)) < .99:
                raise ValueError("One light spans differently oriented approaches")
            lane, error = builder.match(stop)
            stop_lanes.append(lane)
            errors.append(error)
            x, y, z = stop["position"]
            for side in (-1, 1):
                candidates.append([x+side*normal[0]*stop["width"]/2,
                                   y+side*normal[1]*stop["width"]/2, z])
            tag(lane["element"], "source:opendrive:road_id", stop["road_id"])
            tag(lane["element"], "source:opendrive:lane_id", stop["lane_id"])
        candidates.sort(key=lambda p: p[0]*normal[0]+p[1]*normal[1])
        stop_way = builder.way([candidates[0], candidates[-1]], {"type": "stop_line"})
        light_ways = []
        for box in signal["boxes"]:
            x, y, z = box["center"]
            # A horizontal baseline plus height uses the actual physical light-box extents.
            yaw = math.radians(box["yaw"])
            ex, ey, ez = box["extent"]
            half_width = abs(normal[0]*math.cos(yaw)+normal[1]*math.sin(yaw))*ex + \
                abs(-normal[0]*math.sin(yaw)+normal[1]*math.cos(yaw))*ey
            light_ways.append(builder.way(
                [[x-normal[0]*half_width, y-normal[1]*half_width, z-ez],
                 [x+normal[0]*half_width, y+normal[1]*half_width, z-ez]],
                {"type": "traffic_light", "subtype": "red_yellow_green", "height": 2*ez,
                 "source:opendrive:signal_id": signal["opendrive_id"]}))
        reg = builder.relation([("way", stop_way, "ref_line")]+
                               [("way", wid, "refers") for wid in light_ways],
                               {"type": "regulatory_element", "subtype": "traffic_light",
                                "source:opendrive:signal_id": signal["opendrive_id"]})
        linked = {lane["id"]: lane for lane in stop_lanes}
        for affected in signal["affected"]:
            # Match at the connector midpoint; shared junction entry points are ambiguous.
            wp = wmap.get_waypoint_xodr(affected["road_id"], affected["lane_id"],
                                       road_lengths[affected["road_id"]]/2)
            if wp is None:
                raise ValueError("Affected connector is absent from OpenDRIVE")
            try:
                lane, _ = builder.match(waypoint_record(wp))
                linked[lane["id"]] = lane
            except ValueError:
                report["unmatched_connectors"].append({"signal": signal["opendrive_id"],
                                                       "road": wp.road_id, "lane": wp.lane_id})
        for lane in linked.values():
            builder.attach(lane, reg)
        manifest["groups"].append({"opendrive_id": signal["opendrive_id"],
                                   "group_id": reg, "stop_line_id": stop_way,
                                   "approach_lanelet_ids": [l["id"] for l in stop_lanes],
                                   "lanelet_ids": sorted(linked)})
        report["signals"].append({"opendrive_id": signal["opendrive_id"],
                                  "group_id": reg, "max_center_error_m": max(errors)})
    if len(source.get("stop_signs", [])) != len(xroot.findall('.//signal[@type="206"]')):
        raise ValueError("Recapture source: native OpenDRIVE stop signs are missing")
    for sign in source["stop_signs"]:
        normal = [-sign["stops"][0]["forward"][1], sign["stops"][0]["forward"][0]]
        x, y, z = sign["position"]
        half = sign["width"]/2
        sign_way = builder.way([[x-half*normal[0], y-half*normal[1], z],
                                [x+half*normal[0], y+half*normal[1], z]],
                               {"type": "traffic_sign", "subtype": "stop_sign", "height": sign["height"],
                                "source:opendrive:signal_id": sign["opendrive_id"]})
        for stop in sign["stops"]:
            # A trigger can sit just inside a connector; midpoint disambiguates
            # overlapping turn branches while its actual stopping plane stays unchanged.
            mid = wmap.get_waypoint_xodr(stop["road_id"], stop["lane_id"], road_lengths[stop["road_id"]]/2)
            lane, _ = builder.match(waypoint_record(mid))
            x, y, z = stop["position"]
            n = [-stop["forward"][1], stop["forward"][0]]
            half = stop["width"]/2
            line = builder.way([[x-half*n[0], y-half*n[1], z], [x+half*n[0], y+half*n[1], z]],
                               {"type": "stop_line", "source:geometry": "physical_stop_trigger_projection"})
            reg = builder.relation([("way", sign_way, "refers"), ("way", line, "ref_line")],
                                   {"type": "regulatory_element", "subtype": "traffic_sign",
                                    "source:opendrive:signal_id": sign["opendrive_id"]})
            builder.attach(lane, reg)
            report["stop_signs"].append({"opendrive_id": sign["opendrive_id"], "regulatory_id": reg,
                                         "stop_line_id": line, "lanelet_id": lane["id"],
                                         "trigger_projection_error_m": stop["lateral_projection_error_m"]})
    # Use explicit OpenDRIVE turnRelation rather than guessing from traffic-light orientation.
    for road in xroot.findall("road"):
        for ref in road.findall("./signals/signalReference"):
            vector = ref.find("./userData/vectorSignal")
            turn = vector.get("turnRelation", "").lower() if vector is not None else ""
            if turn not in ("left", "right", "straight"):
                continue
            for validity in ref.findall("validity"):
                a, b = int(validity.get("fromLane")), int(validity.get("toLane"))
                for lane_id in range(min(a, b), max(a, b)+1):
                    if not lane_id:
                        continue
                    wp = wmap.get_waypoint_xodr(int(road.get("id")), lane_id,
                                               float(road.get("length"))/2)
                    if wp is None:
                        continue
                    try:
                        lane, _ = builder.match(waypoint_record(wp))
                    except ValueError:
                        continue
                    existing = tags(lane["element"]).get("turn_direction")
                    if existing and existing != turn:
                        raise ValueError(f"Conflicting turn labels for lanelet {lane['id']}")
                    tag(lane["element"], "turn_direction", turn)
                    tag(lane["element"], "source:opendrive:road_id", wp.road_id)
                    tag(lane["element"], "source:opendrive:lane_id", wp.lane_id)
                    report["turn_directions"][str(lane["id"])] = turn
    outlines, current = [], []
    for point in wmap.get_crosswalks():
        position = xyz(point)
        if current and math.dist(position, current[0]) < 1e-5:
            outlines.append(current)
            current = []
        else:
            current.append(position)
    if current or len(outlines) != len(xroot.findall('.//object[@type="crosswalk"]')):
        raise ValueError("CARLA crosswalk outlines do not match the OpenDRIVE object count")
    outlines.sort(key=lambda ps: (round(Polygon(ps).centroid.x, 4), round(Polygon(ps).centroid.y, 4)))
    for index, points in enumerate(outlines):
        left, right = crosswalk_boundaries(points)
        polygon = Polygon(points)
        mean_z = sum(p[2] for p in points)/len(points)
        linked = [builder.lanes[i] for i in builder.index.query(polygon, predicate="intersects")
                  if abs(builder.lanes[i]["center"].interpolate(
                      builder.lanes[i]["center"].project(polygon.centroid)).z-mean_z) < 2 and
                  builder.lanes[i]["query_polygon"].intersection(polygon).area > .05]
        if not linked:
            raise ValueError(f"Crosswalk {index} intersects no road")
        left_way = builder.way(left, {"type": "virtual"})
        right_way = builder.way(right, {"type": "virtual"})
        polygon_way = builder.way(points+[points[0]], {"type": "crosswalk", "area": "yes"})
        crosswalk = builder.relation([("way", left_way, "left"), ("way", right_way, "right")],
                                     {"type": "lanelet", "subtype": "crosswalk", "location": "urban",
                                      "one_way": "no", "participant:pedestrian": "yes",
                                      "participant:vehicle": "no", "participant:bicycle": "no"})
        reg = builder.relation([("relation", crosswalk, "refers"),
                                ("way", polygon_way, "crosswalk_polygon")],
                               {"type": "regulatory_element", "subtype": "crosswalk"})
        for lane in linked:
            builder.attach(lane, reg)
        report["crosswalks"].append({"lanelet_id": crosswalk, "regulatory_id": reg,
                                    "linked_road_lanelets": sorted(l["id"] for l in linked),
                                    "area_m2": polygon.area})
    report["summary"] = {"traffic_light_groups": len(manifest["groups"]),
                          "signal_approach_lanes": sum(len(g["approach_lanelet_ids"]) for g in manifest["groups"]),
                          "crosswalks": len(report["crosswalks"]),
                          "turn_directions": dict(Counter(report["turn_directions"].values())),
                          "removed_duplicate_boundary_points": sum(x["removed"] for x in builder.deduplicated),
                          "unmatched_signal_connectors": len(report["unmatched_connectors"]),
                          "native_stop_signs": len(source["stop_signs"]),
                          "stop_sign_lane_regulations": len(report["stop_signs"]),
                          "unmapped_scene_stop_triggers": len(source.get("scene_stops", []))}
    # Keep the original road IDs/boundaries; only adjacent duplicate coordinates were removed.
    order = {"MetaInfo": 0, "node": 1, "way": 2, "relation": 3}
    root[:] = sorted(root, key=lambda e: (order.get(e.tag, 4), int(e.get("id", "0"))))
    ET.indent(root, space="  ")
    data = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    manifest["map_sha256"] = digest(data)
    report["map_sha256"] = manifest["map_sha256"]
    return data, manifest, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True, help="Original unaugmented Town05 OSM")
    parser.add_argument("--xodr", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True, help="Read/write reproducible CARLA snapshot")
    parser.add_argument("--capture", action="store_true", help="Capture read-only live signal geometry first")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--output", type=Path, required=True, help="Candidate OSM, must differ from input")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if args.map.resolve() == args.output.resolve():
        parser.error("Use a separate candidate output; validate and back up before installation")
    xodr = args.xodr.read_text()
    if args.capture:
        source = capture_source(args.host, args.port, xodr)
        args.source.write_text(json.dumps(source, indent=2)+"\n")
    else:
        source = json.loads(args.source.read_text())
    data, manifest, report = augment(args.map, xodr, source)
    args.output.write_bytes(data)
    args.manifest.write_text(json.dumps(manifest, indent=2)+"\n")
    args.report.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report["summary"], indent=2))


if __name__ == "__main__":
    main()
