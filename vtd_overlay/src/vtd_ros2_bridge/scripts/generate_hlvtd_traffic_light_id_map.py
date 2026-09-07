#!/usr/bin/env python3
"""Map 9910 approach IDs to existing Lanelet2 traffic-light groups.

The Host table is keyed by OpenDRIVE road and lane sign, not individual lamp
IDs. Match that approach's exit plane, drivable lane widths and travel heading
against each regulated stop line. Never add map rules or choose a nearest
unrelated signal when the association is missing or ambiguous.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

from generate_traffic_light_id_map import (
    angle_difference,
    commonroad_lane_context,
    traffic_light_group_geometry,
)


def polynomial(record: ET.Element, ds: float) -> float:
    return sum(float(record.get(key, "0")) * ds**i for i, key in enumerate("abcd"))


def lane_intervals(road: ET.Element, s: float, sign: int) -> list[tuple[float, float]]:
    """Drivable t intervals at the approach exit, including laneOffset."""
    if road.get("rule", "RHT") != "RHT":
        raise ValueError("This lane-sign contract is only validated for the RHT map")
    offsets = [r for r in road.findall("./lanes/laneOffset") if float(r.get("s")) <= s]
    offset = max(offsets, key=lambda r: float(r.get("s"))) if offsets else None
    t = polynomial(offset, s - float(offset.get("s"))) if offset is not None else 0.0
    sections = [r for r in road.findall("./lanes/laneSection") if float(r.get("s")) <= s]
    if not sections:
        return []
    section = max(sections, key=lambda r: float(r.get("s")))
    ds = s - float(section.get("s"))
    side = "left" if sign > 0 else "right"
    intervals = []
    for lane in sorted(section.findall(f"./{side}/lane"), key=lambda r: abs(int(r.get("id")))):
        records = [r for r in lane.findall("width") if float(r.get("sOffset")) <= ds]
        if not records:
            raise ValueError(f"Missing lane width on road {road.get('id')} lane {lane.get('id')}")
        record = max(records, key=lambda r: float(r.get("sOffset")))
        width = polynomial(record, ds - float(record.get("sOffset")))
        if not math.isfinite(width) or width < -1.0e-6:
            raise ValueError(f"Invalid lane width on road {road.get('id')}")
        end = t + sign * max(width, 0.0)
        if width > 1.0e-3 and lane.get("type") in {"driving", "entry", "exit", "onRamp", "offRamp"}:
            intervals.append((min(t, end), max(t, end)))
        t = end
    return intervals


def approach_geometry(opendrive: Path, contract: dict) -> tuple[list[dict], list[dict]]:
    from crdesigner.map_conversion.opendrive.odr2cr.opendrive_parser.parser import parse_opendrive

    raw = ET.parse(opendrive).getroot()
    roads = {int(r.get("id")): r for r in raw.findall("road")}
    parsed = {r.id: r for r in parse_opendrive(opendrive).roads}
    expected_fields = ["road_id", "lane_sign", "hlvtd_id", "green_mode"]
    if contract.get("fields") != expected_fields:
        raise ValueError("Unexpected Host approach table schema")
    seen_ids, seen_roads = set(), set()
    valid, rejected = [], []
    for row in contract["entries"]:
        if len(row) != 4 or any(type(v) is not int for v in row):
            raise ValueError("Host approach rows must contain four integers")
        road_id, sign, api_id, green_mode = row
        if sign not in {-1, 1} or api_id <= 0 or green_mode not in {3, 4, 5}:
            raise ValueError(f"Invalid Host approach: {row}")
        if api_id in seen_ids or (road_id, sign) in seen_roads:
            raise ValueError(f"Ambiguous Host approach: {row}")
        seen_ids.add(api_id)
        seen_roads.add((road_id, sign))
        info = dict(zip(expected_fields, row))
        road = roads.get(road_id)
        if road is None:
            rejected.append({**info, "reason": "road_missing_from_opendrive"})
            continue
        length = float(road.get("length"))
        if not math.isfinite(length) or length <= 1.0e-6:
            raise ValueError(f"Invalid approach road length: {road_id}")
        s = 0.0 if sign > 0 else length - 1.0e-7
        intervals = lane_intervals(road, s, sign)
        if not intervals:
            rejected.append({**info, "reason": "no_drivable_exit_lane"})
            continue
        plan = parsed[road_id].plan_view
        position, tangent, _, _ = plan.calc(s, compute_curvature=False)
        x, y, tangent = float(position[0]), float(position[1]), float(tangent)
        if not all(math.isfinite(v) for v in (x, y, tangent)):
            raise ValueError(f"Invalid approach geometry: {road_id}")
        heading = tangent + (math.pi if sign > 0 else 0.0)
        valid.append({**info, "x": x, "y": y, "tangent": tangent,
                      "heading": math.atan2(math.sin(heading), math.cos(heading)),
                      "lane_t_intervals": intervals})
    return valid, rejected


def generate(opendrive: Path, osm: Path, commonroad: Path, contract_path: Path, longitudinal: float,
             lateral: float, heading_tolerance: float) -> tuple[list[tuple[int, int]], dict]:
    if any(not math.isfinite(v) or v < 0 for v in (longitudinal, lateral, heading_tolerance)):
        raise ValueError("Mapping tolerances must be finite and nonnegative")
    contract = json.loads(contract_path.read_text())
    approaches, rejected = approach_geometry(opendrive, contract)
    # Some OSM boundary ways run opposite to the lane's travel direction.
    # Use the same source-lane headings as the existing verified map generator.
    _, lane_headings = commonroad_lane_context(commonroad)
    positions, lanes, group_ids = traffic_light_group_geometry(ET.parse(osm).getroot(), lane_headings)
    matches, excluded = [], []
    for group_id in sorted(group_ids):
        if group_id not in positions or not lanes.get(group_id):
            excluded.append({"group_id": group_id, "reason": "no_stop_line_or_regulated_lane"})
            continue
        if any(source not in lane_headings for _, source, _ in lanes[group_id]):
            excluded.append({"group_id": group_id, "reason": "missing_source_lane_heading"})
            continue
        x, y = positions[group_id]
        candidates = []
        for a in approaches:
            dx, dy = x - a["x"], y - a["y"]
            along = dx * math.cos(a["tangent"]) + dy * math.sin(a["tangent"])
            t = -dx * math.sin(a["tangent"]) + dy * math.cos(a["tangent"])
            error = max(angle_difference(yaw, a["heading"]) for _, _, yaw in lanes[group_id])
            if abs(along) > longitudinal or error > heading_tolerance:
                continue
            if not any(lo - lateral <= t <= hi + lateral for lo, hi in a["lane_t_intervals"]):
                continue
            candidates.append({"hlvtd_id": a["hlvtd_id"], "road_id": a["road_id"],
                               "lane_sign": a["lane_sign"], "group_id": group_id,
                               "longitudinal_error_m": round(along, 6),
                               "lateral_t_m": round(t, 6), "heading_error_rad": round(error, 6),
                               "lanelet_ids": sorted({lane for lane, _, _ in lanes[group_id]})})
        if len(candidates) == 1:
            matches.append(candidates[0])
        else:
            excluded.append({"group_id": group_id,
                             "reason": "ambiguous_approach" if candidates else "no_matching_approach",
                             "candidate_ids": [c["hlvtd_id"] for c in candidates]})
    if not matches:
        raise ValueError("No valid 9910 traffic-light associations; refusing an empty map")
    pairs = sorted((m["hlvtd_id"], m["group_id"]) for m in matches)
    mapped_ids = {p[0] for p in pairs}
    for a in approaches:
        if a["hlvtd_id"] not in mapped_ids:
            rejected.append({key: a[key] for key in [*contract["fields"]]} |
                            {"reason": "no_unambiguous_regulated_stop_line"})
    audit = {
        "host_contract_sha256": hashlib.sha256(contract_path.read_bytes()).hexdigest(),
        "host_plugin_sha256": contract["source_plugin_sha256"],
        "opendrive_sha256": hashlib.sha256(opendrive.read_bytes()).hexdigest(),
        "lanelet2_sha256": hashlib.sha256(osm.read_bytes()).hexdigest(),
        "commonroad_sha256": hashlib.sha256(commonroad.read_bytes()).hexdigest(),
        "tolerances": {"longitudinal_m": longitudinal, "lateral_m": lateral,
                       "heading_rad": heading_tolerance},
        "host_approaches": len(contract["entries"]), "mapped_hlvtd_ids": len(mapped_ids),
        "mapped_groups": len(pairs), "existing_groups": len(group_ids),
        "excluded_reasons": dict(Counter(e["reason"] for e in excluded)),
        "unmapped_approaches": sorted(rejected, key=lambda a: a["hlvtd_id"]),
        "excluded_groups": excluded, "matches": matches,
    }
    return pairs, audit


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("opendrive", type=Path)
    parser.add_argument("osm", type=Path)
    parser.add_argument("host_approaches", type=Path)
    parser.add_argument("--commonroad", type=Path, required=True,
                        help="Source lane travel headings (OSM way orientation is not sufficient)")
    parser.add_argument("--longitudinal-tolerance", type=float, default=10.0)
    parser.add_argument("--lateral-tolerance", type=float, default=0.5)
    parser.add_argument("--heading-tolerance", type=float, default=0.35)
    parser.add_argument("--audit-report", type=Path)
    args = parser.parse_args()
    pairs, audit = generate(args.opendrive, args.osm, args.commonroad, args.host_approaches,
                            args.longitudinal_tolerance, args.lateral_tolerance,
                            args.heading_tolerance)
    print("# 9910 approach IDs; NOT individual RDB/OpenDRIVE lamp IDs.")
    print("# Matched by Host road/lane sign, exit stop-line position and travel heading.")
    print(f"# lanelet2_sha256={audit['lanelet2_sha256']}")
    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow(("hlvtd_id", "autoware_group_id"))
    writer.writerows(pairs)
    if args.audit_report is not None:
        args.audit_report.write_text(json.dumps(audit, indent=2, sort_keys=True) + "\n")
    print(json.dumps({k: audit[k] for k in ["host_approaches", "mapped_hlvtd_ids",
                                           "mapped_groups", "existing_groups", "excluded_reasons"]}),
          file=sys.stderr)


if __name__ == "__main__":
    main()
