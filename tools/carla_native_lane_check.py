#!/usr/bin/env python3
"""Conservative CARLA native Driving-lane occupancy query.

CARLA's non-projecting query can reject points inside an internal lane seam
because the nearest sampled waypoint also has a longitudinal offset. This
helper verifies each of the two actual native lane widths; it never widens the
road or projects an arbitrary off-road position onto a driving lane.

A direct native hit is returned unchanged (including its direction). The caller
must still enforce the expected travel direction for its scenario.
"""
from __future__ import annotations

import math
import carla


def _components(waypoint, position):
    transform = waypoint.transform
    delta = position-transform.location
    right, forward = transform.get_right_vector(), transform.get_forward_vector()
    return (delta.x*right.x+delta.y*right.y,
            delta.x*forward.x+delta.y*forward.y, delta.z)


def _identity(waypoint):
    return waypoint.road_id, waypoint.section_id, waypoint.lane_id


def _record(waypoint, position):
    across, along, height = _components(waypoint, position)
    return {'road_id':waypoint.road_id,'section_id':waypoint.section_id,
            'lane_id':waypoint.lane_id,'s':waypoint.s,
            'half_width_m':waypoint.lane_width/2,'across_m':across,
            'along_m':along,'height_above_center_m':height}


def query_driving_lane(native_map, position):
    """Return ``(waypoint, evidence)``; evidence is set only for verified seams.

    Fallback requires a same-road/section/direction adjacent Driving lane, no
    junction, a shared edge (within native float precision), <=0.5 m initial
    longitudinal offset and <0.5 m height error. After refining OpenDRIVE s,
    the position must be inside a lane's OWN width (+1e-6 m arithmetic tolerance)
    and within 0.05 m of their internal edge. Invalid/unknown cases return None.
    """
    direct = native_map.get_waypoint(position, project_to_road=False,
                                     lane_type=carla.LaneType.Driving)
    if direct is not None:
        return direct, None
    projected = native_map.get_waypoint(position, project_to_road=True,
                                        lane_type=carla.LaneType.Driving)
    if projected is None or projected.is_junction or projected.lane_id == 0 or \
            projected.lane_type != carla.LaneType.Driving:
        return None, None
    across, along, height = _components(projected, position)
    half_width = projected.lane_width/2
    if half_width <= 0 or abs(abs(across)-half_width) > .05 or \
            abs(along) > .5 or abs(height) >= .5:
        return None, None
    side = -1 if across < 0 else 1
    neighbor = projected.get_left_lane() if side < 0 else projected.get_right_lane()
    if neighbor is None or neighbor.is_junction or neighbor.lane_type != carla.LaneType.Driving or \
            neighbor.road_id != projected.road_id or neighbor.section_id != projected.section_id or \
            neighbor.lane_id*projected.lane_id <= 0 or abs(neighbor.lane_id-projected.lane_id) != 1:
        return None, None
    # Verify these are touching edges, not merely two road-like waypoints with
    # matching identifiers. CARLA stores transforms as float32; 1 mm here is
    # a shared-edge matching tolerance, NOT a lane-width expansion.
    a, b = projected.transform, neighbor.transform
    ar, br = a.get_right_vector(), b.get_right_vector()
    edge_a = a.location+carla.Location(x=side*ar.x*half_width,y=side*ar.y*half_width)
    edge_b = b.location-carla.Location(x=side*br.x*neighbor.lane_width/2,
                                      y=side*br.y*neighbor.lane_width/2)
    shared_edge_gap = math.hypot(edge_a.x-edge_b.x, edge_a.y-edge_b.y)
    if shared_edge_gap > .001 or abs(edge_a.z-edge_b.z) > .001:
        return None, None
    for candidate, shared_side in ((projected,side),(neighbor,-side)):
        before = _record(candidate, position)
        refined = native_map.get_waypoint_xodr(candidate.road_id,candidate.lane_id,
            candidate.s+before['along_m']*(-1 if candidate.lane_id > 0 else 1))
        if refined is None or refined.is_junction or _identity(refined) != _identity(candidate) or \
                refined.lane_type != carla.LaneType.Driving:
            continue
        after = _record(refined, position)
        lateral, width = after['across_m'], after['half_width_m']
        if width <= 0 or lateral*shared_side < 0 or not width-.05 <= abs(lateral) <= width+1e-6 or \
                abs(after['along_m']) > .05 or abs(after['height_above_center_m']) >= .5:
            continue
        return refined, {'reason':'verified_native_internal_lane_seam',
            'xyz':[position.x,position.y,position.z],
            'projected':_record(projected,position),'neighbor':_record(neighbor,position),
            'selected_before_refinement':before,'selected':after,
            'shared_edge_gap_m':shared_edge_gap,'lane_width_tolerance_m':1e-6}
    return None, None
