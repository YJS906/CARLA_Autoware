#!/usr/bin/env python3
"""Map-match an ordered CSV route and set it through the Autoware AD API."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Callable, Iterable, Sequence


class RouteCsvError(ValueError):
    """Raised when a route CSV cannot be interpreted safely."""


class MapMatchError(RuntimeError):
    """Raised when CSV points cannot be matched to one connected road route."""


class RouteSetError(RuntimeError):
    """Raised when Autoware rejects or cannot receive the route."""


@dataclass(frozen=True)
class RoutePoint:
    seq: int
    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    lanelet_id: int | None = None


@dataclass(frozen=True)
class LaneletCandidate:
    """A CSV point projected onto one candidate road lanelet."""

    route_point: RoutePoint
    lanelet_id: int
    lanelet: Any
    centerline: tuple[tuple[float, float, float], ...]
    left_bound: tuple[tuple[float, float, float], ...]
    right_bound: tuple[tuple[float, float, float], ...]
    x: float
    y: float
    z: float
    yaw: float
    centerline_distance: float
    polygon_distance: float
    arc_length: float
    lanelet_length: float
    heading_error: float
    forced: bool = False


@dataclass(frozen=True)
class MatchedRoute:
    """Globally selected, topology-consistent lanelets and corrected points."""

    raw_points: tuple[RoutePoint, ...]
    candidate_sets: tuple[tuple[LaneletCandidate, ...], ...]
    selected: tuple[LaneletCandidate, ...]
    transition_paths: tuple[tuple[Any, ...], ...]
    route_lanelets: tuple[Any, ...]
    topology_ok: bool
    direction_ok: bool
    goal_ok: bool
    goal_left_clearance: float
    goal_right_clearance: float

    @property
    def route_lanelet_ids(self) -> tuple[int, ...]:
        return tuple(int(lanelet.id) for lanelet in self.route_lanelets)


def _parse_finite_float(value: str | None, field: str, line: int) -> float:
    if value is None or not value.strip():
        raise RouteCsvError(f"line {line}: '{field}' is empty")
    try:
        result = float(value)
    except ValueError as error:
        raise RouteCsvError(
            f"line {line}: '{field}' must be a number, got {value!r}"
        ) from error
    if not math.isfinite(result):
        raise RouteCsvError(f"line {line}: '{field}' must be finite, got {value!r}")
    return result


def _parse_seq(value: str | None, line: int) -> int:
    if value is None or not value.strip():
        raise RouteCsvError(f"line {line}: 'seq' is empty")
    try:
        result = int(value)
    except ValueError as error:
        raise RouteCsvError(
            f"line {line}: 'seq' must be an integer, got {value!r}"
        ) from error
    if result < 1:
        raise RouteCsvError(f"line {line}: 'seq' must be at least 1, got {result}")
    return result


def _parse_lanelet_id(value: str | None, line: int) -> int | None:
    if value is None or not value.strip():
        return None
    try:
        result = int(value)
    except ValueError as error:
        raise RouteCsvError(
            f"line {line}: 'lanelet_id' must be an integer, got {value!r}"
        ) from error
    if result <= 0:
        raise RouteCsvError(
            f"line {line}: 'lanelet_id' must be greater than zero, got {result}"
        )
    return result


def load_route_csv(path: str | Path) -> list[RoutePoint]:
    """Read and validate ``seq,x,y[,z,yaw,lanelet_id]`` route data."""
    csv_path = Path(path)
    try:
        route_file = csv_path.open("r", encoding="utf-8-sig", newline="")
    except OSError as error:
        raise RouteCsvError(f"cannot open {csv_path}: {error}") from error

    with route_file:
        reader = csv.DictReader(route_file)
        if reader.fieldnames is None:
            raise RouteCsvError("CSV header is missing")

        columns: dict[str, str] = {}
        for raw_name in reader.fieldnames:
            if raw_name is None:
                raise RouteCsvError("CSV header contains an empty column name")
            name = raw_name.strip().lower()
            if not name:
                raise RouteCsvError("CSV header contains an empty column name")
            if name in columns:
                raise RouteCsvError(f"CSV header contains duplicate column {name!r}")
            columns[name] = raw_name

        missing = sorted({"seq", "x", "y"} - columns.keys())
        if missing:
            raise RouteCsvError(
                "CSV header must contain seq,x,y; missing " + ", ".join(missing)
            )

        points: list[RoutePoint] = []
        seen_sequences: set[int] = set()
        for row in reader:
            if None in row:
                raise RouteCsvError(
                    f"line {reader.line_num}: row has more values than the CSV header"
                )
            if all(value is None or not value.strip() for value in row.values()):
                continue

            seq = _parse_seq(row[columns["seq"]], reader.line_num)
            if seq in seen_sequences:
                raise RouteCsvError(
                    f"line {reader.line_num}: duplicate sequence number {seq}"
                )
            seen_sequences.add(seq)

            x = _parse_finite_float(row[columns["x"]], "x", reader.line_num)
            y = _parse_finite_float(row[columns["y"]], "y", reader.line_num)
            z = 0.0
            if "z" in columns:
                raw_z = row[columns["z"]]
                if raw_z is not None and raw_z.strip():
                    z = _parse_finite_float(raw_z, "z", reader.line_num)

            yaw = None
            if "yaw" in columns:
                raw_yaw = row[columns["yaw"]]
                if raw_yaw is not None and raw_yaw.strip():
                    yaw = _parse_finite_float(raw_yaw, "yaw", reader.line_num)

            lanelet_id = None
            if "lanelet_id" in columns:
                lanelet_id = _parse_lanelet_id(
                    row[columns["lanelet_id"]], reader.line_num
                )

            points.append(
                RoutePoint(
                    seq=seq,
                    x=x,
                    y=y,
                    z=z,
                    yaw=yaw,
                    lanelet_id=lanelet_id,
                )
            )

    if len(points) < 2:
        raise RouteCsvError("route must contain at least start (seq 1) and goal (seq 2)")

    points.sort(key=lambda point: point.seq)
    expected_sequences = list(range(1, len(points) + 1))
    actual_sequences = [point.seq for point in points]
    if actual_sequences != expected_sequences:
        raise RouteCsvError(
            "'seq' values must be consecutive from 1; got "
            + ", ".join(str(seq) for seq in actual_sequences)
        )

    for previous, current in zip(points, points[1:]):
        if math.hypot(current.x - previous.x, current.y - previous.y) <= 1.0e-9:
            raise RouteCsvError(
                f"seq {previous.seq} and {current.seq} have the same x,y position"
            )

    return points


def point_yaw(points: Sequence[RoutePoint], index: int) -> float:
    """Return the CSV heading hint used while selecting a directed lanelet."""
    point = points[index]
    if point.yaw is not None:
        return point.yaw
    if index == 0:
        source, target = point, points[1]
    elif index == len(points) - 1:
        source, target = points[index - 1], point
    else:
        source, target = points[index - 1], points[index + 1]
        if math.hypot(target.x - source.x, target.y - source.y) <= 1.0e-9:
            source, target = points[index - 1], point
    return math.atan2(target.y - source.y, target.x - source.x)


def quaternion_z_w(yaw: float) -> tuple[float, float]:
    """Return the planar quaternion components for a yaw angle."""
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def route_summary(points: Sequence[RoutePoint]) -> str:
    checkpoint_count = max(0, len(points) - 2)
    start = points[0]
    goal = points[-1]
    return (
        f"start=seq {start.seq} ({start.x:.3f}, {start.y:.3f}), "
        f"checkpoints={checkpoint_count}, "
        f"goal=seq {goal.seq} ({goal.x:.3f}, {goal.y:.3f})"
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_difference(first: float, second: float) -> float:
    return abs(_normalize_angle(first - second))


def _attribute(lanelet: Any, name: str, default: str = "") -> str:
    if name not in lanelet.attributes:
        return default
    return str(lanelet.attributes[name])


def _xyz_points(line_string: Iterable[Any]) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        (float(point.x), float(point.y), float(getattr(point, "z", 0.0)))
        for point in line_string
    )


def _project_to_polyline(
    points: Sequence[tuple[float, float, float]], x: float, y: float
) -> tuple[float, float, float, float, float, float, float]:
    """Return distance, projected xyz, directed yaw, arc, and total length."""
    if len(points) < 2:
        raise MapMatchError("lanelet centerline has fewer than two points")

    total_length = sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    )
    best: tuple[float, float, float, float, float, float] | None = None
    traversed = 0.0
    for first, second in zip(points, points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        segment_length = math.hypot(dx, dy)
        if segment_length <= 1.0e-9:
            continue
        ratio = max(
            0.0,
            min(
                1.0,
                ((x - first[0]) * dx + (y - first[1]) * dy)
                / (segment_length * segment_length),
            ),
        )
        projected_x = first[0] + ratio * dx
        projected_y = first[1] + ratio * dy
        projected_z = first[2] + ratio * (second[2] - first[2])
        distance = math.hypot(x - projected_x, y - projected_y)
        candidate = (
            distance,
            projected_x,
            projected_y,
            projected_z,
            math.atan2(dy, dx),
            traversed + ratio * segment_length,
        )
        if best is None or candidate[0] < best[0]:
            best = candidate
        traversed += segment_length

    if best is None:
        raise MapMatchError("lanelet centerline contains only zero-length segments")
    return (*best, total_length)


def _distance_to_polyline(
    points: Sequence[tuple[float, float, float]], x: float, y: float
) -> float:
    return _project_to_polyline(points, x, y)[0]


def _lanelet_kind(lanelet: Any) -> str:
    turn_direction = _attribute(lanelet, "turn_direction").lower()
    if turn_direction and turn_direction != "straight":
        return "intersection"
    if _attribute(lanelet, "subtype").lower() == "intersection":
        return "intersection"
    return "road"


class LaneletMapMatcher:
    """Select a connected sequence of nearby directed road lanelets."""

    def __init__(
        self,
        map_path: str | Path,
        *,
        candidate_radius: float,
        candidate_count: int,
        direction_threshold: float,
        vehicle_width: float,
        goal_margin: float,
    ) -> None:
        try:
            import lanelet2
        except ImportError as error:
            raise MapMatchError(
                "lanelet2 Python bindings are unavailable; run through ./set_route "
                "or source the Autoware environment"
            ) from error

        self.lanelet2 = lanelet2
        self.map_path = Path(map_path)
        if not self.map_path.is_file():
            raise MapMatchError(f"lanelet map does not exist: {self.map_path}")
        self.candidate_radius = candidate_radius
        self.candidate_count = candidate_count
        self.direction_threshold = direction_threshold
        self.vehicle_width = vehicle_width
        self.goal_margin = goal_margin
        self._lanelet_length_cache: dict[int, float] = {}
        self._path_cache: dict[tuple[int, int], tuple[Any, ...] | None] = {}

        try:
            projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(0, 0))
            self.map = lanelet2.io.load(str(self.map_path), projector)
        except Exception as error:
            raise MapMatchError(
                f"failed to load lanelet map {self.map_path}: {error}"
            ) from error

        local_point_count = 0
        for point in self.map.pointLayer:
            if "local_x" in point.attributes and "local_y" in point.attributes:
                point.x = float(point.attributes["local_x"])
                point.y = float(point.attributes["local_y"])
                local_point_count += 1
        if local_point_count == 0:
            raise MapMatchError(
                "the map has no local_x/local_y tags; this CSV matcher currently "
                "expects an Autoware Local-projector lanelet map"
            )

        try:
            traffic_rules = lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                lanelet2.traffic_rules.Participants.Vehicle,
            )
            self.routing_graph = lanelet2.routing.RoutingGraph(
                self.map, traffic_rules
            )
        except Exception as error:
            raise MapMatchError(
                f"failed to build lanelet routing graph: {error}"
            ) from error

    def _is_road(self, lanelet: Any) -> bool:
        return _attribute(lanelet, "subtype").lower() in {
            "road",
            "road_shoulder",
        }

    def _lanelet_by_id(self, lanelet_id: int) -> Any:
        try:
            lanelet = self.map.laneletLayer[lanelet_id]
        except Exception as error:
            raise MapMatchError(f"lanelet {lanelet_id} does not exist in the map") from error
        if not self._is_road(lanelet):
            raise MapMatchError(f"lanelet {lanelet_id} is not a drivable road lanelet")
        return lanelet

    def _candidate(
        self,
        point: RoutePoint,
        lanelet: Any,
        polygon_distance: float,
        heading_hint: float,
        *,
        forced: bool,
    ) -> LaneletCandidate:
        centerline = _xyz_points(lanelet.centerline)
        left_bound = _xyz_points(lanelet.leftBound)
        right_bound = _xyz_points(lanelet.rightBound)
        (
            centerline_distance,
            x,
            y,
            z,
            yaw,
            arc_length,
            lanelet_length,
        ) = _project_to_polyline(centerline, point.x, point.y)
        return LaneletCandidate(
            route_point=point,
            lanelet_id=int(lanelet.id),
            lanelet=lanelet,
            centerline=centerline,
            left_bound=left_bound,
            right_bound=right_bound,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            centerline_distance=centerline_distance,
            polygon_distance=float(polygon_distance),
            arc_length=arc_length,
            lanelet_length=lanelet_length,
            heading_error=_angle_difference(yaw, heading_hint),
            forced=forced,
        )

    def candidates_for_point(
        self,
        point: RoutePoint,
        heading_hint: float,
        forced_lanelet_id: int | None = None,
    ) -> tuple[LaneletCandidate, ...]:
        if forced_lanelet_id is not None:
            lanelet = self._lanelet_by_id(forced_lanelet_id)
            candidate = self._candidate(
                point,
                lanelet,
                0.0,
                heading_hint,
                forced=True,
            )
            return (candidate,)

        query = self.lanelet2.core.BasicPoint2d(point.x, point.y)
        search_count = max(50, self.candidate_count * 8)
        nearest = self.lanelet2.geometry.findNearest(
            self.map.laneletLayer, query, search_count
        )
        candidates: list[LaneletCandidate] = []
        for polygon_distance, lanelet in nearest:
            if not self._is_road(lanelet):
                continue
            candidate = self._candidate(
                point,
                lanelet,
                float(polygon_distance),
                heading_hint,
                forced=False,
            )
            if candidate.centerline_distance <= self.candidate_radius:
                candidates.append(candidate)

        candidates.sort(key=self._base_candidate_key)
        if not candidates:
            raise MapMatchError(
                f"seq {point.seq}: no road lanelet centerline found within "
                f"{self.candidate_radius:.1f} m of ({point.x:.3f}, {point.y:.3f})"
            )
        return tuple(candidates[: self.candidate_count])

    @staticmethod
    def _base_candidate_key(candidate: LaneletCandidate) -> tuple[float, int]:
        reverse_penalty = max(
            0.0, candidate.heading_error - math.pi / 2.0
        ) * 20.0
        score = (
            candidate.centerline_distance * 2.0
            + candidate.heading_error * 4.0
            + reverse_penalty
        )
        return score, candidate.lanelet_id

    def _shortest_path(self, first: Any, second: Any) -> tuple[Any, ...] | None:
        key = (int(first.id), int(second.id))
        if key not in self._path_cache:
            try:
                path = self.routing_graph.shortestPath(first, second)
            except Exception:
                path = None
            self._path_cache[key] = tuple(path) if path else None
        return self._path_cache[key]

    def _lanelet_length(self, lanelet: Any) -> float:
        lanelet_id = int(lanelet.id)
        if lanelet_id not in self._lanelet_length_cache:
            centerline = _xyz_points(lanelet.centerline)
            self._lanelet_length_cache[lanelet_id] = _project_to_polyline(
                centerline, centerline[0][0], centerline[0][1]
            )[-1]
        return self._lanelet_length_cache[lanelet_id]

    def _transition(
        self,
        previous: LaneletCandidate,
        current: LaneletCandidate,
    ) -> tuple[float, tuple[Any, ...]] | None:
        if previous.lanelet_id == current.lanelet_id:
            if current.arc_length + 1.0 < previous.arc_length:
                return None
            path = (previous.lanelet,)
            path_distance = max(0.0, current.arc_length - previous.arc_length)
        else:
            path = self._shortest_path(previous.lanelet, current.lanelet)
            if not path:
                return None
            path_distance = max(
                0.0, previous.lanelet_length - previous.arc_length
            )
            path_distance += sum(
                self._lanelet_length(lanelet) for lanelet in path[1:-1]
            )
            path_distance += current.arc_length

        raw_distance = math.hypot(
            current.route_point.x - previous.route_point.x,
            current.route_point.y - previous.route_point.y,
        )
        excessive_detour = max(
            0.0, path_distance - raw_distance * 2.5 - 20.0
        )
        if excessive_detour > 200.0:
            return None
        distance_mismatch = abs(path_distance - raw_distance)
        transition_cost = (
            distance_mismatch * 0.025
            + excessive_detour * 0.20
            + max(0, len(path) - 1) * 0.02
        )
        return transition_cost, path

    def match(
        self,
        points: Sequence[RoutePoint],
        overrides: dict[int, int] | None = None,
    ) -> MatchedRoute:
        overrides = overrides or {}
        candidate_sets = tuple(
            self.candidates_for_point(
                point,
                point_yaw(points, index),
                overrides.get(point.seq, point.lanelet_id),
            )
            for index, point in enumerate(points)
        )

        costs: list[list[float]] = [
            [self._base_candidate_key(candidate)[0] for candidate in candidate_sets[0]]
        ]
        predecessors: list[list[tuple[int, tuple[Any, ...]] | None]] = [
            [None for _ in candidate_sets[0]]
        ]

        for index in range(1, len(candidate_sets)):
            row_costs = [math.inf for _ in candidate_sets[index]]
            row_predecessors: list[tuple[int, tuple[Any, ...]] | None] = [
                None for _ in candidate_sets[index]
            ]
            for current_index, current in enumerate(candidate_sets[index]):
                base_cost = self._base_candidate_key(current)[0]
                for previous_index, previous in enumerate(candidate_sets[index - 1]):
                    if not math.isfinite(costs[index - 1][previous_index]):
                        continue
                    transition = self._transition(previous, current)
                    if transition is None:
                        continue
                    transition_cost, path = transition
                    total_cost = (
                        costs[index - 1][previous_index]
                        + base_cost
                        + transition_cost
                    )
                    if total_cost < row_costs[current_index]:
                        row_costs[current_index] = total_cost
                        row_predecessors[current_index] = previous_index, path
            if not any(math.isfinite(cost) for cost in row_costs):
                previous_ids = ", ".join(
                    str(candidate.lanelet_id)
                    for candidate in candidate_sets[index - 1]
                )
                current_ids = ", ".join(
                    str(candidate.lanelet_id)
                    for candidate in candidate_sets[index]
                )
                raise MapMatchError(
                    f"seq {points[index - 1].seq}->{points[index].seq}: no forward "
                    f"topology path between candidates [{previous_ids}] and "
                    f"[{current_ids}]"
                )
            costs.append(row_costs)
            predecessors.append(row_predecessors)

        selected_indices = [0 for _ in candidate_sets]
        selected_indices[-1] = min(
            range(len(costs[-1])),
            key=lambda candidate_index: costs[-1][candidate_index],
        )
        transition_paths: list[tuple[Any, ...]] = [() for _ in points[1:]]
        for index in range(len(candidate_sets) - 1, 0, -1):
            predecessor = predecessors[index][selected_indices[index]]
            if predecessor is None:
                raise MapMatchError(
                    "internal error while reconstructing lanelet route"
                )
            previous_index, path = predecessor
            transition_paths[index - 1] = path
            selected_indices[index - 1] = previous_index

        selected = tuple(
            candidate_sets[index][candidate_index]
            for index, candidate_index in enumerate(selected_indices)
        )
        route_lanelets: list[Any] = [selected[0].lanelet]
        for path in transition_paths:
            for lanelet in path:
                if int(route_lanelets[-1].id) != int(lanelet.id):
                    route_lanelets.append(lanelet)

        direction_ok = all(
            candidate.heading_error <= self.direction_threshold
            for candidate in selected
        )
        goal = selected[-1]
        goal_point = self.lanelet2.core.BasicPoint2d(goal.x, goal.y)
        try:
            goal_center_inside = bool(
                self.lanelet2.geometry.inside(goal.lanelet, goal_point)
            )
        except Exception:
            goal_center_inside = goal.polygon_distance <= 1.0e-6
        left_clearance = _distance_to_polyline(goal.left_bound, goal.x, goal.y)
        right_clearance = _distance_to_polyline(goal.right_bound, goal.x, goal.y)
        required_clearance = self.vehicle_width * 0.5 + self.goal_margin
        goal_ok = (
            goal_center_inside
            and left_clearance >= required_clearance
            and right_clearance >= required_clearance
        )

        return MatchedRoute(
            raw_points=tuple(points),
            candidate_sets=candidate_sets,
            selected=selected,
            transition_paths=tuple(transition_paths),
            route_lanelets=tuple(route_lanelets),
            topology_ok=True,
            direction_ok=direction_ok,
            goal_ok=goal_ok,
            goal_left_clearance=left_clearance,
            goal_right_clearance=right_clearance,
        )

    def lanelet_from_click(
        self, x: float, y: float, maximum_distance: float
    ) -> tuple[int, float]:
        query = self.lanelet2.core.BasicPoint2d(x, y)
        nearest = self.lanelet2.geometry.findNearest(
            self.map.laneletLayer, query, 50
        )
        choices: list[tuple[float, int]] = []
        for _, lanelet in nearest:
            if not self._is_road(lanelet):
                continue
            centerline = _xyz_points(lanelet.centerline)
            distance = _project_to_polyline(centerline, x, y)[0]
            choices.append((distance, int(lanelet.id)))
        if not choices:
            raise MapMatchError("the clicked point has no nearby road lanelet")
        distance, lanelet_id = min(choices)
        if distance > maximum_distance:
            raise MapMatchError(
                f"clicked point is {distance:.2f} m from lanelet {lanelet_id}; "
                f"maximum is {maximum_distance:.2f} m"
            )
        return lanelet_id, distance


def resolve_map_path(explicit_path: str | None) -> Path:
    if explicit_path:
        return Path(explicit_path).expanduser().resolve()

    configured = os.environ.get("AUTOWARE_LANELET2_MAP")
    if configured:
        return Path(configured).expanduser().resolve()

    relative = os.environ.get(
        "VTD_MAP_RELATIVE_PATH", "HL_FMA_VTD_LivingLab_topology_fixed"
    )
    roots = [
        os.environ.get("VTD_MAP_DIR"),
        "/home/aw/vtd_autoware_maps",
        "/home/a/vtd_autoware_maps",
    ]
    checked: list[Path] = []
    for raw_root in roots:
        if not raw_root:
            continue
        candidate = Path(raw_root) / relative / "lanelet2_map.osm"
        checked.append(candidate)
        if candidate.is_file():
            return candidate.resolve()
    raise MapMatchError(
        "cannot find lanelet2_map.osm; use --map or set AUTOWARE_LANELET2_MAP. "
        "Checked: " + ", ".join(str(path) for path in checked)
    )


def print_match_report(result: MatchedRoute) -> None:
    checkpoint_count = max(0, len(result.raw_points) - 2)
    print(
        f"[CSV] {len(result.raw_points)} route points loaded "
        f"({checkpoint_count} intermediate checkpoints)",
        flush=True,
    )
    print("\n[MAP MATCH]", flush=True)
    for candidate in result.selected:
        print(
            f"seq {candidate.route_point.seq:<3}: "
            f"{_lanelet_kind(candidate.lanelet)} {candidate.lanelet_id} "
            f"(offset {candidate.centerline_distance:.2f} m, "
            f"yaw {math.degrees(candidate.yaw):.1f} deg)",
            flush=True,
        )

    print("\n[ROUTE]", flush=True)
    print(" -> ".join(str(value) for value in result.route_lanelet_ids), flush=True)

    raw_goal = result.raw_points[-1]
    corrected_goal = result.selected[-1]
    print("\n[GOAL]", flush=True)
    print(f"raw       : {raw_goal.x:.3f} {raw_goal.y:.3f}", flush=True)
    print(
        f"corrected : {corrected_goal.x:.3f} {corrected_goal.y:.3f} "
        f"(lanelet {corrected_goal.lanelet_id})",
        flush=True,
    )

    maximum_heading_error = max(
        math.degrees(candidate.heading_error) for candidate in result.selected
    )
    goal_clearance = min(
        result.goal_left_clearance, result.goal_right_clearance
    )
    print("\n[VALIDATION]", flush=True)
    print(f"topology     {'OK' if result.topology_ok else 'FAILED'}", flush=True)
    print(
        f"direction    {'OK' if result.direction_ok else 'FAILED'} "
        f"(max error {maximum_heading_error:.1f} deg)",
        flush=True,
    )
    print(
        f"goal         {'OK' if result.goal_ok else 'FAILED'} "
        f"(minimum side clearance {goal_clearance:.2f} m)",
        flush=True,
    )


def save_corrected_csv(
    result: MatchedRoute, path: str | Path, *, overwrite: bool
) -> None:
    output_path = Path(path)
    if output_path.exists() and not overwrite:
        raise RouteCsvError(
            f"refusing to overwrite existing corrected route: {output_path}; "
            "use --force-output"
        )
    try:
        output_file = output_path.open("w", encoding="utf-8", newline="")
    except OSError as error:
        raise RouteCsvError(f"cannot write {output_path}: {error}") from error
    with output_file:
        writer = csv.writer(output_file)
        writer.writerow(
            ["seq", "x", "y", "z", "yaw", "lanelet_id", "raw_x", "raw_y"]
        )
        for raw, corrected in zip(result.raw_points, result.selected):
            writer.writerow(
                [
                    raw.seq,
                    f"{corrected.x:.9f}",
                    f"{corrected.y:.9f}",
                    f"{corrected.z:.9f}",
                    f"{corrected.yaw:.12f}",
                    corrected.lanelet_id,
                    f"{raw.x:.9f}",
                    f"{raw.y:.9f}",
                ]
            )
    print(f"Corrected route saved: {output_path}", flush=True)


class RoutePreviewPublisher:
    """Publish the four requested CSV route debug MarkerArray topics."""

    RAW_TOPIC = "/debug/csv/raw_checkpoints"
    CANDIDATE_TOPIC = "/debug/csv/candidate_lanelets"
    SELECTED_TOPIC = "/debug/csv/selected_lanelets"
    CORRECTED_TOPIC = "/debug/csv/corrected_checkpoints"

    def __init__(
        self,
        node: Any,
        frame_id: str,
        *,
        state_path: str | None = None,
        state_metadata: dict[str, Any] | None = None,
    ) -> None:
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from visualization_msgs.msg import MarkerArray

        self.node = node
        self.frame_id = frame_id
        self.state_path = state_path
        self.state_metadata = state_metadata or {}
        self._result: MatchedRoute | None = None
        self._raw_points: Sequence[RoutePoint] = ()
        self._keepalive_timer = None
        # In managed mode only the bridge/standalone preview worker publishes these topics.
        # Two independent DELETEALL/ADD writers would otherwise fight each other.
        self.publishers = {}
        if state_path is not None:
            return
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publishers = {
            self.RAW_TOPIC: node.create_publisher(
                MarkerArray, self.RAW_TOPIC, qos
            ),
            self.CANDIDATE_TOPIC: node.create_publisher(
                MarkerArray, self.CANDIDATE_TOPIC, qos
            ),
            self.SELECTED_TOPIC: node.create_publisher(
                MarkerArray, self.SELECTED_TOPIC, qos
            ),
            self.CORRECTED_TOPIC: node.create_publisher(
                MarkerArray, self.CORRECTED_TOPIC, qos
            ),
        }

    def _marker(self, namespace: str, marker_id: int, marker_type: int) -> Any:
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        return marker

    @staticmethod
    def _point(x: float, y: float, z: float) -> Any:
        from geometry_msgs.msg import Point

        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def _delete_all(self) -> Any:
        from visualization_msgs.msg import Marker

        marker = self._marker("csv_route", 0, Marker.POINTS)
        marker.action = Marker.DELETEALL
        return marker

    def _label(
        self,
        namespace: str,
        marker_id: int,
        text: str,
        x: float,
        y: float,
        z: float,
        color: tuple[float, float, float],
    ) -> Any:
        from visualization_msgs.msg import Marker

        marker = self._marker(namespace, marker_id, Marker.TEXT_VIEW_FACING)
        marker.text = text
        marker.pose.position = self._point(x, y, z)
        marker.scale.z = 0.9
        marker.color.r, marker.color.g, marker.color.b = color
        return marker

    def _raw_markers(self, points: Sequence[RoutePoint]) -> Any:
        from visualization_msgs.msg import Marker, MarkerArray

        array = MarkerArray()
        array.markers.append(self._delete_all())
        marker_id = 1
        for point in points:
            marker = self._marker("raw_x", marker_id, Marker.LINE_LIST)
            marker.scale.x = 0.20
            marker.color.r = 1.0
            marker.color.g = 0.05
            marker.color.b = 0.05
            radius = 0.8
            z = point.z + 0.25
            marker.points.extend(
                [
                    self._point(point.x - radius, point.y - radius, z),
                    self._point(point.x + radius, point.y + radius, z),
                    self._point(point.x - radius, point.y + radius, z),
                    self._point(point.x + radius, point.y - radius, z),
                ]
            )
            array.markers.append(marker)
            marker_id += 1
            array.markers.append(
                self._label(
                    "raw_label",
                    marker_id,
                    f"raw seq {point.seq}",
                    point.x,
                    point.y,
                    z + 0.8,
                    (1.0, 0.1, 0.1),
                )
            )
            marker_id += 1
        return array

    def _candidate_markers(self, result: MatchedRoute) -> Any:
        from visualization_msgs.msg import Marker, MarkerArray

        array = MarkerArray()
        array.markers.append(self._delete_all())
        marker_id = 1
        for point, candidates in zip(result.raw_points, result.candidate_sets):
            for rank, candidate in enumerate(candidates):
                marker = self._marker(
                    f"candidate_seq_{point.seq}", marker_id, Marker.LINE_STRIP
                )
                marker.scale.x = 0.09 if rank else 0.14
                marker.color.r = 1.0
                marker.color.g = 0.85
                marker.color.b = 0.0
                marker.color.a = 0.35 if rank else 0.75
                polygon = list(candidate.left_bound)
                polygon.extend(reversed(candidate.right_bound))
                if polygon:
                    polygon.append(polygon[0])
                marker.points.extend(
                    self._point(x, y, z + 0.10) for x, y, z in polygon
                )
                array.markers.append(marker)
                marker_id += 1
            array.markers.append(
                self._label(
                    "candidate_label",
                    marker_id,
                    f"seq {point.seq}: candidates "
                    + ",".join(str(value.lanelet_id) for value in candidates),
                    point.x,
                    point.y,
                    point.z + 1.8,
                    (1.0, 0.85, 0.0),
                )
            )
            marker_id += 1
        return array

    def _selected_markers(self, result: MatchedRoute) -> Any:
        from visualization_msgs.msg import Marker, MarkerArray

        array = MarkerArray()
        array.markers.append(self._delete_all())
        marker_id = 1
        for lanelet in result.route_lanelets:
            marker = self._marker("selected_route", marker_id, Marker.LINE_STRIP)
            marker.scale.x = 0.35
            marker.color.r = 0.05
            marker.color.g = 1.0
            marker.color.b = 0.10
            marker.color.a = 0.95
            marker.points.extend(
                self._point(x, y, z + 0.22)
                for x, y, z in _xyz_points(lanelet.centerline)
            )
            array.markers.append(marker)
            marker_id += 1
        for selected in result.selected:
            array.markers.append(
                self._label(
                    "selected_label",
                    marker_id,
                    f"seq {selected.route_point.seq} -> lanelet "
                    f"{selected.lanelet_id}",
                    selected.x,
                    selected.y,
                    selected.z + 1.0,
                    (0.05, 1.0, 0.10),
                )
            )
            marker_id += 1
        return array

    def _corrected_markers(self, result: MatchedRoute) -> Any:
        from visualization_msgs.msg import Marker, MarkerArray

        array = MarkerArray()
        array.markers.append(self._delete_all())
        marker_id = 1
        if len(result.selected) > 2:
            waypoints = self._marker(
                "corrected_waypoints", marker_id, Marker.SPHERE_LIST
            )
            waypoints.scale.x = 0.85
            waypoints.scale.y = 0.85
            waypoints.scale.z = 0.85
            waypoints.color.r = 0.05
            waypoints.color.g = 0.35
            waypoints.color.b = 1.0
            waypoints.points.extend(
                self._point(point.x, point.y, point.z + 0.35)
                for point in result.selected[1:-1]
            )
            array.markers.append(waypoints)
            marker_id += 1

        for point in result.selected[1:]:
            arrow = self._marker("corrected_yaw", marker_id, Marker.ARROW)
            arrow.scale.x = 0.16
            arrow.scale.y = 0.35
            arrow.scale.z = 0.35
            arrow.color.r = 0.05
            arrow.color.g = 0.35
            arrow.color.b = 1.0
            arrow.points.extend(
                [
                    self._point(point.x, point.y, point.z + 0.45),
                    self._point(
                        point.x + math.cos(point.yaw) * 2.0,
                        point.y + math.sin(point.yaw) * 2.0,
                        point.z + 0.45,
                    ),
                ]
            )
            array.markers.append(arrow)
            marker_id += 1

        goal = result.selected[-1]
        star = self._marker("corrected_goal", marker_id, Marker.LINE_STRIP)
        star.scale.x = 0.28
        star.color.r = 0.05
        star.color.g = 0.35
        star.color.b = 1.0
        star_points: list[tuple[float, float]] = []
        for index in range(10):
            angle = goal.yaw + math.pi / 2.0 + index * math.pi / 5.0
            radius = 1.35 if index % 2 == 0 else 0.55
            star_points.append(
                (
                    goal.x + radius * math.cos(angle),
                    goal.y + radius * math.sin(angle),
                )
            )
        star_points.append(star_points[0])
        star.points.extend(
            self._point(x, y, goal.z + 0.40) for x, y in star_points
        )
        array.markers.append(star)
        marker_id += 1

        for point in result.selected[1:-1]:
            array.markers.append(
                self._label(
                    "corrected_label",
                    marker_id,
                    f"waypoint seq {point.route_point.seq}",
                    point.x,
                    point.y,
                    point.z + 1.2,
                    (0.05, 0.35, 1.0),
                )
            )
            marker_id += 1
        array.markers.append(
            self._label(
                "corrected_label",
                marker_id,
                f"GOAL seq {goal.route_point.seq}",
                goal.x,
                goal.y,
                goal.z + 1.8,
                (0.05, 0.35, 1.0),
            )
        )
        return array

    def render_messages(
        self, points: Sequence[RoutePoint], result: MatchedRoute | None = None,
    ) -> dict[str, Any]:
        """Render visualization only; no state writes, publishers or route API calls."""
        from visualization_msgs.msg import MarkerArray

        if result is None:
            messages = {
                topic: MarkerArray(markers=[self._delete_all()])
                for topic in (
                    self.RAW_TOPIC, self.CANDIDATE_TOPIC,
                    self.SELECTED_TOPIC, self.CORRECTED_TOPIC,
                )
            }
            messages[self.RAW_TOPIC] = self._raw_markers(points)
        else:
            messages = {
                self.RAW_TOPIC: self._raw_markers(result.raw_points),
                self.CANDIDATE_TOPIC: self._candidate_markers(result),
                self.SELECTED_TOPIC: self._selected_markers(result),
                self.CORRECTED_TOPIC: self._corrected_markers(result),
            }
        return messages

    def _publish_current(self) -> None:
        if self._result is None and not self._raw_points:
            return
        messages = self.render_messages(self._raw_points, self._result)
        if self.state_path is not None:
            from csv_route_preview import save_preview_state

            save_preview_state(self.state_path, self.state_metadata, messages)
            return
        for topic, message in messages.items():
            self.publishers[topic].publish(message)

    def publish_raw(self, points: Sequence[RoutePoint]) -> None:
        """Retain raw input even if lanelet matching cannot produce a route."""
        self._result = None
        self._raw_points = tuple(points)
        self._publish_current()

    def publish(self, result: MatchedRoute) -> None:
        self._result = result
        self._publish_current()
        if self.state_path is not None:
            print("\nRoute preview saved; the independent RViz publisher stays alive.", flush=True)
            return
        if self._keepalive_timer is None:
            # Republish so RViz displays added after this process starts also receive
            # the preview, even when the display uses volatile durability.
            self._keepalive_timer = self.node.create_timer(
                1.0, self._publish_current
            )
        print("\nRoute preview published to RViz.", flush=True)


def _wait_until(
    node: Any,
    predicate: Callable[[], bool],
    deadline: float,
    description: str,
) -> None:
    import rclpy

    while rclpy.ok() and not predicate():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise RouteSetError(f"timed out waiting for {description}")
        rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
    if not rclpy.ok():
        raise RouteSetError(f"ROS shut down while waiting for {description}")


def _call_service(
    node: Any, client: Any, request: Any, deadline: float, name: str
) -> Any:
    future = client.call_async(request)
    _wait_until(node, future.done, deadline, f"response from {name}")
    try:
        response = future.result()
    except Exception as error:
        raise RouteSetError(f"{name} call failed: {error}") from error
    if response is None:
        raise RouteSetError(f"{name} returned no response")
    return response


def _status_error(action: str, status: Any) -> RouteSetError:
    message = status.message.strip() if status.message else "no error message"
    return RouteSetError(f"Autoware failed to {action}: code={status.code}, {message}")


def _make_route_request(
    node: Any,
    result: MatchedRoute,
    frame_id: str,
    allow_goal_modification: bool,
) -> Any:
    from autoware_adapi_v1_msgs.srv import SetRoutePoints
    from geometry_msgs.msg import Pose

    def make_pose(candidate: LaneletCandidate) -> Pose:
        pose = Pose()
        pose.position.x = candidate.x
        pose.position.y = candidate.y
        pose.position.z = candidate.z
        pose.orientation.z, pose.orientation.w = quaternion_z_w(candidate.yaw)
        return pose

    request = SetRoutePoints.Request()
    request.header.frame_id = frame_id
    request.header.stamp = node.get_clock().now().to_msg()
    request.option.allow_goal_modification = allow_goal_modification
    request.waypoints = [make_pose(point) for point in result.selected[1:-1]]
    request.goal = make_pose(result.selected[-1])
    return request


def set_autoware_route(
    node: Any,
    result: MatchedRoute,
    *,
    frame_id: str,
    timeout: float,
    start_tolerance: float,
    skip_start_check: bool,
    allow_goal_modification: bool,
) -> None:
    """Set a new route, or use change_route_points when a route is SET."""
    from autoware_adapi_v1_msgs.msg import RouteState
    from autoware_adapi_v1_msgs.srv import ClearRoute, SetRoutePoints
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    deadline = time.monotonic() + timeout
    current_odometry: list[Odometry] = []
    current_route_state = [RouteState.UNKNOWN]

    def on_odometry(message: Odometry) -> None:
        current_odometry[:] = [message]

    def on_route_state(message: RouteState) -> None:
        current_route_state[0] = message.state

    route_state_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    odometry_subscription = node.create_subscription(
        Odometry,
        "/localization/kinematic_state",
        on_odometry,
        QoSProfile(depth=1),
    )
    state_subscription = node.create_subscription(
        RouteState, "/api/routing/state", on_route_state, route_state_qos
    )
    clear_client = node.create_client(ClearRoute, "/api/routing/clear_route")
    set_client = node.create_client(SetRoutePoints, "/api/routing/set_route_points")
    change_client = node.create_client(
        SetRoutePoints, "/api/routing/change_route_points"
    )

    try:
        print("Waiting for Autoware route API, map, and localization...", flush=True)

        def route_api_ready() -> bool:
            state = current_route_state[0]
            if not current_odometry or state in (
                RouteState.UNKNOWN,
                RouteState.CHANGING,
            ):
                return False
            if state == RouteState.UNSET:
                return set_client.service_is_ready()
            if state == RouteState.SET:
                return change_client.service_is_ready()
            if state == RouteState.ARRIVED:
                return (
                    clear_client.service_is_ready()
                    and set_client.service_is_ready()
                )
            return False

        _wait_until(
            node,
            route_api_ready,
            deadline,
            "Autoware route API, map, and localization",
        )

        odometry = current_odometry[0]
        if odometry.header.frame_id and odometry.header.frame_id != frame_id:
            raise RouteSetError(
                "localization frame does not match the CSV route frame: "
                f"{odometry.header.frame_id!r} != {frame_id!r}"
            )

        corrected_start = result.selected[0]
        start_distance = math.hypot(
            odometry.pose.pose.position.x - corrected_start.x,
            odometry.pose.pose.position.y - corrected_start.y,
        )
        if not skip_start_check and start_distance > start_tolerance:
            raise RouteSetError(
                f"vehicle is {start_distance:.2f} m from corrected CSV start seq 1 "
                f"(tolerance {start_tolerance:.2f} m); current="
                f"({odometry.pose.pose.position.x:.3f}, "
                f"{odometry.pose.pose.position.y:.3f}), corrected CSV="
                f"({corrected_start.x:.3f}, {corrected_start.y:.3f})"
            )
        print(
            f"CSV start check: vehicle is {start_distance:.2f} m from "
            "corrected seq 1",
            flush=True,
        )

        state = current_route_state[0]
        if state == RouteState.ARRIVED:
            print("Clearing the completed Autoware route...", flush=True)
            clear_response = _call_service(
                node,
                clear_client,
                ClearRoute.Request(),
                deadline,
                "/api/routing/clear_route",
            )
            if not clear_response.status.success:
                raise _status_error(
                    "clear the completed route", clear_response.status
                )
            _wait_until(
                node,
                lambda: current_route_state[0] == RouteState.UNSET,
                deadline,
                "route state UNSET after clearing the completed route",
            )
            state = RouteState.UNSET

        request = _make_route_request(
            node, result, frame_id, allow_goal_modification
        )
        if state == RouteState.SET:
            service_name = "/api/routing/change_route_points"
            client = change_client
            action = "change the CSV route"
            print(
                f"Changing the active route with {len(request.waypoints)} "
                "corrected checkpoint(s); clear_route is intentionally not used...",
                flush=True,
            )
        else:
            service_name = "/api/routing/set_route_points"
            client = set_client
            action = "set the CSV route"
            print(
                f"Setting {len(request.waypoints)} corrected checkpoint(s) and "
                "the corrected final goal...",
                flush=True,
            )

        response = _call_service(node, client, request, deadline, service_name)
        if not response.status.success:
            raise _status_error(action, response.status)
        _wait_until(
            node,
            lambda: current_route_state[0] in (
                RouteState.SET,
                RouteState.ARRIVED,
            ),
            deadline,
            "route state SET or ARRIVED",
        )
        print("Autoware route was set successfully.", flush=True)
    finally:
        node.destroy_subscription(odometry_subscription)
        node.destroy_subscription(state_subscription)


def apply_rviz_overrides(
    node: Any,
    preview: RoutePreviewPublisher,
    matcher: LaneletMapMatcher,
    points: Sequence[RoutePoint],
    result: MatchedRoute,
    sequences: Sequence[int],
    *,
    topic: str,
    timeout: float,
    maximum_distance: float,
) -> MatchedRoute:
    from geometry_msgs.msg import PointStamped

    point_by_seq = {point.seq: point for point in points}
    invalid = sorted(set(sequences) - point_by_seq.keys())
    if invalid:
        raise RouteCsvError(
            "--override-seq is outside the CSV sequence range: "
            + ", ".join(str(value) for value in invalid)
        )

    clicked_messages: list[PointStamped] = []

    def on_clicked_point(message: PointStamped) -> None:
        clicked_messages.append(message)

    subscription = node.create_subscription(
        PointStamped, topic, on_clicked_point, 10
    )
    overrides = {
        point.seq: point.lanelet_id
        for point in points
        if point.lanelet_id is not None
    }
    try:
        for seq in sequences:
            while True:
                clicked_messages.clear()
                preview.publish(result)
                print(
                    f"[OVERRIDE] Select the RViz Publish Point tool and click the "
                    f"desired lanelet for seq {seq} (topic {topic}).",
                    flush=True,
                )
                deadline = time.monotonic() + timeout
                _wait_until(
                    node,
                    lambda: bool(clicked_messages),
                    deadline,
                    f"RViz click for seq {seq}",
                )
                click = clicked_messages[-1]
                if (
                    click.header.frame_id
                    and click.header.frame_id != preview.frame_id
                ):
                    print(
                        f"Ignoring click in frame {click.header.frame_id!r}; "
                        f"expected {preview.frame_id!r}.",
                        file=sys.stderr,
                        flush=True,
                    )
                    continue
                try:
                    lanelet_id, distance = matcher.lanelet_from_click(
                        click.point.x, click.point.y, maximum_distance
                    )
                    proposed = dict(overrides)
                    proposed[seq] = lanelet_id
                    rematched = matcher.match(points, proposed)
                except MapMatchError as error:
                    print(
                        f"Override rejected for seq {seq}: {error}",
                        file=sys.stderr,
                        flush=True,
                    )
                    continue
                overrides = proposed
                result = rematched
                print(
                    f"[OVERRIDE] seq {seq} -> lanelet {lanelet_id} "
                    f"(click offset {distance:.2f} m)",
                    flush=True,
                )
                break
    finally:
        node.destroy_subscription(subscription)
    return result


def _positive_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError(
            "must be a finite number greater than zero"
        )
    return result


def _nonnegative_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise argparse.ArgumentTypeError(
            "must be a finite number at least zero"
        )
    return result


def _positive_int(value: str) -> int:
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be an integer greater than zero")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Map-match seq,x,y CSV points to connected lanelet centerlines, "
            "preview them in RViz, and set or change the Autoware route."
        )
    )
    parser.add_argument("csv_path", help="route CSV file")
    parser.add_argument("--map", dest="map_path", help="lanelet2_map.osm path")
    parser.add_argument("--frame-id", default="map", help="coordinate frame")
    parser.add_argument(
        "--candidate-radius",
        type=_positive_float,
        default=8.0,
        help="raw-point to centerline search radius in metres (default: 8)",
    )
    parser.add_argument(
        "--candidate-count",
        type=_positive_int,
        default=8,
        help="maximum candidate lanelets retained per point (default: 8)",
    )
    parser.add_argument(
        "--direction-threshold",
        type=_positive_float,
        default=90.0,
        help="maximum selected-lane heading error in degrees (default: 90)",
    )
    parser.add_argument(
        "--vehicle-width",
        type=_positive_float,
        default=1.896,
        help="vehicle width used for goal validation in metres (default: 1.896)",
    )
    parser.add_argument(
        "--goal-margin",
        type=_nonnegative_float,
        default=0.20,
        help="extra required clearance on each side of goal (default: 0.20)",
    )
    parser.add_argument(
        "--timeout",
        type=_positive_float,
        default=300.0,
        help="overall Autoware wait/call timeout in seconds (default: 300)",
    )
    parser.add_argument(
        "--start-tolerance",
        type=_nonnegative_float,
        default=5.0,
        help="distance allowed by optional --check-start (default: 5 m)",
    )
    start_check_group = parser.add_mutually_exclusive_group()
    start_check_group.add_argument(
        "--check-start",
        action="store_true",
        help="enforce proximity between localization and corrected seq 1",
    )
    start_check_group.add_argument(
        "--skip-start-check",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--allow-goal-modification",
        action="store_true",
        help="allow downstream Autoware modules to modify the corrected goal",
    )
    parser.add_argument(
        "--allow-direction-mismatch",
        action="store_true",
        help="submit even when selected lane direction exceeds the threshold",
    )
    parser.add_argument(
        "--strict-validation",
        action="store_true",
        help="block route submission when direction or goal validation fails",
    )
    parser.add_argument(
        "--preview-wait",
        type=_nonnegative_float,
        default=1.0,
        help="seconds to expose the RViz preview before submission (default: 1)",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="do not publish RViz MarkerArray previews",
    )
    parser.add_argument(
        "--preview-only",
        action="store_true",
        help="publish preview without setting a route (managed launcher saves and exits)",
    )
    parser.add_argument(
        "--preview-state",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--override-seq",
        type=_positive_int,
        action="append",
        default=[],
        metavar="SEQ",
        help="wait for an RViz /clicked_point lanelet override for this sequence",
    )
    parser.add_argument(
        "--clicked-point-topic",
        default="/clicked_point",
        help="RViz PointStamped topic used by --override-seq",
    )
    parser.add_argument(
        "--click-timeout",
        type=_positive_float,
        default=300.0,
        help="seconds to wait for each RViz override click (default: 300)",
    )
    parser.add_argument(
        "--click-radius",
        type=_positive_float,
        default=5.0,
        help="maximum click-to-centerline distance in metres (default: 5)",
    )
    parser.add_argument(
        "--output-csv",
        help="save corrected points, yaw, and lanelet IDs to this CSV",
    )
    parser.add_argument(
        "--force-output",
        action="store_true",
        help="allow --output-csv to replace an existing file",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="map-match, validate, and print without connecting to ROS",
    )
    return parser


def _spin_for(node: Any, seconds: float) -> None:
    import rclpy

    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(
            node, timeout_sec=min(0.1, deadline - time.monotonic())
        )


def _keep_preview_alive(node: Any, message: str) -> None:
    import rclpy

    print(message, flush=True)
    print(
        "RViz preview remains active and is republished every second; "
        "press Ctrl-C to exit.",
        flush=True,
    )
    rclpy.spin(node)


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    if not args.frame_id.strip():
        parser.error("--frame-id cannot be empty")
    if args.dry_run and args.override_seq:
        parser.error("--dry-run cannot be combined with --override-seq")
    if args.preview_only and args.no_preview:
        parser.error("--preview-only cannot be combined with --no-preview")
    if args.preview_state and args.no_preview:
        parser.error("--preview-state cannot be combined with --no-preview")

    node = None
    try:
        points = load_route_csv(args.csv_path)
        map_path = resolve_map_path(args.map_path)
        print(f"Loading lanelet map: {map_path}", flush=True)
        if not args.dry_run:
            import rclpy
            from rclpy.node import Node

            rclpy.init()
            node = Node("csv_route_setter")
            metadata = {}
            if args.preview_state:
                from csv_route_preview import map_digest

                metadata = {
                    "source_csv": os.environ.get("AUTOWARE_CSV_SOURCE_PATH", args.csv_path),
                    "csv_text": Path(args.csv_path).read_text(encoding="utf-8-sig"),
                    "map_sha256": map_digest(map_path),
                    "frame_id": args.frame_id,
                }
            preview = RoutePreviewPublisher(
                node, args.frame_id,
                state_path=args.preview_state,
                state_metadata=metadata,
            )
            if args.preview_state:
                preview.publish_raw(points)
        matcher = LaneletMapMatcher(
            map_path,
            candidate_radius=args.candidate_radius,
            candidate_count=args.candidate_count,
            direction_threshold=math.radians(args.direction_threshold),
            vehicle_width=args.vehicle_width,
            goal_margin=args.goal_margin,
        )
        result = matcher.match(points)

        if args.dry_run:
            print_match_report(result)
            if not result.direction_ok and not args.allow_direction_mismatch:
                raise MapMatchError(
                    "selected lanelet direction exceeds --direction-threshold"
                )
            if not result.goal_ok:
                raise MapMatchError(
                    "corrected goal does not have enough in-lane vehicle clearance"
                )
            if args.output_csv:
                save_corrected_csv(
                    result, args.output_csv, overwrite=args.force_output
                )
            return 0

        if args.override_seq:
            result = apply_rviz_overrides(
                node,
                preview,
                matcher,
                points,
                result,
                args.override_seq,
                topic=args.clicked_point_topic,
                timeout=args.click_timeout,
                maximum_distance=args.click_radius,
            )

        print_match_report(result)
        # Visualize a matched result even when strict validation or route API
        # submission rejects it. Saving a preview never authorizes a route.
        if not args.no_preview:
            preview.publish(result)
            _spin_for(node, args.preview_wait)
        validation_errors = []
        if not result.direction_ok and not args.allow_direction_mismatch:
            validation_errors.append(
                "selected lanelet direction exceeds --direction-threshold"
            )
        if not result.goal_ok:
            validation_errors.append(
                "corrected goal does not have enough in-lane vehicle clearance"
            )
        for validation_error in validation_errors:
            print(f"WARNING: {validation_error}", file=sys.stderr, flush=True)
        if args.strict_validation and validation_errors:
            raise MapMatchError("; ".join(validation_errors))
        if args.output_csv:
            save_corrected_csv(
                result, args.output_csv, overwrite=args.force_output
            )

        if args.preview_only:
            if not args.preview_state:
                _keep_preview_alive(node, "Preview-only mode.")
            return 0

        try:
            set_autoware_route(
                node,
                result,
                frame_id=args.frame_id,
                timeout=args.timeout,
                start_tolerance=args.start_tolerance,
                skip_start_check=not args.check_start,
                allow_goal_modification=args.allow_goal_modification,
            )
        except RouteSetError as error:
            print(f"ERROR: {error}", file=sys.stderr, flush=True)
            if not args.no_preview and not args.preview_state:
                _keep_preview_alive(
                    node, "Route submission failed, but the preview will stay alive."
                )
            return 1

        if not args.no_preview and not args.preview_state:
            _keep_preview_alive(
                node, "Route submission finished; keeping the preview alive."
            )
    except (RouteCsvError, MapMatchError, RouteSetError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        return 1
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr, flush=True)
        return 130
    finally:
        if node is not None:
            node.destroy_node()
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
