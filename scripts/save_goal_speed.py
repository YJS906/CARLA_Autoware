#!/usr/bin/env python3
"""Set 30 km/h on the road cross-section at the current RViz/Autoware goal.

Only lanelet speed_limit tags are patched. Geometry and all other XML bytes are
preserved. A dry run has no map side effects. Runtime map reload is not performed.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time

from lxml import etree
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

STATE = Path.home() / '.local/state/autoware-goal-speed'
SPEED = '30 km/h'
INDEX_VERSION = 2
EDGE_TOLERANCE = 0.25  # m: duplicate/rounded boundaries, never jump a median.
SNAP_DISTANCE = 1.0    # m: fail instead of selecting a distant road.


def command(args, **kwargs):
    return subprocess.run(args, check=True, text=True, capture_output=True,
                          timeout=kwargs.pop('timeout', 15), **kwargs).stdout


def signature(path):
    s = path.stat()
    return [s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns]


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, separators=(',', ':'))
            stream.write('\n')
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def runtime(container=None):
    if not container:
        names = command(['docker', 'ps', '--format', '{{.Names}}']).splitlines()
        names = [n for n in names if n.startswith('vtd-autoware-run-')]
        if len(names) != 1:
            raise ValueError(f'실행 중인 Autoware가 정확히 하나여야 합니다: {names}')
        container = names[0]
    info = json.loads(command(['docker', 'inspect', container]))[0]
    if not info['State']['Running']:
        raise ValueError('Autoware 컨테이너가 실행 중이 아닙니다.')
    return container, info


def active_map(info):
    env = dict(x.split('=', 1) for x in info['Config']['Env'] if '=' in x)
    relative = env.get('VTD_MAP_RELATIVE_PATH')
    if not relative:
        raise ValueError('실행 중인 VTD_MAP_RELATIVE_PATH를 찾을 수 없습니다.')
    target = Path('/home/aw/vtd_autoware_maps') / relative / 'lanelet2_map.osm'
    for mount in sorted(info['Mounts'], key=lambda m: len(m['Destination']), reverse=True):
        try:
            suffix = target.relative_to(mount['Destination'])
        except ValueError:
            continue
        return (Path(mount['Source']) / suffix).resolve()
    raise ValueError(f'활성 지도에 대응하는 호스트 경로가 없습니다: {target}')


def cached_goal(container):
    def read():
        try:
            data = json.loads(command(['docker', 'exec', container, 'cat',
                                       '/tmp/autoware-goal-speed.json']))
            if data.get('version') == 1 and time.time() - data.get('alive_at', 0) < 3:
                return data
        except (subprocess.SubprocessError, json.JSONDecodeError):
            pass
        return None

    data = read()
    if not data:
        watcher = Path(__file__).with_name('goal_speed_watch.py')
        command(['docker', 'cp', str(watcher), container + ':/tmp/goal_speed_watch.py'])
        launch = ('source /opt/ros/jazzy/setup.bash; source /opt/autoware/setup.bash; '
                  'source /opt/selfcar_overlay/setup.bash; '
                  'exec python3 /tmp/goal_speed_watch.py '
                  '>/tmp/goal-speed-watch.log 2>&1')
        command(['docker', 'exec', '-d', container, 'bash', '-lc', launch])
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        data = read()
        if data and data.get('goal'):
            goal = data['goal']
            if goal['frame'].lstrip('/') != 'map':
                raise ValueError(f'Goal 좌표계가 map이 아닙니다: {goal["frame"]}')
            if not all(math.isfinite(goal[k]) for k in ('x', 'y', 'z')):
                raise ValueError('Goal 좌표가 유효하지 않습니다.')
            return goal
        time.sleep(0.1)
    raise ValueError('현재 Goal Pose를 받지 못했습니다. RViz에서 2D Goal Pose를 지정하세요.')


def make_index(path):
    projector = path.with_name('map_projector_info.yaml')
    if projector.exists() and not re.search(r'projector_type:\s*Local\s*$', projector.read_text(), re.M):
        raise ValueError('이 스크립트는 local_x/local_y를 사용하는 Local 지도용입니다.')
    before = signature(path)
    nodes, ways, pending, roads = {}, {}, [], []
    for _, element in etree.iterparse(str(path), events=('end',), tag=('node', 'way', 'relation')):
        eid = int(element.get('id'))
        tags = {t.get('k'): t.get('v') for t in element.findall('tag')}
        if element.tag == 'node':
            if 'local_x' in tags and 'local_y' in tags:
                nodes[eid] = (float(tags['local_x']), float(tags['local_y']))
        elif element.tag == 'way':
            ways[eid] = [int(n.get('ref')) for n in element.findall('nd')]
        elif tags.get('type') == 'lanelet' and tags.get('subtype') in ('road', 'highway'):
            members = {m.get('role'): int(m.get('ref')) for m in element.findall('member')
                       if m.get('type') == 'way' and m.get('role') in ('left', 'right')}
            if not all(k in members for k in ('left', 'right')):
                raise ValueError(f'차선 {eid}의 좌우 경계가 없습니다.')
            # Repairs may append new ways/nodes after the lanelet that refers to
            # them. Resolve members after parsing, never depend on XML order.
            pending.append((eid, members))
        element.clear()
        while element.getprevious() is not None:
            del element.getparent()[0]
    for eid, members in pending:
        try:
            left = [nodes[n] for n in ways[members['left']]]
            right = [nodes[n] for n in ways[members['right']]]
        except KeyError as exc:
            raise ValueError(f'차선 {eid}의 local 좌표가 누락됐습니다: {exc}') from exc
        if len(left) < 2 or len(right) < 2:
            raise ValueError(f'차선 {eid}의 경계가 너무 짧습니다.')
        same = math.dist(left[0], right[0]) + math.dist(left[-1], right[-1])
        reverse = math.dist(left[0], right[-1]) + math.dist(left[-1], right[0])
        if reverse < same:
            right = list(reversed(right))
        roads.append({'id': eid, 'left': left, 'right': right})
    if signature(path) != before:
        raise ValueError('지도 인덱스 생성 도중 파일이 변경됐습니다. 다시 실행하세요.')
    if not roads:
        raise ValueError('도로 lanelet을 찾지 못했습니다.')
    return {'version': INDEX_VERSION, 'map': str(path), 'signature': before, 'roads': roads}


def load_index(path, state):
    key = hashlib.sha256(str(path).encode()).hexdigest()[:16]
    cache = state / ('index-' + key + '.json')
    try:
        index = json.loads(cache.read_text())
        if index['version'] == INDEX_VERSION and index['signature'] == signature(path):
            return index, cache, True
    except (OSError, ValueError, KeyError):
        pass
    index = make_index(path)
    write_json(cache, index)
    return index, cache, False


def centerline(road):
    a, b = LineString(road['left']), LineString(road['right'])
    # OSM bounds are normally co-directed; tolerate reversed imported coordinates.
    same = Point(a.coords[0]).distance(Point(b.coords[0])) + Point(a.coords[-1]).distance(Point(b.coords[-1]))
    reverse = Point(a.coords[0]).distance(Point(b.coords[-1])) + Point(a.coords[-1]).distance(Point(b.coords[0]))
    if reverse < same:
        b = LineString(list(b.coords)[::-1])
    count = max(2, math.ceil(max(a.length, b.length) / 2) + 1)
    points = []
    for i in range(count):
        p, q = a.interpolate(i / (count - 1), normalized=True), b.interpolate(i / (count - 1), normalized=True)
        points.append(((p.x + q.x) / 2, (p.y + q.y) / 2))
    return LineString(points)


def tangent(line, point):
    s = line.project(point)
    a, b = line.interpolate(max(0, s - 1)), line.interpolate(min(line.length, s + 1))
    length = a.distance(b)
    if length < 1e-6:
        raise ValueError('길이가 0인 도로 중심선입니다.')
    return ((b.x - a.x) / length, (b.y - a.y) / length)


def intervals(geometry, cut):
    if geometry.is_empty:
        return []
    if geometry.geom_type in ('LineString', 'Point'):
        values = [cut.project(Point(p)) for p in geometry.coords]
        return [(min(values), max(values))]
    if hasattr(geometry, 'geoms'):
        return [v for part in geometry.geoms for v in intervals(part, cut)]
    return []


def select_roads(index, goal):
    point = Point(goal['x'], goal['y'])
    roads = index['roads']
    polygons = [Polygon(r['left'] + list(reversed(r['right']))) for r in roads]
    tree = STRtree(polygons)
    seeds = [int(i) for i in tree.query(point.buffer(1e-7)) if polygons[int(i)].distance(point) < 1e-7]
    if not seeds:
        nearest = int(tree.nearest(point))
        distance = polygons[nearest].distance(point)
        if distance > SNAP_DISTANCE:
            raise ValueError(f'Goal이 도로에서 {distance:.2f} m 떨어져 있습니다. 지도를 바꾸지 않았습니다.')
        seeds = [nearest]
    for i in seeds:
        if not polygons[i].is_valid:
            raise ValueError(f'Goal의 차선 {roads[i]["id"]} 폴리곤이 유효하지 않습니다.')
    lines = {}

    def axis(i, p):
        if i not in lines:
            lines[i] = centerline(roads[i])
        return tangent(lines[i], p)

    seed = min(seeds, key=lambda i: centerline(roads[i]).distance(point))
    direction = axis(seed, point)
    for i in seeds:
        other = axis(i, point)
        if abs(direction[0] * other[0] + direction[1] * other[1]) < math.cos(math.radians(35)):
            raise ValueError('Goal이 서로 교차하는 도로에 겹칩니다. 교차로 밖의 도로에 Goal을 지정하세요.')
    # Search a cross-section, not successor/predecessor topology. This cannot walk
    # down a road network or reach a crossing street just because endpoints meet.
    half = max(100.0, max(max(p.bounds[2] - p.bounds[0], p.bounds[3] - p.bounds[1]) for p in polygons))
    normal = (-direction[1], direction[0])
    cut = LineString([(point.x - half * normal[0], point.y - half * normal[1]),
                      (point.x + half * normal[0], point.y + half * normal[1])])
    sections = []
    for j in tree.query(cut):
        i = int(j)
        polygon = polygons[i]
        if not polygon.is_valid:
            if polygon.distance(point) < 30:
                raise ValueError(f'인접 도로 {roads[i]["id"]}의 폴리곤이 유효하지 않습니다.')
            continue
        for lo, hi in intervals(polygon.intersection(cut), cut):
            if hi - lo < 0.05:
                continue
            mid = cut.interpolate((lo + hi) / 2)
            other = axis(i, mid)
            if abs(direction[0] * other[0] + direction[1] * other[1]) >= math.cos(math.radians(35)):
                sections.append((lo, hi, i))
    selected = {i for lo, hi, i in sections if i in seeds}
    if not selected:
        raise ValueError('Goal의 도로 횡단면을 찾지 못했습니다.')
    active = [(lo, hi) for lo, hi, i in sections if i in selected]
    low, high = min(v[0] for v in active), max(v[1] for v in active)
    changed = True
    while changed:
        changed = False
        for lo, hi, i in sections:
            if i not in selected and lo <= high + EDGE_TOLERANCE and hi >= low - EDGE_TOLERANCE:
                selected.add(i)
                low, high = min(low, lo), max(high, hi)
                changed = True
    selected_ids = sorted(roads[i]['id'] for i in selected)
    return selected_ids, {'seed_lanelet_ids': sorted(roads[i]['id'] for i in seeds),
        'cross_section_width_m': round(high - low, 3),
        'goal_distance_to_road_m': round(min(polygons[i].distance(point) for i in seeds), 6),
        'opposite_direction_lanelet_ids': sorted(roads[i]['id'] for i in selected
            if sum(a*b for a,b in zip(axis(i, point), direction)) < 0)}


RELATION = re.compile(rb'<relation\b[^>]*\bid=["\x27](\d+)["\x27][^>]*>.*?</relation>', re.S)
TAG = re.compile(rb'<tag\b[^>]*/>')
VALUE = re.compile(rb'\bv\s*=\s*(["\x27])(.*?)\1', re.S)


def patch_bytes(original, ids):
    wanted = set(ids)
    found, changes, pieces = set(), [], []
    cursor = 0
    for match in RELATION.finditer(original):
        rid = int(match[1])
        if rid not in wanted:
            continue
        found.add(rid)
        old = match[0]
        element = etree.fromstring(old)
        attributes = {t.get('k'): t.get('v') for t in element.findall('tag')}
        if attributes.get('type') != 'lanelet' or attributes.get('subtype') not in ('road', 'highway'):
            raise ValueError(f'{rid}는 도로 lanelet이 아닙니다.')
        speed_tags = [t for t in TAG.finditer(old) if etree.fromstring(t[0]).get('k') == 'speed_limit']
        if len(speed_tags) > 1:
            raise ValueError(f'{rid}에 speed_limit 태그가 중복돼 있습니다.')
        new = old
        if speed_tags:
            tag = speed_tags[0]
            value = VALUE.search(tag[0])
            if not value:
                raise ValueError(f'{rid}의 speed_limit 값 형식이 잘못됐습니다.')
            replacement = tag[0][:value.start(2)] + SPEED.encode() + tag[0][value.end(2):]
            new = old[:tag.start()] + replacement + old[tag.end():]
        else:
            new = old.replace(b'</relation>', b'  <tag k="speed_limit" v="30 km/h"/>\n  </relation>', 1)
        if new != old:
            changes.append({'lanelet_id': rid, 'old_speed_limit': attributes.get('speed_limit'),
                            'new_speed_limit': SPEED})
        pieces.extend((original[cursor:match.start()], new))
        cursor = match.end()
    if found != wanted:
        raise ValueError(f'지도에서 찾지 못한 차선 ID: {sorted(wanted - found)}')
    pieces.append(original[cursor:])
    return b''.join(pieces), changes


def save_map(path, ids, index, index_file, state, report, dry_run):
    if signature(path) != index['signature']:
        raise ValueError('선택 이후 지도가 변경됐습니다. 다시 실행하세요.')
    original = path.read_bytes()
    patched, changes = patch_bytes(original, ids)
    report.update({'changed_lanelets': changes, 'changed_count': len(changes),
                   'status': 'preview' if dry_run else ('saved' if changes else 'already_30_kmh')})
    if dry_run or not changes:
        return
    # Validate the byte patch before touching the original; never reserialize the map.
    undo, _ = patch_bytes(patched, ids)
    if undo != patched:
        raise ValueError('속도 태그 변경 검증 실패')
    if signature(path) != index['signature']:
        raise ValueError('저장 도중 지도가 변경됐습니다. 다시 실행하세요.')
    stamp = time.strftime('%Y%m%d-%H%M%S') + f'-{time.time_ns() % 1_000_000_000:09d}'
    history = state / 'backups' / stamp
    history.mkdir(parents=True)
    backup = history / path.name
    # Reflink when available; original bytes remain recoverable on all filesystems.
    command(['cp', '--reflink=auto', '--preserve=mode,timestamps', str(path), str(backup)], timeout=30)
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(patched)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, path.stat().st_mode & 0o777)
        if signature(path) != index['signature']:
            raise ValueError('저장 직전 지도가 변경됐습니다. 다시 실행하세요.')
        os.replace(temporary, path)
        parent_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    report.update({'backup': str(backup), 'before_sha256': hashlib.sha256(original).hexdigest(),
                   'after_sha256': hashlib.sha256(patched).hexdigest(),
                   'runtime_reload_required': True})
    index['signature'] = signature(path)
    write_json(index_file, index)
    write_json(history / 'change.json', report)


def main():
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='대상 확인만 하고 지도는 변경하지 않음')
    parser.add_argument('--container', help='자동 검색 대신 사용할 Autoware 컨테이너')
    parser.add_argument('--map', type=Path, help='활성 지도 대신 사용할 파일 (복사본 검사 등에 사용)')
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'), help='ROS 대신 map 좌표 지정')
    parser.add_argument('--state-dir', type=Path, default=STATE)
    args = parser.parse_args()
    args.state_dir.mkdir(parents=True, exist_ok=True)
    try:
        if args.goal:
            if not args.map:
                raise ValueError('--goal을 사용할 때는 --map도 지정해야 합니다.')
            if not all(math.isfinite(x) for x in args.goal):
                raise ValueError('Goal 좌표가 유효하지 않습니다.')
            goal = dict(x=args.goal[0], y=args.goal[1], z=0., frame='map', source='explicit')
            container = None
        else:
            container, info = runtime(args.container)
            goal = cached_goal(container)
        path = args.map.resolve() if args.map else active_map(info)
        if not path.is_file():
            raise ValueError(f'지도 파일이 없습니다: {path}')
        lock_key = hashlib.sha256(str(path).encode()).hexdigest()[:16]
        with open(args.state_dir / (lock_key + '.lock'), 'w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            index, index_file, cache_hit = load_index(path, args.state_dir)
            ids, selection = select_roads(index, goal)
            report = {'goal': goal, 'map': str(path), 'container': container,
                      'speed_limit': SPEED, 'lanelet_ids': ids, 'selected_count': len(ids),
                      'geometry_cache_hit': cache_hit, **selection}
            save_map(path, ids, index, index_file, args.state_dir, report, args.dry_run)
        report['elapsed_seconds'] = round(time.monotonic() - started, 3)
        write_json(args.state_dir / ('last-preview.json' if args.dry_run else 'last-save.json'), report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
    except (ValueError, OSError, subprocess.SubprocessError, etree.Error) as exc:
        print(json.dumps({'status': 'error', 'message': str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
