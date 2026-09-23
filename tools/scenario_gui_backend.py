"""Local scenario editor operations. Never ticks, reloads, or engages Autoware."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import carla

ROOT = Path(__file__).resolve().parents[1]
STATE = Path.home() / '.local/state/carla_pp/scenario-gui'
TRAFFIC_CONFIG = ROOT / 'config/carla/traffic/city_dense.json'
ROLE_PREFIX = 'scenario_gui_'
MODES = {'vehicle': {'parked', 'autopilot'}, 'pedestrian': {'wait', 'walk'}}


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + f'.{os.getpid()}.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    os.replace(tmp, path)


def number(value, low, high, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{label}: {low}~{high} 범위의 숫자를 입력하세요.')
    return value


def validate_scenario(data):
    if not isinstance(data, dict) or data.get('schema') != 1 or data.get('map') != 'Town05_Opt':
        raise ValueError('Town05 시나리오 파일이 아닙니다.')
    actors = data.get('actors')
    if not isinstance(actors, list) or len(actors) > 100:
        raise ValueError('시나리오 객체는 최대 100개입니다.')
    ids = set()
    for a in actors:
        if not isinstance(a, dict) or a.get('kind') not in MODES or a.get('mode') not in MODES[a['kind']]:
            raise ValueError('객체 종류 또는 동작이 잘못됐습니다.')
        aid = a.get('id')
        if not isinstance(aid, str) or not aid or len(aid) > 80 or aid in ids:
            raise ValueError('객체 ID가 잘못됐거나 중복됐습니다.')
        ids.add(aid)
        for key in ('x', 'y', 'z'):
            number(a.get(key), -5000, 5000, key)
        number(a.get('yaw'), -360, 360, '방향')
        number(a.get('speed'), 0, 60 if a['kind'] == 'vehicle' else 3, '속도')
        target = a.get('target')
        if target is not None:
            if not isinstance(target, list) or len(target) != 2:
                raise ValueError('보행자 도착 지점이 잘못됐습니다.')
            for v in target:
                number(v, -5000, 5000, '도착 지점')
        if a['mode'] == 'walk' and target is None:
            raise ValueError('걷는 보행자의 도착 지점을 지도에서 지정하세요.')
    return data


def read_map(path):
    root = ET.parse(path).getroot()
    def tags(e):
        return {t.get('k'): t.get('v') for t in e.findall('tag')}
    nodes = {n.get('id'): (float(tags(n)['local_x']), float(tags(n)['local_y'])) for n in root.findall('node')}
    ways = {w.get('id'): [nodes[n.get('ref')] for n in w.findall('nd')] for w in root.findall('way')}
    roads, crossings, stops = [], [], []
    for r in root.findall('relation'):
        t = tags(r)
        if t.get('type') == 'lanelet' and t.get('subtype') == 'road':
            left = ways[r.find("member[@role='left']").get('ref')]
            right = list(ways[r.find("member[@role='right']").get('ref')])
            if math.dist(left[0], right[0]) + math.dist(left[-1], right[-1]) > math.dist(left[0], right[-1]) + math.dist(left[-1], right[0]):
                right.reverse()
            roads.append(left + right[::-1])
        if t.get('type') == 'regulatory_element' and t.get('subtype') == 'crosswalk':
            crossings.append(ways[r.find("member[@role='crosswalk_polygon']").get('ref')])
    for w in root.findall('way'):
        if tags(w).get('type') == 'stop_line':
            stops.append(ways[w.get('id')])
    return {'roads': roads, 'crossings': crossings, 'stops': stops}


class ScenarioEngine:
    def __init__(self, state_dir=STATE):
        self.state_dir = Path(state_dir)
        self.registry_path = self.state_dir / 'owned.json'
        self.client = None
        self.world = None
        self.world_id = None
        self.owned = {}
        self.walkers = {}
        self.last_snapshot = None
        self.last_wall = 0
        self.fps = 0
        self.map = None
        self._load_registry()

    def _load_registry(self):
        if self.registry_path.exists():
            data = json.loads(self.registry_path.read_text())
            self.world_id = data.get('world_id')
            self.owned = {int(k): v for k, v in data.get('actors', {}).items()}

    def persist(self):
        atomic_json(self.registry_path, {'world_id': self.world_id, 'actors': self.owned})

    def connect(self):
        if self.client is None:
            self.client = carla.Client('127.0.0.1', 2000)
            self.client.set_timeout(3)
        world = self.client.get_world()
        if self.map is None or world.id != self.world_id:
            self.map = world.get_map()
        name = self.map.name.rsplit('/', 1)[-1]
        if name not in ('Town05', 'Town05_Opt'):
            raise ValueError(f'현재 지도는 {name}입니다. 이 편집기는 Town05용입니다.')
        if self.world_id is not None and world.id != self.world_id:
            self.owned.clear()
            self.walkers.clear()
            self.persist_for_world(world.id)
        self.world, self.world_id = world, world.id
        return world

    def persist_for_world(self, world_id):
        self.world_id = world_id
        self.persist()

    def verified_actor(self, actor_id):
        record = self.owned.get(actor_id)
        if not record:
            return None
        actor = self.world.get_actor(actor_id)
        if actor and actor.type_id == record['type'] and actor.attributes.get('role_name') == record['role'] and record['role'].startswith(ROLE_PREFIX):
            return actor
        return None

    def status(self):
        w = self.connect()
        snap = w.get_snapshot()
        now = time.monotonic()
        if self.last_snapshot and now > self.last_wall:
            self.fps = (snap.frame - self.last_snapshot.frame) / (now - self.last_wall)
        self.last_snapshot, self.last_wall = snap, now
        actors = []
        for actor in w.get_actors():
            if not actor.type_id.startswith(('vehicle.', 'walker.pedestrian.')):
                continue
            sample = snap.find(actor.id)
            if sample is None:
                continue
            t = sample.get_transform()
            actors.append({'id': actor.id, 'kind': 'vehicle' if actor.type_id.startswith('vehicle.') else 'pedestrian',
                           'ego': actor.attributes.get('role_name') in ('ego_vehicle', 'hero', 'ego'),
                           'owned': actor.id in self.owned and actor.attributes.get('role_name') == self.owned[actor.id]['role'],
                           'x': t.location.x, 'y': -t.location.y, 'yaw': -t.rotation.yaw})
        # Only directly controlled GUI pedestrians; no server ticks or world settings.
        for actor_id, action in list(self.walkers.items()):
            actor = self.verified_actor(actor_id)
            if actor is None:
                self.walkers.pop(actor_id, None)
                continue
            pos = actor.get_location()
            dx, dy = action['target'][0] - pos.x, -action['target'][1] - pos.y
            distance = math.hypot(dx, dy)
            speed = action['speed'] if distance > .35 else 0.0
            actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(dx/max(distance,.001), dy/max(distance,.001), 0), speed=speed))
            if not speed:
                self.walkers.pop(actor_id, None)
        return {'map': 'Town05', 'world_id': w.id, 'fps': max(0, self.fps), 'actors': actors,
                'owned': len([a for a in actors if a['owned']]), 'draft_active': bool(self.owned)}

    def snap(self, kind, x, y):
        w = self.connect()
        lane_type = carla.LaneType.Driving if kind == 'vehicle' else carla.LaneType.Sidewalk
        waypoint = self.map.get_waypoint(carla.Location(float(x), -float(y), 0), project_to_road=True, lane_type=lane_type)
        if waypoint is None or math.hypot(waypoint.transform.location.x-x, waypoint.transform.location.y+y) > 20:
            raise ValueError('가까운 차로/보도를 찾지 못했습니다. 도로 가까이를 클릭하세요.')
        t = waypoint.transform
        return {'x': t.location.x, 'y': -t.location.y, 'z': t.location.z + .5, 'yaw': -t.rotation.yaw}

    def stop(self):
        self.connect()
        failed = []
        for actor_id in list(self.owned):
            actor = self.verified_actor(actor_id)
            if actor is None:
                self.owned.pop(actor_id, None)
                self.walkers.pop(actor_id, None)
                continue
            if actor.destroy():
                self.owned.pop(actor_id, None)
                self.walkers.pop(actor_id, None)
            else:
                failed.append(actor_id)
        self.persist()
        if failed:
            raise RuntimeError(f'삭제하지 못한 시나리오 객체: {failed}. 다시 정리를 눌러 주세요.')
        return '이 편집기로 실행한 객체를 정리했습니다.'

    def start(self, data):
        validate_scenario(data)
        w = self.connect()
        if self.owned:
            raise ValueError('실행 중인 시나리오를 먼저 정리한 뒤 실행하세요.')
        if not data['actors']:
            raise ValueError('지도에 차량이나 보행자를 먼저 배치하세요.')
        # Refuse immediately overlapping ego placement before creating anything.
        egos = [a.get_location() for a in w.get_actors().filter('vehicle.*') if a.attributes.get('role_name') in ('ego_vehicle','hero','ego')]
        for spec in data['actors']:
            if any(math.hypot(e.x-spec['x'], e.y+spec['y']) < 6 for e in egos):
                raise ValueError('주행 차량에서 6m 이상 떨어진 위치에 배치하세요.')
        library = w.get_blueprint_library()
        try:
            for spec in data['actors']:
                blueprint = library.find('vehicle.tesla.model3' if spec['kind']=='vehicle' else 'walker.pedestrian.0001')
                role = ROLE_PREFIX + uuid.uuid4().hex
                blueprint.set_attribute('role_name', role)
                if blueprint.has_attribute('color'):
                    blueprint.set_attribute('color', '36,175,195')
                if blueprint.has_attribute('is_invincible'):
                    blueprint.set_attribute('is_invincible', 'false')
                t = carla.Transform(carla.Location(spec['x'], -spec['y'], spec['z']), carla.Rotation(yaw=-spec['yaw']))
                actor = w.try_spawn_actor(blueprint, t)
                if actor is None:
                    raise ValueError('배치 위치가 다른 객체와 겹칩니다. 위치를 옮겨 주세요.')
                self.owned[actor.id] = {'role': role, 'type': actor.type_id, 'spec_id': spec['id']}
                self.persist()
                if spec['mode'] == 'autopilot':
                    tm = self.client.get_trafficmanager(8000)
                    tm.auto_lane_change(actor, False)
                    tm.distance_to_leading_vehicle(actor, 3.0)
                    tm.ignore_lights_percentage(actor, 0)
                    tm.ignore_vehicles_percentage(actor, 0)
                    tm.ignore_walkers_percentage(actor, 0)
                    tm.set_desired_speed(actor, spec['speed'])
                    actor.set_autopilot(True, 8000)
                elif spec['kind'] == 'vehicle':
                    actor.apply_control(carla.VehicleControl(brake=1, hand_brake=True))
                elif spec['mode'] == 'walk':
                    self.walkers[actor.id] = {'target': spec['target'], 'speed': spec['speed']}
                else:
                    actor.apply_control(carla.WalkerControl(speed=0))
        except BaseException:
            self.stop()
            raise
        return f'시나리오 객체 {len(self.owned)}개를 실행했습니다.'

    def traffic(self, vehicles, walkers):
        for v, limit in ((vehicles, 300), (walkers, 800)):
            number(v, 0, limit, '개체 수')
            if not isinstance(v, int):
                raise ValueError('개체 수는 정수여야 합니다.')
        config = json.loads(TRAFFIC_CONFIG.read_text())
        config.update(vehicles=vehicles, walkers=walkers)
        atomic_json(TRAFFIC_CONFIG, config)
        result = subprocess.run([str(ROOT/'scripts/carla/carla_city_traffic'), 'start'], capture_output=True, text=True, timeout=10)
        if result.returncode:
            raise RuntimeError('설정은 저장됐지만 배경 교통을 시작하지 못했습니다: '+result.stderr[-300:])
        return f'주변 차량 {vehicles}대 · 보행자 {walkers}명으로 조정 중입니다.'
