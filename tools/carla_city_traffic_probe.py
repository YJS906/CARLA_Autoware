#!/usr/bin/env python3
"""Read-only population/motion/frame-rate sample; never advances CARLA."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time

import carla


def population(world, snapshot):
    result = {}
    for actor in world.get_actors():
        if not actor.type_id.startswith(('vehicle.', 'walker.pedestrian.')):
            continue
        state = snapshot.find(actor.id)
        if state is None:
            continue
        loc = state.get_transform().location
        velocity = state.get_velocity()
        result[actor.id] = dict(type=actor.type_id, role=actor.attributes.get('role_name',''),
            xyz=[loc.x,loc.y,loc.z], speed_mps=velocity.length())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=20)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if not 5 <= args.duration <= 60:
        parser.error('duration must be 5..60 wall seconds')
    client=carla.Client('127.0.0.1',2000);client.set_timeout(10)
    world=client.get_world();first=world.wait_for_tick(5)
    initial=population(world,first);start=time.monotonic();last=start;intervals=[];last_frame=first.frame
    while time.monotonic()-start<args.duration:
        snap=world.wait_for_tick(5)
        now=time.monotonic()
        if snap.frame<=last_frame:continue
        intervals.append((now-last)/(snap.frame-last_frame));last=now;last_frame=snap.frame
    elapsed=last-start;sim_elapsed=snap.timestamp.elapsed_seconds-first.timestamp.elapsed_seconds
    final=population(world,snap)
    egos=[a for a in final.values() if a['role'] in ('ego_vehicle','hero','ego')]
    def is_group(a, group):
        return a['type'].startswith('vehicle.' if group=='vehicles' else 'walker.pedestrian.') and a not in egos
    groups={}
    for kind in ('vehicles','walkers'):
        rows=[(i,a) for i,a in final.items() if is_group(a,kind)]
        distances=[math.dist(a['xyz'],initial[i]['xyz']) for i,a in rows if i in initial]
        groups[kind]={'count':len(rows), 'moving_now_reported':sum(a['speed_mps']>.2 for _,a in rows),
            'survived_sample':len(distances),'moved_over_1m':sum(v>1 for v in distances),
            'mean_reported_speed_mps':statistics.mean(a['speed_mps'] for _,a in rows) if rows else 0,
            'mean_displacement_speed_mps':statistics.mean(distances)/sim_elapsed if distances and sim_elapsed>0 else 0,
            'within_100m_of_ego':sum(any(math.dist(a['xyz'],e['xyz'])<100 for e in egos) for _,a in rows)}
    intervals.sort()
    result={'utc_epoch':time.time(),'world_id':world.id,'map':world.get_map().name,
        'duration_wall_s':elapsed,'world_fps':(snap.frame-first.frame)/elapsed,
        'simulation_speed_ratio':(snap.timestamp.elapsed_seconds-first.timestamp.elapsed_seconds)/elapsed,
        'motion_note':'AI walker reported velocity may be zero; movement is also verified from snapshot position changes.',
        'median_frame_wall_s':statistics.median(intervals),
        'p95_frame_wall_s':intervals[min(len(intervals)-1,int(len(intervals)*.95))],
        'groups':groups,'ego':egos,'actors':final}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='actors'}))


if __name__=='__main__':main()
