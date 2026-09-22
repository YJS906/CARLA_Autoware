"""Sequential CARLA trials. Each child verifies STOP before another starts."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess

root=Path('/home/a/carla_pp')
out=Path('/tmp/carla-behavior-repair-20260922/final')
out.mkdir(exist_ok=True)
container='selfcar-carla-autoware'
base='/tmp/carla-validation/'
setup='source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && source /opt/carla_overlay/setup.bash && '
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
metadata={'image':subprocess.check_output(['docker','inspect','--format','{{.Image}}',container],text=True).strip(),
          'map_sha256':sha(Path('/home/a/autoware_data/maps/Town05/lanelet2_map.osm')),
          'mode':'ground_truth',
          'tools_sha256':{n:sha(root/'tools'/n) for n in ['carla_closed_loop_validation.py','carla_validation_actors.py','carla_aeb_observer.py','carla_native_lane_check.py']},
          'completed':[]}
(out/'environment.json').write_text(json.dumps(metadata,indent=2))
cases=[('baseline',1),('vehicle_stop',1),('pedestrian_stop',1),('vehicle_avoidance',1),
       ('pedestrian_stop',2),('vehicle_avoidance',2),('pedestrian_stop',3),('vehicle_avoidance',3)]
for case,run in cases:
    name=f'{case}_run{run}'
    if case=='baseline':
        args=['python3',base+'carla_closed_loop_validation.py','--case',case,'--input-mode','ground_truth',
              '--speed-kmh','20','--duration','45','--isolate-traffic','--output',base+name+'.json']
    else:
        args=['python3',base+'run_observed.py','--case',case,'--run',str(run),'--duration','90']
    print(json.dumps({'starting':name}),flush=True)
    with (out/(name+'.log')).open('w') as logfile:
        process=subprocess.run(['docker','exec',container,'bash','-lc',setup+shlex.join(args)],stdout=logfile,stderr=subprocess.STDOUT)
    for suffix in (['.json'] if case=='baseline' else ['.json','_aeb.json','_aeb.log']):
        subprocess.run(['docker','cp',container+':'+base+name+suffix,str(out/(name+suffix))],check=True)
    result=json.loads((out/(name+'.json')).read_text())
    entry={'name':name,'exit_code':process.returncode,'success':result['success'],'result':result['result']}
    metadata['completed'].append(entry)
    (out/'environment.json').write_text(json.dumps(metadata,indent=2))
    print(json.dumps(entry),flush=True)
    if process.returncode or not result['success'] or not result['cleanup'].get('stop_verified'):
        raise SystemExit('Suite halted; inspect '+str(out/(name+'.log')))
print('All eight trials completed successfully',flush=True)
