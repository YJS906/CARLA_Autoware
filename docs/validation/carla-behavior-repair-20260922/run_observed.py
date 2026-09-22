"""Own a read-only AEB observer for exactly one ordinary validation process."""
import argparse
from pathlib import Path
import signal
import subprocess
import time

p=argparse.ArgumentParser()
p.add_argument('--case',required=True)
p.add_argument('--run',type=int,default=1)
p.add_argument('--duration',type=float,default=60)
a=p.parse_args()
base=Path('/tmp/carla-validation')
stem=f'{a.case}_run{a.run}'
log=base/(stem+'_aeb.log')
with log.open('w') as output:
    observer=subprocess.Popen(['python3',str(base/'carla_aeb_observer.py'),'--duration','180',
        '--output',str(base/(stem+'_aeb.json'))],stdout=output,stderr=output)
    try:
        deadline=time.monotonic()+15
        while 'AEB_OBSERVER_READY' not in log.read_text():
            if observer.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('AEB observer not ready: '+log.read_text())
            time.sleep(.1)
        result=subprocess.run(['python3',str(base/'carla_closed_loop_validation.py'),
            '--case',a.case,'--input-mode','ground_truth','--speed-kmh','20',
            '--duration',str(a.duration),'--output',str(base/(stem+'.json'))])
    finally:
        if observer.poll() is None:
            observer.send_signal(signal.SIGINT)
            observer.wait(timeout=10)
raise SystemExit(result.returncode)
