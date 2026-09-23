import psutil,time,json,subprocess,pathlib,statistics
out=pathlib.Path('/home/a/carla_pp/docs/validation/carla-load-20260923')
procs={p.pid:p for p in psutil.process_iter()}; [p.cpu_percent() for p in procs.values() if p.is_running()]
def threads():
 d={}
 for pid in (125685,307718,307753,307754):
  try:
   for t in psutil.Process(pid).threads():
    try:n=pathlib.Path(f'/proc/{pid}/task/{t.id}/comm').read_text().strip()
    except:n='?'
    d[f'{pid}:{t.id}:{n}']=t.user_time+t.system_time
  except:pass
 return d
def throttle():
 return {str(p):p.read_text().strip() for p in pathlib.Path('/sys/devices/system/cpu').glob('cpu*/thermal_throttle/*throttle_count')}
start=threads();th0=throttle();t0=time.monotonic();rows=[];psutil.cpu_percent(percpu=True);psutil.cpu_times_percent()
for i in range(40):
 time.sleep(1)
 row={'wall':time.time(),'cpu_per_core':psutil.cpu_percent(percpu=True),'cpu_times':psutil.cpu_times_percent()._asdict(),'memory':psutil.virtual_memory()._asdict(),'swap':psutil.swap_memory()._asdict(),'load':psutil.getloadavg(),'temperatures':{k:[x._asdict() for x in v] for k,v in psutil.sensors_temperatures().items()},'pressure':{k:pathlib.Path('/proc/pressure/'+k).read_text() for k in ('cpu','memory','io')},'processes':[]}
 for pid,p in procs.items():
  try:
   cpu=p.cpu_percent()
   if cpu>3:row['processes'].append({'pid':pid,'name':p.name(),'cpu_percent':cpu,'rss':p.memory_info().rss})
  except (psutil.NoSuchProcess,psutil.AccessDenied):pass
 row['gpu']=subprocess.check_output(['nvidia-smi','--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw,clocks.sm','--format=csv,noheader,nounits'],text=True).strip()
 rows.append(row)
end=threads();elapsed=time.monotonic()-t0
result={'elapsed':elapsed,'samples':rows,'thread_cpu_percent':sorted([{'thread':k,'cpu_percent':(v-start[k])/elapsed*100} for k,v in end.items() if k in start],key=lambda x:-x['cpu_percent'])[:25],'thermal_throttle_before':th0,'thermal_throttle_after':throttle()}
(out/'host-load.json').write_text(json.dumps(result,indent=2))
print(json.dumps({'cpu_mean_percent':statistics.mean(statistics.mean(r['cpu_per_core']) for r in rows),'top_threads':result['thread_cpu_percent'][:8],'gpu_first_last':[rows[0]['gpu'],rows[-1]['gpu']],'available_gib_min':min(r['memory']['available'] for r in rows)/2**30,'swap_io_delta':{k:rows[-1]['swap'][k]-rows[0]['swap'][k] for k in ('sin','sout')}}))
