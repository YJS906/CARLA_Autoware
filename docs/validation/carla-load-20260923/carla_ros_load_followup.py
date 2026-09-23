import rclpy,time,json,statistics
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from collections import defaultdict
rclpy.init();n=Node('carla_readonly_load_probe');q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
clock=[None];rows=defaultdict(list);diags={};subs=[]
def sec(s):return s.sec+s.nanosec*1e-9
def cb(topic,msg):
 now=time.monotonic();st=None
 if topic=='/clock':clock[0]=sec(msg.clock);st=clock[0]
 elif hasattr(msg,'header'):st=sec(msg.header.stamp)
 elif hasattr(msg,'stamp'):st=sec(msg.stamp)
 d={'wall':now,'stamp':st,'age_sim_s':clock[0]-st if st is not None and clock[0] is not None else None}
 if hasattr(msg,'data') and isinstance(msg.data,(float,int)):d['value']=msg.data
 if hasattr(msg,'objects'):d['objects']=len(msg.objects)
 if topic=='/diagnostics':
  for s in msg.status:
   level=int.from_bytes(s.level,'little') if isinstance(s.level,bytes) else int(s.level)
   if level:
    key=s.name+'|'+s.message
    diags[key]={'name':s.name,'level':level,'message':s.message,'values':{v.key:v.value for v in s.values},'count':diags.get(key,{}).get('count',0)+1}
 rows[topic].append(d)
base={'/clock','/localization/kinematic_state','/perception/object_recognition/tracking/objects','/perception/object_recognition/objects','/planning/trajectory','/control/command/control_cmd','/control/trajectory_follower/control_cmd','/diagnostics'}
time.sleep(1)
for topic,types in n.get_topic_names_and_types():
 if topic in base or (topic.endswith(('processing_time_ms','cyclic_time_ms')) and 'Float64Stamped' in types[0]):
  subs.append(n.create_subscription(get_message(types[0]),topic,lambda m,t=topic:cb(t,m),q))
t0=time.monotonic()
while time.monotonic()-t0<20:rclpy.spin_once(n,timeout_sec=.05)
def stats(v):
 v=sorted(v);return {'min':v[0],'mean':statistics.mean(v),'p95':v[min(len(v)-1,int(len(v)*.95))],'max':v[-1]} if v else None
summary={}
for topic,rs in rows.items():
 gaps=[b['wall']-a['wall'] for a,b in zip(rs,rs[1:])];ages=[r['age_sim_s'] for r in rs if r['age_sim_s'] is not None]
 summary[topic]={'count':len(rs),'wall_hz':(len(rs)-1)/(rs[-1]['wall']-rs[0]['wall']) if len(rs)>1 else None,'wall_gap_s':stats(gaps),'age_sim_s':stats(ages),'first_age':ages[0] if ages else None,'last_age':ages[-1] if ages else None,'value_ms':stats([r['value'] for r in rs if 'value' in r]),'objects':stats([r['objects'] for r in rs if 'objects' in r])}
result={'duration_wall_s':time.monotonic()-t0,'summary':summary,'non_ok_diagnostics':list(diags.values()),'raw':rows}
open('/tmp/carla-ros-load-followup.json','w').write(json.dumps(result,indent=2));print(json.dumps({'streams':{t:v for t,v in summary.items() if t in base},'non_ok_diagnostic_count':len(diags)}));n.destroy_node();rclpy.shutdown()
