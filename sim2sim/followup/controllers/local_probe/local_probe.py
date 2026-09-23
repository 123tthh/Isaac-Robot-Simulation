"""Native controller measurements: no WebSocket, no position teleport writes."""
from pathlib import Path
import json,os,sys
from omnisim import Supervisor
r=Supervisor();dt=int(r.getBasicTimeStep());mode=sys.argv[1]
sys.path.insert(0,str(Path(os.environ['OMNISIM_HOME'])/'projects/default/controllers/harness_supervisor'))
import observe
out=Path(os.environ['OMNI_PROBE_OUTPUT']);result={'mode':mode,'dt_ms':dt,'samples':[]}
def save():out.write_text(json.dumps(result,indent=2)+'\n')
if mode=='contact':
 node=r.getSelf();result['initial']={'t':r.getTime(),'position':node.getPosition()}
 for step in range(round(2560/dt)):
  if r.step(dt)==-1:break
  result['samples'].append({'step':step+1,'t':r.getTime(),'position':node.getPosition(),'velocity':node.getVelocity(),'contacts':[{'point':p.point,'node_id':p.node_id,'depth':p.depth} for p in node.getContactPoints()]})
 result['final']=result['samples'][-1];z=result['final']['position'][2]
 result['stable_on_floor']=abs(z-.045)<.003 and abs(result['final']['velocity'][2])<.01
else:
 index=observe.joint_write_index(r,'R1');result['limits']=[{k:v for k,v in e.items() if k not in ['node','params']} for e in index]
 result['harness_snapshot']=observe.list_joints(r,'R1',{},r.getTime()*1000)
 names=['left_joint7_motor','right_joint7_motor'] if mode=='joint' else [e['name'] for e in index]
 devices={};sensors={}
 for e in index:
  m=r.getDevice(e['name'])
  if m is None:continue
  sensor=m.getPositionSensor()
  if sensor:sensor.enable(dt)
  if m.getMinPosition()!=m.getMaxPosition():m.setPosition(max(m.getMinPosition(),min(0,m.getMaxPosition())))
  if e['name'] in names:
   devices[e['name']]=m;sensors[e['name']]=sensor
 result['motor_device_limits']={n:{'min':m.getMinPosition(),'max':m.getMaxPosition()} for n,m in devices.items()}
 phases=[0,.25,-.25,0] if mode!='range' else [0,-2,-3.5,-6.283,0]
 for target in phases:
  goals={n:(max(-2, min(target,2)) if n=='j_inside_motor' else target) for n in devices}
  # Intent comes from authored probe bounds, never from the possibly defective readback.
  for n,m in devices.items():m.setVelocity(min(1,m.getMaxVelocity()));m.setPosition(goals[n])
  for step in range(round((1.5 if mode!='range' else 8)*1000/dt)):
   if r.step(dt)==-1:break
   if step%10==0:result['samples'].append({'t':r.getTime(),'goals':goals,'sensor_positions':{n:s.getValue() if s else None for n,s in sensors.items()}})
  result.setdefault('phases',[]).append({'target':target,'goals':goals,'t':r.getTime(),'sensor_positions':{n:s.getValue() if s else None for n,s in sensors.items()},'endpoint_orientation':{e['name']:e['node'].getField('endPoint').getSFNode().getOrientation() for e in index if e['name'] in names},'harness_snapshot':observe.list_joints(r,'R1',{},r.getTime()*1000)})
  save()
save();print('LOCAL_PROBE_COMPLETE',out,flush=True);r.simulationQuit(0)
