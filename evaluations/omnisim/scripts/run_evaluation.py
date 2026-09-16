#!/usr/bin/env python3
"""Replay a fixed gripper cycle; change only a rigid target object's pose."""
import argparse,csv,json,math,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
EVAL=ROOT/'evaluations/omnisim'

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--url',default='http://127.0.0.1:6989');a=p.parse_args()
 out=EVAL/'results';out.mkdir(exist_ok=True)
 def rpc(path,data=None):
  req=urllib.request.Request(a.url+path,None if data is None else json.dumps(data).encode(),{'Content-Type':'application/json'})
  with urllib.request.urlopen(req,timeout=120) as r:return json.load(r)
 template=(EVAL/'worlds/r1_gripper.omniworld').read_text()
 # Same rigid cube and fixed open/close sequence; only the cube translation varies.
 rows=[]
 for i,pose in enumerate(([0,.35,.8],[.05,.35,.8],[.10,.35,.8])):
  world=EVAL/f'worlds/pose_{i}.omniworld'
  world.write_text(template+'''\nDEF TARGET Robot {
 physicsBackend "newton"
 translation %s
 name "target"
 children [Shape { appearance PBRAppearance {baseColor 0.8 0.2 0.1 roughness 1} geometry Box {size 0.04 0.04 0.04} }]
 boundingObject Box {size 0.04 0.04 0.04}
 physics Physics {density -1 mass 0.05}
}\n'''%' '.join(map(str,pose)))
  d={'initial_pose':pose,'load':rpc('/world/load',{'path':str(world),'wait_s':60})}
  d['mapping']=rpc('/robot/R1/joints')
  d['open']=rpc('/robot/R1/joints/set',{'joints':{'left_PGIA_joint1_motor':.08,'left_PGIA_joint2_motor':.08},'settle_steps':80})
  d['contacts_open']=rpc('/sim/contacts')
  d['close']=rpc('/robot/R1/joints/set',{'joints':{'left_PGIA_joint1_motor':.04,'left_PGIA_joint2_motor':.04},'settle_steps':80})
  d['contacts_close']=rpc('/sim/contacts')
  d['target_final']=rpc('/scene/node/TARGET')
  (out/f'pose_{i}.json').write_text(json.dumps(d,indent=2))
  joint_errors=[abs(j['error']) for j in d['close'].get('joints',{}).values()]
  contact_text=json.dumps(d['contacts_close'])
  row={'variant':i,'object_pose':str(pose),'max_gripper_error_m':max(joint_errors,default=float('nan')),
       'object_contact_reported':'TARGET' in contact_text,
       'terminal_object_displacement_m':math.dist(pose,d['target_final']['position']),'gripper_cycle_complete':bool(joint_errors) and max(joint_errors)<.003,
       'grasp_completion':'not demonstrated','commanded_range_fraction':(.08-.04)/(.12-.04)}
  rows.append(row);print(row,flush=True)
 with (out/'summary.csv').open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
 (out/'summary.json').write_text(json.dumps(rows,indent=2))

if __name__=='__main__':main()
