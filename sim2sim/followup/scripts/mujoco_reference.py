#!/usr/bin/env python3
"""Local primitive parameter sweep; not an OmniSim/Newton patched-build validation."""
from pathlib import Path
import json,mujoco
OUT=Path(__file__).resolve().parents[3]/'outputs/omnisim_followup'/('mujoco_'+mujoco.__version__); OUT.mkdir(parents=True,exist_ok=True)
results=[]
cases=[('thin_box_16ms',.016,.05),('thin_box_8ms',.008,.05),('thin_box_4ms',.004,.05),('thick_box_16ms',.016,.2),('plane_16ms',.016,'plane'),('no_floor_16ms',.016,None)]
for name,dt,thickness in cases:
 if thickness=='plane':ground='<geom name="floor" type="plane" size="3 3 0.1" pos="0 0 0.025"/>'
 elif thickness is None:ground=''
 else:ground=f'<geom name="floor" type="box" size="3 3 {thickness/2}" pos="0 0 {.025-thickness/2}"/>'
 xml=f'<mujoco><option timestep="{dt}" gravity="0 0 -9.81"/><worldbody>{ground}<body name="cube" pos="0 0.35 0.8"><freejoint/><geom name="cube_geom" type="box" size="0.02 0.02 0.02" mass="0.05"/></body></worldbody></mujoco>'
 model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model);body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'cube')
 samples=[]
 for step in range(round(2.56/dt)):
  mujoco.mj_step(model,data);mujoco.mj_forward(model,data)
  samples.append({'step':step+1,'t':float(data.time),'z':float(data.xpos[body,2]),'contacts':int(data.ncon)})
 end=samples[-1];stable=abs(end['z']-.045)<.002 and end['contacts']>0
 results.append({'case':name,'dt':dt,'floor_thickness':thickness,'version':mujoco.__version__,'final':end,'stable_on_floor':stable,'samples':samples})
 (OUT/f'mujoco_{name}.xml').write_text(xml+'\n')
 print(name,end,stable,flush=True)
(OUT/'mujoco_primitive_control.json').write_text(json.dumps({'scope':'Independent native MuJoCo control, not an OmniSim patch validation; same floor top z=0.025 in all supported cases','results':results},indent=2)+'\n')
